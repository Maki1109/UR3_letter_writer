"""Cầu nối Gazebo Sim (gz-sim / Ignition Fortress): "gắp ảo" (virtual gripper).

UR3/UR3e mặc định không có tay gắp. Để khối di chuyển theo robot trong
Gazebo, khi skill close_gripper() được gọi, node này bám vị trí khung tool0
(qua TF) và liên tục đặt pose của khối ngay dưới flange bằng service gz-sim
gốc "/world/<world>/set_pose" (được ros_gz_bridge lộ ra dưới dạng service
ROS 2 kiểu ros_gz_interfaces/srv/SetEntityPose — xem launch/llm_robot.launch.py).
Khi open_gripper(), khối được đặt chính xác lên vùng đích.

Các khối được khai báo <kinematic>true</kinematic> + <gravity>false</gravity>
trong SDF (xem launch: hàm _box_sdf) nên không bị trọng lực/va chạm xô lệch;
chúng chỉ đổi chỗ khi node này gọi set_pose.

Không có service "get pose" tương đương trong gz-sim/ros_gz, nên vị trí ban
đầu của vật lấy thẳng từ config/scene.yaml (cũng chính là toạ độ dùng để
spawn vật trong launch) thay vì đọc lại từ simulator lúc khởi động.

Ghi chú: sau khi thả vật (đặc biệt sau một chuỗi set_pose liên tục lúc đang
"gắp"), vật kinematic có thể trôi nhẹ theo vận tốc engine vật lý tự suy ra từ
các lần đổi pose gần nhau (dù đã tắt gravity). Để triệt tiêu, GazeboBridge
định kỳ gửi lại đúng pose hiện tại của MỌI vật KHÔNG đang bị cầm (xem
resting_positions_fn) — vài lần gọi set_pose liên tiếp cùng toạ độ sẽ đưa
vận tốc suy ra về 0.
"""
from __future__ import annotations

import threading
from typing import Callable, Dict, Optional, Tuple

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.time import Time
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from tf2_ros import Buffer, TransformListener

DEFAULT_WORLD_NAME = "llm_table_world"  # phải khớp <world name="..."> trong worlds/llm_table.world


class GazeboBridge:
    def __init__(self, node: Node, frame_id: str, world_name: str = DEFAULT_WORLD_NAME,
                 tool_frame: str = "tool0", enabled: bool = True,
                 resting_positions_fn: Optional[Callable[[], Dict[str, Tuple[float, float, float]]]] = None):
        self.node = node
        self.frame_id = frame_id
        self.tool_frame = tool_frame
        self.enabled = enabled
        self.resting_positions_fn = resting_positions_fn
        self._cb = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._carry: Optional[Tuple[str, float]] = None   # (entity, z-offset dưới tool0)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        if not enabled:
            return
        self.set_cli = node.create_client(
            SetEntityPose, f"/world/{world_name}/set_pose", callback_group=self._cb)
        self.timer = node.create_timer(0.03, self._follow_tool, callback_group=self._cb)
        if resting_positions_fn is not None:
            self.keepalive_timer = node.create_timer(0.2, self._keepalive, callback_group=self._cb)

    # ------------------------------------------------------------------
    def wait_ready(self, timeout: float = 10.0) -> bool:
        if not self.enabled:
            return False
        ok = self.set_cli.wait_for_service(timeout_sec=timeout)
        if not ok:
            self.node.get_logger().warn(
                "gz-sim 'set_pose' service not found -> chạy KHÔNG có đồng bộ Gazebo "
                "(kiểm tra bridge trong launch và world/llm_table.world).")
            self.enabled = False
        return ok

    def set_pose(self, entity: str, x: float, y: float, z: float) -> bool:
        if not self.enabled:
            return False
        req = SetEntityPose.Request()
        req.entity = Entity(name=entity, type=Entity.MODEL)
        req.pose.position.x, req.pose.position.y, req.pose.position.z = x, y, z
        req.pose.orientation.w = 1.0
        res = self._call(self.set_cli, req, 1.0)
        return bool(res and res.success)

    # ------------------------------------------------------------------
    def start_carry(self, entity: str, z_offset: float) -> None:
        """Bắt đầu "gắp": khối bám theo tool0, tâm khối thấp hơn tool0 z_offset."""
        with self._lock:
            self._carry = (entity, z_offset)

    def stop_carry(self) -> None:
        with self._lock:
            self._carry = None

    def _follow_tool(self) -> None:
        with self._lock:
            carry = self._carry
        if carry is None or not self.enabled:
            return
        entity, dz = carry
        try:
            tf = self.tf_buffer.lookup_transform(self.frame_id, self.tool_frame, Time())
        except Exception:
            return
        t = tf.transform.translation
        # tool0 chúc xuống => tâm khối nằm ngay dưới flange theo trục -z của world
        self._set_async(entity, t.x, t.y, t.z - dz)

    def _keepalive(self) -> None:
        """Gửi lại pose hiện tại của mọi vật đang KHÔNG bị cầm, để triệt tiêu
        trôi vận tốc dư (xem ghi chú ở đầu file)."""
        if self.resting_positions_fn is None:
            return
        with self._lock:
            carried = self._carry[0] if self._carry else None
        try:
            positions = self.resting_positions_fn()
        except Exception:
            return
        for entity, (x, y, z) in positions.items():
            if entity != carried:
                self._set_async(entity, x, y, z)

    def _set_async(self, entity: str, x: float, y: float, z: float) -> None:
        req = SetEntityPose.Request()
        req.entity = Entity(name=entity, type=Entity.MODEL)
        req.pose.position.x, req.pose.position.y, req.pose.position.z = x, y, z
        req.pose.orientation.w = 1.0
        self.set_cli.call_async(req)

    def _call(self, client, req, timeout: float):
        fut = client.call_async(req)
        ev = threading.Event()
        fut.add_done_callback(lambda _f: ev.set())
        if not ev.wait(timeout):
            return None
        return fut.result()
