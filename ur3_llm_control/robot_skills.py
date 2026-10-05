"""Robot Skills: các hành vi mức cao, chuyển động qua MoveIt 2, gắp bằng tay gắp thật.

Skill mức cao (LLM được phép gọi, xem task_validator.SKILL_SPECS):
    detect_objects(), check_zone(zone), pick(object), place(object, zone), home()
    (zone của place có thể là "temporary_position": hệ thống tự tìm chỗ trống)
Skill / hàm nội bộ (LLM KHÔNG gọi trực tiếp):
    find_object(object), find_free_position(object), open_gripper(), close_gripper()

Mọi chuyển động tay máy đi qua MoveIt 2 (pymoveit2: /move_action,
/compute_cartesian_path, /execute_trajectory) nên MoveIt kiểm tra giới hạn khớp,
tự va chạm (kể cả tay gắp) và va chạm với bàn / các khối (lấy từ camera, đưa vào
Planning Scene). Tay gắp: 2 ngón điều khiển vị trí qua ros2_control; vật được giữ
bằng lực kẹp + ma sát trong Gazebo — chương trình không bao giờ đặt pose của vật.

Mỗi skill trả về một SkillStatus; chi tiết (vd. "zone_b occupied by blue_cube")
ghi vào self.last_detail để Skill Executor in ra.
"""
from __future__ import annotations

import math
import threading
import time
from enum import Enum
from typing import Callable, Dict, List, Optional, Tuple

from pymoveit2 import MoveIt2
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from .free_space import find_free_position
from .scene_config import TEMP_POSITION, SceneConfig, WorldState

UR_JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
GRIPPER_LINKS = ["gripper_palm", "gripper_left_finger", "gripper_right_finger", "tool0", "wrist_3_link"]


class SkillStatus(str, Enum):
    SUCCESS = "SUCCESS"
    FAILED = "FAILED"
    INVALID_OBJECT = "INVALID_OBJECT"
    INVALID_ZONE = "INVALID_ZONE"
    PRECONDITION_FAILED = "PRECONDITION_FAILED"   # vd: place khi không cầm vật
    OBJECT_NOT_FOUND = "OBJECT_NOT_FOUND"         # camera không thấy vật
    ZONE_OCCUPIED = "ZONE_OCCUPIED"
    NO_FREE_POSITION = "NO_FREE_POSITION"
    GRASP_FAILED = "GRASP_FAILED"                 # ngón tay khép hết = không kẹp được vật
    PERCEPTION_FAILED = "PERCEPTION_FAILED"       # không có ảnh camera
    PLANNING_FAILED = "PLANNING_FAILED"
    EXECUTION_FAILED = "EXECUTION_FAILED"


def tool_quat(yaw: float) -> Tuple[float, float, float, float]:
    """Tool chúc xuống (quay 180° quanh x) rồi xoay yaw quanh z của base: Rz(yaw) * Rx(pi)."""
    return (math.cos(yaw / 2.0), math.sin(yaw / 2.0), 0.0, 0.0)


def yaw_quat(yaw: float) -> Tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class Gripper:
    """Tay gắp 2 ngón: lệnh vị trí cho gripper_controller, đọc vị trí ngón từ /joint_states."""

    def __init__(self, node: Node, cfg: dict):
        self.node = node
        self.joints = list(cfg.get("joints", ["gripper_left_finger_joint", "gripper_right_finger_joint"]))
        self.open_pos = float(cfg.get("open_position", 0.04))
        self.close_pos = float(cfg.get("close_position", 0.0))
        self.anchor_pos = float(cfg.get("anchor_position", 0.02))
        self.min_gap = float(cfg.get("grasp_min_gap", 0.03))
        self.max_gap = float(cfg.get("grasp_max_gap", 0.05))
        self.settle_time = float(cfg.get("settle_time", 2.5))
        self.pub = node.create_publisher(Float64MultiArray, cfg.get("command_topic",
                                                                   "/gripper_controller/commands"), 10)
        self._lock = threading.Lock()
        self._pos: Dict[str, float] = {}
        node.create_subscription(JointState, "/joint_states", self._on_js, 20,
                                 callback_group=ReentrantCallbackGroup())

    def _on_js(self, msg: JointState) -> None:
        with self._lock:
            for n, p in zip(msg.name, msg.position):
                if n in self.joints:
                    self._pos[n] = p

    def positions(self) -> List[float]:
        with self._lock:
            return [round(self._pos.get(j, float("nan")), 4) for j in self.joints]

    def gap(self) -> Optional[float]:
        """Khe hở giữa hai mặt trong của ngón (m)."""
        with self._lock:
            if not all(j in self._pos for j in self.joints):
                return None
            return sum(self._pos[j] for j in self.joints)

    def command(self, q: List[float]) -> Optional[float]:
        """Gửi lệnh vị trí [ngón trái, ngón phải] và chờ đến khi ngón dừng (hoặc hết thời gian)."""
        self.pub.publish(Float64MultiArray(data=list(q)))
        t0 = time.time()
        last, still_since = None, None
        while time.time() - t0 < self.settle_time:
            time.sleep(0.05)
            g = self.gap()
            if g is None:
                continue
            if last is not None and abs(g - last) < 2e-4:
                still_since = still_since or time.time()
                if time.time() - still_since > 0.3 and time.time() - t0 > 0.4:
                    break
            else:
                still_since = None
            last = g
        return self.gap()

    def open(self) -> Optional[float]:
        return self.command([self.open_pos, self.open_pos])

    def close(self) -> Optional[float]:
        """Ngón tựa tới nửa cạnh vật, ngón kẹp đóng hết (bị vật chặn) -> kẹp chặt, vật ở giữa."""
        return self.command([self.anchor_pos, self.close_pos])

    def is_holding(self, gap: Optional[float]) -> bool:
        return gap is not None and self.min_gap <= gap <= self.max_gap


class RobotSkills:
    def __init__(self, node: Node, scene: SceneConfig, world: WorldState,
                 perceive: Callable[[], Optional[Dict]],
                 group_name: str = "ur_manipulator", base_link: str = "base_link",
                 ee_link: str = "tool0"):
        """perceive(): chụp ảnh MỚI và trả về {tên: Detection} (None nếu không có ảnh)."""
        self.node = node
        self.log = node.get_logger()
        self.scene = scene
        self.world = world
        self.perceive = perceive
        self.ee_link = ee_link
        self.base_link = base_link
        self.last_detail = ""
        self.at_home = False

        m, g = scene.motion, scene.gripper
        self.approach_h = float(m.get("approach_height", 0.10))
        self.grasp_z = float(m.get("grasp_height", 0.075))
        self.release_z = float(m.get("release_height", 0.078))
        self.tcp = float(g.get("tcp_offset", 0.062))
        self.home_joints = list(m.get("home_joints", [0, -1.5708, 0, -1.5708, 0, 0]))
        self.ik_seed = list(m.get("ik_seed_joints", [0, -1.5708, 1.5708, -1.5708, -1.5708, 0]))
        self.cart_step = float(m.get("cartesian_max_step", 0.005))
        self.cart_min_fraction = float(m.get("cartesian_min_fraction", 0.95))
        self.visible_ratio = float(scene.perception.get("visible_area_ratio", 0.70))
        # pymoveit2.plan(..., timeout_sec=None) có thể chờ vô thời hạn nếu move_group không
        # trả lời -> luôn chặn trên bằng planning_time * planning_attempts (+ dư).
        self.plan_timeout = float(m.get("planning_time", 5.0)) * int(m.get("planning_attempts", 10)) + 10.0
        self.free_move_attempts = 8

        self.gripper = Gripper(node, g)
        self.moveit2 = MoveIt2(
            node=node, joint_names=UR_JOINTS, base_link_name=base_link,
            end_effector_name=ee_link, group_name=group_name,
            callback_group=ReentrantCallbackGroup())
        self._try_set("max_velocity", float(m.get("max_velocity", 0.3)))
        self._try_set("max_acceleration", float(m.get("max_acceleration", 0.3)))
        self._try_set("allowed_planning_time", float(m.get("planning_time", 5.0)))
        self._try_set("num_planning_attempts", int(m.get("planning_attempts", 10)))

    def _try_set(self, attr: str, value) -> None:
        try:
            setattr(self.moveit2, attr, value)
        except Exception as e:  # khác biệt giữa các phiên bản pymoveit2
            self.log.warn(f"pymoveit2: cannot set {attr}: {e}")

    # ==================================================================
    # Planning scene: bàn (cố định) + các khối (từ camera)
    # ==================================================================
    def setup_planning_scene(self) -> None:
        s = self.scene
        # Hạ mặt bàn trong planning scene 4 mm: khối vừa được attach (đang chạm bàn) không
        # bị MoveIt coi là "đang va chạm" làm hỏng start state của đoạn nhấc lên.
        margin = 0.004
        size = (s.table_size[0], s.table_size[1], s.table_size[2] - margin)
        center = (s.table_center[0], s.table_center[1], s.table_center[2] - margin / 2.0)
        self.moveit2.add_collision_box(id="table", size=size, position=center,
                                       quat_xyzw=(0.0, 0.0, 0.0, 1.0), frame_id=s.frame_id)

    def _sync_collision_objects(self) -> None:
        c = self.scene.cube_size
        for obj in self.scene.objects:
            if obj == self.world.holding:
                continue
            pos = self.world.positions.get(obj)
            if pos is None:
                self.moveit2.remove_collision_object(id=obj)
                continue
            x, y, yaw = pos
            self.moveit2.add_collision_box(
                id=obj, size=(c, c, c), position=(x, y, self.scene.cube_center_z()),
                quat_xyzw=yaw_quat(yaw), frame_id=self.scene.frame_id)

    # ==================================================================
    # Nhận thức (camera)
    # ==================================================================
    def refresh_world(self) -> Optional[Dict]:
        """Chụp ảnh mới -> cập nhật vị trí mọi khối KHÔNG bị cầm + planning scene.

        Khối bị che khuất một phần (visible_ratio thấp) giữ vị trí cũ nếu đã biết.
        Khối không thấy: nếu robot đang ở home (không che camera) thì xoá khỏi world.
        """
        dets = self.perceive()
        if dets is None:
            return None
        for obj in self.scene.objects:
            if obj == self.world.holding:
                continue
            d = dets.get(obj)
            if d is not None and (d.visible_ratio >= self.visible_ratio or obj not in self.world.positions):
                self.world.positions[obj] = d.pose
            elif d is None and self.at_home:
                self.world.positions.pop(obj, None)
        self.world.stamp = time.time()
        self._sync_collision_objects()
        return dets

    def detect_objects(self) -> SkillStatus:
        if self.refresh_world() is None:
            self.last_detail = "no camera image"
            return SkillStatus.PERCEPTION_FAILED
        seen = [f"{o}@{self.world.location_of(o)}" for o in self.scene.objects
                if self.world.location_of(o) != "not detected"]
        self.last_detail = f"{len(seen)} objects: " + ", ".join(seen)
        return SkillStatus.SUCCESS

    def check_zone(self, zone: str) -> SkillStatus:
        if zone not in self.scene.zones:
            return SkillStatus.INVALID_ZONE
        if self.refresh_world() is None:
            self.last_detail = "no camera image"
            return SkillStatus.PERCEPTION_FAILED
        occ = self.world.occupant_of(zone)
        self.last_detail = f"{zone} occupied by {occ}" if occ else f"{zone} is free"
        return SkillStatus.SUCCESS

    def find_object(self, obj: str) -> Optional[Tuple[float, float, float]]:
        """Vị trí mới nhất của vật từ camera. Nếu vật bị tay máy che khuất -> về home, chụp lại."""
        dets = self.perceive()
        d = dets.get(obj) if dets else None
        if (d is None or d.visible_ratio < self.visible_ratio) and not self.at_home:
            self.log.info(f"find_object({obj}): occluded/not visible -> moving home to look again")
            if self.home() == SkillStatus.SUCCESS:
                dets = self.perceive()
                d = dets.get(obj) if dets else None
        if d is not None:
            self.world.positions[obj] = d.pose
            self._sync_collision_objects()
            return d.pose
        return None

    # ==================================================================
    # Primitive chuyển động (đều qua MoveIt 2)
    # ==================================================================
    def _tool0_z(self, tcp_z: float) -> float:
        """tool0 nằm phía trên tâm vùng kẹp (TCP) một đoạn tcp_offset (tool chúc xuống)."""
        return tcp_z + self.tcp

    def _move_free(self, xyz: Tuple[float, float, float], yaw: float = 0.0) -> SkillStatus:
        """Chuyển động tự do (OMPL) tới pose của tool0, tool chúc xuống, xoay yaw.

        Giải IK với seed cố định (ik_seed_joints: nhánh elbow-up hướng ra trước) rồi plan theo
        khớp; loại quỹ đạo "xoay cả vòng" (nghiệm IK lệch 2π).
        """
        quat = tool_quat(yaw)
        goal = self._ik_from_home(xyz, quat)
        for _ in range(self.free_move_attempts):
            if goal is not None:
                traj = self.moveit2.plan(joint_positions=goal, joint_names=UR_JOINTS,
                                         timeout_sec=self.plan_timeout)
            else:
                traj = self.moveit2.plan(position=xyz, quat_xyzw=quat,
                                         frame_id=self.scene.frame_id, target_link=self.ee_link,
                                         timeout_sec=self.plan_timeout)
            if traj is None:
                self.log.info("free move: plan failed, retrying")
            elif not self._is_compact(traj):
                self.log.info("free move: rejected wide-swing plan, retrying")
            else:
                st = self._execute(traj)
                if st == SkillStatus.SUCCESS:
                    self.at_home = False
                return st
            if goal is None:
                goal = self._ik_from_home(xyz, quat)
        return SkillStatus.PLANNING_FAILED

    def _ik_from_home(self, xyz, quat):
        js = self.moveit2.compute_ik(position=xyz, quat_xyzw=quat, ik_link_name=self.ee_link,
                                     start_joint_state=self.ik_seed, timeout_sec=5.0)
        if js is None:
            return None
        by_name = dict(zip(js.name, js.position))
        if not all(j in by_name for j in UR_JOINTS):
            return None
        goal = [float(by_name[j]) for j in UR_JOINTS]
        return goal if all(abs(v) <= math.pi + 0.05 for v in goal) else None

    @staticmethod
    def _is_compact(traj) -> bool:
        pts = traj.points
        if not pts:
            return False
        for i in range(len(pts[0].positions)):
            vals = [p.positions[i] for p in pts]
            if max(vals) - min(vals) > 3.3 or abs(vals[-1]) > math.pi + 0.05:
                return False
        return True

    def _move_linear(self, xyz: Tuple[float, float, float], yaw: float = 0.0) -> SkillStatus:
        """Chuyển động thẳng Cartesian (hạ/nâng tool) — compute_cartesian_path."""
        traj = self.moveit2.plan(position=xyz, quat_xyzw=tool_quat(yaw),
                                 frame_id=self.scene.frame_id, target_link=self.ee_link,
                                 cartesian=True, cartesian_max_step=self.cart_step,
                                 cartesian_fraction_threshold=self.cart_min_fraction,
                                 timeout_sec=self.plan_timeout)
        return self._execute(traj)

    def _execute(self, traj, timeout_sec: float = 45.0) -> SkillStatus:
        if traj is None:
            return SkillStatus.PLANNING_FAILED
        self.moveit2.execute(traj)
        ok = self.moveit2.wait_until_executed(timeout_sec=timeout_sec, cancel_on_timeout=True)
        return SkillStatus.SUCCESS if ok in (True, None) else SkillStatus.EXECUTION_FAILED

    # ==================================================================
    # Tay gắp
    # ==================================================================
    def open_gripper(self) -> SkillStatus:
        gap = self.gripper.open()
        if self.world.holding is not None:
            self.moveit2.detach_collision_object(id=self.world.holding)
            self.world.holding = None
        return SkillStatus.SUCCESS if gap is not None else SkillStatus.EXECUTION_FAILED

    def close_gripper(self, obj: str) -> Tuple[SkillStatus, Optional[float]]:
        """Khép ngón; nếu khe hở dừng lại ≈ cạnh khối => đã kẹp được vật -> attach vào
        planning scene (MoveIt tính cả vật đang cầm khi tránh va chạm)."""
        gap = self.gripper.close()
        self.log.info(f"close_gripper({obj}): fingers {self.gripper.positions()} gap={gap}")
        if not self.gripper.is_holding(gap):
            return SkillStatus.GRASP_FAILED, gap
        self.moveit2.attach_collision_object(id=obj, link_name=self.ee_link, touch_links=GRIPPER_LINKS)
        self.world.holding = obj
        return SkillStatus.SUCCESS, gap

    # ==================================================================
    # Skill mức cao
    # ==================================================================
    def home(self) -> SkillStatus:
        traj = self.moveit2.plan(joint_positions=self.home_joints, joint_names=UR_JOINTS,
                                 timeout_sec=self.plan_timeout)
        st = self._execute(traj)
        self.at_home = st == SkillStatus.SUCCESS
        return st

    def pick(self, obj: str, attempts: int = 2) -> SkillStatus:
        if obj not in self.scene.objects:
            return SkillStatus.INVALID_OBJECT
        if self.world.holding is not None:
            return SkillStatus.PRECONDITION_FAILED
        st = SkillStatus.FAILED
        for attempt in range(attempts):
            pose = self.find_object(obj)                              # 1. camera: vật ở đâu
            if pose is None:
                self.last_detail = "camera cannot see the object"
                return SkillStatus.OBJECT_NOT_FOUND
            x, y, yaw = pose
            z_pre = self._tool0_z(self.grasp_z + self.approach_h)
            z_grasp = self._tool0_z(self.grasp_z)
            self.gripper.open()                                       # 2. mở tay gắp
            st = self._move_free((x, y, z_pre), yaw)                  # 3. tới phía trên vật
            if st != SkillStatus.SUCCESS:
                return st
            st = self._move_linear((x, y, z_grasp), yaw)              # 4. hạ thẳng xuống
            if st != SkillStatus.SUCCESS:
                return st
            st, gap = self.close_gripper(obj)                         # 5. kẹp
            if st == SkillStatus.SUCCESS:
                st = self._move_linear((x, y, z_pre), yaw)            # 6. nhấc thẳng lên
                if st != SkillStatus.SUCCESS:
                    return st
                gap = self.gripper.gap()
                if self.gripper.is_holding(gap):                      # 7. vẫn còn kẹp vật?
                    self.world.positions.pop(obj, None)
                    self.last_detail = (f"at x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.0f}deg, "
                                        f"finger gap {gap * 1000:.0f} mm")
                    return SkillStatus.SUCCESS
                self.log.warn(f"pick({obj}): object slipped (gap={gap})")
            self.log.warn(f"pick({obj}): grasp failed (gap={gap}), attempt {attempt + 1}/{attempts}")
            self.open_gripper()
            self._move_linear((x, y, z_pre), yaw)
            self.moveit2.detach_collision_object(id=obj)
            self.world.holding = None
            st = SkillStatus.GRASP_FAILED
            self.last_detail = f"finger gap {gap * 1000:.0f} mm" if gap is not None else ""
        return st

    def find_free_position(self, obj: str) -> Optional[Tuple[float, float]]:
        """Chỗ trống trên bàn (từ trạng thái camera mới nhất)."""
        self.refresh_world()
        return find_free_position(self.scene, self.world, obj)

    def place(self, obj: str, zone: str) -> SkillStatus:
        if obj not in self.scene.objects:
            return SkillStatus.INVALID_OBJECT
        if zone not in self.scene.zones and zone != TEMP_POSITION:
            return SkillStatus.INVALID_ZONE
        if self.world.holding != obj:
            return SkillStatus.PRECONDITION_FAILED

        if zone == TEMP_POSITION:
            xy = self.find_free_position(obj)
            if xy is None:
                self.last_detail = "no free spot on the table"
                return SkillStatus.NO_FREE_POSITION
            x, y = xy
            self.last_detail = f"free spot x={x:.3f} y={y:.3f}"
        else:
            self.refresh_world()                                      # zone còn trống không?
            occ = self.world.occupant_of(zone)
            if occ is not None and occ != obj:
                self.last_detail = f"{zone} occupied by {occ}"
                return SkillStatus.ZONE_OCCUPIED
            x, y = self.scene.zones[zone].center_xy
            self.last_detail = ""

        yaw = 0.0                                                     # đặt thẳng hàng với bàn
        z_pre = self._tool0_z(self.release_z + self.approach_h)
        z_release = self._tool0_z(self.release_z)
        st = self._move_free((x, y, z_pre), yaw)                      # 1. tới phía trên đích
        if st != SkillStatus.SUCCESS:
            return st
        st = self._move_linear((x, y, z_release), yaw)                # 2. hạ thẳng xuống
        if st != SkillStatus.SUCCESS:
            return st
        self.log.info(f"place({obj}): before release fingers {self.gripper.positions()}")
        self.open_gripper()                                           # 3. mở tay -> vật nằm trên bàn
        self.world.positions[obj] = (x, y, yaw)                       # ước lượng; camera sẽ cập nhật
        self._sync_collision_objects()
        return self._move_linear((x, y, z_pre), yaw)                  # 4. rút lên
