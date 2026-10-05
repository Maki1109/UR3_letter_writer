"""CameraPerception: lớp ROS 2 bọc CubeDetector (perception.py).

- Nhận ảnh /overhead_camera/image + /overhead_camera/camera_info (ros_gz_bridge).
- capture(): chờ một ảnh MỚI (chụp sau thời điểm gọi, theo sim time) rồi nhận dạng
  -> {tên khối: Detection}. Chỉ chụp khi tay máy đứng yên (giữa các skill).
- Xuất ảnh debug /perception/debug_image (khung bao các khối) và MarkerArray
  /perception/markers (khối đo được + zone xanh = trống / đỏ = có vật) cho RViz.
"""
from __future__ import annotations

import threading
import time
from typing import Dict, Optional

import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import Marker, MarkerArray

from .perception import CubeDetector, Detection
from .robot_skills import yaw_quat
from .scene_config import SceneConfig, WorldState


class CameraPerception:
    def __init__(self, node: Node, scene: SceneConfig):
        self.node = node
        self.scene = scene
        self.detector = CubeDetector(scene)
        cam = scene.camera
        cb = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._img: Optional[Image] = None
        self._K = None
        node.create_subscription(Image, cam.get("image_topic", "/overhead_camera/image"),
                                 self._on_image, qos_profile_sensor_data, callback_group=cb)
        node.create_subscription(CameraInfo, cam.get("info_topic", "/overhead_camera/camera_info"),
                                 self._on_info, qos_profile_sensor_data, callback_group=cb)
        self.debug_pub = node.create_publisher(Image, "perception/debug_image", 1)
        self.marker_pub = node.create_publisher(MarkerArray, "perception/markers", 1)
        self.last: Dict[str, Detection] = {}

    def _on_image(self, msg: Image) -> None:
        with self._lock:
            self._img = msg

    def _on_info(self, msg: CameraInfo) -> None:
        if msg.k[0] > 0:
            self._K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def wait_ready(self, timeout: float = 30.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self._lock:
                if self._img is not None:
                    return True
            time.sleep(0.1)
        return False

    def capture(self, timeout: float = 5.0) -> Optional[Dict[str, Detection]]:
        """Chờ ảnh chụp SAU lúc gọi (tay máy đã dừng) rồi nhận dạng các khối."""
        t_req = self.node.get_clock().now()
        t0 = time.time()
        msg = None
        while time.time() - t0 < timeout:
            with self._lock:
                m = self._img
            if m is not None and Time.from_msg(m.header.stamp) > t_req:
                msg = m
                break
            time.sleep(0.03)
        if msg is None:
            with self._lock:
                msg = self._img                     # không có ảnh mới: dùng ảnh gần nhất
            if msg is None:
                return None
            self.node.get_logger().warn("camera: no fresh image, using the latest one")
        rgb = self._to_rgb(msg)
        dets = self.detector.detect(rgb, self._K)
        self.last = dets
        self._publish_debug(msg, rgb, dets)
        return dets

    @staticmethod
    def _to_rgb(msg: Image) -> np.ndarray:
        a = np.frombuffer(bytes(msg.data), np.uint8)
        ch = 4 if msg.encoding in ("rgba8", "bgra8") else 3
        a = a.reshape(msg.height, msg.step // ch if msg.step else msg.width, ch)[:, :msg.width, :3]
        if msg.encoding.startswith("bgr"):
            a = a[..., ::-1]
        return np.ascontiguousarray(a)

    def _publish_debug(self, src: Image, rgb: np.ndarray, dets: Dict[str, Detection]) -> None:
        img = self.detector.annotate(rgb, dets)
        out = Image()
        out.header = src.header
        out.height, out.width = img.shape[:2]
        out.encoding = "rgb8"
        out.step = out.width * 3
        out.data = img.tobytes()
        self.debug_pub.publish(out)

    # ------------------------------------------------------------------
    def publish_world(self, world: WorldState) -> None:
        """Marker RViz: khối (theo camera) + zone (xanh = trống, đỏ = có vật) + nhãn."""
        arr = MarkerArray()
        stamp = self.node.get_clock().now().to_msg()
        frame = self.scene.frame_id
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        mid = 0

        def mk(ns, mtype):
            nonlocal mid
            m = Marker()
            m.header.frame_id, m.header.stamp = frame, stamp
            m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
            m.pose.orientation.w = 1.0
            mid += 1
            return m

        top = self.scene.table_top_z
        for z in self.scene.zones.values():
            occ = world.occupant_of(z.name)
            sq = mk("zones", Marker.CUBE)
            sq.pose.position.x, sq.pose.position.y = z.center_xy
            sq.pose.position.z = top + 0.001
            sq.scale.x = sq.scale.y = self.scene.zone_size
            sq.scale.z = 0.002
            sq.color.r, sq.color.g, sq.color.b, sq.color.a = (0.9, 0.2, 0.2, 0.6) if occ else (0.2, 0.8, 0.2, 0.6)
            tx = mk("zone_labels", Marker.TEXT_VIEW_FACING)
            tx.pose.position.x, tx.pose.position.y = z.center_xy
            tx.pose.position.z = top + 0.10
            tx.scale.z = 0.025
            tx.color.r = tx.color.g = tx.color.b = tx.color.a = 1.0
            tx.text = f"{z.label}: {occ or 'free'}"
            arr.markers += [sq, tx]
        c = self.scene.cube_size
        for obj, pos in world.positions.items():
            if pos is None or obj == world.holding:
                continue
            spec = self.scene.objects[obj]
            cube = mk("detected_objects", Marker.CUBE)
            cube.pose.position.x, cube.pose.position.y = pos[0], pos[1]
            cube.pose.position.z = self.scene.cube_center_z()
            q = yaw_quat(pos[2])
            cube.pose.orientation.x, cube.pose.orientation.y, cube.pose.orientation.z, cube.pose.orientation.w = q
            cube.scale.x = cube.scale.y = cube.scale.z = c * 1.05
            cube.color.r, cube.color.g, cube.color.b = spec.rgba[:3]
            cube.color.a = 0.5
            arr.markers.append(cube)
        self.marker_pub.publish(arr)
