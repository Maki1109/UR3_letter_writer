"""Đọc cấu hình cảnh (scene.yaml), thông tin sinh viên và tính nhiệm vụ cá nhân.

Module này KHÔNG phụ thuộc rclpy để có thể unit-test offline.

Bài 03: vị trí các khối KHÔNG lấy từ cấu hình. WorldState được điền từ kết quả
camera (perception.py); scene.yaml chỉ mô tả hạ tầng cố định (bàn, zone, camera,
tay gắp) và màu của từng khối để camera nhận dạng.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import yaml

# Bảng quy ước của đề bài 02: P -> (màu zone A, zone B, zone C)
PERSONAL_TABLE: Dict[int, Tuple[str, str, str]] = {
    0: ("red", "yellow", "blue"),
    1: ("red", "blue", "yellow"),
    2: ("yellow", "red", "blue"),
    3: ("yellow", "blue", "red"),
    4: ("blue", "red", "yellow"),
    5: ("blue", "yellow", "red"),
}

# Tên "zone ảo" để đặt vật ra một chỗ trống trên bàn (do hệ thống tự tìm)
TEMP_POSITION = "temporary_position"

Pose2D = Tuple[float, float, float]          # (x, y, yaw) trên mặt bàn, hệ base_link


@dataclass
class ObjectSpec:
    name: str
    color: str
    rgba: List[float]
    hue: Tuple[float, float]                 # ngưỡng màu (độ) cho camera


@dataclass
class ZoneSpec:
    name: str
    label: str
    center_xy: Tuple[float, float]
    rgba: List[float]


@dataclass
class SceneConfig:
    frame_id: str
    table_center: List[float]
    table_size: List[float]
    cube_size: float
    objects: Dict[str, ObjectSpec]
    zones: Dict[str, ZoneSpec]
    zone_size: float
    occupancy_radius: float
    camera: dict = field(default_factory=dict)
    perception: dict = field(default_factory=dict)
    free_space: dict = field(default_factory=dict)
    gripper: dict = field(default_factory=dict)
    motion: dict = field(default_factory=dict)

    @property
    def table_top_z(self) -> float:
        return self.table_center[2] + self.table_size[2] / 2.0

    def cube_center_z(self) -> float:
        return self.table_top_z + self.cube_size / 2.0

    def table_bounds(self, margin: float = 0.0) -> Tuple[float, float, float, float]:
        cx, cy = self.table_center[0], self.table_center[1]
        hx, hy = self.table_size[0] / 2.0 - margin, self.table_size[1] / 2.0 - margin
        return cx - hx, cx + hx, cy - hy, cy + hy

    def zone_at(self, xy: Tuple[float, float]) -> Optional[str]:
        """Trả về tên zone chứa điểm xy (nếu có)."""
        for z in self.zones.values():
            if math.dist(xy[:2], z.center_xy) < self.occupancy_radius:
                return z.name
        return None


def load_scene(path: str) -> SceneConfig:
    with open(path, "r", encoding="utf-8") as f:
        d = yaml.safe_load(f)
    objects = {
        name: ObjectSpec(name, o["color"], list(o["rgba"]), tuple(o.get("hue", [0, 360])))
        for name, o in d["objects"].items()
    }
    zones = {
        name: ZoneSpec(name, z.get("label", name), tuple(z["center"]),
                       list(z.get("rgba", [0.6, 0.6, 0.6, 1.0])))
        for name, z in d["zones"].items()
    }
    return SceneConfig(
        frame_id=d.get("frame_id", "base_link"),
        table_center=list(d["table"]["center"]),
        table_size=list(d["table"]["size"]),
        cube_size=float(d["cube_size"]),
        objects=objects,
        zones=zones,
        zone_size=float(d.get("zone_size", 0.08)),
        occupancy_radius=float(d.get("occupancy_radius", 0.045)),
        camera=dict(d.get("camera", {})),
        perception=dict(d.get("perception", {})),
        free_space=dict(d.get("free_space", {})),
        gripper=dict(d.get("gripper", {})),
        motion=dict(d.get("motion", {})),
    )


@dataclass
class StudentTask:
    student_name: str
    student_id: str
    last_two: int
    p: int
    zone_to_color: Dict[str, str]          # {"zone_a": "blue", ...}
    zone_to_object: Dict[str, str]         # {"zone_a": "blue_cube", ...}

    @property
    def object_to_zone(self) -> Dict[str, str]:
        return {o: z for z, o in self.zone_to_object.items()}


def compute_student_task(student_name: str, student_id: str,
                         scene: Optional[SceneConfig] = None) -> StudentTask:
    """P = (2 chữ số cuối MSSV) mod 6, tra bảng quy ước của đề."""
    sid = str(student_id).strip()
    digits = "".join(ch for ch in sid if ch.isdigit())
    if len(digits) < 2:
        raise ValueError(f"student_id '{student_id}' phải có ít nhất 2 chữ số")
    last_two = int(digits[-2:])
    p = last_two % 6
    colors = PERSONAL_TABLE[p]
    zone_to_color = {"zone_a": colors[0], "zone_b": colors[1], "zone_c": colors[2]}

    # Map màu -> tên object (dựa vào scene nếu có, mặc định "<màu>_cube")
    color_to_obj = {c: f"{c}_cube" for c in ("red", "yellow", "blue")}
    if scene is not None:
        color_to_obj.update({o.color: o.name for o in scene.objects.values()})
    zone_to_object = {z: color_to_obj[c] for z, c in zone_to_color.items()}
    return StudentTask(student_name, sid, last_two, p, zone_to_color, zone_to_object)


def load_student(path: str, scene: Optional[SceneConfig] = None) -> StudentTask:
    with open(path, "r", encoding="utf-8") as f:
        d = yaml.safe_load(f)
    return compute_student_task(d["student_name"], str(d["student_id"]), scene)


class WorldState:
    """Trạng thái thế giới: vật nằm ở đâu, robot đang cầm gì.

    positions[obj]:
      (x, y, yaw)  vị trí camera đo được trên mặt bàn
      None         vật nằm trên bàn ở chỗ chưa xác định toạ độ (chỉ dùng khi Plan
                   Validator mô phỏng place(obj, temporary_position))
      (không có khoá) camera không thấy vật
    """

    def __init__(self, scene: SceneConfig,
                 positions: Optional[Dict[str, Optional[Pose2D]]] = None,
                 holding: Optional[str] = None):
        self.scene = scene
        self.positions: Dict[str, Optional[Pose2D]] = dict(positions or {})
        self.holding: Optional[str] = holding
        self.stamp: float = 0.0               # thời điểm (giây) của ảnh camera gần nhất

    def copy(self) -> "WorldState":
        w = WorldState(self.scene, dict(self.positions), self.holding)
        w.stamp = self.stamp
        return w

    def is_known(self, obj: str) -> bool:
        return obj == self.holding or obj in self.positions

    def zone_of(self, obj: str) -> Optional[str]:
        if obj == self.holding:
            return None
        pos = self.positions.get(obj)
        return self.scene.zone_at(pos[:2]) if pos is not None else None

    def location_of(self, obj: str) -> str:
        if self.holding == obj:
            return "gripper"
        if obj not in self.positions:
            return "not detected"
        z = self.zone_of(obj)
        return z if z else "table"

    def occupant_of(self, zone: str) -> Optional[str]:
        for obj in self.positions:
            if obj != self.holding and self.zone_of(obj) == zone:
                return obj
        return None

    def free_zones(self) -> List[str]:
        return [z for z in self.scene.zones if self.occupant_of(z) is None]

    def describe(self, with_coords: bool = False) -> str:
        lines = []
        for obj in self.scene.objects:
            loc = self.location_of(obj)
            text = {"table": "on the table (not in any zone)",
                    "gripper": "held by the gripper",
                    "not detected": "NOT DETECTED by the camera"}.get(loc, f"in {loc}")
            pos = self.positions.get(obj)
            if with_coords and pos is not None and loc != "gripper":
                text += f"  [x={pos[0]:.3f} y={pos[1]:.3f} yaw={math.degrees(pos[2]):.0f}deg]"
            lines.append(f"- {obj}: {text}")
        for zone in self.scene.zones:
            occ = self.occupant_of(zone)
            lines.append(f"- {zone}: {'occupied by ' + occ if occ else 'free'}")
        lines.append(f"- robot is holding: {self.holding or 'nothing'}")
        return "\n".join(lines)
