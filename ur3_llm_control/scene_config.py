"""Đọc cấu hình cảnh (scene.yaml), thông tin sinh viên và tính nhiệm vụ cá nhân.

Module này KHÔNG phụ thuộc rclpy để có thể unit-test offline.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import yaml

# Bảng quy ước của đề: P -> (màu zone A, zone B, zone C)
PERSONAL_TABLE: Dict[int, Tuple[str, str, str]] = {
    0: ("red", "yellow", "blue"),
    1: ("red", "blue", "yellow"),
    2: ("yellow", "red", "blue"),
    3: ("yellow", "blue", "red"),
    4: ("blue", "red", "yellow"),
    5: ("blue", "yellow", "red"),
}


@dataclass
class ObjectSpec:
    name: str
    color: str
    rgba: List[float]
    start_xy: Tuple[float, float]


@dataclass
class ZoneSpec:
    name: str
    label: str
    center_xy: Tuple[float, float]
    rgba: List[float]
    is_temporary: bool = False


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
    motion: dict = field(default_factory=dict)

    @property
    def table_top_z(self) -> float:
        return self.table_center[2] + self.table_size[2] / 2.0

    def cube_center_z(self) -> float:
        return self.table_top_z + self.cube_size / 2.0

    def zone_at(self, xy: Tuple[float, float]) -> Optional[str]:
        """Trả về tên zone chứa điểm xy (nếu có)."""
        for z in self.zones.values():
            if math.dist(xy, z.center_xy) < self.occupancy_radius:
                return z.name
        return None


def load_scene(path: str) -> SceneConfig:
    with open(path, "r", encoding="utf-8") as f:
        d = yaml.safe_load(f)
    objects = {
        name: ObjectSpec(name, o["color"], list(o["rgba"]), tuple(o["start"]))
        for name, o in d["objects"].items()
    }
    zones = {
        name: ZoneSpec(name, z.get("label", name), tuple(z["center"]),
                       list(z.get("rgba", [0.6, 0.6, 0.6, 1.0])),
                       bool(z.get("is_temporary", False)))
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
        occupancy_radius=float(d.get("occupancy_radius", 0.03)),
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
    color_to_obj = {f"{c}": f"{c}_cube" for c in ("red", "yellow", "blue")}
    if scene is not None:
        color_to_obj = {o.color: o.name for o in scene.objects.values()}
    zone_to_object = {z: color_to_obj[c] for z, c in zone_to_color.items()}
    return StudentTask(student_name, sid, last_two, p, zone_to_color, zone_to_object)


def load_student(path: str, scene: Optional[SceneConfig] = None) -> StudentTask:
    with open(path, "r", encoding="utf-8") as f:
        d = yaml.safe_load(f)
    return compute_student_task(d["student_name"], str(d["student_id"]), scene)


class WorldState:
    """Trạng thái logic của thế giới: vật nằm ở đâu, robot đang cầm gì.

    Được dùng bởi cả Plan Validator (mô phỏng trước kế hoạch) lẫn
    Robot Skills (cập nhật sau mỗi skill thực thi thành công).
    """

    def __init__(self, scene: SceneConfig,
                 positions: Optional[Dict[str, Tuple[float, float]]] = None,
                 holding: Optional[str] = None):
        self.scene = scene
        self.positions: Dict[str, Tuple[float, float]] = dict(
            positions if positions is not None
            else {n: o.start_xy for n, o in scene.objects.items()})
        self.holding: Optional[str] = holding

    def copy(self) -> "WorldState":
        return WorldState(self.scene, dict(self.positions), self.holding)

    def location_of(self, obj: str) -> str:
        if self.holding == obj:
            return "gripper"
        z = self.scene.zone_at(self.positions[obj])
        return z if z else "table (start area)"

    def occupant_of(self, zone: str) -> Optional[str]:
        for obj, xy in self.positions.items():
            if obj == self.holding:
                continue
            if self.scene.zone_at(xy) == zone:
                return obj
        return None

    def describe(self) -> str:
        lines = []
        for obj in self.scene.objects:
            lines.append(f"- {obj}: {self.location_of(obj)}")
        for zone in self.scene.zones:
            occ = self.occupant_of(zone)
            lines.append(f"- {zone}: {'occupied by ' + occ if occ else 'free'}")
        lines.append(f"- robot is holding: {self.holding or 'nothing'}")
        return "\n".join(lines)
