"""Nhận dạng khối từ ảnh camera RGB nhìn xuống bàn (thuần numpy, không cần OpenCV).

Các bước cho mỗi ảnh:
  1. RGB -> HSV; với mỗi khối, lấy mặt nạ pixel có hue nằm trong ngưỡng màu của
     khối (scene.yaml: objects.<tên>.hue) và đủ bão hoà / đủ sáng.
  2. Chiếu mọi pixel lên mặt phẳng nằm ngang z = mặt trên của khối (tia từ tâm
     camera qua pixel cắt mặt phẳng) bằng nội tham số (camera_info) + pose camera
     (ngoại tham số, cố định) -> toạ độ (x, y) trong base_link. Bỏ pixel ngoài bàn.
  3. Lấy thành phần liên thông lớn nhất của mặt nạ; giữ phần sáng (mặt trên của
     khối) để ước lượng tâm (x, y) và góc xoay yaw (hình chữ nhật bao nhỏ nhất).
  4. Kiểm tra diện tích so với diện tích kỳ vọng của mặt trên khối: quá nhỏ =>
     nhiễu/không thấy; nhỏ hơn ngưỡng "visible" => đang bị che khuất một phần.

Từ các khối đo được, WorldState suy ra zone nào trống / có vật (scene_config.py).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from .scene_config import SceneConfig


@dataclass
class Detection:
    name: str
    x: float
    y: float
    yaw: float              # rad, trong (-pi/4, pi/4] (khối lập phương đối xứng 90 độ)
    pixels: int             # số pixel của blob
    visible_ratio: float    # diện tích mặt trên đo được / diện tích kỳ vọng
    bbox: Tuple[int, int, int, int]   # (u_min, v_min, u_max, v_max) trên ảnh

    @property
    def pose(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.yaw)


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def rgb_to_hsv(rgb: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """rgb uint8 (H, W, 3) -> hue (độ, 0..360), saturation (0..1), value (0..1)."""
    x = rgb.astype(np.float32) / 255.0
    r, g, b = x[..., 0], x[..., 1], x[..., 2]
    mx = x.max(axis=-1)
    mn = x.min(axis=-1)
    d = mx - mn
    s = np.where(mx > 1e-6, d / np.maximum(mx, 1e-6), 0.0)
    dd = np.maximum(d, 1e-6)
    h = np.where(mx == r, ((g - b) / dd) % 6.0,
                 np.where(mx == g, (b - r) / dd + 2.0, (r - g) / dd + 4.0)) * 60.0
    h = np.where(d < 1e-6, 0.0, h)
    return h, s, mx


def hue_mask(h: np.ndarray, lo: float, hi: float) -> np.ndarray:
    return (h >= lo) & (h <= hi) if lo <= hi else (h >= lo) | (h <= hi)


def largest_component(mask: np.ndarray) -> np.ndarray:
    """Thành phần liên thông (4-lân cận) lớn nhất của mặt nạ nhị phân."""
    ys, xs = np.nonzero(mask)
    if len(ys) == 0:
        return mask
    remaining = set(zip(ys.tolist(), xs.tolist()))
    best: List[Tuple[int, int]] = []
    while remaining:
        seed = remaining.pop()
        comp, stack = [seed], [seed]
        while stack:
            v, u = stack.pop()
            for nb in ((v + 1, u), (v - 1, u), (v, u + 1), (v, u - 1)):
                if nb in remaining:
                    remaining.remove(nb)
                    comp.append(nb)
                    stack.append(nb)
        if len(comp) > len(best):
            best = comp
    out = np.zeros_like(mask)
    vv, uu = zip(*best)
    out[list(vv), list(uu)] = True
    return out


def min_area_yaw(points: np.ndarray) -> float:
    """Góc xoay (rad, trong (-pi/4, pi/4]) của hình chữ nhật bao có diện tích nhỏ nhất.

    Dùng min/max thật (blob đã là một thành phần liên thông sạch): cắt phân vị sẽ "gọt"
    góc hình vuông không đều theo góc quay và làm lệch kết quả vài độ.
    """
    if len(points) < 5:
        return 0.0
    c = points - points.mean(axis=0)

    def area(deg: float) -> float:
        a = math.radians(deg)
        ca, sa = math.cos(a), math.sin(a)
        u = c[:, 0] * ca + c[:, 1] * sa
        v = -c[:, 0] * sa + c[:, 1] * ca
        return float((u.max() - u.min()) * (v.max() - v.min()))

    coarse = min(np.arange(-45.0, 45.0, 1.0), key=area)
    fine = min(np.arange(coarse - 1.0, coarse + 1.0, 0.1), key=area)
    a = math.radians(fine)
    return (a + math.pi / 4) % (math.pi / 2) - math.pi / 4


class CubeDetector:
    def __init__(self, scene: SceneConfig):
        self.scene = scene
        cam = scene.camera
        self.cam_pos = np.array(cam["pose"][:3], dtype=float)
        self.cam_rot = rpy_to_matrix(*cam["pose"][3:6])
        p = scene.perception
        self.min_s = float(p.get("min_saturation", 0.45))
        self.min_v = float(p.get("min_value", 0.25))
        self.min_area_ratio = float(p.get("min_area_ratio", 0.35))
        self.visible_ratio = float(p.get("visible_area_ratio", 0.70))
        self.table_margin = float(p.get("table_margin", 0.0))
        self.top_z = scene.table_top_z + scene.cube_size
        self._grid_key = None
        self._grid_xy: Optional[np.ndarray] = None
        self._on_table: Optional[np.ndarray] = None
        self._px_area = 1.0

    @staticmethod
    def default_intrinsics(width: int, height: int, hfov: float) -> Tuple[float, float, float, float]:
        f = width / (2.0 * math.tan(hfov / 2.0))
        return f, f, width / 2.0, height / 2.0

    # ------------------------------------------------------------------
    def _prepare(self, width: int, height: int, K: Tuple[float, float, float, float]) -> None:
        key = (width, height, tuple(round(k, 3) for k in K))
        if key == self._grid_key:
            return
        fx, fy, cx, cy = K
        u, v = np.meshgrid(np.arange(width) + 0.5, np.arange(height) + 0.5)
        # Hệ link camera của Gazebo: x = hướng nhìn, y = trái ảnh, z = lên ảnh
        d_link = np.stack([np.ones_like(u), -(u - cx) / fx, -(v - cy) / fy], axis=-1)
        d = d_link @ self.cam_rot.T
        t = (self.top_z - self.cam_pos[2]) / np.where(np.abs(d[..., 2]) < 1e-9, -1e-9, d[..., 2])
        xy = self.cam_pos[:2] + t[..., None] * d[..., :2]
        x0, x1, y0, y1 = self.scene.table_bounds(self.table_margin)
        on_table = (t > 0) & (xy[..., 0] >= x0) & (xy[..., 0] <= x1) & (xy[..., 1] >= y0) & (xy[..., 1] <= y1)
        self._grid_key, self._grid_xy, self._on_table = key, xy, on_table
        # diện tích (m^2) một pixel tại tâm ảnh trên mặt phẳng khối
        dist = abs(self.cam_pos[2] - self.top_z)
        self._px_area = (dist / fx) * (dist / fy)

    def expected_pixels(self) -> float:
        return self.scene.cube_size ** 2 / self._px_area

    # ------------------------------------------------------------------
    def detect(self, rgb: np.ndarray, K: Optional[Tuple[float, float, float, float]] = None,
               skip: Tuple[str, ...] = ()) -> Dict[str, Detection]:
        height, width = rgb.shape[:2]
        if K is None:
            K = self.default_intrinsics(width, height, float(self.scene.camera.get("horizontal_fov", 1.0)))
        self._prepare(width, height, K)
        h, s, v = rgb_to_hsv(rgb)
        base = (s >= self.min_s) & (v >= self.min_v) & self._on_table
        expected = self.expected_pixels()
        out: Dict[str, Detection] = {}
        for name, spec in self.scene.objects.items():
            if name in skip:
                continue
            mask = base & hue_mask(h, *spec.hue)
            if mask.sum() < self.min_area_ratio * expected:
                continue
            blob = largest_component(mask)
            n = int(blob.sum())
            if n < self.min_area_ratio * expected:
                continue
            # mặt trên (sáng nhất) — mặt bên tối hơn và bị chiếu lệch theo phối cảnh
            vals = v[blob]
            top = blob & (v >= np.percentile(vals, 90) * 0.88)
            if top.sum() < 0.3 * n:
                top = blob
            pts = self._grid_xy[top]
            x, y = float(pts[:, 0].mean()), float(pts[:, 1].mean())
            yaw = min_area_yaw(pts)
            vv, uu = np.nonzero(blob)
            out[name] = Detection(name, x, y, yaw, n, float(top.sum()) / expected,
                                  (int(uu.min()), int(vv.min()), int(uu.max()), int(vv.max())))
        return out

    # ------------------------------------------------------------------
    def annotate(self, rgb: np.ndarray, detections: Dict[str, Detection]) -> np.ndarray:
        """Ảnh debug: khung bao + chữ thập tâm cho từng khối, khung zone (xanh = trống, đỏ = có vật)."""
        img = rgb.copy()
        for d in detections.values():
            u0, v0, u1, v1 = d.bbox
            col = np.array([255, 255, 255], np.uint8)
            img[v0:v1 + 1, [max(u0 - 2, 0), min(u1 + 2, img.shape[1] - 1)]] = col
            img[[max(v0 - 2, 0), min(v1 + 2, img.shape[0] - 1)], u0:u1 + 1] = col
        return img
