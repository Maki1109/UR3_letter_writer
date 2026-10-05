"""find_free_position(): tìm một chỗ trống trên bàn để đặt tạm vật (temporary_position).

Duyệt lưới ứng viên trong vùng làm việc (scene.yaml: free_space) và giữ các điểm:
  - nằm trong tầm với của tay máy (bán kính <= max_reach tính từ trục robot),
  - cách tâm MỌI khối camera đang thấy >= min_dist_objects (đủ chỗ cho ngón tay mở),
  - cách tâm mọi zone >= min_dist_zones (không đặt đè/che zone).
Trong các điểm hợp lệ, chọn điểm gần vật cần dời nhất (quãng đường mang vật ngắn),
ưu tiên điểm thoáng (khoảng cách tới vật gần nhất lớn) khi bằng nhau.
Thuần Python — unit test offline được.
"""
from __future__ import annotations

import math
from typing import Iterable, Optional, Tuple

from .scene_config import SceneConfig, WorldState


def find_free_position(scene: SceneConfig, world: WorldState, obj: Optional[str] = None,
                       extra_obstacles: Iterable[Tuple[float, float]] = ()
                       ) -> Optional[Tuple[float, float]]:
    fs = scene.free_space
    x0, x1 = fs.get("x_range", [0.21, 0.45])
    y0, y1 = fs.get("y_range", [-0.30, 0.30])
    step = float(fs.get("step", 0.02))
    d_obj = float(fs.get("min_dist_objects", 0.10))
    d_zone = float(fs.get("min_dist_zones", 0.10))
    reach = float(fs.get("max_reach", 0.43))
    tx0, tx1, ty0, ty1 = scene.table_bounds(margin=scene.cube_size)

    obstacles = [p[:2] for o, p in world.positions.items()
                 if p is not None and o != obj and o != world.holding]
    obstacles += list(extra_obstacles)
    zones = [z.center_xy for z in scene.zones.values()]
    origin = world.positions.get(obj) if obj else None
    ref = origin[:2] if origin is not None else (0.30, 0.0)

    best, best_key = None, None
    nx = int(round((x1 - x0) / step)) + 1
    ny = int(round((y1 - y0) / step)) + 1
    for i in range(nx):
        x = x0 + i * step
        for j in range(ny):
            y = y0 + j * step
            if not (tx0 <= x <= tx1 and ty0 <= y <= ty1):
                continue
            if math.hypot(x, y) > reach:
                continue
            if any(math.dist((x, y), z) < d_zone for z in zones):
                continue
            clearance = min((math.dist((x, y), o) for o in obstacles), default=1.0)
            if clearance < d_obj:
                continue
            key = (round(math.dist((x, y), ref), 3), -round(clearance, 3))
            if best_key is None or key < best_key:
                best, best_key = (round(x, 3), round(y, 3)), key
    return best
