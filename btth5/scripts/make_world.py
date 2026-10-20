#!/usr/bin/env python3
"""Sinh world 'room_objects' = world 'room' của bài 04 (cùng tường, vách, hộp -> dùng lại được maps/map.yaml)
+ các vật thể thuộc lớp COCO để YOLO nhận diện: người, ghế, sofa, trụ cứu hỏa.

Vật thể đặt sát tường / góc phòng, không chắn khe đi qua vách A, B.
Chạy:  python3 btth5/scripts/make_world.py
  -> overlay/share/turtlebot4_ignition_bringup/worlds/room_objects.sdf (+ các model ở cùng thư mục)

Mô hình lấy từ Gazebo Fuel. Nếu world include thẳng URL Fuel, mỗi lần khởi động Gazebo lại hỏi server Fuel
phiên bản mới nhất và treo ở "server may be busy downloading resources" khi mạng chập chờn. Vì vậy script
chép model từ cache ~/.ignition/fuel (tải về một lần nếu chưa có) vào thư mục worlds của overlay, đổi URI mesh
sang đường dẫn tương đối và include bằng model://<tên> -> Gazebo chạy offline.
"""
import math
import os
import re
import shutil
import subprocess

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
WORLDS = os.path.join(ROOT, 'overlay', 'share', 'turtlebot4_ignition_bringup', 'worlds')
SRC = os.path.join(WORLDS, 'room.sdf')
OUT = os.path.join(WORLDS, 'room_objects.sdf')
FUEL = 'https://fuel.gazebosim.org/1.0'
CACHE = os.path.expanduser('~/.ignition/fuel/fuel.gazebosim.org')

# Model Fuel: tên local -> (owner, tên model trên Fuel, phiên bản)
MODELS = {
    'b5_standing_person': ('OpenRobotics', 'Standing person', 3),
    'b5_sofa': ('OpenRobotics', 'Sofa', 3),
    'b5_fire_hydrant': ('OpenRobotics', 'Fire hydrant', 3),
    'b5_casual_female': ('plateau', 'Casual female', 1),
    'b5_chair': ('OpenRobotics', 'Chair', 3),
}
# (tên trong world, model, x, y, yaw độ)  – phòng x ∈ [-2, 5], y ∈ [-2.5, 2.5]
OBJECTS = [
    ('person_left', 'b5_standing_person', -1.5, 1.9, -45),   # góc tây bắc phòng trái
    ('sofa', 'b5_sofa', -1.55, -0.2, 0),                     # sát tường tây phòng trái
    ('fire_hydrant', 'b5_fire_hydrant', 1.85, -2.1, 0),      # góc tây nam hành lang giữa
    ('person_right', 'b5_casual_female', 3.0, 2.1, -120),    # góc đông bắc hành lang giữa (sau hộp box_2)
    ('chair', 'b5_chair', 1.8, 2.15, -90),                   # góc tây bắc hành lang giữa (trên khe vách A)
]


def cached_dir(owner, name, ver):
    for n in (name.lower(), name.lower().replace(' ', '%20')):
        d = os.path.join(CACHE, owner.lower(), 'models', n, str(ver))
        if os.path.isfile(os.path.join(d, 'model.sdf')):
            return d
    return None


def fetch(owner, name, ver):
    d = cached_dir(owner, name, ver)
    if d is None:
        url = f'{FUEL}/{owner}/models/{name.replace(" ", "%20")}/{ver}'
        print('Tải', url)
        subprocess.run(['ign', 'fuel', 'download', '-u', url], check=True, timeout=300)
        d = cached_dir(owner, name, ver)
    if d is None:
        raise SystemExit(f'Không tải được model {owner}/{name}')
    return d


def install_model(local, owner, name, ver):
    dst = os.path.join(WORLDS, local)
    if os.path.isfile(os.path.join(dst, 'model.sdf')):
        return
    src = fetch(owner, name, ver)
    shutil.copytree(src, dst, dirs_exist_ok=True)
    # URI tuyệt đối tới Fuel -> đường dẫn tương đối trong thư mục model
    sdf_path = os.path.join(dst, 'model.sdf')
    with open(sdf_path) as f:
        sdf = f.read()
    sdf = re.sub(r'https://fuel\.[a-z]+\.org/1\.0/[^<]*?/files/', '', sdf, flags=re.I)
    with open(sdf_path, 'w') as f:
        f.write(sdf)
    # Một số mesh (.dae) gọi texture bằng tên file trần -> chép texture cạnh mesh để Gazebo tìm thấy
    tex = os.path.join(dst, 'materials', 'textures')
    if os.path.isdir(tex) and os.path.isdir(os.path.join(dst, 'meshes')):
        for t in os.listdir(tex):
            shutil.copy(os.path.join(tex, t), os.path.join(dst, 'meshes', t))


def include(name, model, x, y, yaw):
    return f"""
    <include>
      <name>{name}</name>
      <uri>model://{model}</uri>
      <static>true</static>
      <pose>{x} {y} 0 0 0 {math.radians(yaw):.4f}</pose>
    </include>"""


for local, spec in MODELS.items():
    install_model(local, *spec)
with open(SRC) as f:
    sdf = f.read()
sdf = sdf.replace("<world name='room'>", "<world name='room_objects'>")
sdf = sdf.replace('  </world>', ''.join(include(*o) for o in OBJECTS) + '\n  </world>')
with open(OUT, 'w') as f:
    f.write(sdf)
print('Đã ghi', os.path.normpath(OUT))
