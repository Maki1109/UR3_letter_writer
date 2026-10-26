#!/usr/bin/env python3
"""Sinh world 'open_arena' cho bài 06: sân trống 7 x 7 m, chỉ có tường bao, để robot xoay / tiến-lùi /
chạy vòng tròn mà không va chạm.

Robot spawn tại (0, 0) trên dock (dock nằm phía trước robot, x = +0.157). Sau khi undock robot quay mặt
về phía -x, nên sân được đặt lệch về phía -x: x ∈ [-5, 2], y ∈ [-3.5, 3.5]. Vùng hoạt động (đường tròn
bán kính 0.5 m, tiến-lùi 0.4 m) cách tường gần nhất > 2 m.

Physics, ánh sáng, sàn lấy nguyên từ world 'room' của bài 04 (overlay/.../worlds/room.sdf), chỉ thay
phần vật thể. Chạy:  python3 btth6/scripts/make_world.py
  -> overlay/share/turtlebot4_ignition_bringup/worlds/open_arena.sdf
"""
import os
import re
import subprocess
import sys

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
WORLDS = os.path.join(ROOT, 'overlay', 'share', 'turtlebot4_ignition_bringup', 'worlds')
SRC = os.path.join(WORLDS, 'room.sdf')
OUT = os.path.join(WORLDS, 'open_arena.sdf')

H, T = 0.5, 0.15                            # chiều cao, độ dày tường (m)
X0, X1, Y0, Y1 = -5.0, 2.0, -3.5, 3.5       # biên sân
WALLS = [   # (tên, tâm x, tâm y, kích thước x, kích thước y)
    ('wall_north', (X0 + X1) / 2, Y1, X1 - X0 + T, T),
    ('wall_south', (X0 + X1) / 2, Y0, X1 - X0 + T, T),
    ('wall_west', X0, 0.0, T, Y1 - Y0 + T),
    ('wall_east', X1, 0.0, T, Y1 - Y0 + T),
]
# Vòng tròn mờ trên sàn đánh dấu vùng hoạt động (chỉ có visual, không có collision)
ZONE = (-1.3, 0.0, 1.4)                     # tâm x, tâm y, bán kính


def box(name, x, y, sx, sy, sz, color):
    return f"""
    <model name='{name}'>
      <static>true</static>
      <pose>{x} {y} {sz / 2} 0 0 0</pose>
      <link name='link'>
        <collision name='collision'><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>
        <visual name='visual'>
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>
      </link>
    </model>"""


def zone(x, y, r):
    return f"""
    <model name='activity_zone'>
      <static>true</static>
      <pose>{x} {y} 0.001 0 0 0</pose>
      <link name='link'>
        <visual name='visual'>
          <geometry><cylinder><radius>{r}</radius><length>0.001</length></cylinder></geometry>
          <material><ambient>0.45 0.55 0.65 1</ambient><diffuse>0.45 0.55 0.65 1</diffuse></material>
        </visual>
      </link>
    </model>"""


if not os.path.isfile(SRC):     # overlay chưa có room.sdf -> sinh bằng script của bài 04
    subprocess.run([sys.executable, os.path.join(ROOT, 'scripts', 'make_room_world.py')], check=True)
sdf = open(SRC).read()
head = sdf[:sdf.index('</model>', sdf.index("<model name='ground_plane'>")) + len('</model>')]
head = re.sub(r"<world name='[^']*'>", "<world name='open_arena'>", head)
models = ''.join(box(n, x, y, sx, sy, H, '0.75 0.75 0.78 1') for n, x, y, sx, sy in WALLS)
models += zone(*ZONE)

with open(OUT, 'w') as f:
    f.write(head + models + '\n  </world>\n</sdf>\n')
print('Đã ghi', os.path.normpath(OUT))
