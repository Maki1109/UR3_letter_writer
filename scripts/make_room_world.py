#!/usr/bin/env python3
"""Sinh world nhỏ 'room' cho bài thực hành: phòng 7 x 5 m, 2 vách ngăn (hành lang chữ S) và 3 hộp.

Robot spawn tại (0,0) trên dock (dock ở phía sau robot), nên cách tường >= 2 m mỗi phía.
Chạy:  python3 scripts/make_room_world.py   -> overlay/share/turtlebot4_ignition_bringup/worlds/room.sdf
Quét hết map ~1 phút ở 0.3 m/s: ra đầu bên phải qua khe phía trên vách A, vòng về qua khe
phía dưới vách B.
"""
import os

OUT = os.path.join(os.path.dirname(__file__), '..', 'overlay', 'share',
                   'turtlebot4_ignition_bringup', 'worlds', 'room.sdf')

H = 1.0          # chiều cao tường (m) – cao hơn LiDAR (~0.3 m)
T = 0.15         # độ dày tường (m)
X0, X1, Y0, Y1 = -2.0, 5.0, -2.5, 2.5     # biên phòng

# (tên, tâm x, tâm y, kích thước x, kích thước y)
BOXES = [
    ('wall_north', (X0 + X1) / 2, Y1, X1 - X0 + T, T),
    ('wall_south', (X0 + X1) / 2, Y0, X1 - X0 + T, T),
    ('wall_west', X0, 0.0, T, Y1 - Y0 + T),
    ('wall_east', X1, 0.0, T, Y1 - Y0 + T),
    # vách A (x=1.4): từ tường nam lên tới y=0.5, chừa khe 2 m phía bắc
    ('partition_a', 1.4, (Y0 + 0.5) / 2, T, 0.5 - Y0),
    # vách B (x=3.4): từ tường bắc xuống tới y=-0.5, chừa khe 2 m phía nam
    ('partition_b', 3.4, (Y1 - 0.5) / 2, T, Y1 + 0.5),
]
OBSTACLES = [   # (tên, x, y, kích thước)
    ('box_1', 0.4, -1.5, 0.5),
    ('box_2', 2.4, 1.6, 0.5),
    ('box_3', 4.2, -1.4, 0.4),
]


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


models = ''.join(box(n, x, y, sx, sy, H, '0.75 0.75 0.78 1') for n, x, y, sx, sy in BOXES)
models += ''.join(box(n, x, y, s, s, 0.6, '0.8 0.4 0.2 1') for n, x, y, s in OBSTACLES)

sdf = f"""<?xml version="1.0"?>
<sdf version='1.8'>
  <world name='room'>
    <physics name='1ms' type='ignored'>
      <max_step_size>0.003</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <plugin name='ignition::gazebo::systems::Physics' filename='ignition-gazebo-physics-system' />
    <plugin name='ignition::gazebo::systems::UserCommands' filename='ignition-gazebo-user-commands-system' />
    <plugin name='ignition::gazebo::systems::SceneBroadcaster' filename='ignition-gazebo-scene-broadcaster-system' />
    <plugin name='ignition::gazebo::systems::Contact' filename='ignition-gazebo-contact-system' />
    <light name='sun' type='directional'>
      <cast_shadows>0</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.1 -0.9</direction>
    </light>
    <gravity>0 0 -9.8</gravity>
    <scene>
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>0</shadows>
    </scene>
    <model name='ground_plane'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision>
        <visual name='visual'>
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.35 0.4 0.35 1</ambient><diffuse>0.35 0.4 0.35 1</diffuse></material>
        </visual>
      </link>
    </model>{models}
  </world>
</sdf>
"""

os.makedirs(os.path.dirname(OUT), exist_ok=True)
with open(OUT, 'w') as f:
    f.write(sdf)
print('Đã ghi', os.path.normpath(OUT))
