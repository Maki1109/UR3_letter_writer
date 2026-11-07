#!/usr/bin/env python3
"""Giao diện bản đồ riêng (OpenCV): khu vực, robot, lộ trình và đường đi dự kiến, câu lệnh và trạng thái.

Vẽ lại bản đồ bài 04 với các khu vực trong config/zones.yaml (tô màu, tên, điểm đích x, cửa □), rồi cập nhật 5 lần/s:
  - robot (hình tròn + mũi hướng) từ /amcl_pose;
  - đường đi dự kiến của CẢ lộ trình (/voice_nav/planned_path, đường xanh đậm), phần chưa đi;
  - đường Nav2 đang bám ở chặng hiện tại (/plan, đường mảnh màu cam);
  - các điểm dừng đánh số 1, 2, 3... (xám = đã qua, cam = đang tới, xanh = sắp tới) từ /voice_nav/route;
  - câu lệnh cuối (/voice_command), trạng thái nhận dạng (/speech/state) và kết quả (/voice_nav/status).
Bấm chuột trái vào một khu vực để gửi "Đi đến khu vực X" (giống nói câu đó); phím s lưu ảnh vào figures/,
q/Esc đóng cửa sổ. `-p snapshot:=file.png` ghi ảnh định kỳ, `-p show:=false` không mở cửa sổ (chạy trên máy không màn hình).

    python3 nodes/map_view_node.py --ros-args -p use_sim_time:=true
"""
import json
import math
import os
import sys
import time

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from PIL import Image, ImageDraw, ImageFont
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

HERE = os.path.dirname(os.path.abspath(__file__))
B7_DIR = os.path.dirname(HERE)
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
COLORS = [(231, 76, 60), (46, 204, 113), (52, 152, 219), (241, 196, 15), (155, 89, 182), (26, 188, 156),
          (230, 126, 34), (149, 165, 166)]                  # RGB, giống figures/zones.png
LEG_RGB = {'done': (140, 140, 140), 'current': (255, 140, 0), 'pending': (25, 90, 240)}
WINDOW = 'Voice Nav - ban do'                               # tiêu đề cửa sổ OpenCV không hiện được dấu
PANEL = 120                                                 # chiều cao khung chữ phía dưới (px)


class MapView(Node):
    def __init__(self):
        super().__init__('map_view')
        zones_file = self.declare_parameter('zones_file', os.path.join(B7_DIR, 'config', 'zones.yaml')).value
        self.scale = self.declare_parameter('scale', 6).value
        self.show = self.declare_parameter('show', True).value
        self.snapshot = self.declare_parameter('snapshot', '').value
        with open(zones_file) as f:
            cfg = yaml.safe_load(f)
        self.zones = cfg['zones']
        self.doors = cfg.get('doors', [])
        map_yaml = os.path.normpath(os.path.join(os.path.dirname(zones_file), cfg['map']))
        with open(map_yaml) as f:
            meta = yaml.safe_load(f)
        img = cv2.imread(os.path.join(os.path.dirname(map_yaml), meta['image']), cv2.IMREAD_GRAYSCALE)
        self.res = float(meta['resolution'])
        self.ox, self.oy = float(meta['origin'][0]), float(meta['origin'][1])
        self.h = img.shape[0]
        self.base = self.render_base(img)
        self.font = ImageFont.truetype(FONT, 15)
        self.font_big = ImageFont.truetype(FONT, 19)

        self.pose = None            # (x, y, yaw)
        self.path, self.plan = [], []
        self.route = {}
        self.command, self.status, self.speech = '', '', ''
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, latched)
        self.create_subscription(Path, '/voice_nav/planned_path', lambda m: setattr(self, 'path', self.xy(m)),
                                 latched)
        self.create_subscription(Path, '/plan', lambda m: setattr(self, 'plan', self.xy(m)), 10)
        self.create_subscription(String, '/voice_nav/route', self.on_route, latched)
        self.create_subscription(String, '/voice_nav/status', lambda m: setattr(self, 'status', m.data), 10)
        self.create_subscription(String, '/voice_command', lambda m: setattr(self, 'command', m.data), 10)
        self.create_subscription(String, '/speech/state', lambda m: setattr(self, 'speech', m.data), 10)
        self.cmd_pub = self.create_publisher(String, '/voice_command', 10)
        self.last_snap = 0.0
        if self.show:
            cv2.namedWindow(WINDOW)
            cv2.setMouseCallback(WINDOW, self.on_click)
        self.create_timer(0.2, self.draw)
        self.get_logger().info('Bấm chuột vào một khu vực để đi tới đó; s = lưu ảnh, q = thoát')

    # --- tọa độ ---------------------------------------------------------------------------------------------
    def to_img(self, x, y):
        return ((x - self.ox) / self.res * self.scale, (self.h - (y - self.oy) / self.res) * self.scale)

    def to_world(self, u, v):
        return self.ox + u / self.scale * self.res, self.oy + (self.h - v / self.scale) * self.res

    @staticmethod
    def xy(msg):
        return [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def render_base(self, img):
        s = self.scale
        rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        big = cv2.resize(rgb, None, fx=s, fy=s, interpolation=cv2.INTER_NEAREST)
        over = big.copy()
        for k, z in enumerate(self.zones):
            pts = np.array([self.to_img(x, y) for x, y in z['polygon']], np.int32)
            cv2.fillPoly(over, [pts], COLORS[k % len(COLORS)])
        big = cv2.addWeighted(big, 0.5, over, 0.5, 0)
        big[cv2.resize(img, None, fx=s, fy=s, interpolation=cv2.INTER_NEAREST) == 0] = 0
        return big

    # --- callback -------------------------------------------------------------------------------------------
    def on_pose(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (p.x, p.y, 2 * math.atan2(q.z, q.w))

    def on_route(self, msg):
        try:
            self.route = json.loads(msg.data)
        except ValueError:
            self.route = {}

    def on_click(self, event, u, v, *_):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        x, y = self.to_world(u, v)
        for z in self.zones:
            pts = np.array([self.to_img(px, py) for px, py in z['polygon']], np.float32)
            if cv2.pointPolygonTest(pts, (float(u), float(v)), False) >= 0:
                text = f"Đi đến khu vực {z['id']}"
                self.cmd_pub.publish(String(data=text))
                self.get_logger().info(f'Bấm ({x:.2f}, {y:.2f}) -> "{text}"')
                return

    # --- vẽ -------------------------------------------------------------------------------------------------
    def leg_state(self, i):
        st, cur = self.route.get('state'), self.route.get('current', 0)
        if st == 'done' or i < cur:
            return 'done'
        if i == cur and st in ('running', 'failed'):
            return 'current'
        return 'pending'

    def draw(self):
        img = Image.new('RGB', (self.base.shape[1], self.base.shape[0] + PANEL), (250, 250, 250))
        img.paste(Image.fromarray(self.base), (0, 0))
        d = ImageDraw.Draw(img)
        for dr in self.doors:
            x, y = self.to_img(*dr['point'])
            d.rectangle([x - 5, y - 5, x + 5, y + 5], fill=(255, 255, 255), outline=(0, 0, 0), width=2)
        for z in self.zones:
            cx, cy = self.to_img(*z['center'])
            gx, gy = self.to_img(*z['goal'])
            d.line([gx - 7, gy - 7, gx + 7, gy + 7], fill=(200, 0, 0), width=3)
            d.line([gx - 7, gy + 7, gx + 7, gy - 7], fill=(200, 0, 0), width=3)
            label = f"{z['id']}: {z['name']}"
            tw = d.textlength(label, font=self.font_big)
            tx = min(max(cx - tw / 2, 4), img.width - tw - 8)
            d.rectangle([tx - 3, cy - 40, tx + tw + 3, cy - 16], fill=(255, 255, 255))
            d.text((tx, cy - 40), label, font=self.font_big, fill=(0, 0, 0))
        if len(self.plan) > 1:              # đường Nav2 đang bám (chặng hiện tại)
            d.line([self.to_img(*p) for p in self.plan], fill=(255, 140, 0), width=2)
        if len(self.path) > 1:              # đường dự kiến của cả lộ trình
            d.line([self.to_img(*p) for p in self.path], fill=(25, 90, 240), width=4)
        n = 0
        for i, lg in enumerate(self.route.get('legs', [])):
            x, y = self.to_img(lg['x'], lg['y'])
            col = LEG_RGB[self.leg_state(i)]
            if lg['door']:
                d.rectangle([x - 6, y - 6, x + 6, y + 6], fill=col, outline=(0, 0, 0))
            else:
                n += 1
                d.ellipse([x - 13, y - 13, x + 13, y + 13], fill=col, outline=(0, 0, 0), width=2)
                d.text((x - 5, y - 10), str(n), font=self.font_big, fill=(255, 255, 255))
        if self.pose:
            x, y = self.to_img(*self.pose[:2])
            r = 0.17 / self.res * self.scale            # bán kính TurtleBot 4 ~0.17 m
            d.ellipse([x - r, y - r, x + r, y + r], fill=(40, 40, 40), outline=(255, 255, 255), width=2)
            yaw = self.pose[2]
            d.line([x, y, x + r * 1.4 * math.cos(yaw), y - r * 1.4 * math.sin(yaw)], fill=(255, 255, 0), width=4)

        y0 = self.base.shape[0] + 6
        route = ' → '.join(self.route.get('route', [])) or '-'
        state = {'planning': 'đang lập đường đi', 'running': 'đang chạy', 'done': 'hoàn thành', 'failed': 'lỗi',
                 'stopped': 'đã dừng', 'idle': 'chờ lệnh'}.get(self.route.get('state', 'idle'), '')
        lines = [(f'Câu lệnh: "{self.command}"' if self.command else 'Câu lệnh: (chưa có)', (0, 0, 0)),
                 (f'Lộ trình: {route}   [{state}]   Micro: {self.speech or "-"}', (25, 60, 160)),
                 (self.status or '', (180, 0, 0) if 'Không' in self.status or 'lỗi' in self.status.lower()
                  else (0, 110, 0)),
                 ('Bấm khu vực để đi tới  ·  xanh: dự kiến cả lộ trình  ·  cam: Nav2 đang bám  ·  □ cửa  ✕ đích',
                  (90, 90, 90))]
        for i, (t, c) in enumerate(lines):
            d.text((8, y0 + i * 27), t[:110], font=self.font, fill=c)

        frame = cv2.cvtColor(np.asarray(img), cv2.COLOR_RGB2BGR)
        now = time.time()
        if self.snapshot and now - self.last_snap > 2.0:
            cv2.imwrite(self.snapshot, frame)
            self.last_snap = now
        if self.show:
            cv2.imshow(WINDOW, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                out = os.path.join(B7_DIR, 'figures', f'map_view_{time.strftime("%H%M%S")}.png')
                cv2.imwrite(out, frame)
                self.get_logger().info(f'Đã lưu {out}')
            elif key in (ord('q'), 27):
                rclpy.try_shutdown()


def main():
    rclpy.init(args=sys.argv)
    node = MapView()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
