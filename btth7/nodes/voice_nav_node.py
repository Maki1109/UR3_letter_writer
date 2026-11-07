#!/usr/bin/env python3
"""Phần 4 – Điều hướng bằng giọng nói: /voice_command (văn bản) -> phân tích -> NavigateToPose của Nav2.

  - Đọc config/zones.yaml (sinh bởi tools/segment_map.py): id, tên, bí danh, điểm đích, đường bao và các "cửa"
    giữa hai khu vực kề nhau.
  - Mỗi câu lệnh được CommandParser chuyển thành lộ trình có bao nhiêu điểm cũng được, ví dụ
    "Đi từ phòng A qua hành lang C rồi đến phòng B" -> [A, C, B] với A là điểm xuất phát. Robot đang ở trong A
    (vị trí /amcl_pose nằm trong đường bao của A) thì bỏ A, còn đang ở chỗ khác thì đi qua A trước.
  - Lập kế hoạch cả lộ trình ngay khi nhận lệnh: giữa hai khu vực không kề nhau, robot đi qua các cửa (BFS trên đồ
    thị khu vực). Mỗi cửa và mỗi điểm đích là một chặng, tức một goal NavigateToPose. Hướng tại cửa nhìn về chặng
    sau; hướng tại điểm đích là hướng từ chặng trước tới, để tới nơi robot không phải quay tại chỗ.
  - Xem trước đường đi: trước khi chạy, mỗi chặng được gửi tới planner của Nav2 (action /compute_path_to_pose,
    cùng planner bt_navigator dùng). Các đoạn được nối thành /voice_nav/planned_path (nav_msgs/Path). Planner lỗi
    ở chặng nào thì chặng đó vẽ bằng đoạn thẳng. Khi robot đi, path chỉ còn phần chưa đi.
  - Lệnh mới thay lệnh cũ: goal đang chạy bị hủy rồi lộ trình mới bắt đầu. "Dừng lại" hủy goal đang chạy.
  - Câu có khu vực không tồn tại (hoặc không hiểu) bị từ chối và báo danh sách khu vực hợp lệ. Robot giữ nguyên
    việc đang làm.
  - Robot đang trên dock thì được undock trước chặng đầu tiên.
  - Topic ra: /voice_nav/status (String, câu báo kết quả), /voice_nav/route (String JSON: lộ trình, các chặng,
    chặng hiện tại, trạng thái – map_view_node dùng), /voice_nav/planned_path, và /zones/markers (MarkerArray:
    khu vực tô màu, tên, điểm đích; các điểm của lộ trình đánh số 1, 2, 3...: xám = đã qua, cam = đang tới,
    xanh = sắp tới).

    python3 nodes/voice_nav_node.py --ros-args -p use_sim_time:=true
"""
import json
import math
import os
import sys

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import DockStatus
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

HERE = os.path.dirname(os.path.abspath(__file__))
B7_DIR = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from command_parser import CommandParser  # noqa: E402

COLORS = [(0.91, 0.30, 0.24), (0.18, 0.80, 0.44), (0.20, 0.60, 0.86), (0.95, 0.77, 0.06), (0.61, 0.35, 0.71),
          (0.10, 0.74, 0.61), (0.90, 0.49, 0.13), (0.58, 0.65, 0.65)]     # giống figures/zones.png
LEG_COLORS = {'done': (0.55, 0.55, 0.55), 'current': (1.0, 0.55, 0.0), 'pending': (0.1, 0.35, 0.95)}


def inside(poly, x, y):
    """Điểm (x, y) có nằm trong đa giác không (ray casting)."""
    c = False
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            c = not c
    return c


class VoiceNavNode(Node):
    def __init__(self):
        super().__init__('voice_nav')
        zones_file = self.declare_parameter('zones_file', os.path.join(B7_DIR, 'config', 'zones.yaml')).value
        topic = self.declare_parameter('command_topic', '/voice_command').value
        self.auto_undock = self.declare_parameter('undock', True).value
        self.goal_timeout = self.declare_parameter('goal_timeout', 240.0).value
        self.preview = self.declare_parameter('preview', True).value     # xem trước đường đi bằng planner

        with open(zones_file) as f:
            cfg = yaml.safe_load(f)
        self.frame = cfg.get('frame_id', 'map')
        self.zones = {str(z['id']): z for z in cfg['zones']}
        self.parser = CommandParser(cfg['zones'])
        self.doors = {}             # đồ thị khu vực: doors[a][b] = điểm cửa (x, y) giữa a và b
        for d in cfg.get('doors', []):
            a, b = d['zones']
            self.doors.setdefault(a, {})[b] = self.doors.setdefault(b, {})[a] = tuple(d['point'])

        self.pose = None            # (x, y) từ AMCL
        self.docked = None          # None = chưa biết (/dock_status chỉ publish khi đổi trạng thái) -> thử undock
        self.run = 0                # tăng mỗi lệnh mới: kết quả của goal/plan cũ (đã bị thay) bị bỏ qua
        self.route = []             # các khu vực của lệnh hiện tại
        self.legs = []              # các chặng: dict(zone, door, x, y, yaw, path=[(x, y)...])
        self.leg_i = 0              # chặng đang chạy
        self.state = 'idle'         # idle | planning | running | done | failed | stopped
        self.text = ''
        self.goal_handle = None
        self.goal_t0 = None
        self.last_fb = 0.0

        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.planner = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.undock_client = ActionClient(self, Undock, 'undock')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/voice_nav/status', 10)
        self.route_pub = self.create_publisher(String, '/voice_nav/route', latched)
        self.path_pub = self.create_publisher(Path, '/voice_nav/planned_path', latched)
        self.marker_pub = self.create_publisher(MarkerArray, '/zones/markers', latched)
        self.create_subscription(String, topic, self.on_command, 10)
        # AMCL chỉ publish khi robot di chuyển; TRANSIENT_LOCAL để nhận ngay pose cuối cùng
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, latched)
        self.create_subscription(DockStatus, '/dock_status', self.on_dock, qos_profile_sensor_data)
        self.create_timer(1.0, self.on_timer)
        self.publish_all()

        names = ', '.join(f"{z['id']} ({z['name']})" for z in self.zones.values())
        self.get_logger().info(f'{len(self.zones)} khu vực: {names}. Đợi câu lệnh trên {topic} ...')

    # --- trạng thái robot ------------------------------------------------------------------------------------
    def on_pose(self, msg):
        p = msg.pose.pose.position
        first = self.pose is None
        self.pose = (p.x, p.y)
        if first:
            z = self.current_zone()
            self.get_logger().info(f'Vị trí robot ({p.x:.2f}, {p.y:.2f}) – khu vực {z or "?"}')

    def on_dock(self, msg):
        self.docked = msg.is_docked

    def zone_at(self, xy):
        return next((zid for zid, z in self.zones.items() if inside(z['polygon'], *xy)), None) if xy else None

    def current_zone(self):
        return self.zone_at(self.pose)

    def say(self, text, error=False):
        self.status_pub.publish(String(data=text))
        # rclpy không cho cùng một dòng log đổi mức (info/error) giữa các lần gọi -> tách 2 nhánh
        if error:
            self.get_logger().error(text)
        else:
            self.get_logger().info(text)

    def zname(self, zid):
        return f"{zid} ({self.zones[zid]['name']})"

    # --- câu lệnh -------------------------------------------------------------------------------------------
    def on_command(self, msg):
        text = msg.data.strip()
        cmd = self.parser.parse(text)
        self.get_logger().info(f'Câu lệnh: "{text}" -> {cmd.describe(self.zones)}')
        if cmd.intent == 'unknown_zone':
            self.say(cmd.describe(self.zones), error=True)
            return
        if cmd.intent == 'not_understood':
            self.say(f'Không tìm thấy khu vực nào trong câu "{text}" – bỏ qua', error=True)
            return
        if cmd.intent == 'stop':
            self.run += 1
            self.cancel()
            self.state = 'stopped'
            self.say('Đã dừng theo lệnh')
            self.publish_all()
            return

        route = list(cmd.route)
        here = self.current_zone()
        if cmd.origin:
            if here == cmd.origin:
                route.pop(0)
                self.get_logger().info(f'Robot đang ở {self.zname(here)}: bỏ qua điểm xuất phát')
            else:
                self.get_logger().info(f'Robot đang ở {self.zname(here) if here else "ngoài các khu vực"}: '
                                       f'tới điểm xuất phát {self.zname(cmd.origin)} trước')
        self.run += 1
        self.cancel()
        self.text, self.route = text, route
        self.legs, self.leg_i = self.make_legs(route), 0
        self.say('Lộ trình: ' + ' -> '.join(self.zname(z) for z in route) +
                 f' ({len(self.legs)} chặng, qua {sum(1 for lg in self.legs if lg["door"])} cửa)')
        self.state = 'planning'
        self.publish_all()
        if self.preview and self.planner.wait_for_server(timeout_sec=2.0):
            self.plan_leg(self.run, 0)
        else:
            self.start(self.run)

    # --- lập lộ trình ---------------------------------------------------------------------------------------
    def door_path(self, src, dst):
        """BFS trên đồ thị khu vực -> các cửa (tên, x, y) phải đi qua từ src tới dst."""
        if src is None or src == dst:
            return []
        parent, todo = {src: None}, [src]
        while todo:
            a = todo.pop(0)
            for b in self.doors.get(a, {}):
                if b not in parent:
                    parent[b] = a
                    todo.append(b)
        if dst not in parent:
            return []
        path, b = [], dst
        while parent[b] is not None:
            path.append((f'{parent[b]}–{b}', *self.doors[parent[b]][b]))
            b = parent[b]
        return path[::-1]

    def make_legs(self, route):
        """Cả lộ trình thành danh sách chặng (cửa + điểm đích). Khu vực xuất phát = khu vực chứa điểm trước."""
        legs, prev = [], self.pose
        for zid in route:
            for door, x, y in self.door_path(self.zone_at(prev), zid):
                legs.append({'zone': zid, 'door': door, 'x': x, 'y': y})
                prev = (x, y)
            gx, gy = self.zones[zid]['goal']
            legs.append({'zone': zid, 'door': None, 'x': gx, 'y': gy})
            prev = (gx, gy)
        prev = self.pose
        for i, lg in enumerate(legs):
            if lg['door'] and i + 1 < len(legs):         # tại cửa: nhìn về chặng sau
                nx, ny = legs[i + 1]['x'], legs[i + 1]['y']
                lg['yaw'] = math.atan2(ny - lg['y'], nx - lg['x'])
            else:                                       # tại điểm đích: hướng đi tới
                lg['yaw'] = math.atan2(lg['y'] - prev[1], lg['x'] - prev[0]) if prev else 0.0
            lg['path'] = []
            prev = (lg['x'], lg['y'])
        return legs

    def pose_stamped(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = self.frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return p

    def plan_leg(self, run, i):
        """Xem trước: gọi planner Nav2 cho từng chặng, nối tiếp nhau (mỗi chặng bắt đầu ở điểm cuối chặng trước)."""
        if run != self.run:
            return
        if i >= len(self.legs):
            total = sum(math.dist(a, b) for lg in self.legs for a, b in zip(lg['path'], lg['path'][1:]))
            self.say(f'Đường đi dự kiến: {total:.1f} m – xem /voice_nav/planned_path trên RViz hoặc map_view')
            self.publish_all()
            self.start(run)
            return
        lg = self.legs[i]
        goal = ComputePathToPose.Goal()
        goal.goal = self.pose_stamped(lg['x'], lg['y'], lg['yaw'])
        if i > 0:
            p = self.legs[i - 1]
            goal.start, goal.use_start = self.pose_stamped(p['x'], p['y'], p['yaw']), True
        else:
            goal.use_start = False                      # từ vị trí hiện tại của robot

        def straight():
            a = (self.legs[i - 1]['x'], self.legs[i - 1]['y']) if i > 0 else self.pose
            lg['path'] = ([a] if a else []) + [(lg['x'], lg['y'])]
            self.plan_leg(run, i + 1)

        def on_result(f):
            res = f.result()
            if res.status == GoalStatus.STATUS_SUCCEEDED and res.result.path.poses:
                lg['path'] = [(p.pose.position.x, p.pose.position.y) for p in res.result.path.poses]
                self.plan_leg(run, i + 1)
            else:
                self.get_logger().warn(f'    planner không tìm được đường cho chặng {i + 1} – vẽ đoạn thẳng')
                straight()

        def on_accept(f):
            h = f.result()
            if not h.accepted:
                straight()
                return
            h.get_result_async().add_done_callback(on_result)
        self.planner.send_goal_async(goal).add_done_callback(on_accept)

    # --- chạy lộ trình --------------------------------------------------------------------------------------
    def cancel(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
            self.goal_t0 = None

    def start(self, run):
        if run != self.run:
            return
        if not self.nav.wait_for_server(timeout_sec=5.0):
            self.fail('Nav2 chưa sẵn sàng (/navigate_to_pose) – chạy 03_nav2.sh rồi nói lại')
            return
        self.state = 'running'
        self.publish_all()
        if self.auto_undock and self.docked is not False:
            self.do_undock(run)
        else:
            self.send_leg(run)

    def fail(self, text):
        self.state = 'failed'
        self.say(text, error=True)
        self.publish_all()

    def send_leg(self, run):
        if run != self.run:
            return
        if self.leg_i >= len(self.legs):
            self.state = 'done'
            self.publish_all()
            return
        lg = self.legs[self.leg_i]
        where = f"cửa {lg['door']}" if lg['door'] else 'đích'
        self.say(f"[{self.leg_i + 1}/{len(self.legs)}] Đang đi tới {self.zname(lg['zone'])}: {where} "
                 f"({lg['x']:.2f}, {lg['y']:.2f}), hướng {math.degrees(lg['yaw']):.0f}°")
        self.publish_all()
        goal = NavigateToPose.Goal()
        goal.pose = self.pose_stamped(lg['x'], lg['y'], lg['yaw'])
        fut = self.nav.send_goal_async(goal, feedback_callback=self.on_feedback)
        fut.add_done_callback(lambda f: self.on_goal_response(f, run))

    def on_goal_response(self, fut, run):
        handle = fut.result()
        if run != self.run:                     # đã có lệnh mới trong lúc chờ Nav2 nhận goal
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.fail(f"Nav2 từ chối goal tới {self.zname(self.legs[self.leg_i]['zone'])}")
            return
        self.goal_handle = handle
        self.goal_t0 = self.get_clock().now()
        self.last_fb = 0.0
        handle.get_result_async().add_done_callback(lambda f: self.on_result(f, run))

    def on_feedback(self, fb):
        if self.goal_t0 is None or self.leg_i >= len(self.legs):
            return
        t = (self.get_clock().now() - self.goal_t0).nanoseconds * 1e-9
        if t - self.last_fb >= 3.0:
            self.last_fb = t
            f = fb.feedback
            q = f.current_pose.pose.position
            self.get_logger().info(f"    -> chặng {self.leg_i + 1} ({self.legs[self.leg_i]['zone']}): còn "
                                   f'{f.distance_remaining:.2f} m, t={t:.0f}s, robot ({q.x:.2f}, {q.y:.2f}), '
                                   f'recoveries={f.number_of_recoveries}')

    def on_result(self, fut, run):
        if run != self.run:
            return                              # goal của lệnh cũ (bị lệnh mới hoặc "dừng" thay thế)
        self.goal_handle, self.goal_t0 = None, None
        status = fut.result().status
        lg = self.legs[self.leg_i]
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.leg_i += 1
            if lg['door']:
                self.get_logger().info(f"    qua cửa {lg['door']}")
            else:
                last = self.leg_i >= len(self.legs)
                self.say(f"Đã đến {self.zname(lg['zone'])}" + (' – hoàn thành lộ trình' if last else ''))
            self.send_leg(run)
        else:
            name = {GoalStatus.STATUS_ABORTED: 'Nav2 không tới được', GoalStatus.STATUS_CANCELED: 'Đã hủy goal'}
            self.fail(f"{name.get(status, f'Lỗi (status {status})')} {self.zname(lg['zone'])} – dừng lộ trình")

    def do_undock(self, run):
        if not self.undock_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Không có action /undock – bỏ qua undock')
            self.auto_undock = False
            self.send_leg(run)
            return
        self.say('Robot đang trên dock: undock trước' if self.docked else 'Kiểm tra dock: undock nếu cần')
        self.docked = False                     # chỉ thử một lần; /dock_status sẽ cập nhật nếu robot vẫn docked

        def done(f):
            if not f.result().accepted:
                self.get_logger().warn('Undock bị từ chối – đi tiếp')
                self.send_leg(run)
                return
            f.result().get_result_async().add_done_callback(lambda _: self.send_leg(run))
        self.undock_client.send_goal_async(Undock.Goal()).add_done_callback(done)

    def on_timer(self):
        if self.goal_t0 is not None and self.goal_handle is not None:
            t = (self.get_clock().now() - self.goal_t0).nanoseconds * 1e-9
            if t > self.goal_timeout:
                self.say(f"Quá {self.goal_timeout:.0f}s chưa tới {self.zname(self.legs[self.leg_i]['zone'])} – hủy",
                         error=True)
                self.goal_handle.cancel_goal_async()

    # --- hiển thị: RViz + map_view --------------------------------------------------------------------------
    def leg_state(self, i):
        if self.state in ('done',) or i < self.leg_i:
            return 'done'
        if i == self.leg_i and self.state in ('running', 'failed'):
            return 'current'
        return 'pending'

    def publish_all(self):
        active = self.legs[self.leg_i]['zone'] if self.state == 'running' and self.leg_i < len(self.legs) else None
        info = {'text': self.text, 'route': self.route, 'state': self.state, 'current': self.leg_i,
                'legs': [{k: lg[k] for k in ('zone', 'door', 'x', 'y')} for lg in self.legs]}
        self.route_pub.publish(String(data=json.dumps(info, ensure_ascii=False)))

        path = Path()
        path.header.frame_id, path.header.stamp = self.frame, self.get_clock().now().to_msg()
        if self.state in ('planning', 'running'):     # chỉ phần chưa đi
            for lg in self.legs[self.leg_i:]:
                for x, y in lg['path']:
                    path.poses.append(self.pose_stamped(x, y, 0.0))
        self.path_pub.publish(path)
        self.marker_pub.publish(self.zone_markers(active))

    def marker(self, ns, k, mtype, stamp):
        m = Marker()
        m.header.frame_id, m.header.stamp = self.frame, stamp
        m.ns, m.id, m.type, m.action = ns, k, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def zone_markers(self, active):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        clear = Marker()
        clear.action = Marker.DELETEALL                 # xóa điểm lộ trình của lệnh cũ
        arr.markers.append(clear)
        for k, (zid, z) in enumerate(self.zones.items()):
            r, g, b = COLORS[k % len(COLORS)]
            on = zid == active
            fill = self.marker('zone_fill', k, Marker.CUBE_LIST, now)     # ô 0.1 m bên trong đường bao
            fill.scale.x, fill.scale.y, fill.scale.z = 0.1, 0.1, 0.01
            fill.color = ColorRGBA(r=r, g=g, b=b, a=0.55 if on else 0.25)
            xs = [p[0] for p in z['polygon']]
            ys = [p[1] for p in z['polygon']]
            x = min(xs) + 0.05
            while x < max(xs):
                y = min(ys) + 0.05
                while y < max(ys):
                    if inside(z['polygon'], x, y):
                        fill.points.append(Point(x=x, y=y, z=0.01))
                    y += 0.1
                x += 0.1
            edge = self.marker('zone_edge', k, Marker.LINE_STRIP, now)
            edge.scale.x = 0.06 if on else 0.03
            edge.color = ColorRGBA(r=r * 0.6, g=g * 0.6, b=b * 0.6, a=1.0)
            edge.points = [Point(x=float(px), y=float(py), z=0.02) for px, py in z['polygon'] + z['polygon'][:1]]
            text = self.marker('zone_name', k, Marker.TEXT_VIEW_FACING, now)
            text.pose.position.x, text.pose.position.y, text.pose.position.z = z['center'][0], z['center'][1], 0.5
            text.scale.z = 0.3
            text.color = ColorRGBA(r=0.05, g=0.05, b=0.05, a=1.0)
            text.text = f"{zid}: {z['name']}"
            goal = self.marker('zone_goal', k, Marker.CYLINDER, now)
            goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = z['goal'][0], z['goal'][1], 0.05
            goal.scale.x = goal.scale.y = 0.15
            goal.scale.z = 0.1
            goal.color = ColorRGBA(r=0.85, g=0.0, b=0.0, a=1.0)
            arr.markers += [fill, edge, text, goal]
        # các điểm của lộ trình: số thứ tự tại điểm đích, khối nhỏ tại cửa
        n = 0
        for i, lg in enumerate(self.legs):
            r, g, b = LEG_COLORS[self.leg_state(i)]
            if lg['door']:
                m = self.marker('route_door', i, Marker.CUBE, now)
                m.scale.x = m.scale.y = m.scale.z = 0.12
            else:
                n += 1
                m = self.marker('route_stop', i, Marker.SPHERE, now)
                m.scale.x = m.scale.y = m.scale.z = 0.25
                t = self.marker('route_label', i, Marker.TEXT_VIEW_FACING, now)
                t.pose.position.x, t.pose.position.y, t.pose.position.z = lg['x'], lg['y'] + 0.25, 0.4
                t.scale.z = 0.28
                t.color = ColorRGBA(r=r, g=g, b=b, a=1.0)
                t.text = f"{n}. {lg['zone']}"
                arr.markers.append(t)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = lg['x'], lg['y'], 0.12
            m.color = ColorRGBA(r=r, g=g, b=b, a=0.95)
            arr.markers.append(m)
        return arr


def main():
    rclpy.init(args=sys.argv)
    node = VoiceNavNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.cancel()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
