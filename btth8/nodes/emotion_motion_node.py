#!/usr/bin/env python3
"""Node điều khiển: /emotion (std_msgs/String) -> quỹ đạo tương ứng -> /cmd_vel (geometry_msgs/Twist).

    happy    -> cong sang trái     (v = 0.15 m/s, w = +0.45 rad/s, 4 s)
    sad      -> cong sang phải     (v = 0.15 m/s, w = -0.45 rad/s, 4 s)
    neutral  -> dừng tại chỗ
    angry    -> lùi nhẹ 0.2 m      (đo bằng /odom)
    surprise -> xoay đúng 1 vòng   (tích luỹ yaw từ /odom)
    fear     -> tiến nhẹ 0.2 m     (đo bằng /odom)
    none     -> dừng (không thấy mặt / mất tín hiệu từ detector)

Mỗi biểu cảm ổn định kích hoạt một lượt chuyển động; biểu cảm khác xuất hiện thì lượt đang chạy bị thay thế.
Giữ nguyên biểu cảm sau khi lượt kết thúc `repeat_delay` giây thì lặp lại (repeat:=true).
Trạng thái được phát trên /emotion_motion/status (JSON) để cửa sổ camera hiển thị.
"""
import json
import math
import os
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from motion_plans import Action, GoHome, default_actions, Executor  # noqa: E402

try:
    from irobot_create_msgs.action import Undock
    from irobot_create_msgs.msg import DockStatus
except ImportError:                    # chạy với robot không phải Create 3 / TurtleBot 4
    Undock = DockStatus = None


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class EmotionMotion(Node):
    def __init__(self):
        super().__init__('emotion_motion')
        P = {'cmd_vel_topic': '/cmd_vel', 'rate': 20.0, 'repeat': True, 'repeat_delay': 1.5,
             'interruptible': True, 'stale_timeout': 1.5, 'auto_undock': True, 'backup_override': True,
             'arc_speed': 0.15, 'arc_turn': 0.45, 'arc_time': 4.0, 'step_speed': 0.10,
             'back_dist': 0.20, 'fwd_dist': 0.20, 'spin_speed': 1.0, 'after_undock_forward': 1.0}
        for k, v in P.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in P}
        self.actions = default_actions(self.p)

        self.cmd_pub = self.create_publisher(Twist, self.p['cmd_vel_topic'], 10)
        self.status_pub = self.create_publisher(String, '/emotion_motion/status', 10)
        self.create_subscription(String, '/emotion', self.on_emotion, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.pose = None               # (x, y, yaw) từ /odom
        self.label = 'none'            # biểu cảm mới nhất nhận được
        self.label_time = None         # lúc nhận /emotion gần nhất (phát hiện detector chết)
        self.ex = None                 # Executor của lượt hiện tại
        self.ex_label = None
        self.done_time = None
        self.count = 0                 # số lượt đã chạy
        self.idle_zero = 0             # số lệnh 0 còn phải gửi sau khi dừng (đảm bảo robot đứng yên)
        self.ready = not self.p['auto_undock'] or DockStatus is None
        self.docked = None
        self.intro = None              # đoạn tiến ra sau khi undock
        self.home = None               # vị trí (x, y, yaw) trong khung odom lúc sẵn sàng
        self.gohome = None             # bộ điều khiển đang đưa robot về home
        self.create_subscription(String, '/emotion_motion/command', self.on_command, 10)

        if not self.ready:
            self.create_subscription(DockStatus, '/dock_status', self.on_dock,
                                     QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.undock_client = ActionClient(self, Undock, '/undock')
            self.undock_sent = False
            self.create_timer(8.0, self.dock_timeout)
        if self.p['backup_override']:
            self.set_backup_override()

        self.dt = 1.0 / self.p['rate']
        self.create_timer(self.dt, self.tick)
        self.get_logger().info('Sẵn sàng. Ánh xạ: ' + ', '.join(
            f'{e}->{a.text}' for e, a in self.actions.items() if e != 'none'))

    # ---------- Create 3: cho phép lùi, rời dock ----------
    def set_backup_override(self):
        """Create 3 chặn lùi quá xa khi không có cảm biến cliff phía sau (hazard BACKUP_LIMIT).
        Đặt safety_override = backup_only trên /motion_control để lệnh 'lùi nhẹ' không bị chặn."""
        cli = self.create_client(SetParameters, '/motion_control/set_parameters')
        if not cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('Không thấy /motion_control – bỏ qua safety_override')
            return
        req = SetParameters.Request()
        req.parameters = [ParamMsg(name='safety_override', value=ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value='backup_only'))]
        cli.call_async(req).add_done_callback(
            lambda f: self.get_logger().info('motion_control: safety_override = backup_only'))

    def on_dock(self, msg):
        self.docked = msg.is_docked
        if self.ready or self.undock_sent:
            return
        if not msg.is_docked:
            self.ready = True
            self.get_logger().info('Robot không ở trên dock – bắt đầu nhận lệnh biểu cảm')
            return
        if not self.undock_client.wait_for_server(timeout_sec=0.5):
            return
        self.undock_sent = True
        self.get_logger().info('Robot đang ở trên dock -> gửi /undock ...')
        self.undock_client.send_goal_async(Undock.Goal()).add_done_callback(self.on_undock_accepted)

    def on_undock_accepted(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().warn('Undock bị từ chối – vẫn tiếp tục')
            self.ready = True
            return
        gh.get_result_async().add_done_callback(self.on_undocked)

    def on_undocked(self, _):
        d = self.p['after_undock_forward']
        if d > 0:                      # tiến ra giữa sân cho rộng chỗ (undock để robot sát dock)
            self.get_logger().info(f'Đã rời dock – tiến ra {d:.1f} m')
            self.intro = Executor(Action('intro', 'Ra giua san', 0.2, 0.0, 'distance', d), self.pose)
        else:
            self.get_logger().info('Đã rời dock – bắt đầu nhận lệnh biểu cảm')
            self.ready = True

    def dock_timeout(self):
        if not self.ready and self.docked is None:
            self.get_logger().warn('Không có /dock_status sau 8 s – bỏ qua bước undock')
            self.ready = True

    # ---------- đầu vào ----------
    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_command(self, msg):
        c = msg.data.strip().lower()
        if c == 'home':
            if self.home is None or self.pose is None:
                self.get_logger().warn('Chưa có vị trí home (robot chưa sẵn sàng)')
                return
            self.gohome = GoHome(self.home)
            self.ex = None
            self.get_logger().info('VỀ CHỖ CŨ: x=%.2f y=%.2f yaw=%.0f°' % (
                self.home[0], self.home[1], math.degrees(self.home[2])))
        elif c == 'sethome' and self.pose is not None:
            self.home = self.pose
            self.get_logger().info('Đã đặt home tại vị trí hiện tại')
        elif c == 'cancel':
            self.gohome = None

    def on_emotion(self, msg):
        lab = msg.data.strip().lower()
        if lab not in self.actions:
            return
        self.label_time = self.get_clock().now()
        if lab != self.label:
            self.get_logger().info(f'Biểu cảm: {self.label} -> {lab}')
        self.label = lab

    # ---------- vòng điều khiển ----------
    def start(self, lab):
        self.ex = Executor(self.actions[lab], self.pose)
        self.ex_label = lab
        self.done_time = None
        if not self.ex.done:
            self.count += 1
            self.get_logger().info(f'Lượt {self.count}: {lab} -> {self.actions[lab].text}')

    def tick(self):
        now = self.get_clock().now()
        lab = self.label
        if self.label_time is None or (now - self.label_time).nanoseconds * 1e-9 > self.p['stale_timeout']:
            lab = 'none'               # không nhận /emotion -> dừng an toàn
        if self.intro is not None:
            v, w, done = self.intro.step(self.dt, self.pose)
            self.publish(v, w)
            if done:
                self.intro = None
                self.ready = True
                self.home = self.pose
                self.get_logger().info('Sẵn sàng – bắt đầu nhận lệnh biểu cảm')
            self.status_pub.publish(String(data=json.dumps({
                'emotion': 'none', 'action': 'Ra giua san', 'running': True, 'progress': self.intro.progress()
                if self.intro else 1.0, 'v': v, 'w': w, 'ready': False, 'count': 0})))
            return
        if not self.ready:
            lab = 'none'
            if self.home is None and self.pose is not None:
                self.home = self.pose          # không undock: home = vị trí lúc bắt đầu

        if self.gohome is not None:
            v, w, done = self.gohome.step(self.pose)
            self.publish(v, w)
            if done:
                self.gohome = None
                self.idle_zero = 5
                self.get_logger().info('Đã về chỗ cũ')
            self.status_pub.publish(String(data=json.dumps({
                'emotion': lab, 'action': 'VE CHO CU', 'running': not done, 'progress': 0.0, 'v': v, 'w': w,
                'ready': True, 'count': self.count})))
            return

        running = self.ex is not None and not self.ex.done
        if lab != self.ex_label:
            # neutral/none luôn dừng ngay; biểu cảm khác thay thế lượt đang chạy (trừ khi interruptible=false)
            if not running or self.p['interruptible'] or lab in ('neutral', 'none'):
                self.start(lab)
        elif not running and self.p['repeat'] and lab not in ('neutral', 'none'):
            if self.done_time is None:
                self.done_time = now
            elif (now - self.done_time).nanoseconds * 1e-9 >= self.p['repeat_delay']:
                self.start(lab)

        v = w = 0.0
        if self.ex is not None and not self.ex.done:
            v, w, _ = self.ex.step(self.dt, self.pose)
        if v or w:
            self.idle_zero = 5
            self.publish(v, w)
        elif self.idle_zero > 0:       # gửi vài lệnh 0 rồi im để không giành /cmd_vel với teleop
            self.idle_zero -= 1
            self.publish(0.0, 0.0)

        st = {'emotion': lab, 'action': self.actions[self.ex_label].text if self.ex_label else '',
              'running': bool(self.ex and not self.ex.done), 'progress': self.ex.progress() if self.ex else 1.0,
              'v': v, 'w': w, 'ready': self.ready, 'count': self.count}
        self.status_pub.publish(String(data=json.dumps(st)))

    def publish(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.cmd_pub.publish(t)

    def stop(self):
        try:
            for _ in range(3):
                self.publish(0.0, 0.0)
        except Exception:              # context đã đóng (Ctrl+C)
            pass


def main():
    rclpy.init()
    node = EmotionMotion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
