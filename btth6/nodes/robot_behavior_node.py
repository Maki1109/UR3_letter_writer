#!/usr/bin/env python3
"""Phần 3 – robot_behavior_node: /detected_gesture -> máy trạng thái -> /cmd_vel cho TurtleBot 4.

  Cử chỉ        Hành vi
  Wave          xoay trái-phải chào lại (chạy ngay khi robot đang chờ, không cần xác nhận)
  Pointing_Up   CHỌN hành vi tiến-lùi 4 lần
  Victory       CHỌN hành vi chạy theo quỹ đạo tròn
  Thumb_Up      XÁC NHẬN: thực hiện hành vi đã chọn (hoặc tiếp tục hành vi đang tạm dừng)
  Open_Palm     DỪNG ngay lập tức (hành vi chuyển sang PAUSED, Thumb_Up để chạy tiếp)
  Closed_Fist   HỦY hành vi / lựa chọn hiện tại, về trạng thái chờ (IDLE)

  UNDOCKING -> IDLE --Pointing_Up/Victory--> SELECTED --Thumb_Up--> EXECUTING --xong--> IDLE
                                                                    |  ^
                                                        Open_Palm / |  | Thumb_Up
                                                      mất tín hiệu  v  |
                                                                    PAUSED      (Closed_Fist: mọi trạng thái -> IDLE)

Chống kích hoạt liên tục: node chỉ phản ứng khi nhãn ổn định ĐỔI sang một cử chỉ mới (sườn lên); giữ nguyên
cử chỉ không lặp lệnh, và cùng một cử chỉ phải cách lần trước ít nhất `cooldown` giây. Riêng Open_Palm luôn
được xử lý ngay (an toàn).

Mất tín hiệu: không nhận /detected_gesture quá `signal_timeout` giây (node nhận diện tắt, webcam lỗi) hoặc
/odom quá `odom_timeout` giây -> dừng robot, hành vi chuyển sang PAUSED.

Quãng đường và góc quay đo bằng /odom (vòng kín), nên đúng kích thước kể cả khi Gazebo chạy chậm hơn thời
gian thực. Khi khởi động, nếu robot đang trên dock thì node tự undock rồi tiến ra `clear_dock` m để có chỗ lùi.

Create 3 mặc định chặn lùi quá ~0.5 m (hazard BACKUP_LIMIT, robot đứng im giữa lượt lùi), nên node đặt tham số
/motion_control safety_override = backup_only (`safety_override`). Sân mô phỏng phẳng, không có vực.
"""
import math
import time
import unicodedata

import rclpy
from geometry_msgs.msg import Twist
from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import DockStatus
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

NAMES = {'wave': 'chào (xoay trái-phải)', 'forward_back': 'tiến-lùi x4', 'circle': 'quỹ đạo tròn',
         'clear_dock': 'rời khỏi dock'}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class RobotBehaviorNode(Node):
    def __init__(self):
        super().__init__('robot_behavior_node')
        self.declare_parameter('linear_speed', 0.2)      # m/s
        self.declare_parameter('fb_distance', 0.4)       # m mỗi lượt tiến / lùi
        self.declare_parameter('fb_repeats', 4)
        self.declare_parameter('circle_radius', 0.5)     # m
        self.declare_parameter('circle_speed', 0.2)      # m/s
        self.declare_parameter('wave_angle', 0.6)        # rad mỗi bên (~35°)
        self.declare_parameter('wave_speed', 1.2)        # rad/s
        self.declare_parameter('wave_repeats', 2)
        self.declare_parameter('cooldown', 1.5)          # s giữa 2 lần cùng một lệnh
        self.declare_parameter('signal_timeout', 1.0)    # s không có /detected_gesture -> dừng
        self.declare_parameter('odom_timeout', 0.5)
        self.declare_parameter('auto_undock', True)
        self.declare_parameter('clear_dock', 1.0)        # m tiến ra sau khi undock
        self.declare_parameter('safety_override', 'backup_only')   # '' = giữ nguyên cấu hình Create 3
        p = lambda n: self.get_parameter(n).value   # noqa: E731
        self.prm = {n: p(n) for n in ('linear_speed', 'fb_distance', 'fb_repeats', 'circle_radius',
                                      'circle_speed', 'wave_angle', 'wave_speed', 'wave_repeats', 'clear_dock')}
        self.cooldown, self.signal_timeout, self.odom_timeout = p('cooldown'), p('signal_timeout'), p('odom_timeout')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/robot_behavior/state', 10)
        self.create_subscription(String, '/detected_gesture', self.on_gesture, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)

        self.state = 'IDLE'
        self.selected = None        # hành vi đã chọn, chờ Thumb_Up
        self.active = None          # hành vi đang chạy / tạm dừng
        self.plan, self.seg_i, self.seg = [], 0, None
        self.last_label, self.last_cmd_t = 'None', {}
        self.t_gesture = None       # thời điểm nhận /detected_gesture gần nhất (đồng hồ thật)
        self.odom, self.t_odom = None, None
        self.stop_ticks = 0         # số chu kỳ còn gửi vận tốc 0 sau khi dừng

        if p('auto_undock'):
            self.state = 'UNDOCKING'
            self.dock_status, self.undock_sent = None, False
            self.create_subscription(DockStatus, '/dock_status', self.on_dock, qos_profile_sensor_data)
            self.undock_client = ActionClient(self, Undock, '/undock')
            self.t_start = time.monotonic()
        if p('safety_override'):
            self.set_safety_override(p('safety_override'))
        self.create_timer(0.05, self.control)       # 20 Hz
        self.create_timer(0.5, self.publish_state)
        self.get_logger().info('robot_behavior_node sẵn sàng, chờ /detected_gesture')

    def set_safety_override(self, mode):
        cli = self.create_client(SetParameters, '/motion_control/set_parameters')
        if not cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn('Không gọi được /motion_control/set_parameters: robot có thể không lùi đủ xa')
            return
        val = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=mode)
        cli.call_async(SetParameters.Request(parameters=[Parameter(name='safety_override', value=val)]))
        self.get_logger().info(f'motion_control: safety_override = {mode}')

    # ---------------------------------------------------------------- input
    def on_odom(self, msg):
        pose = msg.pose.pose
        self.odom = (pose.position.x, pose.position.y, yaw_of(pose.orientation))
        self.t_odom = time.monotonic()

    def on_dock(self, msg):
        if self.dock_status is None:
            self.dock_status = msg.is_docked
            if not msg.is_docked:
                self.get_logger().info('Robot không ở trên dock, bỏ qua undock')
                self.state = 'IDLE'

    def try_undock(self):
        """Gửi goal /undock khi đã biết robot đang trên dock và action server sẵn sàng (gọi từ vòng 20 Hz)."""
        if self.dock_status and not self.undock_sent and self.undock_client.server_is_ready():
            self.undock_sent = True
            self.get_logger().info('Robot đang trên dock -> undock')
            self.undock_client.send_goal_async(Undock.Goal()).add_done_callback(self.on_undock_accepted)

    def on_undock_accepted(self, fut):
        fut.result().get_result_async().add_done_callback(self.on_undocked)

    def on_undocked(self, _):
        self.get_logger().info(f'Undock xong, tiến ra {self.prm["clear_dock"]} m để có chỗ lùi')
        self.start('clear_dock')

    def on_gesture(self, msg):
        self.t_gesture = time.monotonic()
        label = msg.data
        if label == self.last_label:
            return                  # giữ nguyên cử chỉ -> không lặp lệnh
        self.last_label = label
        if label == 'None':
            return
        now = time.monotonic()
        if label != 'Open_Palm' and now - self.last_cmd_t.get(label, -1e9) < self.cooldown:
            self.get_logger().info(f'Bỏ qua {label} (cooldown)')
            return
        self.last_cmd_t[label] = now
        self.dispatch(label)

    # ---------------------------------------------------------------- state machine
    def dispatch(self, g):
        s = self.state
        if s == 'UNDOCKING' or (s == 'CLEARING' and g != 'Open_Palm'):
            self.get_logger().info(f'{g}: robot đang rời dock, chưa nhận lệnh')
            return
        if g == 'Open_Palm':
            if s in ('EXECUTING', 'CLEARING'):
                self.halt('PAUSED', f'DỪNG ngay: {NAMES[self.active]} tạm dừng, Thumb_Up để tiếp tục, '
                                    f'Closed_Fist để hủy')
            else:
                self.halt(s, 'DỪNG (robot đứng yên)')
        elif g == 'Closed_Fist':
            if s != 'IDLE':
                what = NAMES.get(self.active or self.selected, '')
                self.selected = self.active = None
                self.plan, self.seg = [], None
                self.halt('IDLE', f'HỦY {what} -> trạng thái chờ')
        elif g in ('Pointing_Up', 'Victory'):
            if s in ('IDLE', 'SELECTED'):
                self.selected = 'forward_back' if g == 'Pointing_Up' else 'circle'
                self.state = 'SELECTED'
                self.get_logger().info(f'CHỌN: {NAMES[self.selected]} -> giơ Thumb_Up để xác nhận')
            else:
                self.get_logger().info(f'{g}: đang {s.lower()} {NAMES[self.active]}, bỏ qua '
                                       '(Closed_Fist để hủy trước)')
        elif g == 'Thumb_Up':
            if s == 'SELECTED':
                self.get_logger().info(f'XÁC NHẬN: thực hiện {NAMES[self.selected]}')
                self.start(self.selected)
                self.selected = None
            elif s == 'PAUSED' and self.active:
                self.get_logger().info(f'XÁC NHẬN: tiếp tục {NAMES[self.active]}')
                self.state = 'EXECUTING'
                self.begin_segment(resume=True)
            elif s == 'EXECUTING':
                self.get_logger().info(f'Thumb_Up: đang thực hiện {NAMES[self.active]}')
            else:
                self.get_logger().info('Thumb_Up: chưa chọn hành vi (Pointing_Up / Victory)')
        elif g == 'Wave':
            if s == 'IDLE':
                self.get_logger().info('Wave: chào lại')
                self.start('wave')
            else:
                self.get_logger().info(f'Wave: robot đang {s}, bỏ qua')
        else:
            self.get_logger().info(f'{g}: không gán hành vi')

    def halt(self, new_state, reason):
        self.state = new_state
        self.cmd_pub.publish(Twist())
        self.stop_ticks = 10        # gửi tiếp vận tốc 0 trong 0.5 s
        self.get_logger().warn(reason)

    # ---------------------------------------------------------------- behaviours
    def build_plan(self, name):
        """Danh sách đoạn chuyển động: ('lin', quãng đường có dấu, tốc độ) | ('rot', góc có dấu, tốc độ góc)
        | ('arc', góc có dấu, vận tốc dài, vận tốc góc) | ('wait', giây)."""
        P = self.prm
        if name == 'wave':
            a, w = P['wave_angle'], P['wave_speed']
            plan = [('rot', a, w)]
            for _ in range(P['wave_repeats']):
                plan += [('rot', -2 * a, w), ('rot', 2 * a, w)]
            return plan + [('rot', -a, w)]
        if name == 'forward_back':
            d, v = P['fb_distance'], P['linear_speed']
            plan = []
            for _ in range(P['fb_repeats']):
                plan += [('lin', d, v), ('wait', 0.3), ('lin', -d, v), ('wait', 0.3)]
            return plan
        if name == 'circle':
            v = P['circle_speed']
            return [('arc', 2 * math.pi, v, v / P['circle_radius'])]
        if name == 'clear_dock':
            return [('lin', P['clear_dock'], P['linear_speed'])]
        raise ValueError(name)

    def start(self, name):
        self.active, self.plan, self.seg_i = name, self.build_plan(name), 0
        self.state = 'CLEARING' if name == 'clear_dock' else 'EXECUTING'
        self.begin_segment()

    def begin_segment(self, resume=False):
        """Bắt đầu đoạn self.seg_i; khi resume thì chỉ đi nốt phần còn lại của đoạn."""
        if self.seg_i >= len(self.plan):
            self.finish()
            return
        kind, amount = self.plan[self.seg_i][0], self.plan[self.seg_i][1]
        if resume and self.seg is not None:
            amount = math.copysign(self.seg['remaining'], amount)
        self.seg = {'kind': kind, 'target': abs(amount), 'sign': math.copysign(1, amount),
                    'args': self.plan[self.seg_i][2:], 'remaining': abs(amount),
                    'start': self.odom, 'yaw_acc': 0.0, 'yaw_prev': self.odom[2] if self.odom else 0.0,
                    't0': time.monotonic()}

    def finish(self):
        done = self.active
        self.plan, self.seg, self.active = [], None, None
        self.halt('IDLE', f'Hoàn thành {NAMES[done]} -> trạng thái chờ')

    def segment_cmd(self):
        """Tính Twist cho đoạn hiện tại từ /odom; trả None khi đoạn đã xong."""
        sg, cmd = self.seg, Twist()
        x, y, yaw = self.odom
        if sg['kind'] == 'wait':
            sg['remaining'] = sg['target'] - (time.monotonic() - sg['t0'])
            return cmd if sg['remaining'] > 0 else None
        if sg['start'] is None:
            sg['start'], sg['yaw_prev'] = self.odom, yaw
        if sg['kind'] == 'lin':
            done = math.hypot(x - sg['start'][0], y - sg['start'][1])
            ramp = 0.08
        else:
            sg['yaw_acc'] += abs(wrap(yaw - sg['yaw_prev']))
            sg['yaw_prev'] = yaw
            done = sg['yaw_acc']
            ramp = 0.25
        sg['remaining'] = sg['target'] - done
        if sg['remaining'] <= 0.005:
            return None
        k = max(0.25, min(1.0, sg['remaining'] / ramp))    # giảm tốc khi gần tới đích
        s = sg['sign']
        if sg['kind'] == 'lin':
            cmd.linear.x = s * k * sg['args'][0]
        elif sg['kind'] == 'rot':
            cmd.angular.z = s * k * sg['args'][0]
        else:   # arc: giữ tỉ lệ v/w để bán kính không đổi
            v, w = sg['args']
            cmd.linear.x, cmd.angular.z = k * v, s * k * w
        return cmd

    # ---------------------------------------------------------------- 20 Hz loop
    def control(self):
        now = time.monotonic()
        if self.state in ('EXECUTING', 'CLEARING'):
            if self.state == 'EXECUTING' and (self.t_gesture is None or now - self.t_gesture > self.signal_timeout):
                self.halt('PAUSED', f'MẤT TÍN HIỆU /detected_gesture > {self.signal_timeout} s -> dừng robot')
                return
            if self.odom is None or now - self.t_odom > self.odom_timeout:
                self.halt('PAUSED', 'Mất /odom -> dừng robot')
                return
            cmd = self.segment_cmd()
            if cmd is None:
                self.seg_i += 1
                self.seg = None
                self.begin_segment()
                return
            self.cmd_pub.publish(cmd)
        elif self.stop_ticks > 0:
            self.stop_ticks -= 1
            self.cmd_pub.publish(Twist())
        if self.state == 'UNDOCKING':
            self.try_undock()
            if self.dock_status is None and now - self.t_start > 30.0:
                self.get_logger().warn('Không nhận được /dock_status sau 30 s, bỏ qua undock')
                self.state = 'IDLE'

    def publish_state(self):
        s = self.state
        if s == 'SELECTED':
            text = f'SELECTED: {NAMES[self.selected]} (Thumb_Up de xac nhan)'
        elif s in ('EXECUTING', 'PAUSED', 'CLEARING'):
            prog = f' {self.seg_i + 1}/{len(self.plan)}' if self.plan else ''
            text = f'{s}: {NAMES.get(self.active, "")}{prog}'
        else:
            text = s
        # cv2.putText không vẽ được dấu tiếng Việt -> bỏ dấu cho cửa sổ webcam
        self.state_pub.publish(String(data=strip_accents(text)))


def strip_accents(text):
    text = text.replace('đ', 'd').replace('Đ', 'D')
    return ''.join(c for c in unicodedata.normalize('NFD', text) if unicodedata.category(c) != 'Mn')


def main():
    rclpy.init()
    node = RobotBehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
