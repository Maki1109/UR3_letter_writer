"""Quỹ đạo ứng với từng biểu cảm (không phụ thuộc ROS, để test được độc lập).

Mỗi hành động là một đoạn chuyển động (v, w) với điều kiện dừng:
  - 'time'     chạy đúng `limit` giây           (quỹ đạo cong: bán kính R = v / w)
  - 'distance' tới khi đi được `limit` mét      (đo bằng /odom; không có odom thì quy ra thời gian)
  - 'angle'    tới khi xoay được `limit` radian (tích luỹ yaw từ /odom; không có odom thì quy ra thời gian)
"""
from dataclasses import dataclass
import math


@dataclass
class Action:
    emotion: str
    text: str          # mô tả hiển thị (không dấu để vẽ được bằng cv2.putText)
    v: float = 0.0     # m/s
    w: float = 0.0     # rad/s (+ = trái / ngược chiều kim đồng hồ)
    until: str = 'time'
    limit: float = 0.0


def default_actions(p=None):
    """p: dict tham số ghi đè (tên giống tham số ROS của emotion_motion_node)."""
    p = p or {}
    g = lambda k, d: float(p.get(k, d))
    v_arc, w_arc, t_arc = g('arc_speed', 0.15), g('arc_turn', 0.45), g('arc_time', 4.0)
    return {
        'happy':    Action('happy', 'Cong sang TRAI', v_arc, +w_arc, 'time', t_arc),
        'sad':      Action('sad', 'Cong sang PHAI', v_arc, -w_arc, 'time', t_arc),
        'neutral':  Action('neutral', 'DUNG tai cho'),
        'angry':    Action('angry', 'LUI nhe', -g('step_speed', 0.10), 0.0, 'distance', g('back_dist', 0.20)),
        'surprise': Action('surprise', 'XOAY 1 vong', 0.0, g('spin_speed', 1.0), 'angle', 2 * math.pi),
        'fear':     Action('fear', 'TIEN nhe', g('step_speed', 0.10), 0.0, 'distance', g('fwd_dist', 0.20)),
        'none':     Action('none', 'DUNG (khong thay mat)'),
    }


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Executor:
    """Chạy một Action theo từng bước điều khiển.

        ex = Executor(action, pose)          # pose = (x, y, yaw) từ odom hoặc None
        v, w, done = ex.step(dt, pose)
    """

    def __init__(self, action, pose=None):
        self.a = action
        self.t = 0.0
        self.dist = 0.0
        self.angle = 0.0
        self.last = pose
        self.cmd = (0.0, 0.0)          # lệnh (v, w) của bước trước – dùng khi không có odom
        self.done = action.v == 0.0 and action.w == 0.0

    def progress(self):
        a = self.a
        if self.done or a.limit <= 0:
            return 1.0
        cur = {'time': self.t, 'distance': self.dist, 'angle': self.angle}[a.until]
        return min(1.0, cur / a.limit)

    def step(self, dt, pose=None):
        if self.done:
            return 0.0, 0.0, True
        a = self.a
        self.t += dt
        if pose is not None and self.last is not None:
            self.dist += math.hypot(pose[0] - self.last[0], pose[1] - self.last[1])
            self.angle += abs(wrap(pose[2] - self.last[2]))
        elif pose is None:             # không có odom: ước lượng theo lệnh vận tốc đã gửi
            self.dist += abs(self.cmd[0]) * dt
            self.angle += abs(self.cmd[1]) * dt
        self.last = pose
        # Thời gian tối đa an toàn cho đoạn đo bằng odom (gấp 3 lần thời gian lý thuyết + 2 s)
        nominal = a.limit / max(abs(a.v if a.until == 'distance' else a.w), 1e-3) if a.until != 'time' else a.limit
        if self.progress() >= 1.0 or self.t > 3 * nominal + 2.0:
            self.done = True
            return 0.0, 0.0, True
        # Giảm tốc ở 15% cuối của đoạn xoay/đoạn thẳng để dừng đúng chỗ (tránh quá đà do quán tính)
        k = 1.0
        if a.until in ('distance', 'angle'):
            k = max(0.3, min(1.0, (1.0 - self.progress()) / 0.15))
        self.cmd = (a.v * k, a.w * k)
        return a.v * k, a.w * k, False


class GoHome:
    """Về điểm (x, y, yaw) trong khung odom: quay mặt về phía điểm -> đi thẳng (có chỉnh hướng) -> quay về hướng cũ.

        gh = GoHome((hx, hy, hyaw)); v, w, done = gh.step(pose)
    """

    def __init__(self, home, v_max=0.2, w_max=0.8, tol_d=0.05, tol_a=0.05):
        self.home, self.v_max, self.w_max, self.tol_d, self.tol_a = home, v_max, w_max, tol_d, tol_a
        self.phase = 'turn'
        self.done = False

    def step(self, pose):
        if self.done or pose is None:
            return 0.0, 0.0, self.done
        hx, hy, hyaw = self.home
        dx, dy = hx - pose[0], hy - pose[1]
        d = math.hypot(dx, dy)
        clamp = lambda v, m: max(-m, min(m, v))
        if self.phase == 'turn':
            if d < self.tol_d:
                self.phase = 'final'
            else:
                e = wrap(math.atan2(dy, dx) - pose[2])
                if abs(e) < self.tol_a:
                    self.phase = 'drive'
                else:
                    return 0.0, clamp(1.5 * e, self.w_max) if abs(e) > 0.3 else math.copysign(0.45, e), False
        if self.phase == 'drive':
            if d < self.tol_d:
                self.phase = 'final'
            else:
                e = wrap(math.atan2(dy, dx) - pose[2])
                if abs(e) > 0.6:               # lệch quá nhiều (bị đẩy) -> quay lại
                    self.phase = 'turn'
                    return 0.0, 0.0, False
                v = min(self.v_max, max(0.05, 0.8 * d)) * (1.0 - min(abs(e), 0.5))
                return v, clamp(2.0 * e, self.w_max), False
        e = wrap(hyaw - pose[2])
        if abs(e) < self.tol_a:
            self.done = True
            return 0.0, 0.0, True
        return 0.0, clamp(1.5 * e, self.w_max) if abs(e) > 0.3 else math.copysign(0.45, e), False
