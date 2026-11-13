#!/usr/bin/env python3
"""Kiểm thử trên Gazebo (không cần webcam): lần lượt phát từng biểu cảm lên /emotion, ghi lại /odom, đo kết quả
và vẽ quỹ đạo ra figures/trajectories.png. Cần sim + emotion_motion_node đang chạy (scripts/test_run.sh lo việc đó).

    python3 tests/run_trajectories.py [emotion ...]       mặc định: happy sad neutral angry surprise fear
"""
import json
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXPECT = {'happy': 'cong trái', 'sad': 'cong phải', 'neutral': 'đứng yên', 'angry': 'lùi 0.2 m',
          'surprise': 'xoay 360°', 'fear': 'tiến 0.2 m'}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Runner(Node):
    def __init__(self):
        super().__init__('trajectory_test')
        self.pub = self.create_publisher(String, '/emotion', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(String, '/emotion_motion/status', self.on_status, 10)
        self.pose, self.status, self.rec = None, None, None
        self.unwrapped = 0.0

    def on_odom(self, m):
        p = m.pose.pose
        yaw = yaw_of(p.orientation)
        if self.pose is not None:
            d = math.atan2(math.sin(yaw - self.pose[2]), math.cos(yaw - self.pose[2]))
            self.unwrapped += d
        self.pose = (p.position.x, p.position.y, yaw)
        if self.rec is not None:
            self.rec.append((p.position.x, p.position.y, self.unwrapped))

    def on_status(self, m):
        self.status = json.loads(m.data)

    def spin_for(self, sec, label=None):
        end = time.time() + sec
        while time.time() < end:
            if label:
                self.pub.publish(String(data=label))
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait(self, cond, timeout):
        end = time.time() + timeout
        while time.time() < end and not cond():
            self.pub.publish(String(data='neutral'))
            rclpy.spin_once(self, timeout_sec=0.1)
        return cond()

    def run_one(self, e):
        """Giữ biểu cảm e tới khi lượt đầu tiên kết thúc (repeat_delay 1.5 s nên không bị lặp), rồi về neutral."""
        self.spin_for(1.0, 'neutral')
        c0 = self.status['count']
        self.rec = []
        t0 = time.time()
        while time.time() - t0 < 15:
            self.spin_for(0.1, e)
            st = self.status
            if e == 'neutral' and time.time() - t0 > 3:
                break
            if st['count'] > c0 and not st['running'] and st['emotion'] == e:
                break
        dur = time.time() - t0
        self.spin_for(1.0, 'neutral')                  # để robot dừng hẳn, odom ổn định
        tr, self.rec = self.rec, None
        return tr, dur


def main():
    emos = sys.argv[1:] or ['happy', 'sad', 'neutral', 'angry', 'surprise', 'fear']
    rclpy.init()
    n = Runner()
    print('Đợi /odom và emotion_motion_node sẵn sàng (undock + tiến ra giữa sân) ...', flush=True)
    if not n.wait(lambda: n.pose is not None and n.status and n.status['ready'], 180):
        sys.exit('Hết giờ chờ: emotion_motion_node chưa sẵn sàng')
    results, trajs = [], {}
    for e in emos:
        tr, dur = n.run_one(e)
        x0, y0, a0 = tr[0]
        x1, y1, a1 = tr[-1]
        c, s = math.cos(-a0), math.sin(-a0)            # đổi sang hệ toạ độ robot lúc bắt đầu
        local = [((x - x0) * c - (y - y0) * s, (x - x0) * s + (y - y0) * c) for x, y, _ in tr]
        dx, dy = local[-1]
        dyaw = math.degrees(a1 - a0)
        path = sum(math.dist(local[i], local[i + 1]) for i in range(len(local) - 1))
        results.append((e, dx, dy, dyaw, path, dur))
        trajs[e] = (local, [a - a0 for *_, a in tr])
        print(f'{e:9s} ({EXPECT[e]:10s}): tiến {dx:+.2f} m, ngang {dy:+.2f} m, xoay {dyaw:+7.1f}°, '
              f'quãng đường {path:.2f} m, {dur:.1f} s', flush=True)
    n.destroy_node()
    rclpy.try_shutdown()

    ok = {
        'happy': lambda dx, dy, a, p: dy > 0.15 and a > 45,
        'sad': lambda dx, dy, a, p: dy < -0.15 and a < -45,
        'neutral': lambda dx, dy, a, p: p < 0.02 and abs(a) < 2,
        'angry': lambda dx, dy, a, p: -0.26 < dx < -0.14 and abs(a) < 10,
        'surprise': lambda dx, dy, a, p: 340 < abs(a) < 380 and p < 0.1,
        'fear': lambda dx, dy, a, p: 0.14 < dx < 0.26 and abs(a) < 10,
    }
    good = [ok[e](dx, dy, a, p) for e, dx, dy, a, p, _ in results]
    print('KẾT QUẢ:', f'{sum(good)}/{len(good)} đạt', ' '.join(f'{r[0]}={"OK" if g else "SAI"}'
                                                            for r, g in zip(results, good)))
    with open(os.path.join(ROOT, 'logs', 'trajectories.json'), 'w') as f:
        json.dump({'results': results, 'trajs': trajs}, f)
    plot(results, trajs)


def plot(results, trajs):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('(không có matplotlib – bỏ qua hình)')
        return
    fig, axes = plt.subplots(1, len(results), figsize=(3.2 * len(results), 3.6))
    axes = axes if len(results) > 1 else [axes]
    for ax, (e, dx, dy, a, p, _) in zip(axes, results):
        pts, yaws = trajs[e]
        xs, ys = [q[0] for q in pts], [q[1] for q in pts]
        if e == 'surprise':                            # xoay tại chỗ: vẽ góc yaw theo một vòng tròn bán kính 0.2
            r = [0.05 + 0.15 * i / max(len(yaws) - 1, 1) for i in range(len(yaws))]
            ax.plot([ri * math.cos(t) for ri, t in zip(r, yaws)], [ri * math.sin(t) for ri, t in zip(r, yaws)],
                    color='tab:purple', lw=2)
            ax.text(0, -0.32, f'yaw {a:+.0f}° (xoắn ốc = góc)', ha='center', fontsize=8)
        else:
            ax.plot(xs, ys, lw=2.5, color='tab:blue')
            ax.plot(xs[-1], ys[-1], 'o', color='tab:red')
        ax.annotate('', xy=(0.15, 0), xytext=(0, 0), arrowprops=dict(arrowstyle='->', color='green', lw=2))
        ax.plot(0, 0, 's', color='green')
        ax.set_title(f'{e}\n{EXPECT[e]}', fontsize=10)
        ax.set_xlim(-0.45, 0.6)
        ax.set_ylim(-0.6, 0.6)
        ax.set_aspect('equal')
        ax.grid(alpha=0.3)
        ax.set_xlabel('x robot (m)')
    axes[0].set_ylabel('y robot (m)')
    fig.suptitle('Quỹ đạo đo bằng /odom trong Gazebo (hệ toạ độ robot lúc bắt đầu, mũi tên xanh = hướng đầu)',
                 fontsize=10)
    fig.tight_layout()
    out = os.path.join(ROOT, 'figures', 'trajectories.png')
    fig.savefig(out, dpi=130)
    print('Hình:', out)


if __name__ == '__main__':
    main()
