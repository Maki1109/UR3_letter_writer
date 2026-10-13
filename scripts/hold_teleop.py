#!/usr/bin/env python3
"""Teleop trong terminal: giữ phím là chạy, nhả phím là tự dừng (không cần Enter).

Terminal không báo được lúc nhả phím, nên dùng tính năng auto-repeat của bàn phím:
giữ phím thì terminal liên tục gửi ký tự; ngừng nhận ký tự thì dừng robot.
  W / ↑  tiến        S / ↓  lùi
  A / ←  quay trái   D / →  quay phải      (giữ 2 phím cùng lúc có thể không được
  Z / X (hoặc - / +)  giảm / tăng tốc độ     tuỳ bàn phím; hãy thử xen kẽ W rồi A)
  Space / K  dừng ngay     Q hoặc Ctrl+C  thoát
LƯU Ý: phải TẮT bộ gõ tiếng Việt (ibus-bamboo) khi dùng chữ W A S D, nếu không giữ phím
sẽ sinh ra ư/â/đ... Phím mũi tên không bị ảnh hưởng bởi bộ gõ.
"""
import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist

# m/s, rad/s – tính theo thời gian mô phỏng; RTF khi chạy ~0.45 nên tốc độ thực tế
# nhìn thấy chỉ bằng ~1/2. Z/X để chỉnh. Khi mapping: tối đa ~0.35 m/s, và quay <= 0.4 rad/s
# (quay nhanh làm SLAM ghép scan sai -> tường nhân đôi / bản đồ ma).
LIN_START, ANG_START = 0.3, 0.4
RATE_HZ = 20
# Độ trễ trước khi bàn phím bắt đầu lặp (GNOME mặc định 500 ms). Lần nhấn đầu tiên phải
# giữ robot chạy ít nhất khoảng này, nếu không sẽ dừng trước khi auto-repeat kịp bắt đầu.
FIRST_PRESS_HOLD = float(os.environ.get('KEY_REPEAT_DELAY_MS', 500)) / 1000 + 0.12
REPEAT_HOLD = 0.12                  # khi đã lặp (30 ms/ký tự): im quá ngần này = đã nhả

KEYS = {                            # phím -> (hệ số linear, hệ số angular)
    'w': (1, 0), 's': (-1, 0), 'a': (0, 1), 'd': (0, -1),
    'i': (1, 0), ',': (-1, 0), 'j': (0, 1), 'l': (0, -1),
    '\x1b[A': (1, 0), '\x1b[B': (-1, 0), '\x1b[D': (0, 1), '\x1b[C': (0, -1),
}


def read_key(fd, timeout):
    """Trả về một phím (hoặc chuỗi escape của phím mũi tên), None nếu hết thời gian chờ."""
    if not select.select([fd], [], [], timeout)[0]:
        return None
    data = os.read(fd, 32).decode(errors='ignore')
    if not data:
        return None
    # Nhiều ký tự lặp có thể dồn lại trong một lần đọc: lấy phím cuối cùng
    if '\x1b' in data:
        data = '\x1b' + data.rsplit('\x1b', 1)[1]
    return data[-3:] if data.startswith('\x1b') else data[-1]


def main():
    rclpy.init()
    node = rclpy.create_node('hold_teleop')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)               # nhận phím ngay, không cần Enter, không echo

    lin_speed, ang_speed = LIN_START, ANG_START
    active = (0, 0)                 # hướng đang chạy
    last_key, last_time, repeating = None, 0.0, False
    deadline = 0.0
    ime_warn = False

    print(__doc__)
    try:
        while rclpy.ok():
            key = read_key(fd, 1.0 / RATE_HZ)
            now = time.monotonic()
            if key is not None:
                key = key.lower() if len(key) == 1 else key
                if ord(key[0]) > 127:       # ư â đ ... => bộ gõ tiếng Việt đang bật
                    ime_warn = True
                elif key in KEYS:
                    # Cùng phím đến dồn dập => đang giữ (auto-repeat)
                    repeating = (key == last_key and now - last_time < 0.2)
                    active = KEYS[key]
                    deadline = now + (REPEAT_HOLD if repeating else FIRST_PRESS_HOLD)
                    last_key, last_time = key, now
                elif key in (' ', 'k'):
                    active, deadline, last_key = (0, 0), 0.0, None
                elif key in ('x', '+', '='):
                    lin_speed, ang_speed = min(lin_speed * 1.2, 0.46), min(ang_speed * 1.2, 1.9)
                elif key in ('z', '-', '_'):
                    lin_speed, ang_speed = max(lin_speed / 1.2, 0.05), max(ang_speed / 1.2, 0.1)
                elif key in ('q', '\x03'):
                    break
            if now > deadline:
                active, last_key = (0, 0), None
            if not rclpy.ok():              # bị kill từ ngoài (SIGTERM)
                break

            msg = Twist()
            msg.linear.x = lin_speed * active[0]
            msg.angular.z = ang_speed * active[1]
            pub.publish(msg)
            warn = "  !! BỘ GÕ TIẾNG VIỆT ĐANG BẬT: tắt nó (Super+Space) hoặc dùng phím mũi tên" if ime_warn else ""
            sys.stdout.write(f"\r\x1b[Klin {msg.linear.x:+.2f} m/s  ang {msg.angular.z:+.2f} rad/s"
                             f"   (tối đa {lin_speed:.2f} / {ang_speed:.2f}){warn}")
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        if rclpy.ok():
            for _ in range(5):      # gửi nhiều lần để chắc chắn robot nhận lệnh dừng
                pub.publish(Twist())
                time.sleep(0.05)
            node.destroy_node()
            rclpy.shutdown()
        print("\nĐã dừng robot và thoát.")


if __name__ == '__main__':
    main()
