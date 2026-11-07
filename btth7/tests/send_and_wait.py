#!/usr/bin/env python3
"""Gửi lần lượt các câu lệnh lên /voice_command, đợi /voice_nav/status báo kết thúc, in bảng kết quả.

    python3 tests/send_and_wait.py "Đi đến khu vực A" "Di chuyển từ khu vực A đến khu vực B" ...
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

DONE = ('hoàn thành', 'Không có khu vực', 'dừng lộ trình', 'Không tìm thấy', 'chưa sẵn sàng', 'Đã dừng',
        'từ chối')
TIMEOUT = 400.0     # giây (đồng hồ thật) cho mỗi câu lệnh


def main():
    rclpy.init()
    node = Node('voice_test')
    pub = node.create_publisher(String, '/voice_command', 10)
    log = []
    node.create_subscription(String, '/voice_nav/status', lambda m: log.append(m.data), 10)
    # đợi đúng node voice_nav (map_view cũng subscribe /voice_command), thêm 1 s cho kết nối DDS ổn định
    while not any(i.node_name == 'voice_nav' for i in node.get_subscriptions_info_by_topic('/voice_command')):
        rclpy.spin_once(node, timeout_sec=0.5)
    time.sleep(1.0)
    results = []
    for cmd in sys.argv[1:]:
        print(f'\n=== "{cmd}"', flush=True)
        log.clear()
        pub.publish(String(data=cmd))
        t0, seen, last = time.time(), 0, ''
        while time.time() - t0 < TIMEOUT:
            rclpy.spin_once(node, timeout_sec=0.2)
            for line in log[seen:]:
                print(f'  [{time.time() - t0:5.0f}s] {line}', flush=True)
                last = line
            seen = len(log)
            if any(d in last for d in DONE):
                break
        results.append((cmd, last or 'HẾT GIỜ', time.time() - t0))
    print('\n===== Kết quả =====')
    for cmd, last, t in results:
        print(f'  {cmd:<45} {t:5.0f}s  {last}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
