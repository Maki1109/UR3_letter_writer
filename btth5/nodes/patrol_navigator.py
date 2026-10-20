#!/usr/bin/env python3
"""Phần 3 – Điều hướng tự động qua các goal bằng Nav2 Simple Commander.

Đọc config/goals.yaml, đặt initial pose cho AMCL, đợi Nav2 active, rời dock nếu robot đang trên dock,
rồi gửi lần lượt Goal 1 -> Goal 2 -> Goal 3 qua action /navigate_to_pose (BasicNavigator.goToPose).
Goal có khóa `via` (danh sách [x, y, yaw]) được gửi bằng goThroughPoses (/navigate_through_poses): robot đi qua
các điểm trung gian đó rồi mới tới goal – dùng để dẫn robot qua chính giữa khe hẹp.
Robot tự lập kế hoạch đường đi và tránh vật cản; không cần chuột hay teleop.

    python3 nodes/patrol_navigator.py [goals.yaml] --ros-args -p use_sim_time:=true
"""
import math
import os
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import TaskResult
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator  # BasicNavigator + dock/undock

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_GOALS = os.path.join(HERE, '..', 'config', 'goals.yaml')
GOAL_TIMEOUT = 180.0     # giây (thời gian mô phỏng) cho mỗi goal


def make_pose(nav, x, y, yaw_deg):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    yaw = math.radians(yaw_deg)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def main():
    rclpy.init(args=sys.argv)
    args = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    with open(args[0] if args else DEFAULT_GOALS) as f:
        cfg = yaml.safe_load(f)

    nav = TurtleBot4Navigator()
    log = nav.get_logger()

    ip = cfg['initial_pose']
    log.info(f"Đặt initial pose ({ip['x']}, {ip['y']}, {ip['yaw']}°)")
    nav.setInitialPose(make_pose(nav, ip['x'], ip['y'], ip['yaw']))
    nav.waitUntilNav2Active()       # đợi AMCL + bt_navigator active

    if nav.getDockedStatus():
        nav.undock()

    results = []
    for i, g in enumerate(cfg['goals'], 1):
        log.info(f"==> [{i}/{len(cfg['goals'])}] {g['name']}: x={g['x']:.2f} y={g['y']:.2f} yaw={g['yaw']:.0f}°")
        goal = make_pose(nav, g['x'], g['y'], g['yaw'])
        if g.get('via'):
            log.info(f"    qua điểm trung gian: {g['via']}")
            nav.goThroughPoses([make_pose(nav, *v) for v in g['via']] + [goal])
        else:
            nav.goToPose(goal)
        t0, last = nav.get_clock().now(), 0.0
        while not nav.isTaskComplete():
            time.sleep(0.1)     # không quay vòng bận, nhường CPU cho Gazebo/Nav2
            fb = nav.getFeedback()
            if fb is None:
                continue
            elapsed = (nav.get_clock().now() - t0).nanoseconds * 1e-9
            if elapsed - last >= 3.0:
                last = elapsed
                log.info(f'    còn {fb.distance_remaining:.2f} m, t={elapsed:.0f}s, '
                         f'recoveries={fb.number_of_recoveries}')
            if elapsed > GOAL_TIMEOUT:
                log.warn('    quá thời gian, hủy goal')
                nav.cancelTask()
        res = nav.getResult()
        elapsed = (nav.get_clock().now() - t0).nanoseconds * 1e-9
        ok = res == TaskResult.SUCCEEDED
        results.append((g['name'], ok, elapsed))
        # rclpy không cho cùng một dòng log đổi mức (info/error) giữa các lần gọi -> tách 2 nhánh
        if ok:
            log.info(f"    {g['name']}: ĐẾN NƠI sau {elapsed:.0f}s")
            time.sleep(cfg.get('dwell', 0.0))   # đứng lại cho YOLO quan sát vật thể tại goal
        else:
            log.error(f"    {g['name']}: {res.name} sau {elapsed:.0f}s")

    log.info('===== Kết quả =====')
    for name, ok, t in results:
        log.info(f"  {name:<28} {'OK' if ok else 'THẤT BẠI'}  {t:.0f}s")
    nav.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if all(ok for _, ok, _ in results) else 1)


if __name__ == '__main__':
    main()
