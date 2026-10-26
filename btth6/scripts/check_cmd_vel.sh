#!/usr/bin/env bash
# Phần 1 – Kiểm tra robot nhận lệnh vận tốc qua /cmd_vel: undock, quay tại chỗ 0.5 rad/s, so yaw trước/sau.
source "$(dirname "$0")/env.sh"
yaw() {  # yaw (độ) từ /odom
  ros2 topic echo --once /odom --field pose.pose.orientation | $PY -c \
    "import sys,math,re;d=dict(re.findall(r'(\w): (\S+)',sys.stdin.read()));print(round(math.degrees(2*math.atan2(float(d['z']),float(d['w']))),1))"
}
echo "== /cmd_vel =="; ros2 topic info /cmd_vel
echo "== Undock (bị từ chối nếu robot đã rời dock) =="
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}" | tail -2
echo "== Quay tại chỗ 0.5 rad/s trong 6 s qua /cmd_vel =="
Y0=$(yaw)
timeout 6 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}" > /dev/null
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" > /dev/null
sleep 1
echo "Yaw trước: ${Y0}°  ->  sau: $(yaw)°  (thay đổi = robot nhận lệnh vận tốc)"
