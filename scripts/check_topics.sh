#!/usr/bin/env bash
# Phần 1 – Kiểm tra /scan, /odom, /cmd_vel
# Thời lượng tính theo đồng hồ thật; máy render bằng CPU có real-time factor ~0.3 nên đặt dư.
source "$(dirname "$0")/env.sh"

yaw() {  # yaw (độ) từ /odom
  ros2 topic echo --once /odom --field pose.pose.orientation | python3 -c \
    "import sys,math,re;d=dict(re.findall(r'(\w): (\S+)',sys.stdin.read()));print(round(math.degrees(2*math.atan2(float(d['z']),float(d['w']))),1))"
}

for t in /scan /odom /cmd_vel; do
  echo "== $t =="
  ros2 topic info "$t"
done

echo "== Tần số /scan (10 s) =="
timeout 10 ros2 topic hz /scan | tail -2
echo "== Tần số /odom (10 s) =="
timeout 10 ros2 topic hz /odom | tail -2

echo "== Một bản tin /scan =="
ros2 topic echo --once /scan --field ranges | python3 -c \
  "import sys,re;v=[float(x) for x in re.findall(r'\d+\.\d+',sys.stdin.read())];print(f'{len(v)} tia hữu hạn, min {min(v):.2f} m, max {max(v):.2f} m')"
echo "== Pose hiện tại từ /odom =="
ros2 topic echo --once /odom --field pose.pose.position

echo "== Undock robot (robot spawn trên dock; bị từ chối nếu đã undock) =="
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}" | tail -2

echo "== Gửi lệnh quay tại chỗ 0.5 rad/s trong 8 s qua /cmd_vel =="
Y0=$(yaw)
timeout 8 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}" > /dev/null
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" > /dev/null
sleep 1
Y1=$(yaw)
echo "Yaw trước: ${Y0}°  ->  sau: ${Y1}°  (thay đổi = robot nhận lệnh vận tốc)"
