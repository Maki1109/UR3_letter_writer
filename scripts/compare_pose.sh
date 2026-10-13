#!/usr/bin/env bash
# Phần 3 – So sánh pose AMCL (map) với pose thật trong Gazebo.
# Khi mapping, frame map trùng vị trí spawn, nên pose Gazebo (world) - pose spawn ≈ pose AMCL.
source "$(dirname "$0")/env.sh"
echo "== AMCL /amcl_pose =="
ros2 topic echo --once /amcl_pose --field pose.pose
echo "== map -> base_link (tf) =="
timeout 3 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null | head -6
echo "== Ground truth Gazebo (world, model turtlebot4) =="
ign model -m turtlebot4 -p 2>/dev/null | sed -n '/Pose/,/^$/p' || echo "(không đọc được qua ign CLI – xem trực tiếp trong Gazebo)"
