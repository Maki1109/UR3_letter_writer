#!/usr/bin/env bash
# Phần 3 – Đặt initial pose cho AMCL. Robot spawn tại dock = gốc frame map khi mapping,
# nên mặc định là (0,0,0). Có thể dùng "2D Pose Estimate" trên RViz thay cho script này.
# Dùng: set_initial_pose.sh [x] [y] [yaw_rad]
source "$(dirname "$0")/env.sh"
X="${1:-0.0}"; Y="${2:-0.0}"; YAW="${3:-0.0}"
read QZ QW < <(python3 -c "import math;print(math.sin($YAW/2), math.cos($YAW/2))")
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "
header: {frame_id: map}
pose:
  pose:
    position: {x: $X, y: $Y, z: 0.0}
    orientation: {z: $QZ, w: $QW}
  covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]
"
