#!/usr/bin/env bash
# Phần 2 – SLAM Toolbox (online async) với thời gian mô phỏng.
# Dùng config/slam.yaml (đã chỉnh cho phòng nhỏ: không loop closure, chỉ xử lý scan khi robot đã di chuyển).
source "$(dirname "$0")/env.sh"
ros2 launch turtlebot4_navigation slam.launch.py use_sim_time:=true \
  params:="$WS_DIR/config/slam.yaml" "$@"
