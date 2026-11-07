#!/usr/bin/env bash
# RViz2: bản đồ + các khu vực đã phân vùng (/zones/markers), robot, LaserScan, costmap, đường đi.
source "$(dirname "$0")/env.sh"
ros2 run rviz2 rviz2 -d "$B7_DIR/config/voice_nav.rviz" --ros-args -p use_sim_time:=true
