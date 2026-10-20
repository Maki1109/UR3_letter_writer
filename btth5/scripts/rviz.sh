#!/usr/bin/env bash
# RViz2: bản đồ, robot, LaserScan, particle AMCL, costmap, đường đi + ảnh YOLO (/yolo/image_annotated).
source "$(dirname "$0")/env.sh"
ros2 run rviz2 rviz2 -d "$B5_DIR/config/nav_yolo.rviz" --ros-args -p use_sim_time:=true
