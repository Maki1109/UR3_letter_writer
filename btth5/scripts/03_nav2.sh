#!/usr/bin/env bash
# Nav2 (planner, controller, costmaps, bt_navigator) với config/nav2.yaml (giảm inflation cho khe hẹp).
# Chạy sau 02_localization.sh.
source "$(dirname "$0")/env.sh"
ros2 launch turtlebot4_navigation nav2.launch.py use_sim_time:=true \
  params_file:="$B5_DIR/config/nav2.yaml" "$@"
