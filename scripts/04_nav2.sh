#!/usr/bin/env bash
# Phần 4 – Nav2 (planner, controller, costmaps, BT navigator) – chạy SAU khi localization đã active
source "$(dirname "$0")/env.sh"
ros2 launch turtlebot4_navigation nav2.launch.py use_sim_time:=true "$@"
