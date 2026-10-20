#!/usr/bin/env bash
# Phần 1 (tiếp) – AMCL + map_server nạp bản đồ của bài 04 (../maps/map.yaml).
source "$(dirname "$0")/env.sh"
MAP_FILE="${1:-$MAP_DIR/$MAP_NAME.yaml}"
[ -f "$MAP_FILE" ] || { echo "Không thấy $MAP_FILE"; exit 1; }
ros2 launch turtlebot4_navigation localization.launch.py map:="$MAP_FILE" use_sim_time:=true
