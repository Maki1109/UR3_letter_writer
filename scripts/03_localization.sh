#!/usr/bin/env bash
# Phần 3 – AMCL + map_server với bản đồ đã lưu
source "$(dirname "$0")/env.sh"
MAP_FILE="${1:-$MAP_DIR/$MAP_NAME.yaml}"
[ -f "$MAP_FILE" ] || { echo "Không thấy $MAP_FILE – hãy chạy save_map.sh trước"; exit 1; }
ros2 launch turtlebot4_navigation localization.launch.py \
  map:="$MAP_FILE" use_sim_time:=true
