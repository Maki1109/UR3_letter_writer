#!/usr/bin/env bash
# Phần 2 – Lưu bản đồ: maps/map.yaml + maps/map.pgm (+ pose graph để có thể tiếp tục mapping)
# Nếu đã có map cũ, nó được sao lưu thành map_YYYYmmdd_HHMMSS.* thay vì bị ghi đè mất.
source "$(dirname "$0")/env.sh"
mkdir -p "$MAP_DIR"

if [ -e "$MAP_DIR/$MAP_NAME.yaml" ]; then
  TS=$(date +%Y%m%d_%H%M%S)
  for ext in yaml pgm posegraph data; do
    [ -e "$MAP_DIR/$MAP_NAME.$ext" ] && mv "$MAP_DIR/$MAP_NAME.$ext" "$MAP_DIR/${MAP_NAME}_$TS.$ext"
  done
  echo "Đã sao lưu map cũ -> $MAP_DIR/${MAP_NAME}_$TS.*"
fi

ros2 run nav2_map_server map_saver_cli -f "$MAP_DIR/$MAP_NAME" \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0

# Tuỳ chọn: lưu pose graph của SLAM Toolbox (.posegraph/.data)
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$MAP_DIR/$MAP_NAME'}" || true

ls -l "$MAP_DIR"
