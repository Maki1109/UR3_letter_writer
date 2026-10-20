#!/usr/bin/env bash
# Phần 1 – TurtleBot 4 trong Gazebo, world room_objects (= world room đã quét map ở bài 04 + vật thể cho YOLO).
# Robot spawn trên dock tại gốc bản đồ (0, 0, 0).
source "$(dirname "$0")/env.sh"
$PY "$B5_DIR/scripts/make_world.py" || exit 1
exec "$WS_DIR/scripts/01_sim.sh" "$@"
