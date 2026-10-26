#!/usr/bin/env bash
# Phần 3 – robot_behavior_node (/detected_gesture -> /cmd_vel). Tự undock nếu robot đang trên dock.
#   ./scripts/03_behavior.sh -p circle_radius:=0.6 -p fb_distance:=0.5
source "$(dirname "$0")/env.sh"
ARGS=(); [ $# -gt 0 ] && ARGS=(--ros-args "$@")
exec $PY "$B6_DIR/nodes/robot_behavior_node.py" "${ARGS[@]}"
