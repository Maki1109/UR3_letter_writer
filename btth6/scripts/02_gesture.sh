#!/usr/bin/env bash
# Phần 2 – gesture_recognition_node (webcam -> MediaPipe -> /detected_gesture)
#   ./scripts/02_gesture.sh -p camera:=1 -p min_score:=0.7
source "$(dirname "$0")/env.sh"
ARGS=(); [ $# -gt 0 ] && ARGS=(--ros-args "$@")
exec $PY "$B6_DIR/nodes/gesture_recognition_node.py" "${ARGS[@]}"
