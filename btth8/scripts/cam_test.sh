#!/usr/bin/env bash
# Kiểm tra webcam + DeepFace, không cần ROS/Gazebo: cửa sổ camera với khung mặt và biểu cảm từng frame.
#   ./cam_test.sh [device]
source "$(dirname "$0")/env.sh"
exec $PY "$B8_DIR/tests/cam_test.py" "${1:-0}"
