#!/usr/bin/env bash
# Phần 4 – Node điều hướng: /voice_command -> phân tích câu lệnh -> NavigateToPose (Nav2).
source "$(dirname "$0")/env.sh"
[ -f "$B7_DIR/config/zones.yaml" ] || $PY "$B7_DIR/tools/segment_map.py"
exec $PY "$B7_DIR/nodes/voice_nav_node.py" --ros-args -p use_sim_time:=true "$@"
