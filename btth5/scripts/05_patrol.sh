#!/usr/bin/env bash
# Phần 3 – Node điều hướng tự động: initial pose -> undock -> Goal 1 -> Goal 2 -> Goal 3 (config/goals.yaml).
source "$(dirname "$0")/env.sh"
exec $PY "$B5_DIR/nodes/patrol_navigator.py" "${1:-$B5_DIR/config/goals.yaml}" --ros-args -p use_sim_time:=true
