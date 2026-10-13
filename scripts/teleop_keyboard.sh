#!/usr/bin/env bash
# Teleop kiểu cũ trong terminal (i/j/l/, ; k = dừng; robot chạy tiếp cho tới khi bấm k)
source "$(dirname "$0")/env.sh"
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p speed:=0.2 -p turn:=0.4
