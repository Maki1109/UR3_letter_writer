#!/usr/bin/env bash
# Phần 1 – Khởi chạy TurtleBot 4 trong Gazebo (robot spawn tại dock, gốc map = (0,0))
source "$(dirname "$0")/env.sh"
D="$(cd "$(dirname "$0")" && pwd)"
# Sau khi GUI Gazebo sẵn sàng, tự đặt camera nhìn từ trên xuống toàn phòng
(
  for _ in $(seq 1 90); do
    sleep 3
    ign service -l 2>/dev/null | grep -q "^/gui/move_to/pose" && break
  done
  sleep 8
  for _ in 1 2 3; do "$D/top_view.sh" >/dev/null 2>&1 && break; sleep 4; done
) &
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py \
  world:="$WORLD" model:="$MODEL" rviz:=false "$@"
