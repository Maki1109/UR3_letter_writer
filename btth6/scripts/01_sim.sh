#!/usr/bin/env bash
# Phần 1 – TurtleBot 4 trong Gazebo, world open_arena (sân trống 7 x 7 m). Robot spawn trên dock tại (0, 0).
source "$(dirname "$0")/env.sh"
$PY "$B6_DIR/scripts/make_world.py" >/dev/null || exit 1
# Camera Gazebo: nhìn xiên từ trên xuống vùng hoạt động (01_sim.sh của bài 04 đặt camera cho world room)
(
  for _ in $(seq 1 90); do
    sleep 3
    ign service -l 2>/dev/null | grep -q "^/gui/move_to/pose" && break
  done
  sleep 10
  for _ in 1 2 3; do "$B6_DIR/scripts/view.sh" >/dev/null 2>&1 && break; sleep 4; done
) &
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py \
  world:="$WORLD" model:="$MODEL" rviz:=false "$@"
