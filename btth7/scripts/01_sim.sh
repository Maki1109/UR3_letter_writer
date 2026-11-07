#!/usr/bin/env bash
# Gazebo + TurtleBot 4, world room_objects (bài 05). Robot spawn trên dock tại gốc bản đồ (0, 0, 0).
source "$(dirname "$0")/env.sh"
exec "$B5_DIR/scripts/01_sim.sh" "$@"
