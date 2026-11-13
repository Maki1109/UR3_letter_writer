#!/usr/bin/env bash
# Phần 1 – Gazebo + TurtleBot 4, world open_arena (bài 06). Robot spawn trên dock tại (0, 0).
source "$(dirname "$0")/env.sh"
exec "$B6_DIR/scripts/01_sim.sh" "$@"
