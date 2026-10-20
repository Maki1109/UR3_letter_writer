#!/usr/bin/env bash
# Đặt initial pose cho AMCL (mặc định robot trên dock = (0, 0, 0)). Dùng: set_initial_pose.sh [x] [y] [yaw_rad]
source "$(dirname "$0")/env.sh"
exec "$WS_DIR/scripts/set_initial_pose.sh" "$@"
