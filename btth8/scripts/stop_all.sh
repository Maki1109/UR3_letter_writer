#!/usr/bin/env bash
# Tắt sạch bài 08: các node Python của btth8 (chạy bằng /usr/bin/python3) + Gazebo và mọi node ROS.
D="$(cd "$(dirname "$0")" && pwd)"
pkill -INT -f "btth8/nodes/[e]motion_|btth8/tests/[r]un_trajectories"; sleep 1
pkill -9 -f "btth8/nodes/[e]motion_|btth8/tests/[r]un_trajectories"
"$D/../../scripts/stop_all.sh"
