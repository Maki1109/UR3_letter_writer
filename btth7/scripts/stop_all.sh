#!/usr/bin/env bash
# Tắt sạch bài 07: node giọng nói + điều hướng, rồi Gazebo, Nav2 và mọi node ROS (../btth5/scripts/stop_all.sh).
D="$(cd "$(dirname "$0")" && pwd)"
P="btth7/nodes/[s]peech_recognition_node.py|btth7/nodes/[v]oice_nav_node.py|btth7/nodes/[m]ap_view_node.py|[a]record -q -D"
pkill -INT -f "$P"; sleep 1; pkill -9 -f "$P"
"$D/../../btth5/scripts/stop_all.sh"
