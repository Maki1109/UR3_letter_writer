#!/usr/bin/env bash
# Tắt sạch bài 06: các node Python của btth6 (chạy bằng /usr/bin/python3 nên script của bài 04 không bắt được)
# + Gazebo và mọi node ROS (../scripts/stop_all.sh).
D="$(cd "$(dirname "$0")" && pwd)"
pkill -INT -f "btth6/nodes/[g]esture_recognition_node.py|btth6/nodes/[r]obot_behavior_node.py"; sleep 1
pkill -9 -f "btth6/nodes/[g]esture_recognition_node.py|btth6/nodes/[r]obot_behavior_node.py"
"$D/../../scripts/stop_all.sh"
