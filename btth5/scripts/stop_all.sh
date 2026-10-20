#!/usr/bin/env bash
# Tắt sạch bài 05: Gazebo + mọi node ROS (../scripts/stop_all.sh) + các node Python của btth5
# (chạy bằng /usr/bin/python3 nên script của bài 04 không bắt được).
D="$(cd "$(dirname "$0")" && pwd)"
pkill -INT -f "btth5/nodes/[y]olo_detector.py|btth5/nodes/[p]atrol_navigator.py"; sleep 1
pkill -9 -f "btth5/nodes/[y]olo_detector.py|btth5/nodes/[p]atrol_navigator.py"
"$D/../../scripts/stop_all.sh"
