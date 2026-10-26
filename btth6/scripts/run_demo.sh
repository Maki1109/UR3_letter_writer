#!/usr/bin/env bash
# Chạy toàn bộ demo bài 06 trong một cửa sổ terminal nhiều tab:
#   Sim -> (robot có trong Gazebo) -> Behavior (tự undock + tiến ra khỏi dock) -> Gesture (cửa sổ webcam)
D="$(cd "$(dirname "$0")" && pwd)"
"$D/stop_all.sh" >/dev/null 2>&1
mkdir -p "$D/../logs"                     # log của tab Behavior: btth6/logs/behavior.log
wait_robot="source $D/env.sh; echo 'Đợi robot xuất hiện trong Gazebo...'; \
  until timeout 5 ros2 topic echo --once /dock_status >/dev/null 2>&1; do sleep 2; done"
tab() { gnome-terminal --tab --title="$1" -- bash -c "$2; exec bash"; sleep 1; }
gnome-terminal --window --title="Sim" -- bash -c "$D/01_sim.sh; exec bash"
sleep 1
tab Behavior "$wait_robot && $D/03_behavior.sh 2>&1 | tee $D/../logs/behavior.log"
tab Gesture "$wait_robot && $D/02_gesture.sh"
echo "Đã mở các tab. Đợi 30–60 s cho Gazebo; robot tự undock rồi tiến ra giữa sân, sau đó điều khiển bằng cử chỉ."
