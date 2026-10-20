#!/usr/bin/env bash
# Chạy toàn bộ demo không cần terminal tab (log vào btth5/logs/), dùng để kiểm thử. Ctrl+C hoặc stop_all.sh để dừng.
D="$(cd "$(dirname "$0")" && pwd)"; L="$D/../logs"; mkdir -p "$L"
"$D/stop_all.sh" >/dev/null 2>&1
source "$D/env.sh"
"$D/01_sim.sh" > "$L/sim.log" 2>&1 &
until timeout 5 ros2 topic echo --once /dock_status >/dev/null 2>&1; do sleep 2; done
"$D/02_localization.sh" > "$L/localization.log" 2>&1 &
"$D/04_yolo.sh" ${YOLO_ARGS} > "$L/yolo.log" 2>&1 &
sleep 10; "$D/03_nav2.sh" > "$L/nav2.log" 2>&1 &
"$D/rviz.sh" > "$L/rviz.log" 2>&1 &
sleep 15; "$D/05_patrol.sh" 2>&1 | tee "$L/patrol.log"
