#!/usr/bin/env bash
# Chạy toàn bộ demo bài 08 trong một cửa sổ terminal nhiều tab: Sim -> Motion -> Detector.
D="$(cd "$(dirname "$0")" && pwd)"; B5="$D"
"$D/stop_all.sh" >/dev/null 2>&1
tab() { gnome-terminal --tab --title="$1" -- bash -c "$2; exec bash"; sleep 1; }
gnome-terminal --window --title="Sim" -- bash -c "$D/01_sim.sh; exec bash"
sleep 1
tab Motion "$B5/wait_for.sh robot && $D/02_motion.sh"
tab Detector "$B5/wait_for.sh robot && $D/03_detector.sh $*"
echo "Đã mở các tab. Đợi Gazebo lên (30–60 s); robot tự undock rồi tiến ra giữa sân, sau đó làm biểu cảm trước webcam."
