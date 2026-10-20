#!/usr/bin/env bash
# Chạy toàn bộ demo bài 05 trong một cửa sổ terminal nhiều tab, theo đúng thứ tự phụ thuộc:
#   Sim -> Localization -> Init pose (tự đặt initial pose) -> Nav2 -> RViz (mở khi Nav2 đã active,
#   nên có sẵn bản đồ + robot, không báo lỗi) -> YOLO -> Patrol.
# Đặt PATROL=0 để không tự chạy node điều hướng (tự gõ ./scripts/05_patrol.sh khi đã sẵn sàng quay video).
D="$(cd "$(dirname "$0")" && pwd)"
"$D/stop_all.sh" >/dev/null 2>&1
tab() { gnome-terminal --tab --title="$1" -- bash -c "$2; exec bash"; sleep 1; }
gnome-terminal --window --title="Sim" -- bash -c "$D/01_sim.sh; exec bash"
sleep 1
tab Localization "$D/wait_for.sh robot && $D/02_localization.sh"
tab "Init pose" "$D/auto_initial_pose.sh 0 0 0"
tab Nav2 "$D/wait_for.sh localization && $D/03_nav2.sh"
# RViz mở sau cùng (khi Gazebo đã ổn định và Nav2 active) để không bị treo "not responding" lúc khởi động
tab RViz "$D/wait_for.sh nav2 && $D/rviz.sh"
tab YOLO "$D/wait_for.sh rviz && sleep 5 && $D/04_yolo.sh"
[ "${PATROL:-1}" = 1 ] && tab Patrol "$D/wait_for.sh rviz && sleep 10 && $D/05_patrol.sh"
echo "Đã mở các tab. Đợi 1–3 phút: RViz mở ra khi robot đã định vị và Nav2 sẵn sàng."
