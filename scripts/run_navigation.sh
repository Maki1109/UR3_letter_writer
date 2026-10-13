#!/usr/bin/env bash
# Cảnh 3-5: mở 1 cửa sổ 3 tab: localization (load maps/map.yaml), Nav2, RViz.
# Cần Gazebo đang chạy sẵn (./scripts/01_sim.sh). Sau khi RViz lên: bấm 2D Pose Estimate,
# đợi particle hội tụ rồi mới đặt Nav2 Goal.
D="$(cd "$(dirname "$0")" && pwd)"
MAP="${1:-$D/../maps/map.yaml}"
[ -f "$MAP" ] || { echo "Không thấy $MAP – chạy save_map.sh trước"; exit 1; }
gnome-terminal --window --title="Localization" -- bash -c "$D/03_localization.sh '$MAP'; exec bash"
sleep 1
gnome-terminal --tab --title="Nav2" -- bash -c "sleep 10; $D/04_nav2.sh; exec bash"
sleep 1
gnome-terminal --tab --title="RViz" -- bash -c "sleep 4; $D/rviz.sh; exec bash"
echo "Đã mở localization + Nav2 + RViz. Đặt 2D Pose Estimate rồi Nav2 Goal."
