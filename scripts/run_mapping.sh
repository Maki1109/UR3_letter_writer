#!/usr/bin/env bash
# Cảnh 1 (mapping): mở 1 cửa sổ 3 tab: SLAM Toolbox, RViz, teleop giữ-phím.
# Cần Gazebo đang chạy sẵn (./scripts/01_sim.sh). Không tự di chuyển robot.
# Lưu ý: sau "--" gnome-terminal coi mọi thứ còn lại là câu lệnh, nên mỗi tab là một lần gọi riêng.
D="$(cd "$(dirname "$0")" && pwd)"
gnome-terminal --window --title="SLAM" -- bash -c "$D/02_slam.sh; exec bash"
sleep 1
gnome-terminal --tab --title="RViz" -- bash -c "sleep 4; $D/rviz.sh; exec bash"
sleep 1
gnome-terminal --tab --title="Teleop" -- bash -c "sleep 2; $D/teleop.sh; exec bash"
echo "Đã mở SLAM + RViz + teleop. Xong mapping: ./scripts/save_map.sh"
