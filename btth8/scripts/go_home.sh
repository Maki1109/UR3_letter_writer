#!/usr/bin/env bash
# Đưa robot về vị trí + hướng lúc bắt đầu (sau khi undock và tiến ra giữa sân). Cần 02_motion.sh đang chạy.
#   ./go_home.sh            về chỗ cũ          ./go_home.sh sethome    lấy vị trí hiện tại làm "chỗ cũ" mới
source "$(dirname "$0")/env.sh"
ros2 topic pub --once -w 1 /emotion_motion/command std_msgs/msg/String "{data: ${1:-home}}" >/dev/null
echo "Đã gửi lệnh '${1:-home}'"
