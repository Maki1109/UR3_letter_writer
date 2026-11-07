#!/usr/bin/env bash
# Gửi một câu lệnh dạng văn bản lên /voice_command (bỏ qua bước nhận dạng giọng nói).
#   ./say.sh "Di chuyển từ khu vực A đến khu vực B"
source "$(dirname "$0")/env.sh"
[ -n "$1" ] || { echo "Dùng: $0 \"câu lệnh\""; exit 2; }
ros2 topic pub --once -w 1 /voice_command std_msgs/msg/String "{data: \"$*\"}" >/dev/null && echo "Đã gửi: $*"
