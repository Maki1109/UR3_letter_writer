#!/usr/bin/env bash
# Giả lập biểu cảm (bỏ qua webcam): phát nhãn lên /emotion trong N giây (mặc định 3 s, 10 Hz).
#   ./send_emotion.sh surprise        ./send_emotion.sh happy 8
source "$(dirname "$0")/env.sh"
case "$1" in happy|sad|neutral|angry|surprise|fear|none) ;; *)
  echo "Dùng: $0 happy|sad|neutral|angry|surprise|fear [giây]"; exit 2 ;; esac
timeout "${2:-3}" ros2 topic pub -r 10 /emotion std_msgs/msg/String "{data: $1}" >/dev/null
echo "Đã gửi '$1' trong ${2:-3} s"
