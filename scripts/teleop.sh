#!/usr/bin/env bash
# Điều khiển trong terminal: giữ phím (W A S D / mũi tên / i j l ,) là chạy, nhả là tự dừng.
# Z,X đổi tốc độ; Space dừng; Q thoát. Terminal này phải đang được focus.
# Bản cũ (chạy tiếp cho tới khi bấm k): ./scripts/teleop_keyboard.sh
source "$(dirname "$0")/env.sh"
/usr/bin/python3 "$(dirname "$0")/hold_teleop.py"
