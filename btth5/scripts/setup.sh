#!/usr/bin/env bash
# Cài phụ thuộc cho bài 05 (chạy một lần).
#   - gói ROS: Nav2 Simple Commander, cv_bridge (cần sudo)
#   - ultralytics + numpy<2 vào btth5/.pydeps (không đụng tới ~/.local)
#   - tải trọng số YOLO11n, sinh world room_objects
set -e
D="$(cd "$(dirname "$0")/.." && pwd)"
if ! dpkg -s ros-humble-nav2-simple-commander ros-humble-cv-bridge ros-humble-turtlebot4-navigation >/dev/null 2>&1; then
  sudo apt update
  sudo apt install -y ros-humble-turtlebot4-navigation ros-humble-nav2-simple-commander ros-humble-cv-bridge
fi
# torch, opencv, scipy, matplotlib... dùng bản có sẵn; chỉ thêm các gói còn thiếu
/usr/bin/python3 -m pip install --target "$D/.pydeps" --no-deps "numpy<2" ultralytics ultralytics-thop
/usr/bin/python3 -c "import torch" 2>/dev/null || /usr/bin/python3 -m pip install --user torch torchvision \
  --index-url https://download.pytorch.org/whl/cpu
mkdir -p "$D/models"
source "$D/scripts/env.sh"
$PY -c "from ultralytics import YOLO; YOLO('$D/models/yolo11n.pt')"
$PY "$D/scripts/make_world.py"
echo "Xong. Kiểm tra: source btth5/scripts/env.sh && python3 -c 'import cv_bridge, ultralytics'"
