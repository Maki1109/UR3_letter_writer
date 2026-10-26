#!/usr/bin/env bash
# Cài phụ thuộc cho bài 06 (chạy một lần):
#   - mediapipe 0.10.21 + numpy<2 + protobuf 4 vào btth6/.pydeps (không đụng tới ~/.local, không kéo theo
#     opencv-contrib/jax/matplotlib: node chỉ dùng MediaPipe Tasks, OpenCV dùng bản hệ thống python3-opencv)
#   - tải model gesture_recognizer.task, sinh world open_arena
set -e
D="$(cd "$(dirname "$0")/.." && pwd)"
dpkg -s python3-opencv >/dev/null 2>&1 || { sudo apt update && sudo apt install -y python3-opencv; }
/usr/bin/python3 -m pip install --target "$D/.pydeps" --no-deps "mediapipe==0.10.21" "numpy<2" \
  absl-py attrs flatbuffers "protobuf>=4.25.3,<5"
mkdir -p "$D/models"
[ -f "$D/models/gesture_recognizer.task" ] || curl -fSL -o "$D/models/gesture_recognizer.task" \
  https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/latest/gesture_recognizer.task
/usr/bin/python3 "$D/scripts/make_world.py"
source "$D/scripts/env.sh"
$PY -c "import cv2, mediapipe, rclpy; print('OpenCV', cv2.__version__, '| MediaPipe', mediapipe.__version__)"
echo "Xong."
