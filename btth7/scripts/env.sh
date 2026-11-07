# Môi trường chung cho bài 07 (source, không chạy trực tiếp).
# Dùng lại env của bài 04 (overlay ogre2, gỡ biến snap, ROS_DOMAIN_ID...) và world room_objects của bài 05
# (world room đã quét map + người, ghế, sofa... không có trên bản đồ, làm vật cản cho Nav2 tránh).
B7_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export WORLD="${WORLD:-room_objects}"
source "$B7_DIR/../scripts/env.sh"
export B7_DIR B5_DIR="$B7_DIR/../btth5"

# ~/.local có numpy 2.x; whisper/numba/OpenCV hệ thống cần numpy 1.x. setup.sh cài numpy<2, whisper, vosk vào
# btth7/.pydeps và chỉ đặt nó lên đầu PYTHONPATH cho các node của bài này.
export PYTHONPATH="$B7_DIR/.pydeps:$PYTHONPATH"
export PYTHONWARNINGS="ignore"   # scipy hệ thống cảnh báo về numpy 1.26 – vô hại
PY=/usr/bin/python3
