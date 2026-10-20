# Môi trường chung cho bài 05 (source, không chạy trực tiếp).
# Dùng lại env của bài 04 (overlay ogre2, gỡ biến snap, ROS_DOMAIN_ID...) rồi thêm phần riêng cho bài 05.
B5_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export WORLD="${WORLD:-room_objects}"          # world room + người, ghế, sofa, trụ cứu hỏa
source "$B5_DIR/../scripts/env.sh"
export B5_DIR
[ -f "$WS_DIR/overlay/share/turtlebot4_ignition_bringup/worlds/room_objects.sdf" ] || \
  /usr/bin/python3 "$B5_DIR/scripts/make_world.py" >&2

# ~/.local đang có numpy 2.x, làm cv_bridge (biên dịch với numpy 1.x) lỗi "_ARRAY_API not found".
# setup.sh cài numpy<2 + ultralytics vào btth5/.pydeps; đặt lên đầu PYTHONPATH chỉ cho các node của bài này.
export PYTHONPATH="$B5_DIR/.pydeps:$PYTHONPATH"
PY=/usr/bin/python3
