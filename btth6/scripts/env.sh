# Môi trường chung cho bài 06 (source, không chạy trực tiếp).
# Dùng lại env của bài 04 (overlay ogre2, gỡ biến snap, ROS_DOMAIN_ID...) rồi thêm phần riêng cho bài 06.
B6_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export WORLD="${WORLD:-open_arena}"            # sân trống 7 x 7 m, không vật cản
source "$B6_DIR/../scripts/env.sh"
export B6_DIR
[ -f "$WS_DIR/overlay/share/turtlebot4_ignition_bringup/worlds/open_arena.sdf" ] || \
  /usr/bin/python3 "$B6_DIR/scripts/make_world.py" >&2

# ~/.local đang có numpy 2.x, làm cv2 hệ thống (biên dịch với numpy 1.x) lỗi "_ARRAY_API not found".
# setup.sh cài mediapipe + numpy<2 vào btth6/.pydeps; đặt lên đầu PYTHONPATH chỉ cho các node của bài này.
export PYTHONPATH="$B6_DIR/.pydeps:$PYTHONPATH"
export GLOG_minloglevel=2 TF_CPP_MIN_LOG_LEVEL=2     # bớt log INFO/WARNING của MediaPipe
PY=/usr/bin/python3
