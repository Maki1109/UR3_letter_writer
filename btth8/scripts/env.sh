# Môi trường chung cho bài 08 (source, không chạy trực tiếp).
# Dùng lại env của bài 04 (overlay ogre2, gỡ biến snap, ROS_DOMAIN_ID...) và world open_arena của bài 06
# (sân trống 7 x 7 m, chỉ có tường bao thấp – đủ chỗ cho các quỹ đạo cong / xoay / tiến-lùi).
B8_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export WORLD="${WORLD:-open_arena}"
source "$B8_DIR/../scripts/env.sh"
export B8_DIR B6_DIR="$B8_DIR/../btth6"
[ -f "$WS_DIR/overlay/share/turtlebot4_ignition_bringup/worlds/open_arena.sdf" ] || \
  /usr/bin/python3 "$B6_DIR/scripts/make_world.py" >&2

# ~/.local có numpy 2.x, còn cv2 hệ thống và TensorFlow 2.15 cần numpy 1.x. setup.sh cài numpy<2, tensorflow-cpu,
# deepface vào btth8/.pydeps; chỉ đặt nó lên đầu PYTHONPATH cho các node của bài này.
export PYTHONPATH="$B8_DIR/.pydeps:$PYTHONPATH"
export PYTHONWARNINGS="ignore" TF_CPP_MIN_LOG_LEVEL=3 TF_ENABLE_ONEDNN_OPTS=0
export CUDA_VISIBLE_DEVICES=""          # tensorflow-cpu; tránh dò GPU
PY=/usr/bin/python3
