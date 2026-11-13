#!/usr/bin/env bash
# Node điều khiển: /emotion -> quỹ đạo -> /cmd_vel. Tự undock và tiến ra 1 m giữa sân trước khi nhận lệnh.
#   ./02_motion.sh -p repeat:=false          mỗi biểu cảm chỉ chạy một lượt
#   ./02_motion.sh -p arc_time:=6.0          cung tròn dài hơn
source "$(dirname "$0")/env.sh"
exec $PY "$B8_DIR/nodes/emotion_motion_node.py" --ros-args -p use_sim_time:=true "$@"
