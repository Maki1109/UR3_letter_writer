#!/usr/bin/env bash
# Giao diện bản đồ riêng: khu vực, robot, lộ trình + đường đi dự kiến, câu lệnh, trạng thái.
# Bấm chuột vào khu vực để đi tới đó. Thêm -p snapshot:=/tmp/map.png để ghi ảnh định kỳ.
source "$(dirname "$0")/env.sh"
exec $PY "$B7_DIR/nodes/map_view_node.py" --ros-args -p use_sim_time:=true "$@"
