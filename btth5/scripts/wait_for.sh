#!/usr/bin/env bash
# Chặn tới khi một thành phần của demo sẵn sàng (dùng trong run_demo.sh để khởi động đúng thứ tự).
#   wait_for.sh robot          robot đã xuất hiện trong Gazebo (/dock_status có dữ liệu)
#   wait_for.sh localization   map_server + AMCL active
#   wait_for.sh pose           TF map -> base_link đã có (AMCL đã nhận initial pose)
#   wait_for.sh nav2           bt_navigator active
#   wait_for.sh rviz           node RViz đã chạy
source "$(dirname "$0")/env.sh"
active() { ros2 lifecycle get "$1" 2>/dev/null | grep -q '^active'; }
say() { echo "[wait_for] đợi $1 ..."; }
case "$1" in
  robot)        say "robot trong Gazebo"
                until timeout 5 ros2 topic echo --once /dock_status >/dev/null 2>&1; do sleep 2; done ;;
  localization) say "map_server + AMCL active"
                until active /map_server && active /amcl; do sleep 2; done ;;
  pose)         say "TF map -> base_link"
                until timeout 6 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null | grep -q Translation; do sleep 1; done ;;
  nav2)         say "Nav2 active"
                until active /bt_navigator; do sleep 2; done ;;
  rviz)         say "RViz"
                until ros2 node list 2>/dev/null | grep -q '^/rviz'; do sleep 2; done ;;
  *) echo "Dùng: $0 robot|localization|pose|nav2|rviz" >&2; exit 2 ;;
esac
echo "[wait_for] $1: sẵn sàng"
