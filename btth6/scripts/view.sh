#!/usr/bin/env bash
# Đặt camera Gazebo nhìn xiên 45° xuống vùng hoạt động của robot (khi sim đang chạy).
#   ./scripts/view.sh          mặc định
#   ./scripts/view.sh top      nhìn thẳng từ trên xuống
source "$(dirname "$0")/env.sh"
if [ "$1" = top ]; then
  REQ="pose: {position: {x: -1.0, y: 0, z: 6}, orientation: {x: 0, y: 0.7071068, z: 0, w: 0.7071068}}"
else  # camera ở phía nam vùng hoạt động, cao 3.2 m, quay mặt về +y, chúc xuống ~45°
  REQ="pose: {position: {x: -1.1, y: -3.0, z: 3.2}, orientation: {x: -0.2706, y: 0.2706, z: 0.6533, w: 0.6533}}"
fi
ign service -s /gui/move_to/pose --reqtype ignition.msgs.GUICamera --reptype ignition.msgs.Boolean \
  --timeout 3000 --req "$REQ" | grep -q "data: true" && echo "Đã đặt camera Gazebo." \
  || { echo "Không đặt được camera (Gazebo đã chạy xong chưa?)" >&2; exit 1; }
