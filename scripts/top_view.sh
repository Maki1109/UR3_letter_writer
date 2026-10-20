#!/usr/bin/env bash
# Đặt camera 3D của Gazebo nhìn thẳng từ trên xuống, thấy toàn bộ phòng (khi sim đang chạy).
#   ./scripts/top_view.sh              cao 8 m, nhìn xuống giữa phòng (1.5, 0)
#   ./scripts/top_view.sh 10 2 0       cao 10 m, nhìn xuống điểm x=2, y=0
# 01_sim.sh tự gọi script này sau khi Gazebo lên. Trả về mã lỗi != 0 nếu gọi service thất bại.
source "$(dirname "$0")/env.sh"
H="${1:-8}"; X="${2:-1.5}"; Y="${3:-0}"
# orientation = xoay quanh trục y 90° => camera nhìn thẳng xuống
ign service -s /gui/move_to/pose --reqtype ignition.msgs.GUICamera --reptype ignition.msgs.Boolean \
  --timeout 3000 --req "pose: {position: {x: $X, y: $Y, z: $H}, orientation: {x: 0, y: 0.7071068, z: 0, w: 0.7071068}}" \
  | grep -q "data: true" && echo "Camera nhìn xuống từ độ cao $H m tại ($X, $Y)." \
  || { echo "Không đặt được camera (Gazebo đã chạy xong chưa?)" >&2; exit 1; }
