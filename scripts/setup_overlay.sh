#!/usr/bin/env bash
# Sinh thư mục overlay/ từ các gói ROS đã cài (/opt/ros/humble) + áp các bản sửa cần cho bài thực hành,
# và tạo world 'room'. Chạy một lần sau khi clone repo (env.sh tự gọi nếu overlay chưa có).
#   ./scripts/setup_overlay.sh           tạo nếu chưa có
#   ./scripts/setup_overlay.sh --force   xoá và tạo lại
#
# Các bản sửa (chỉ trong overlay, không đụng vào /opt/ros):
#   1. irobot_create_description: Sensors system render bằng ogre2 (ogre1 làm /scan trả toàn range_min
#      trên Mesa/Intel); hạ tần số cảm biến cliff + ir_intensity 62 Hz -> 10 Hz (sim nhanh hơn ~4 lần).
#   2. turtlebot4_description: hạ tần số RPLIDAR 62 Hz -> 10 Hz (đúng với robot thật).
#   3. turtlebot4_ignition_bringup: thêm world worlds/room.sdf (phòng nhỏ 7 x 5 m).
set -e
WS="$(cd "$(dirname "$0")/.." && pwd)"
O="$WS/overlay"
SHARE="${ROS_SHARE:-/opt/ros/humble/share}"

[ "$1" = "--force" ] && rm -rf "$O"
[ -f "$O/.ready" ] && exit 0
for p in irobot_create_description turtlebot4_description turtlebot4_ignition_bringup; do
  [ -d "$SHARE/$p" ] || { echo "Thiếu gói $p. Cài: sudo apt install ros-humble-turtlebot4-simulator" >&2; exit 1; }
done

echo "Đang tạo overlay (một lần, khoảng vài giây)..."
rm -rf "$O"; mkdir -p "$O/share/ament_index/resource_index/packages"
for p in irobot_create_description turtlebot4_description turtlebot4_ignition_bringup; do
  cp -r "$SHARE/$p" "$O/share/"
  touch "$O/share/ament_index/resource_index/packages/$p"
done

sed -i 's|<render_engine>ogre</render_engine>|<render_engine>ogre2</render_engine>|' \
  "$O/share/irobot_create_description/urdf/create3.urdf.xacro"
sed -i 's|update_rate:=62.0 fov|update_rate:=10.0 fov|' \
  "$O/share/irobot_create_description/urdf/sensors/ir_intensity.urdf.xacro"
sed -i 's|<xacro:property name="update_rate"         value="62"/>|<xacro:property name="update_rate"         value="10"/>|' \
  "$O/share/irobot_create_description/urdf/sensors/cliff_sensor.urdf.xacro"
sed -i 's|update_rate="62.0" visualize="1"|update_rate="10.0" visualize="1"|' \
  "$O/share/turtlebot4_description/urdf/sensors/rplidar.urdf.xacro"

/usr/bin/python3 "$WS/scripts/make_room_world.py"

# Kiểm tra các bản sửa đã được áp dụng (sed không báo lỗi nếu không khớp mẫu)
grep -q "ogre2" "$O/share/irobot_create_description/urdf/create3.urdf.xacro" || { echo "Lỗi: không sửa được render_engine" >&2; exit 1; }
grep -q 'update_rate:=10.0' "$O/share/irobot_create_description/urdf/sensors/ir_intensity.urdf.xacro" || { echo "Lỗi: ir_intensity" >&2; exit 1; }
grep -q 'value="10"' "$O/share/irobot_create_description/urdf/sensors/cliff_sensor.urdf.xacro" || { echo "Lỗi: cliff_sensor" >&2; exit 1; }
grep -q 'update_rate="10.0"' "$O/share/turtlebot4_description/urdf/sensors/rplidar.urdf.xacro" || { echo "Lỗi: rplidar" >&2; exit 1; }
touch "$O/.ready"
echo "Overlay sẵn sàng: $O"
