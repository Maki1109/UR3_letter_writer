#!/usr/bin/env bash
# Tắt sạch Gazebo + mọi node ROS. Ctrl+C launch đôi khi để lại "ign gazebo server" mồ côi,
# server cũ vẫn publish /clock, /scan... và gây nhiễu cho lần chạy sau.
pkill -INT -f "ros2 launch" ; sleep 4
pkill -9 -f "ign gazebo"
for p in $(ps -eo pid,args | grep -E "/opt/ros/humble/(lib|bin)/" | grep -v grep | awk '{print $1}'); do
  kill -9 "$p" 2>/dev/null
done
ros2 daemon stop > /dev/null 2>&1
echo "Còn lại: $(ps -eo args | grep -cE '[i]gn gazebo|/opt/ros/[h]umble') tiến trình"
