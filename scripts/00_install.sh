#!/usr/bin/env bash
# Cài TurtleBot 4 Simulator (Ignition Gazebo Fortress) + các công cụ cần cho bài
set -e
sudo apt update
sudo apt install -y \
  ros-humble-turtlebot4-simulator \
  ros-humble-irobot-create-nodes \
  ros-humble-turtlebot4-desktop \
  ros-humble-turtlebot4-navigation \
  ros-humble-slam-toolbox \
  ros-humble-nav2-bringup \
  ros-humble-nav2-map-server \
  ros-humble-teleop-twist-keyboard
echo "Cài đặt xong."
