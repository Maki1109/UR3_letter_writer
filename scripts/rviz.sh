#!/usr/bin/env bash
# RViz2 với cấu hình TurtleBot 4 (robot model, LaserScan, Map, Nav2 Goal, 2D Pose Estimate)
source "$(dirname "$0")/env.sh"
ros2 launch turtlebot4_viz view_robot.launch.py use_sim_time:=true "$@"
