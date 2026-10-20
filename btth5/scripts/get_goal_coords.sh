#!/usr/bin/env bash
# Phần 2 – Lấy tọa độ goal: bấm "Publish Point" trên RViz rồi click lên bản đồ -> in (x, y).
# Hoặc: ./get_goal_coords.sh amcl  -> in pose hiện tại của robot (x, y, yaw độ) từ /amcl_pose.
source "$(dirname "$0")/env.sh"
if [ "$1" = amcl ]; then
  ros2 topic echo --once /amcl_pose --field pose.pose | $PY -c "
import sys, math, yaml
p = yaml.safe_load(sys.stdin.read().split('---')[0]); q = p['orientation']
print(f\"x={p['position']['x']:.2f}  y={p['position']['y']:.2f}  yaw={math.degrees(2*math.atan2(q['z'], q['w'])):.0f}°\")"
else
  echo "Bấm Publish Point trên RViz rồi click vào bản đồ (Ctrl+C để thoát)"
  ros2 topic echo /clicked_point --field point
fi
