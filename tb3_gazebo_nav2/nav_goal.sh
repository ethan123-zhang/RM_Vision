#!/usr/bin/env bash
# 发送 Nav2 导航目标并等待结果（等价 RViz 的 Nav2 Goal）
# 用法: bash nav_goal.sh [x] [y]
WS=/mnt/e/RM/tb3_gazebo_nav2
set +u
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
X="${1:-1.0}"; Y="${2:--0.5}"
echo "[goal] 目标: map (${X}, ${Y})"
timeout 180 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: ${X}, y: ${Y}, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback 2>&1 | grep -vE '^\s*$' | tail -25