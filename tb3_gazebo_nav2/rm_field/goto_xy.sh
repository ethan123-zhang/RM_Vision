#!/usr/bin/env bash
# 命令行让机器人去某点（等价于在 RViz 地图上点一下）。
#   bash goto_xy.sh X Y [YAW]      # YAW 弧度，缺省 0
# 不依赖 /goal_pose 桥：直接发 NavigateToPose action；栈需要在跑（nav2_field.sh 或 nav_gui.sh）。
set +u
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
X="${1:?用法: bash goto_xy.sh X Y [YAW]}"
Y="${2:?用法: bash goto_xy.sh X Y [YAW]}"
YAW="${3:-0}"
source /opt/ros/jazzy/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=42
Q=$(python3 -c "import math; y=float('$YAW'); print(f'{math.sin(y/2):.6f} {math.cos(y/2):.6f}')")
QZ="${Q% *}"; QW="${Q#* }"
echo "目标: ($X, $Y) yaw=$YAW"
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: $X, y: $Y}, orientation: {z: $QZ, w: $QW}}}}" --feedback