#!/usr/bin/env bash
# 驱动测试：原地旋转 2 秒（验证仿真与渲染是活的）
set +u
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
echo "[drive] 原地旋转 (angular.z=0.8, 2s)"
timeout 4 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.8}}" >/dev/null 2>&1
timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1
echo "[drive] 停止"
timeout -s INT 4 ros2 topic echo /odom --once 2>/dev/null | grep -A3 'orientation:' | head -4