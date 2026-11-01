#!/usr/bin/env bash
# 一键停掉 Nav2 TB3 仿真（Gazebo + RViz + 全部 ROS 节点）
echo "[stop] 停止进程..."
pkill -f 'ros2 launch nav2_bringup' 2>/dev/null
pkill -f 'gz sim' 2>/dev/null
pkill -f 'rviz2' 2>/dev/null
pkill -f 'parameter_bridge' 2>/dev/null
pkill -f 'component_container' 2>/dev/null
pkill -f 'robot_state_publisher' 2>/dev/null
sleep 2
left=$(pgrep -fc 'gz sim|rviz2|component_container|parameter_bridge' 2>/dev/null || true)
echo "[stop] 剩余进程数: ${left:-0}"