#!/usr/bin/env bash
# 停掉 nav2_field.sh 拉起来的一整套（launch / gz / bridge / rviz / lifecycle 容器）
pkill -f 'ros2 launch nav2_bringup' 2>/dev/null
sleep 1
pkill -f 'gz sim' 2>/dev/null
pkill -f 'parameter_bridge' 2>/dev/null
pkill -f 'component_container' 2>/dev/null
pkill -f 'robot_state_publisher' 2>/dev/null
pkill -f 'rviz2' 2>/dev/null
pkill -f 'nav2_' 2>/dev/null
sleep 2
echo "残留: $(pgrep -fc 'gz sim|ros2 launch|rviz2|parameter_bridge|component_container' 2>/dev/null || echo 0)"
echo "已停止"