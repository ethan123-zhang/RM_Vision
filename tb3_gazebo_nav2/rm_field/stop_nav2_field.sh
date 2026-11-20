#!/usr/bin/env bash
# 停掉 nav2_field.sh 拉起来的一整套（launch / gz / bridge / rviz / lifecycle 容器）
# 注意：不要再加 `pkill -f 'nav2_'` 这类过宽的 pattern —— 调用者的命令行里只要含
# nav2_field.sh/stop_nav2_field.sh 就会被自己杀掉（2026-11-20 在串联命令里踩到过）。
set +u
pkill -f 'ros2 launch nav2_bringup' 2>/dev/null
sleep 1
pkill -f 'dynamic_obstacle.py' 2>/dev/null
pkill -f 'goal_pose_bridge[.]py' 2>/dev/null
pkill -f 'gz sim' 2>/dev/null
pkill -f 'parameter_bridge' 2>/dev/null
pkill -f 'component_container' 2>/dev/null
pkill -f 'robot_state_publisher' 2>/dev/null
pkill -f 'rviz2' 2>/dev/null
sleep 2
echo "残留: $(pgrep -fc 'gz sim|ros2 launch nav2_bringup|rviz2|parameter_bridge|component_container' 2>/dev/null || echo 0)"
echo "已停止"