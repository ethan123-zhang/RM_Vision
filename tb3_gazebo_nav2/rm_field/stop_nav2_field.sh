#!/usr/bin/env bash
# 停掉 nav2_field.sh 拉起来的一整套（launch / gz / bridge / rviz / lifecycle 容器）。
# 只停本 ROS_DOMAIN_ID 的进程（默认 42，可用 ROS_DOMAIN_ID=43 指定）；别的 domain
# 的栈不受影响 —— 背景见 domain_cleanup.sh 顶部说明。
# 注意：不要再加 `pkill -f 'nav2_'` 这类过宽的 pattern —— 调用者的命令行里只要含
# nav2_field.sh/stop_nav2_field.sh 就会被自己杀掉（2026-11-20 在串联命令里踩到过）。
# 万一现场需要连别的一起清：RM_CLEAN_ALL_DOMAINS=1 bash stop_nav2_field.sh
set +u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
source "$ROOT/domain_cleanup.sh" || { echo "致命: 缺 $ROOT/domain_cleanup.sh"; exit 1; }

echo "=== 停栈（domain $ROS_DOMAIN_ID）==="
# 先停 launch，让它有机会把子进程收掉；再逐类兜底
kill_in_domain TERM 'ros2 launch nav2_bringup' 'dynamic_obstacle.py' \
  'goal_pose_bridge[.]py' 'gz sim' 'parameter_bridge' 'component_container' \
  'robot_state_publisher' 'rviz2'
sleep 2
# gz sim / 组件容器对 TERM 不总是响应，补一轮 KILL
kill_in_domain KILL 'ros2 launch nav2_bringup' 'gz sim' 'parameter_bridge' \
  'component_container' 'dynamic_obstacle.py' 'goal_pose_bridge[.]py'
sleep 1
left=$(count_in_domain 'gz sim' 'ros2 launch nav2_bringup' 'rviz2' \
  'parameter_bridge' 'component_container')
echo "残留: $left 个（domain $ROS_DOMAIN_ID）"
if [ "$left" != 0 ] && [ "${RM_CLEAN_ALL_DOMAINS:-0}" != 1 ]; then
  echo "提示: 若还要连别的 domain 一起清：RM_CLEAN_ALL_DOMAINS=1 bash stop_nav2_field.sh"
fi
echo "已停止"