#!/usr/bin/env bash
# 只启动"/goal_pose 桥"（点地图→导航目标）。改完桥代码或 RViz 配置后不用重起整栈。
# pattern 用 goal_pose_bridge[.]py，避免 pkill 命中调用者自己的命令行。
set +u
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
cd "$ROOT" || exit 1
pkill -f 'python3 goal_pose_bridge[.]py' 2>/dev/null
sleep 1
source /opt/ros/jazzy/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=42
: > logs/goal_pose_bridge.log
setsid nohup python3 goal_pose_bridge.py > logs/goal_pose_bridge.log 2>&1 < /dev/null &
for i in $(seq 1 120); do
  if grep -q 'READY' logs/goal_pose_bridge.log 2>/dev/null; then
    echo "桥就绪: $(grep -m1 READY logs/goal_pose_bridge.log)"
    exit 0
  fi
  if ! pgrep -f 'python3 goal_pose_bridge[.]py' >/dev/null 2>&1; then
    echo "!! 桥进程退出了"; tail -10 logs/goal_pose_bridge.log; exit 1
  fi
  sleep 0.5
done
echo "!! 桥 60 s 内没 READY"; tail -10 logs/goal_pose_bridge.log; exit 1