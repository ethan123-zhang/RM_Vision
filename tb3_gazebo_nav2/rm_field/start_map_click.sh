#!/usr/bin/env bash
# 只启动"地图点选窗口"（不重起仿真栈）。需要 DISPLAY（WSLg 下就是 :0）。
set +u
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
cd "$ROOT" || exit 1
pkill -f 'python3 map_click[.]py' 2>/dev/null
sleep 1
source /opt/ros/jazzy/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=42
export DISPLAY="${DISPLAY:-:0}"
: > logs/map_click.log
setsid nohup python3 map_click.py --map maps/rmul2027_3v3_up.yaml --ppm 70 \
  > logs/map_click.log 2>&1 < /dev/null &
sleep 2
if pgrep -f 'python3 map_click[.]py' >/dev/null 2>&1; then
  echo "地图点选窗口已启动（日志 logs/map_click.log）"
  echo "  左键点地图=去那里；右键=重定位；中键=取消"
else
  echo "!! 地图点选窗口没起来"; tail -10 logs/map_click.log; exit 1
fi