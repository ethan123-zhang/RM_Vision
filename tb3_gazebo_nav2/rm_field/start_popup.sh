#!/usr/bin/env bash
# 只启动"弹出挡板"驱动（刹车验收用；改完 config.popup 后不必重起整栈）。
# pattern 用 dynamic_obstacle[.]py，避免 pkill 命中调用者自己的命令行。
set +u
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
cd "$ROOT" || exit 1
pkill -f 'dynamic_obstacle[.]py' 2>/dev/null
sleep 1
source /opt/ros/jazzy/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=42
: > logs/popup_obstacle.log
setsid nohup python3 dynamic_obstacle.py --world rmul2027_3v3 \
  --config config/avoid_obstacles.json --popup > logs/popup_obstacle.log 2>&1 < /dev/null &
for i in $(seq 1 40); do
  if grep -qE 'HIDDEN|POPPED' logs/popup_obstacle.log 2>/dev/null; then
    echo "挡板驱动就绪: $(head -1 logs/popup_obstacle.log)"
    exit 0
  fi
  sleep 0.25
done
echo "!! 挡板驱动 10 s 内未就绪"
tail -5 logs/popup_obstacle.log
exit 1