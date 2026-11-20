#!/usr/bin/env bash
# 单独重启动态障碍物驱动（改完 config/avoid_obstacles.json 后用，不必重起整个栈）。
# 注意 pkill 的 pattern 用 dynamic_obstacle[.]py：直接写 dynamic_obstacle.py 会把
# 调用者自己的命令行也匹配掉（同名文件在命令行里），2026-11-20 踩过。
set +u
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
cd "$ROOT" || exit 1
pkill -f 'dynamic_obstacle[.]py' 2>/dev/null
sleep 1
source /opt/ros/jazzy/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=42
: > logs/dynamic_obstacle.log
setsid nohup python3 dynamic_obstacle.py --world rmul2027_3v3 \
  --config config/avoid_obstacles.json > logs/dynamic_obstacle.log 2>&1 < /dev/null &
for i in $(seq 1 40); do
  if grep -qE 'WAIT|MOVING|TRIGGER|PARKED' logs/dynamic_obstacle.log 2>/dev/null; then
    echo "驱动就绪: $(head -1 logs/dynamic_obstacle.log)"
    exit 0
  fi
  sleep 0.25
done
echo "!! 动态障碍物驱动 10 s 内未就绪"
tail -5 logs/dynamic_obstacle.log
exit 1