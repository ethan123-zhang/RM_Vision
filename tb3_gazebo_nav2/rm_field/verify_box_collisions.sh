#!/usr/bin/env bash
# box 拼块碰撞验收（自包含：起 drive 世界 -> 跑 verify_box_collisions.py -> 收尾）
# 用法: bash verify_box_collisions.sh
# 退出码: 0 全部通过 / 3 有判据未通过 / 1 环境问题
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
export GZ_SIM_RESOURCE_PATH="$ROOT/models:/opt/ros/jazzy/share/turtlebot3_gazebo/models"
export GZ_WORLD=drive
cd "$ROOT" || exit 1
mkdir -p logs results

pkill -f "gz sim" 2>/dev/null
sleep 1.5
LOG="$ROOT/logs/verify_box_collisions_gz.log"
: > "$LOG"
nohup gz sim -s -r worlds/drive_colored.sdf > "$LOG" 2>&1 &
GZPID=$!
echo "started gz sim -s pid=$GZPID world=drive"

ready=0
for i in $(seq 1 60); do
  if gz service -l 2>/dev/null | grep -q '/world/drive/create'; then
    echo "server ready after ${i}s"
    ready=1
    break
  fi
  sleep 1
done
if [ "$ready" != "1" ]; then
  echo "!! 等不到 /world/drive/create，看 logs/verify_box_collisions_gz.log"
  tail -20 "$LOG"
  pkill -f "gz sim" 2>/dev/null
  exit 1
fi
sleep 2

python3 verify_box_collisions.py
RC=$?
echo "verify_box_collisions.py exit=$RC"

pkill -f "gz sim" 2>/dev/null
sleep 1
exit $RC