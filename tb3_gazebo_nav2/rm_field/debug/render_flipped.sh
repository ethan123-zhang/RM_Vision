#!/usr/bin/env bash
# 出"翻转后"的场地图，与规则手册 图3-4 对比朝向
set +u
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
export GZ_SIM_RESOURCE_PATH="$ROOT/models:/opt/ros/jazzy/share/turtlebot3_gazebo/models"
cd "$ROOT" || exit 1
mkdir -p shots logs
pkill -f "gz sim" 2>/dev/null
sleep 1.5
: > logs/probe_flipped.log
nohup gz sim -s -r --headless-rendering worlds/probe_flipped_field.sdf > logs/probe_flipped.log 2>&1 &
for i in $(seq 1 90); do
  gz topic -l 2>/dev/null | grep -q '/probe/image' && { echo "相机就绪 ${i}s"; break; }
  sleep 1
done
sleep 3
python3 capture_camera.py shots/probe_flipped.png /probe/image 60 | tail -1
pkill -f "gz sim" 2>/dev/null
sleep 1
echo done