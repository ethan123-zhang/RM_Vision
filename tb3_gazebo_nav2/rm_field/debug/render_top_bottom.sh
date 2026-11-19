#!/usr/bin/env bash
# 出"从上往下"和"从下往上"两张图，用来核对台面朝向
set +u
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
export GZ_SIM_RESOURCE_PATH="$ROOT/models:/opt/ros/jazzy/share/turtlebot3_gazebo/models"
cd "$ROOT" || exit 1
mkdir -p shots logs

pkill -f "gz sim" 2>/dev/null
sleep 1.5
: > logs/view_top_bottom.log
nohup gz sim -s -r --headless-rendering worlds/drive_colored_preview.sdf > logs/view_top_bottom.log 2>&1 &
for i in $(seq 1 90); do
  gz topic -l 2>/dev/null | grep -q '/view_top/image' && { echo "相机就绪 ${i}s"; break; }
  sleep 1
done
sleep 3
python3 capture_camera.py shots/view_from_top.png /view_top/image 60 | tail -1
python3 capture_camera.py shots/view_from_bottom.png /view_bottom/image 60 | tail -1
pkill -f "gz sim" 2>/dev/null
sleep 1
echo done