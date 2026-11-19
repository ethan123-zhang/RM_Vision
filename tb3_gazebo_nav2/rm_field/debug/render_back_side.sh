#!/usr/bin/env bash
# 从下往上拍原始（未配色）场地模型的背面
set +u
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
export GZ_SIM_RESOURCE_PATH="$ROOT/models:/opt/ros/jazzy/share/turtlebot3_gazebo/models"
cd "$ROOT" || exit 1
mkdir -p shots logs
pkill -f "gz sim" 2>/dev/null
sleep 1.5
: > logs/preview_back_side.log
nohup gz sim -s -r --headless-rendering worlds/preview_back_side.sdf > logs/preview_back_side.log 2>&1 &
for i in $(seq 1 90); do
  gz topic -l 2>/dev/null | grep -q '/back_view/image' && { echo "相机就绪 ${i}s"; break; }
  sleep 1
done
sleep 3
python3 capture_camera.py shots/back_side_raw.png /back_view/image 60 | tail -1
pkill -f "gz sim" 2>/dev/null
sleep 1
echo done