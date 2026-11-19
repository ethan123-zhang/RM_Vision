#!/usr/bin/env bash
# 出"驾驶配色"预览图（与 GUI 相机同一位姿），并停掉渲染服务器
set +u
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
export GZ_SIM_RESOURCE_PATH="$ROOT/models:/opt/ros/jazzy/share/turtlebot3_gazebo/models"
cd "$ROOT" || exit 1
mkdir -p shots logs

pkill -f "gz sim" 2>/dev/null
sleep 1.5
: > logs/drive_preview.log
nohup gz sim -s -r --headless-rendering worlds/drive_colored_preview.sdf > logs/drive_preview.log 2>&1 &
echo "render pid=$!"

for i in $(seq 1 90); do
  gz topic -l 2>/dev/null | grep -q '/hero/image' && { echo "相机就绪 ${i}s"; break; }
  sleep 1
done
sleep 3

export GZ_WORLD=drive
python3 spawn_model.py waffle "" 0 0 0.10 0.0 | tail -1
sleep 2

python3 capture_camera.py shots/drive_colored_view.png /hero/image 60 | tail -2

pkill -f "gz sim" 2>/dev/null
sleep 1
echo "--- 渲染日志尾部 ---"
tail -4 logs/drive_preview.log
echo done