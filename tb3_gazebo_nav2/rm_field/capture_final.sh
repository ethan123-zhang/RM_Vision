#!/usr/bin/env bash
# 出图三连：斜俯视 / 正俯视 / 近景（都带 TB3 机器人）
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
export GZ_SIM_RESOURCE_PATH="$ROOT/models:/opt/ros/jazzy/share/turtlebot3_gazebo/models"
DB="$ROOT/shots"; mkdir -p "$DB" "$ROOT/logs"
cd "$ROOT" || exit 1

pkill -f "gz sim" 2>/dev/null; sleep 1.5
: > logs/visual.log
nohup gz sim -s -r --headless-rendering worlds/visual_capture.sdf > logs/visual.log 2>&1 &
echo "render server pid=$!"
for i in $(seq 1 60); do
  gz topic -l 2>/dev/null | grep -q '/closeup/image' && { echo "cameras up ${i}s"; break; }
  sleep 1
done
sleep 3
python3 spawn_model.py waffle "" 1.5 1.0 0.10 0.0 | tail -1
sleep 2

timeout 90 python3 capture_camera.py "$DB/final_overview.png" /overview/image 40 | tail -2
timeout 90 python3 capture_camera.py "$DB/final_topdown.png" /topdown/image 40 | tail -2
timeout 90 python3 capture_camera.py "$DB/final_closeup.png" /closeup/image 40 | tail -2

echo "=== 正俯视比例校验 ==="
python3 analyze_topdown.py "$DB/final_topdown.png"
pkill -f "gz sim" 2>/dev/null
echo done