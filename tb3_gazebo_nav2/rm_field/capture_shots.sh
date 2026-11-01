#!/usr/bin/env bash
# 出图取证：无 GUI（headless rendering）抓正俯视 + 斜俯视，先空场地再放机器人
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
export GZ_SIM_RESOURCE_PATH="$ROOT/models:/opt/ros/jazzy/share/turtlebot3_gazebo/models"
DB="$ROOT/shots"
mkdir -p "$DB" "$ROOT/logs"
cd "$ROOT" || exit 1

pkill -f "gz sim" 2>/dev/null; sleep 1
: > logs/visual.log
nohup gz sim -s -r --headless-rendering worlds/visual_capture.sdf > logs/visual.log 2>&1 &
echo "render server pid=$!"
for i in $(seq 1 60); do
  if gz topic -l 2>/dev/null | grep -q '/topdown/image'; then echo "cameras up after ${i}s"; break; fi
  sleep 1
done
sleep 3

echo "--- 1) 空场地 ---"
timeout 90 python3 capture_camera.py "$DB/cam_topdown_empty.png" /topdown/image 40 | tail -3
timeout 90 python3 capture_camera.py "$DB/cam_overview_empty.png" /overview/image 40 | tail -3

echo "--- 2) 放 TurtleBot3 waffle 到场地 (1.5, 1.0) ---"
python3 spawn_model.py waffle "" 1.5 1.0 0.10 0.0 | tail -2
sleep 2

echo "--- 3) 带机器人 ---"
timeout 90 python3 capture_camera.py "$DB/cam_topdown_robot.png" /topdown/image 40 | tail -3
timeout 90 python3 capture_camera.py "$DB/cam_overview_robot.png" /overview/image 40 | tail -3

pkill -f "gz sim" 2>/dev/null
echo done