#!/usr/bin/env bash
# 无 GUI 出图：headless rendering + overview 相机 -> shots/cam_overview.png
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
export GZ_SIM_RESOURCE_PATH="$ROOT/models:/opt/ros/jazzy/share/turtlebot3_gazebo/models"
cd "$ROOT" || exit 1
OUT="${1:-$ROOT/shots/cam_overview.png}"
WORLD="${2:-$ROOT/worlds/visual_capture.sdf}"

pkill -f "gz sim" 2>/dev/null
sleep 1
pkill -f "capture_camera.py" 2>/dev/null
: > logs/visual.log
nohup gz sim -s -r --headless-rendering "$WORLD" > logs/visual.log 2>&1 &
echo "render server pid=$!"
for i in $(seq 1 60); do
  if gz topic -l 2>/dev/null | grep -q '/overview/image'; then
    echo "camera topic up after ${i}s"; break
  fi
  sleep 1
done
sleep 3
timeout 90 python3 "$ROOT/capture_camera.py" "$OUT" /overview/image 40
echo "--- render log (errors) ---"
grep -iE '\[Err\]|error|ogre|render' logs/visual.log | head -8
pkill -f "gz sim" 2>/dev/null
echo done