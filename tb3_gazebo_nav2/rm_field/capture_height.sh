#!/usr/bin/env bash
# 出"台面高度"三张取证图并跑像素验收：
#   height_elevation_wide.png  正立面全景（1920x1080，亮带 12.6 m x 0.503 m）
#   height_elevation_wall.png  墙面近景（1280x720，墙高占画面 ~450 px）
#   height_oblique_low.png     低机位斜视（1600x900，台面+侧壁+地面同框）
# 用法: bash capture_height.sh
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
export GZ_SIM_RESOURCE_PATH="$ROOT/models:/opt/ros/jazzy/share/turtlebot3_gazebo/models"
DB="$ROOT/shots"; mkdir -p "$DB" "$ROOT/logs" "$ROOT/results"
cd "$ROOT" || exit 1

pkill -f "gz sim" 2>/dev/null; sleep 1.5
: > logs/height_render.log
nohup gz sim -s -r --headless-rendering worlds/visual_capture.sdf > logs/height_render.log 2>&1 &
echo "render server pid=$!"

for i in $(seq 1 90); do
  gz topic -l 2>/dev/null | grep -q '/oblique_low/image' && { echo "相机就绪 ${i}s"; break; }
  sleep 1
done
sleep 3
python3 spawn_model.py waffle "" 1.5 1.0 0.10 0.0 | tail -1
sleep 2

timeout 120 python3 capture_camera.py "$DB/height_elevation_wide.png" /elevation_wide/image 60 | tail -3
timeout 120 python3 capture_camera.py "$DB/height_elevation_wall.png" /elevation_wall/image 60 | tail -3
timeout 120 python3 capture_camera.py "$DB/height_oblique_low.png" /oblique_low/image 60 | tail -3

pkill -f "gz sim" 2>/dev/null
sleep 1
echo "=== 像素验收 ==="
python3 verify_height.py "$ROOT/worlds/visual_capture.sdf" "$DB"
echo done