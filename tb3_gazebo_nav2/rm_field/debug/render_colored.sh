#!/usr/bin/env bash
# 出"颜色标注"取证图（橙色=场地侧壁/凸起，蓝色=围挡，深灰=周围地面）
# 前提：先跑过 debug/split_field_mesh.py 生成 debug_field_colored/meshes/*.stl
# 用法: bash debug/render_colored.sh
set +u
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
export GZ_SIM_RESOURCE_PATH="$ROOT/models:/opt/ros/jazzy/share/turtlebot3_gazebo/models"
cd "$ROOT" || exit 1
mkdir -p shots logs

pkill -f "gz sim" 2>/dev/null
sleep 1.5
: > logs/debug_colored_render.log
nohup gz sim -s -r --headless-rendering worlds/debug_height_colored.sdf > logs/debug_colored_render.log 2>&1 &
echo "render pid=$!"

for i in $(seq 1 90); do
  gz topic -l 2>/dev/null | grep -q '/hero/image' && { echo "相机就绪 ${i}s"; break; }
  sleep 1
done
sleep 3

python3 capture_camera.py shots/debug_colored_hero.png /hero/image 60 | tail -2
python3 capture_camera.py shots/debug_colored_elevation.png /elevation_wide/image 60 | tail -2
python3 capture_camera.py shots/debug_colored_wall.png /elevation_wall/image 60 | tail -2

pkill -f "gz sim" 2>/dev/null
sleep 1
echo "--- 渲染日志（尾部）---"
tail -5 logs/debug_colored_render.log
echo done