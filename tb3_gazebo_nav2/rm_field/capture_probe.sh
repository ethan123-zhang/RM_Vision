#!/usr/bin/env bash
# 渲染"GUI 视角"实验图：起无 GUI 服务器 → 生成 TB3 → 抓指定相机话题 → 停。
# 用法: bash capture_probe.sh <输出文件名> [世界文件] [相机话题]
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
export GZ_SIM_RESOURCE_PATH="$ROOT/models:/opt/ros/jazzy/share/turtlebot3_gazebo/models"
OUT="$ROOT/shots/${1:-probe.png}"
WORLD="${2:-$ROOT/worlds/gui_probe.sdf}"
TOPIC="${3:-/gui_view/image}"
cd "$ROOT" || exit 1
pkill -f "gz sim" 2>/dev/null; sleep 1.5
nohup gz sim -s -r --headless-rendering "$WORLD" > logs/probe.log 2>&1 &
echo "server pid=$! world=$(basename "$WORLD") topic=$TOPIC"
for i in $(seq 1 90); do
  gz topic -l 2>/dev/null | grep -q "$TOPIC" && { echo "camera up ${i}s"; break; }
  sleep 1
done
sleep 3
python3 spawn_model.py waffle "" 1.5 1.0 0.10 0.0 | tail -1
sleep 2
timeout 120 python3 capture_camera.py "$OUT" "$TOPIC" 60 | tail -2
pkill -f "gz sim" 2>/dev/null
echo "saved $OUT"