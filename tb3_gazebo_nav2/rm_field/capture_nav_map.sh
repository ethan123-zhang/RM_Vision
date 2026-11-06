#!/usr/bin/env bash
# 顶视渲染 → Nav2 地图：起无 GUI 服务器 → 抓 1600x1100 正俯视图 → make_nav_map.py → 停
# 用法: bash capture_nav_map.sh
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
export GZ_SIM_RESOURCE_PATH="$ROOT/models:/opt/ros/jazzy/share/turtlebot3_gazebo/models"
WORLD="$ROOT/worlds/nav_map_capture.sdf"
TOPIC=/nav_topdown/image
OUT="$ROOT/shots/nav_topdown.png"
cd "$ROOT" || exit 1
mkdir -p logs shots maps results

pkill -f "gz sim" 2>/dev/null; sleep 1.5
nohup gz sim -s -r --headless-rendering "$WORLD" > logs/nav_map_capture.log 2>&1 &
echo "server pid=$! world=$(basename "$WORLD")"
for i in $(seq 1 90); do
  gz topic -l 2>/dev/null | grep -q "$TOPIC" && { echo "camera up ${i}s"; break; }
  sleep 1
done
sleep 3
timeout 150 python3 capture_camera.py "$OUT" "$TOPIC" 90 | tail -3
pkill -f "gz sim" 2>/dev/null
sleep 1

echo
echo "=== 由渲染图生成地图 ==="
python3 make_nav_map.py
RC=$?
echo
echo "=== 落盘文件 ==="
ls -l maps/ 2>/dev/null | tail -5
exit $RC