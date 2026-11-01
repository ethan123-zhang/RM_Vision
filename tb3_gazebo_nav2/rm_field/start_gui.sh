#!/usr/bin/env bash
# 启动带 GUI 的 Gazebo（场地世界），用于人工查看/截图
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
export GZ_SIM_RESOURCE_PATH="$ROOT/models:/opt/ros/jazzy/share/turtlebot3_gazebo/models"
WORLD="${1:-$ROOT/worlds/rmul2027_3v3_field.sdf}"
TAG="${2:-gui}"
mkdir -p "$ROOT/logs" "$ROOT/shots"
pkill -f "gz sim" 2>/dev/null
sleep 1
LOG="$ROOT/logs/${TAG}.log"
: > "$LOG"
cd "$ROOT" || exit 1
nohup gz sim -r "$WORLD" > "$LOG" 2>&1 &
echo "started gz sim (GUI) pid=$!"
for i in $(seq 1 90); do
  if xdotool search --onlyvisible --name "." 2>/dev/null | while read -r w; do
       xdotool getwindowname "$w" 2>/dev/null; done | grep -qi 'gazebo'; then
    echo "Gazebo 窗口已出现 (等待 ${i}s)"
    break
  fi
  sleep 1
done
echo "--- windows ---"
xdotool search --onlyvisible --name "." 2>/dev/null | while read -r w; do
  printf '%s | %s\n' "$(xdotool getwindowname "$w" 2>/dev/null)" "$(xdotool getwindowgeometry "$w" 2>/dev/null | tr '\n' ' ')"
done | head -10
echo "--- log tail ---"
tail -3 "$LOG"