#!/usr/bin/env bash
# 启动/重启 Gazebo Harmonic 服务器（headless，纯物理；给 verify_field.py 用）
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
export GZ_SIM_RESOURCE_PATH="$ROOT/models:/opt/ros/jazzy/share/turtlebot3_gazebo/models"
mkdir -p "$ROOT/logs"
WORLD="${1:-$ROOT/worlds/rmul2027_3v3_field.sdf}"
TAG="${2:-field}"

pkill -f "gz sim" 2>/dev/null
sleep 1
LOG="$ROOT/logs/server_${TAG}.log"
: > "$LOG"
cd "$ROOT" || exit 1
nohup gz sim -s -r "$WORLD" > "$LOG" 2>&1 &
echo "started gz sim -s pid=$! world=$WORLD"
for i in $(seq 1 40); do
  if gz service -l 2>/dev/null | grep -q '/world/rmul2027_3v3/create'; then
    echo "world service ready after ${i}00ms"
    break
  fi
  sleep 1
done
echo "--- topics (selected) ---"
gz topic -l 2>/dev/null | grep -E 'world/rmul2027_3v3' | head -20
echo "--- server log tail ---"
tail -5 "$LOG"