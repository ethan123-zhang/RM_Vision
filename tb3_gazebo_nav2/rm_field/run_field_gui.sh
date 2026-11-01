#!/usr/bin/env bash
# 一键：起 Gazebo GUI + 场地 + 一台 TB3（用于肉眼看比例/碰撞）
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
export GZ_SIM_RESOURCE_PATH="$ROOT/models:/opt/ros/jazzy/share/turtlebot3_gazebo/models"
WORLD="${1:-$ROOT/worlds/rmul2027_3v3_field.sdf}"
mkdir -p "$ROOT/logs" "$ROOT/shots"
cd "$ROOT" || exit 1

pkill -f "gz sim" 2>/dev/null
sleep 1
LOG="$ROOT/logs/gui_field.log"
: > "$LOG"
nohup gz sim -r "$WORLD" > "$LOG" 2>&1 &
echo "gz sim (GUI) pid=$!  world=$WORLD"
for i in $(seq 1 90); do
  if gz service -l 2>/dev/null | grep -q '/world/rmul2027_3v3/create'; then
    echo "服务器就绪 ${i}s"; break
  fi
  sleep 1
done
sleep 8   # 等 GUI 把 15 万面片的场地画出来
python3 "$ROOT/spawn_model.py" waffle "" 1.5 1.0 0.10 0.0 | tail -2
echo "--- windows ---"
xdotool search --onlyvisible --name "." 2>/dev/null | while read -r w; do
  printf '%s | %s\n' "$(xdotool getwindowname "$w" 2>/dev/null)" \
                      "$(xdotool getwindowgeometry "$w" 2>/dev/null | tr '\n' ' ')"
done | head -8
echo "--- log tail ---"
tail -3 "$LOG"