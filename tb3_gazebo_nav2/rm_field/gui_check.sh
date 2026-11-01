#!/usr/bin/env bash
# 起 GUI（用场地专用 gui.config 俯视相机）-> 回读相机位姿 -> 抓窗口 -> 统计
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
export GZ_SIM_RESOURCE_PATH="$ROOT/models:/opt/ros/jazzy/share/turtlebot3_gazebo/models"
POSE="${1:--9.5 -11.5 9.0 0 0.542 0.880}"
cd "$ROOT" || exit 1

bash make_gui_config.sh "$POSE" | tail -2
pkill -f "gz sim" 2>/dev/null; sleep 1.5
: > logs/gui_field.log
nohup gz sim -r --gui-config "$ROOT/config/gui_field.config" \
      "$ROOT/worlds/rmul2027_3v3_field.sdf" > logs/gui_field.log 2>&1 &
echo "gui pid=$!"
for i in $(seq 1 90); do
  gz service -l 2>/dev/null | grep -q '/world/rmul2027_3v3/create' && { echo "server ready ${i}s"; break; }
  sleep 1
done
sleep 12
python3 spawn_model.py waffle "" 1.5 1.0 0.10 0.0 | tail -1
sleep 2

echo "=== 回读 GUI 相机位姿 ==="
timeout 2 gz topic -e -t /gui/camera/pose 2>/dev/null | head -8

WIN=$(xdotool search --onlyvisible --name "Gazebo" | head -1)
echo "win=$WIN"
import -window "$WIN" "$ROOT/shots/gui_view.png" && echo "截图 ok"
python3 gui_stats.py "$ROOT/shots/gui_view.png"