#!/usr/bin/env bash
# 干净重启 GUI：场地 + 俯视相机配置 + 一台 TB3 放在场地中间偏右
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
export GZ_SIM_RESOURCE_PATH="$ROOT/models:/opt/ros/jazzy/share/turtlebot3_gazebo/models"
cd "$ROOT" || exit 1
POSE="${1:--9.5 -11.5 9.0 0 0.542 0.880}"
bash make_gui_config.sh "$POSE" >/dev/null
pkill -f "gz sim" 2>/dev/null; sleep 1.5
: > logs/gui_field.log
nohup gz sim -r --gui-config "$ROOT/config/gui_field.config" \
      "$ROOT/worlds/rmul2027_3v3_field.sdf" > logs/gui_field.log 2>&1 &
echo "gui pid=$!"
for i in $(seq 1 90); do
  gz service -l 2>/dev/null | grep -q '/world/rmul2027_3v3/create' && { echo "server ready ${i}s"; break; }
  sleep 1
done
sleep 10
python3 spawn_model.py waffle "" 1.5 1.0 0.10 0.0 | tail -1
sleep 2
echo "gui 进程: $(pgrep -af 'gz sim gui' | head -1)"