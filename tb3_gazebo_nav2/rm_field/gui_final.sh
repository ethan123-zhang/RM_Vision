#!/usr/bin/env bash
# 最终版：起 GUI（指定 GUI 相机配置）+ 场地 + 机器人，并报告窗口状态
# 用法: bash gui_final.sh [gui config 路径]
#   默认 config/gui_field.config（整场低机位斜视，+y 侧，看得到台子投影）
#   默认世界 = 带 0.30 m 围挡的 rmul2027_3v3_field_fenced.sdf（图纸外的补丁，为"一眼看出高台"）
#   要 1:1 无围挡世界:  WORLD=$PWD/worlds/rmul2027_3v3_field.sdf bash gui_final.sh
#   看侧壁特写:        bash gui_final.sh $PWD/config/gui_wall.config
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
GUICFG="${1:-$ROOT/config/gui_field.config}"
WORLD="${WORLD:-$ROOT/worlds/rmul2027_3v3_field_fenced.sdf}"
export GZ_SIM_RESOURCE_PATH="$ROOT/models:/opt/ros/jazzy/share/turtlebot3_gazebo/models"
cd "$ROOT" || exit 1
pkill -f "gz sim" 2>/dev/null
sleep 2
: > logs/gui_field.log
echo "gui config = $GUICFG"
echo "world      = $WORLD"
nohup gz sim -r --gui-config "$GUICFG" "$WORLD" > logs/gui_field.log 2>&1 &
echo "gui pid=$!"
for i in $(seq 1 90); do
  gz service -l 2>/dev/null | grep -q '/world/rmul2027_3v3/create' && { echo "server ready ${i}s"; break; }
  sleep 1
done
sleep 10
python3 spawn_model.py waffle "" 1.5 1.0 0.10 0.0 | tail -1
sleep 1
echo "=== X 窗口 ==="
for w in $(xdotool search --onlyvisible --name Gazebo); do
  echo "$w $(xdotool getwindowgeometry "$w" | tr '\n' ' ')"
done
echo "=== 渲染进程 ==="
pgrep -af 'gz sim gui' | head -1