#!/usr/bin/env bash
# 驾驶用彩色世界：草绿台面 / 橙色凸起侧壁 / 红色围栏 / 黄色顶沿与警示带 / 深灰场外地面
# 用法: bash gui_colored.sh [gui config 路径]
#   默认 config/gui_drive.config（较高机位 3/4 视，看得到整个台面和四边围栏）
#   换低机位斜视: bash gui_colored.sh $PWD/config/gui_field.config
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
GUICFG="${1:-$ROOT/config/gui_drive.config}"
WORLD="$ROOT/worlds/drive_colored.sdf"
export GZ_WORLD=drive
export GZ_SIM_RESOURCE_PATH="$ROOT/models:/opt/ros/jazzy/share/turtlebot3_gazebo/models"
cd "$ROOT" || exit 1
mkdir -p logs shots

pkill -f "gz sim" 2>/dev/null
sleep 2
: > logs/gui_colored.log
echo "gui config = $GUICFG"
echo "world      = $WORLD"
nohup gz sim -r --gui-config "$GUICFG" "$WORLD" > logs/gui_colored.log 2>&1 &
echo "gui pid=$!"

for i in $(seq 1 90); do
  gz service -l 2>/dev/null | grep -q '/world/drive/create' && { echo "server ready ${i}s"; break; }
  sleep 1
done
sleep 8

python3 spawn_model.py waffle "" 0 0 0.10 0.0 | tail -1
sleep 1

echo "=== X 窗口 ==="
for w in $(xdotool search --onlyvisible --name Gazebo 2>/dev/null); do
  echo "$w $(xdotool getwindowgeometry "$w" 2>/dev/null | tr '\n' ' ')"
done
echo
echo "开车: gz topic -t /cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.15}'"
echo "转向: gz topic -t /cmd_vel -m gz.msgs.Twist -p 'angular: {z: 0.4}'"
echo "停车: gz topic -t /cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.0}, angular: {z: 0.0}'"