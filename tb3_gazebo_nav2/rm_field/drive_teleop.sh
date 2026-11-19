#!/usr/bin/env bash
# 一键：起 Gazebo GUI 世界 + 小车 + 键盘遥控（WASD / 方向键）
# 用法: bash drive_teleop.sh
#   之后在**这个终端窗口**里按 w/s/a/d 开车（键盘焦点要在终端，Gazebo 窗口只用来看）。
#   w/s 前进后退，a/d 转向，q/e/z/c 斜向，空格停车，+/- 调速，[ ] 转向调速，Ctrl+C 退出。
#   注意：每次都是先 pkill 旧仿真再起新的（gui_colored.sh 的行为），确认没有别的仿真在跑。
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
export GZ_WORLD=drive
export GZ_SIM_RESOURCE_PATH="$ROOT/models:/opt/ros/jazzy/share/turtlebot3_gazebo/models"
cd "$ROOT" || exit 1
mkdir -p logs

if gz service -l 2>/dev/null | grep -q '/world/drive/create'; then
  echo "== 复用已在跑的 drive 世界（如果没看到 Gazebo 窗口，说明它是 headless：先 pkill -f 'gz sim' 再重跑）=="
else
  echo "== 启动 GUI 世界 + 小车（约 15~25 s）=="
  bash gui_colored.sh
fi

if ! timeout 8 gz topic -e -t /world/drive/pose/info -n 1 2>/dev/null | grep -q 'name: "waffle"'; then
  echo "== 生成 waffle =="
  python3 spawn_model.py waffle "" 0 0 0.10 0.0 | tail -1
fi

echo
echo "=========================================================="
echo " 键盘遥控：焦点放在这个终端窗口，按 w/s/a/d 开车"
echo "   Gazebo 窗口在 WSLg 里（Alt+Tab 找 Gazebo Sim），只用来看"
echo "=========================================================="
exec python3 keyboard_teleop.py "$@"