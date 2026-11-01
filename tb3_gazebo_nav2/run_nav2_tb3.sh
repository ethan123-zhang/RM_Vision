#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# 启动 Nav2 官方 TurtleBot3 示例
#   ROS 2 Jazzy + Gazebo Harmonic (gz sim 8) + nav2_bringup/tb3_simulation_launch.py
#
# 用法:
#   bash run_nav2_tb3.sh                          # 带 Gazebo GUI + RViz
#   bash run_nav2_tb3.sh headless:=True use_rviz:=False   # 纯后台无界面
# 附注:
#   * 全部日志写到 logs/ 下（见调用方重定向）
#   * ROS_DOMAIN_ID 固定 42，避免与宿主其它 ROS 进程串台
# ---------------------------------------------------------------------------
set +u   # ROS 的 setup.bash 引用未定义变量，不能开 -u
WS=/mnt/e/RM/tb3_gazebo_nav2
source /opt/ros/jazzy/setup.bash

export TURTLEBOT3_MODEL=waffle
export GZ_SIM_RESOURCE_PATH="/opt/ros/jazzy/share/nav2_minimal_tb3_sim/models:/opt/ros/jazzy/share/turtlebot3_gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"
export DISPLAY="${DISPLAY:-:0}"
export ROS_DOMAIN_ID=42

mkdir -p "$WS/logs"
cd "$WS"

echo "[run_nav2_tb3] ROS_DISTRO=$ROS_DISTRO  TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[run_nav2_tb3] 参数: $*"
exec ros2 launch nav2_bringup tb3_simulation_launch.py "$@"