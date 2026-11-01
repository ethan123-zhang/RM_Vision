#!/usr/bin/env bash
# 后台拉起 Nav2 官方 TurtleBot3 示例，并脱离 wsl.exe 调用会话（setsid）
# 日志: logs/nav2_tb3_gui.log
WS=/mnt/e/RM/tb3_gazebo_nav2
cd "$WS" || exit 1
mkdir -p logs
LOG="$WS/logs/nav2_tb3_gui.log"
: > "$LOG"
setsid nohup bash "$WS/run_nav2_tb3.sh" headless:=False use_rviz:=True > "$LOG" 2>&1 < /dev/null &
echo "launcher_pid=$!"
sleep 3
pgrep -af 'gz sim|ros2 launch|rviz2' | head -10