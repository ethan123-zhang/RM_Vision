#!/usr/bin/env bash
# 检查 gz GUI 是否还活着 + 窗口列表 + 日志尾部
set +u
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
echo "=== gz 进程 ==="
pgrep -af 'gz sim' | head -5
echo "=== 窗口列表 ==="
for w in $(xdotool search --onlyvisible --name "." 2>/dev/null); do
  printf '%s | %s | %s\n' "$w" "$(xdotool getwindowname "$w" 2>/dev/null)" \
    "$(xdotool getwindowgeometry "$w" 2>/dev/null | tr '\n' ' ')"
done | head -12
echo "=== gui log tail ==="
tail -15 "$ROOT/logs/gui_field.log"