#!/usr/bin/env bash
# 从 X 侧直接抓 Gazebo 窗口（WSLg 下可能拿到的是附近帧，但内容有效）。
# 用法: bash grab_x11.sh [out.png]
set +u
export DISPLAY=:0
OUT="${1:-/mnt/e/RM/tb3_gazebo_nav2/rm_field/shots/gui_x11.png}"
W=$(xdotool search --onlyvisible --name Gazebo 2>/dev/null | head -1)
echo "window id=$W"
if [ -z "$W" ]; then echo "no window"; exit 1; fi
xdotool windowactivate "$W" 2>/dev/null
sleep 1
import -window "$W" "$OUT" 2>&1 || xwd -id "$W" -out /tmp/gui.xwd
if [ -f /tmp/gui.xwd ]; then convert /tmp/gui.xwd "$OUT"; fi
ls -l "$OUT"