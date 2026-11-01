#!/usr/bin/env bash
# 把 Gazebo 窗口放到固定位置/大小并提到前台（配合 show_gazebo.ps1 抓图）
set +u
WIN=$(xdotool search --onlyvisible --name "Gazebo" | head -1)
if [ -z "$WIN" ]; then echo "no Gazebo window"; exit 1; fi
W=${1:-1180}
H=${2:-980}
xdotool windowsize "$WIN" "$W" "$H" 2>/dev/null
sleep 0.5
xdotool windowmove "$WIN" 20 20 2>/dev/null
xdotool windowactivate "$WIN" 2>/dev/null
xdotool windowraise "$WIN" 2>/dev/null
sleep 0.8
echo "win=$WIN  $(xdotool getwindowgeometry "$WIN" | tr '\n' ' ')"