#!/usr/bin/env bash
# 把 Gazebo 窗口提到前台（WSLg 里最小化的窗口会被放到 -32000，激活后回到屏幕）
set +u
WIN=$(xdotool search --onlyvisible --name "Gazebo" | head -1)
if [ -z "$WIN" ]; then echo "no Gazebo window"; exit 1; fi
echo "win=$WIN"
xdotool windowactivate --sync "$WIN" 2>/dev/null || xdotool windowmap "$WIN"
xdotool windowraise "$WIN" 2>/dev/null
echo "geometry: $(xdotool getwindowgeometry "$WIN" | tr '\n' ' ')"
sleep 0.5