#!/usr/bin/env bash
# 诊断 WSLg 显示布局：X 屏幕尺寸、Gazebo 窗口位置、Windows 侧显示器对应关系。
set +u
export DISPLAY=:0
echo "=== X 屏幕 ==="
xdpyinfo 2>/dev/null | grep -E 'dimensions|depths' | head -3
echo "=== Gazebo 窗口 ==="
for w in $(xdotool search --onlyvisible --name Gazebo 2>/dev/null); do
  echo "id=$w $(xdotool getwindowgeometry $w | tr '\n' ' ')"
done
echo "=== 当前窗口管理器/屏幕数量 ==="
xrandr --query 2>/dev/null | head -8
echo "=== 尝试把窗口挪到 X=(2700,250) ==="
for w in $(xdotool search --onlyvisible --name Gazebo 2>/dev/null); do
  xdotool windowmove "$w" 2700 250
  sleep 0.5
  xdotool getwindowgeometry "$w"
done