#!/usr/bin/env bash
# 把 WSLg 里的 Gazebo 窗口挪进屏幕并放大，方便 Windows 侧抓图。
# 用法: bash onscreen_gui.sh [x y w h]
set +u
X=${1:-80}; Y=${2:-60}; W=${3:-1100}; H=${4:-880}
export DISPLAY=:0
ids=$(xdotool search --onlyvisible --name Gazebo 2>/dev/null)
if [ -z "$ids" ]; then echo "no Gazebo window"; exit 1; fi
for w in $ids; do
  echo "win $w before: $(xdotool getwindowgeometry $w | tr '\n' ' ')"
  xdotool windowmove "$w" "$X" "$Y"
  xdotool windowsize "$w" "$W" "$H"
  xdotool windowactivate --sync "$w" 2>/dev/null || xdotool windowactivate "$w"
  sleep 0.6
  echo "win $w after : $(xdotool getwindowgeometry $w | tr '\n' ' ')"
done