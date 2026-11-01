#!/usr/bin/env bash
# 直接从 X 侧抓 Gazebo 窗口内容（不抢 Windows 焦点，不影响用户）
set +u
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
OUT="${1:-$ROOT/shots/gui_view_x.png}"
which import xwd convert 2>/dev/null
WIN=$(xdotool search --onlyvisible --name "Gazebo" | head -1)
echo "win=$WIN"
if [ -z "$WIN" ]; then echo "没找到 Gazebo 窗口"; exit 1; fi
if command -v import >/dev/null; then
  import -window "$WIN" "$OUT" && echo "import ok"
elif command -v xwd >/dev/null; then
  xwd -id "$WIN" -out /tmp/win.xwd && convert /tmp/win.xwd "$OUT" && echo "xwd ok"
else
  echo "既没有 import 也没有 xwd"; exit 2
fi
ls -la "$OUT" 2>/dev/null