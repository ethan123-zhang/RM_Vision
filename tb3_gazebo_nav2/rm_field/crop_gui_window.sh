#!/usr/bin/env bash
# 从整块虚拟显示器截图里裁出 Gazebo 窗口，并做 ASCII/亮度检查。
# 用法: bash crop_gui_window.sh <monitor_png> <x> <y> <w> <h>
set +u
cd /mnt/e/RM/tb3_gazebo_nav2/rm_field || exit 1
MON="${1:-shots/monitor1_full.png}"
X=${2:-373}; Y=${3:-58}; W=${4:-672}; H=${5:-558}
python3 - "$MON" "$X" "$Y" "$W" "$H" <<'PY'
import sys
from PIL import Image
mon, x, y, w, h = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5])
im = Image.open(mon).convert("RGB")
crop = im.crop((x, y, x + w, y + h))
out = "shots/gui_window_crop.png"
crop.save(out)
print("crop ->", out, crop.size)
PY
python3 crop_ascii.py shots/gui_window_crop.png 0 0 "$W" "$H" 84 2>&1 | tail -26