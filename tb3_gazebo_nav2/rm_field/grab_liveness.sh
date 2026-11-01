#!/usr/bin/env bash
# 隔几秒抓两次，比较差异，确认抓到的不是冻结帧（WSLg 下 import 可能拿到旧帧）。
set +u
export DISPLAY=:0
D=/mnt/e/RM/tb3_gazebo_nav2/rm_field/shots
W=$(xdotool search --onlyvisible --name Gazebo 2>/dev/null | head -1)
echo "window id=$W"
xdotool windowactivate "$W" 2>/dev/null; sleep 1
import -window "$W" "$D/gui_live_a.png"
sleep 3
import -window "$W" "$D/gui_live_b.png"
python3 - <<'PY'
import numpy as np
from PIL import Image
a = np.asarray(Image.open('/mnt/e/RM/tb3_gazebo_nav2/rm_field/shots/gui_live_a.png').convert('RGB')).astype(float)
b = np.asarray(Image.open('/mnt/e/RM/tb3_gazebo_nav2/rm_field/shots/gui_live_b.png').convert('RGB')).astype(float)
d = np.abs(a - b).mean()
frac = (np.abs(a - b).max(axis=2) > 8).mean()
print(f"size {a.shape[1]}x{a.shape[0]}  mean|diff|={d:.3f}  变化像素占比={frac:.2%}")
lum = 0.299 * a[:, :, 0] + 0.587 * a[:, :, 1] + 0.114 * a[:, :, 2]
B = (lum >= 150) & (lum < 236)
rows = np.nonzero(B.mean(axis=1) > 0.05)[0]
if len(rows):
    print(f"台面亮带行范围 y[{rows.min()},{rows.max()}]  居中行={a.shape[0]/2:.0f}")
PY