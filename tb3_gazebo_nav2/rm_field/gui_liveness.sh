#!/usr/bin/env bash
# 判断 GUI 3D 视图是否在刷新：让机器人走 3 秒，比较视口区域变化
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
cd "$ROOT" || exit 1
WIN=$(xdotool search --onlyvisible --name "Gazebo" | head -1)
echo "win=$WIN"
import -window "$WIN" shots/gui_before.png
for i in $(seq 1 30); do
  gz topic -t /cmd_vel -m gz.msgs.Twist -p "linear: {x: 0.4}" >/dev/null 2>&1
  sleep 0.1
done
gz topic -t /cmd_vel -m gz.msgs.Twist -p "linear: {x: 0.0}" >/dev/null 2>&1
sleep 1
import -window "$WIN" shots/gui_after.png
python3 - <<'PY'
import numpy as np
from PIL import Image
a = np.asarray(Image.open("shots/gui_before.png").convert("RGB")).astype(float)
b = np.asarray(Image.open("shots/gui_after.png").convert("RGB")).astype(float)
print("shape", a.shape, b.shape)
h, w, _ = a.shape
vp = (slice(int(h*0.10), int(h*0.92)), slice(0, int(w*0.58)))
d = np.abs(a - b).sum(axis=2)
dv = d[vp]
print("视口区: 变化像素(>30)占比 %.2f%%   平均色差 %.2f" %
      (100.0*(dv > 30).mean(), dv.mean()))
print("整窗:   变化像素(>30)占比 %.2f%%" % (100.0*(d > 30).mean()))
PY