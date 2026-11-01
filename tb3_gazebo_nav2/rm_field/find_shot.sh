#!/usr/bin/env bash
# 找 gui/screenshot 到底存到哪了
set +u
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
echo "=== 最近 15 分钟内的 png ==="
find /mnt/e/RM/tb3_gazebo_nav2/rm_field /tmp /root /home -maxdepth 3 -name '*.png' -newermt '-15 minutes' 2>/dev/null | head -20
echo "=== 全盘 gui_view.png ==="
find / -maxdepth 5 -name 'gui_view*.png' 2>/dev/null | head -5
echo "=== gz 进程 cwd ==="
for p in $(pgrep -f 'gz sim'); do
  echo "$p -> $(readlink /proc/$p/cwd)  cmd=$(tr '\0' ' ' < /proc/$p/cmdline | cut -c1-90)"
done