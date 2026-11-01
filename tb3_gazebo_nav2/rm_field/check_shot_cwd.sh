#!/usr/bin/env bash
# 查 gz gui 的 cwd / 找 /gui/screenshot 的输出文件
set +u
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
echo "=== 进程 cwd ==="
for p in $(pgrep -f 'gz sim'); do
  d=$(readlink /proc/$p/cwd 2>/dev/null)
  echo "$p cwd=$d"
  ls -la "$d" 2>/dev/null | grep -iE '\.png$' | head -5
done
echo "=== rm_field 下最近 png ==="
ls -lat "$ROOT/shots" | head -10
echo "=== 最近 5 分钟内改动的 png（限定几个目录）==="
find "$ROOT" /tmp /root -maxdepth 3 -name '*.png' -newermt '-5 minutes' 2>/dev/null | head -10