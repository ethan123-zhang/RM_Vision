#!/usr/bin/env bash
# 检查 GUI 进程是否在渲染（CPU）+ 机器人到底动没动
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
HZ=$(getconf CLK_TCK)
GUIPID=$(pgrep -f 'gz sim gui' | head -1)
SRVPID=$(pgrep -f 'gz sim server' | head -1)
echo "gui=$GUIPID server=$SRVPID"
for pid in $GUIPID $SRVPID; do
  a=$(awk '{print $14+$15}' /proc/$pid/stat)
  sleep 3
  b=$(awk '{print $14+$15}' /proc/$pid/stat)
  echo "pid=$pid cpu=$(echo "scale=1; ($b-$a)*100/(3*$HZ)" | bc)%"
done
echo "=== 机器人位姿 ==="
timeout 2 gz topic -e -t /world/rmul2027_3v3/pose/info 2>/dev/null | grep -A4 'name: "waffle"' | head -8
echo "=== 机器人是否还在 (topic) ==="
gz topic -l 2>/dev/null | grep -c waffle