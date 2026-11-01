#!/usr/bin/env bash
# 找 gz-sim/GUI 里 <gui><camera> 的用法示例
set +u
echo "=== 含 view_controller 的文件 ==="
grep -rl 'view_controller' /usr/share/gz /usr/share/gz-sim* /opt/ros/jazzy/share 2>/dev/null | head -8
echo
echo "=== 含 gui/camera 的 SDF 示例 ==="
grep -rl 'gui/camera' /usr/share /opt/ros/jazzy/share 2>/dev/null | head -8
echo
echo "=== 逐个打印 user_camera 片段 ==="
for f in $(grep -rl 'user_camera' /usr/share/gz /usr/share/gz-sim* /opt/ros/jazzy/share 2>/dev/null | head -4); do
  echo "== $f"
  grep -A12 'user_camera' "$f" | head -20
  echo
done
echo "=== gz gui 插件目录 ==="
ls /usr/lib/x86_64-linux-gnu/gz-gui-8/plugins 2>/dev/null | head -20