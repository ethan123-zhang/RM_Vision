#!/usr/bin/env bash
# 1) 看 gz-sim 自带世界怎么配 GUI 相机  2) 看 /gui/move_to/pose 的类型
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
echo "=== gz-sim8 自带 worlds ==="
ls /usr/share/gz/gz-sim8/worlds 2>/dev/null | head -5
echo "=== 自带 world 里的 gui/camera 写法 ==="
for f in /usr/share/gz/gz-sim8/worlds/*.sdf; do
  if grep -q '<gui' "$f" 2>/dev/null; then
    echo "-- $f"; grep -A8 '<gui' "$f" | head -12; echo
  fi
done | head -40
echo "=== GUI 插件配置里的默认相机 ==="
grep -rl 'user_camera' /usr/share/gz/gz-gui8 2>/dev/null | head -3
echo "=== /gui/move_to/pose 服务信息 ==="
gz service -i /gui/move_to/pose 2>&1 | head -20
echo "=== /gui/camera/view_control 服务信息 ==="
gz service -i /gui/camera/view_control 2>&1 | head -20