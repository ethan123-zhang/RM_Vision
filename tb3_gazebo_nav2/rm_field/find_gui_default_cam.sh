#!/usr/bin/env bash
# 找 GUI 相机默认位姿来源：~/.gz/sim/8/gui.config 等
set +u
echo "=== ~/.gz 目录 ==="
ls -la ~/.gz/sim/8/ 2>/dev/null | head -10
echo "=== gui.config 里的 camera/pose ==="
if [ -f ~/.gz/sim/8/gui.config ]; then
  grep -n -iE 'camera|pose|view' ~/.gz/sim/8/gui.config | head -30
else
  echo "没有 gui.config"
fi
echo "=== gz-gui8 默认配置里的 -6 / pose ==="
grep -rn -- '-6 ' /usr/share/gz/gz-gui8/ 2>/dev/null | head -5
ls /usr/share/gz/gz-gui8/ 2>/dev/null | head
echo "=== MinimalScene 插件默认 ==="
strings /usr/lib/x86_64-linux-gnu/gz-gui-8/plugins/libMinimalScene.so 2>/dev/null | grep -iE 'user_camera|pose' | head -10