#!/usr/bin/env bash
# 停掉场地仿真（GUI/headless 都适用）
pkill -f "gz sim" 2>/dev/null
sleep 1
if pgrep -f "gz sim" >/dev/null; then
  pkill -9 -f "gz sim" 2>/dev/null
  sleep 0.5
fi
echo "gz 进程: $(pgrep -af 'gz sim' | wc -l) 个残留"
echo "已停止"