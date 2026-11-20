#!/usr/bin/env bash
# 杀掉所有【地图点选】窗口实例。单独成文件是为了避免 pkill 模式命中调用者自己的命令行
# （把 pkill 直接写进 wsl -lc 那一长串里，模式会匹配到 shell 自身，表现为"命令静默退出 1"）。
set +u
pkill -f 'python3 map_click[.]py' 2>/dev/null
sleep 1
pkill -9 -f 'python3 map_click[.]py' 2>/dev/null
sleep 0.5
echo "剩余实例: $(pgrep -cf 'python3 map_click[.]py' 2>/dev/null || echo 0)"