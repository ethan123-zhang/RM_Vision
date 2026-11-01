#!/usr/bin/env bash
# 列出与 GUI 相机相关的 topic/service
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
echo "=== topics ==="
gz topic -l 2>/dev/null | head -30
echo
echo "=== services ==="
gz service -l 2>/dev/null | head -30
echo
echo "=== 含 gui 的 ==="
gz topic -l 2>/dev/null | grep -i gui
gz service -l 2>/dev/null | grep -i gui