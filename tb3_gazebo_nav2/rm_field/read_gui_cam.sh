#!/usr/bin/env bash
# 读回 gz GUI 当前相机位姿（判断 <gui><camera> 是否生效）
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
echo "=== /gui/camera/pose 回读 (3s) ==="
timeout 3 gz topic -e -t /gui/camera/pose 2>&1 | head -30
echo "=== exit=$? ==="