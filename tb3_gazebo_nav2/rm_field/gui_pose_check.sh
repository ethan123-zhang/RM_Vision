#!/usr/bin/env bash
# 读 GUI 自己发布的相机位姿（/gui/camera/pose 是 GUI 的输出，不是设置入口），
# 用来确认 --gui-config 里的 camera_pose 真的生效了。
set +u
export DISPLAY=:0
echo "=== /gui/camera/pose（GUI 当前视角，取 1 条） ==="
timeout 15 gz topic -e -t /gui/camera/pose -n 1 2>&1 | head -20
echo
echo "=== 期望位姿（config/gui_field.config） ==="
grep -o '<camera_pose>[^<]*</camera_pose>' /mnt/e/RM/tb3_gazebo_nav2/rm_field/config/gui_field.config