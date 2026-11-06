#!/usr/bin/env bash
# 跑 Nav2 导航验收（前置：bash nav2_field.sh 已拉起）
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=42
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
cd "$ROOT" || exit 1
python3 verify_nav2_field.py
RC=$?
exit $RC