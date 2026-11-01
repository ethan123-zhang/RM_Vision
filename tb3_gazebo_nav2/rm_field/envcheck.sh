#!/usr/bin/env bash
# 环境自检：ROS / Gazebo / python 依赖 / 关键脚本是否可跑
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
echo "ROS_DISTRO=${ROS_DISTRO:-<empty>}"
gz sim --versions 2>/dev/null | head -2
python3 - <<'PY'
import numpy, PIL
print("numpy", numpy.__version__, "PIL", PIL.__version__)
import gz.transport13, gz.msgs10.image_pb2
print("gz python bindings ok")
PY
free -g | head -2
echo "nproc=$(nproc)"
ls -1 /mnt/e/RM/tb3_gazebo_nav2/rm_field/shots | head -3
echo "--- gz processes ---"
pgrep -af "gz sim" | head -5 || echo "none"