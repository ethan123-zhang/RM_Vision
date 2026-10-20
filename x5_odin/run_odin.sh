#!/usr/bin/env bash
# Odin1 ("hawk", 2207:0019) ROS2 driver launcher for RDK X5
# Headless: does NOT start rviz2 (the stock launch file would fail without X11).
# NOTE: do NOT use `set -u` here — ROS 2 setup.bash references unbound vars.
set -eo pipefail

source /opt/ros/humble/setup.bash
source "$HOME/odin_ws/install/setup.bash"

exec ros2 run odin_ros_driver host_sdk_sample \
  --ros-args \
  -p config_file:="$HOME/odin_ws/src/odin_ros_driver/config/control_command.yaml"
