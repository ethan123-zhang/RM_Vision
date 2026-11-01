#!/usr/bin/env bash
# 环境探测：OS / ROS2 / Gazebo / 图形加速
echo "=========== 1. 系统 ==========="
cat /etc/os-release | grep -E '^(NAME|VERSION|PRETTY_NAME)='
uname -r
echo "WSL_DISTRO_NAME=${WSL_DISTRO_NAME:-<none>}"
echo "WSL_INTEROP=${WSL_INTEROP:-<none>}"

echo
echo "=========== 2. ROS2 ==========="
echo "ROS_DISTRO=<${ROS_DISTRO:-未设置}>"
ls -d /opt/ros/*/ 2>/dev/null || echo "no /opt/ros"
for d in /opt/ros/*/; do
  [ -f "$d/setup.bash" ] || continue
  echo "--- $d"
  ( . "$d/setup.bash" >/dev/null 2>&1 && ros2 --version 2>&1 | head -2 )
done

echo
echo "=========== 3. Gazebo ==========="
command -v gz ign gazebo ros2 2>/dev/null
echo "--- gz sim --versions"
gz sim --versions 2>&1 | head -10
echo "--- gz --version"
gz --version 2>&1 | head -3
echo "--- ign gazebo --version"
ign gazebo --version 2>&1 | head -3
echo "--- gazebo (classic)"
gazebo --version 2>&1 | head -3
echo "--- dpkg gz/ign/gazebo 包"
dpkg -l 2>/dev/null | grep -iE 'gazebo|gz-|ignition|ros-.*-gazebo|ros-.*-ros-gz' | awk '{print $2, $3}' | head -40

echo
echo "=========== 4. 图形加速 ==========="
echo "DISPLAY=${DISPLAY:-<none>}  WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-<none>}"
echo "--- /dev/dri /dev/dxg"
ls -l /dev/dri /dev/dxg 2>&1 | head -20
echo "--- glxinfo -B"
glxinfo -B 2>&1 | head -20
echo "--- vulkaninfo --summary"
vulkaninfo --summary 2>&1 | grep -iE 'deviceName|driverName|apiVersion|GPU id' | head -12
echo "--- nvidia-smi -L"
nvidia-smi -L 2>&1 | head -5
echo "--- /proc/driver/nvidia"
ls /proc/driver/nvidia 2>&1 | head -3
echo "--- mesa 包"
dpkg -l 2>/dev/null | grep -iE 'mesa|d3d12|libgl1|vulkan' | awk '{print $2, $3}' | head -20

echo
echo "=========== 5. Nav2 / TB3 包 ==========="
for d in /opt/ros/*/; do
  [ -f "$d/setup.bash" ] || continue
  ( . "$d/setup.bash" >/dev/null 2>&1
    echo "--- pkgs in $d"
    ros2 pkg list 2>/dev/null | grep -iE 'nav2|turtlebot3|ros_gz|ros_gz_sim|slam_toolbox' | sort | head -60 )
done