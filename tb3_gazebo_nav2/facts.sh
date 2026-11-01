#!/usr/bin/env bash
# 汇总最终事实（版本 / 加速 / 运行状态），用于写运行记录
set +u
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42

echo "== 系统"; grep -E '^PRETTY_NAME' /etc/os-release | tr -d '"'; echo "kernel: $(uname -r)"
echo "== ROS 2"; echo "ROS_DISTRO=$ROS_DISTRO"
dpkg -l ros-jazzy-rclcpp ros-jazzy-ros-base ros-jazzy-rmw-fastrtps-cpp 2>/dev/null | awk '/^ii/{print "  "$2, $3}'
echo "== Gazebo"
gz sim --versions | sed 's/^/  gz-sim /'
dpkg -l gz-harmonic gz-sim8-cli 2>/dev/null | awk '/^ii/{print "  "$2, $3}'
echo "== Nav2 / TurtleBot3"
dpkg -l ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-minimal-tb3-sim \
       ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-navigation2 2>/dev/null \
  | awk '/^ii/{print "  "$2, $3}'
echo "== 图形加速"
glxinfo -B 2>/dev/null | grep -E 'OpenGL renderer|OpenGL version|Accelerated|Device:'
vulkaninfo --summary 2>/dev/null | grep -E 'deviceName|driverName' | head -2 | sed 's/^/  /'
echo "  /dev/dri: $(ls /dev/dri 2>&1)"
echo "  nvidia-smi: $(nvidia-smi -L 2>/dev/null)"
echo "  GL 实现: $(ls /usr/lib/x86_64-linux-gnu/dri/ | grep -c .) 个 DRI 驱动，d3d12_dri.so $([ -f /usr/lib/x86_64-linux-gnu/dri/d3d12_dri.so ] && echo 存在 || echo 缺失)"
echo "== 运行状态"
echo "  节点数: $(ros2 node list 2>/dev/null | wc -l)"
printf "  /scan: "; timeout -s INT 5 ros2 topic hz /scan 2>/dev/null | grep -m1 average
printf "  /odom: "; timeout -s INT 5 ros2 topic hz /odom 2>/dev/null | grep -m1 average
printf "  /tf  : "; timeout -s INT 5 ros2 topic hz /tf 2>/dev/null | grep -m1 average
echo "  实时因子: $(timeout -s INT 6 gz topic -e -t /world/default/stats -n 3 2>/dev/null | grep -m1 real_time_factor)"
echo "  GUI 窗口:"; xdotool search --onlyvisible --name '.' 2>/dev/null | while read -r w; do
  echo "    $(xdotool getwindowname "$w" 2>/dev/null) | $(xdotool getwindowgeometry --shell "$w" 2>/dev/null | grep -E 'WIDTH|HEIGHT' | tr '\n' ' ')"
done
echo "  进程 CPU: $(ps -eo pcpu,args --sort=-pcpu 2>/dev/null | grep -E 'gz sim|rviz2' | grep -v grep | head -3 | tr '\n' ';')"