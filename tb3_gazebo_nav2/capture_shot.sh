#!/usr/bin/env bash
# 抓取 Gazebo / RViz 窗口图像。用法: bash capture_shot.sh <tag>
WS=/mnt/e/RM/tb3_gazebo_nav2
export DISPLAY="${DISPLAY:-:0}"
TAG="${1:-shot}"
mkdir -p "$WS/shots"
grab() {
  local pattern="$1" out="$2"
  local id
  id=$(xdotool search --onlyvisible --name "$pattern" 2>/dev/null | head -1)
  if [ -z "$id" ]; then echo "  未找到窗口: $pattern"; return 1; fi
  import -display :0 -window "$id" "$WS/shots/$out" 2>/dev/null \
    && echo "  $out <- win $id ($(stat -c %s "$WS/shots/$out") bytes)" \
    || echo "  抓图失败: $out"
}
grab 'Gazebo Sim' "${TAG}_gazebo.png"
grab 'RViz'       "${TAG}_rviz.png"
echo "--- /odom 复查:"
set +u; source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=42
timeout -s INT 5 ros2 topic hz /odom 2>/dev/null | grep -m1 average || echo "  /odom 5s 内无数据"
timeout -s INT 5 ros2 topic echo /odom --once 2>/dev/null | grep -A2 'position:' | head -4