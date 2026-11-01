#!/usr/bin/env bash
# 验证 Nav2 官方 TB3 示例运行状态（话题 / 生命周期 / TF / 帧率 / CPU）
WS=/mnt/e/RM/tb3_gazebo_nav2
set +u
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42

echo "=================== 节点 ==================="
ros2 node list 2>/dev/null | sort | sed 's/^/  /'
echo "节点数: $(ros2 node list 2>/dev/null | wc -l)"

echo
echo "=================== 话题频率 ==================="
for t in /clock /scan /odom /cmd_vel; do
  echo -n "  $t : "
  timeout -s INT 6 ros2 topic hz "$t" 2>/dev/null | grep -m1 average || echo "(6s 内无数据)"
done

echo
echo "=================== Nav2 生命周期 ==================="
for n in /amcl /bt_navigator /controller_server /planner_server /map_server /behavior_server; do
  printf "  %-20s " "$n"
  timeout 5 ros2 lifecycle get "$n" 2>&1 | head -1
done

echo
echo "=================== TF 树（关键边） ==================="
timeout -s INT 8 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -m2 -E 'At time|Translation' || echo "  map->odom 暂无（AMCL 未设初始位姿）"
echo "  --- 静态/动态 frame 列表:"
timeout -s INT 5 ros2 topic echo /tf_static --once 2>/dev/null | grep -oE 'frame_id: [a-z_]+' | sort -u | head -10

echo
echo "=================== Gazebo 侧 ==================="
echo "  gz topic 数量: $(gz topic -l 2>/dev/null | wc -l)"
gz topic -l 2>/dev/null | grep -E '/scan|/odom|/cmd_vel|/clock|/camera|/image' | head -10
echo "  --- 仿真物理步/实时因子:"
timeout -s INT 6 gz topic -e -t /world/tb3_sandbox/stats 2>/dev/null | grep -m3 -E 'real_time_factor|iterations' || true

echo
echo "=================== 资源占用（前 8） ==================="
ps -eo pcpu,pmem,rss,comm --sort=-pcpu 2>/dev/null | head -9
echo "  --- 内存:"; free -h | head -2