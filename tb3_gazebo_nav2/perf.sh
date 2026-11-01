#!/usr/bin/env bash
# 性能与实时因子测量（软件渲染下是否跑得动）
set +u
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42

t() { timeout 5 ros2 topic echo /clock --once 2>/dev/null | awk '/sec:/ {print $2; exit}'; }
c1=$(t); sleep 10; c2=$(t)
echo "仿真时钟: ${c1}s -> ${c2}s"
awk -v a="$c1" -v b="$c2" 'BEGIN{ if (b>a) printf "实时因子 RTF ≈ %.2f (1.00 = 实时)\n", (b-a)/10.0; else print "RTF 读取失败" }'

echo
echo "=== 关键话题频率 ==="
for topic in /clock /scan /odom /tf; do
  printf "  %-8s " "$topic"
  timeout -s INT 5 ros2 topic hz "$topic" 2>/dev/null | grep -m1 average || echo "无数据"
done

echo
echo "=== 渲染进程 CPU（llvmpipe 软件光栅）==="
ps -eo pid,pcpu,pmem,etime,args --sort=-pcpu 2>/dev/null | grep -E 'gz sim|rviz2|ruby' | grep -v grep | head -5 | cut -c1-110

echo
echo "=== Gazebo GUI 帧统计 ==="
gz topic -l 2>/dev/null | grep -E 'stats|gui' | head -5
timeout -s INT 6 gz topic -e -t /world/default/stats -n 2 2>/dev/null | grep -E 'real_time_factor|sim_time|iterations' | head -5 || echo "  (无 /world/default/stats，世界名为 tb3_sandbox 时跳过)"