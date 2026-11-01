#!/usr/bin/env bash
# 现场诊断：cmd_vel 订阅情况、RTF、模型位姿、物理是否在跑
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
echo "=== topics with cmd_vel ==="
gz topic -l 2>/dev/null | grep -i cmd_vel
echo
echo "=== topic info /model/waffle/cmd_vel ==="
gz topic -i -t /model/waffle/cmd_vel 2>&1 | head -20
echo
echo "=== world stats ==="
timeout 6 gz topic -e -t /world/rmul2027_3v3/stats -n 2 2>&1 | head -20
echo
echo "=== model list ==="
gz model --list 2>&1 | head -10
echo
echo "=== waffle pose (before) ==="
gz model -m waffle -p 2>&1 | head -10
echo
echo "=== publish twist via CLI 0.5 m/s for 2s ==="
for i in $(seq 1 20); do gz topic -t /model/waffle/cmd_vel -m gz.msgs.Twist -p "linear: {x: 0.5}" >/dev/null 2>&1; sleep 0.1; done
gz topic -t /model/waffle/cmd_vel -m gz.msgs.Twist -p "linear: {x: 0.0}" >/dev/null 2>&1
sleep 1
echo "=== waffle pose (after) ==="
gz model -m waffle -p 2>&1 | head -10
echo
echo "=== server log tail ==="
tail -25 /mnt/e/RM/tb3_gazebo_nav2/rm_field/logs/server_field.log