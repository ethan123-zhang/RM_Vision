#!/usr/bin/env bash
# 快速判别：把"结构网格碰撞"去掉，只留地面 box，看机器人能不能走
set +u
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
export GZ_SIM_RESOURCE_PATH="$ROOT/models:/opt/ros/jazzy/share/turtlebot3_gazebo/models"
export GZ_WORLD=drive
cd "$ROOT" || exit 1
mkdir -p logs
pkill -f "gz sim" 2>/dev/null
sleep 1.5
: > logs/probe_drive.log
nohup gz sim -s -r worlds/drive_colored.sdf > logs/probe_drive.log 2>&1 &
for i in $(seq 1 60); do
  gz service -l 2>/dev/null | grep -q '/world/drive/create' && { echo "server ready ${i}s"; break; }
  sleep 1
done
sleep 2
python3 spawn_model.py waffle "" 0 0 0.10 0.0 | tail -1
sleep 2
echo "--- 发前进 0.15 m/s 共 2 s ---"
gz topic -t /cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.15}'
sleep 2
gz topic -t /cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.0}'
sleep 1
echo "--- waffle 位姿 ---"
timeout 6 gz topic -e -t /world/drive/pose/info -n 1 2>/dev/null | grep -A6 'name: "waffle"' | head -8
pkill -f "gz sim" 2>/dev/null
sleep 1
echo done