#!/usr/bin/env bash
# 起 "Nav2 + RMUL2027 3V3 场地"（默认 headless）。
#   bash nav2_field.sh            # 无 GUI（验收用）
#   bash nav2_field.sh --gui      # 带 Gazebo GUI + RViz（看画面用）
# 关键点：
#   - 场地世界是 xacro（nav2 的 launch 会用 xacro 展成临时 SDF）
#   - 地图用 maps/rmul2027_3v3_fenced.yaml（围挡内侧面以内为自由区）
#   - AMCL 不给初始位姿会让 planner_server 起不来 → 这里按 spawn 点 (0,0) 发三次
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
export ROS_DOMAIN_ID=42
export TURTLEBOT3_MODEL=waffle
export GZ_SIM_RESOURCE_PATH="$ROOT/models:/opt/ros/jazzy/share/turtlebot3_gazebo/models:/opt/ros/jazzy/share/nav2_minimal_tb3_sim/models"
export DISPLAY="${DISPLAY:-:0}"

GUI=0
[ "${1:-}" = "--gui" ] && GUI=1
if [ "$GUI" = 1 ]; then HL=False; RV=True; else HL=True; RV=False; fi

WORLD="$ROOT/worlds/rmul2027_3v3_nav2.sdf.xacro"
MAP="$ROOT/maps/rmul2027_3v3_fenced.yaml"
PARAMS=/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml
LOG="$ROOT/logs/nav2_field.log"
mkdir -p "$ROOT/logs"
[ -f "$WORLD" ] || { echo "缺 world: $WORLD"; exit 1; }
[ -f "$MAP" ]    || { echo "缺 map: $MAP（先跑 bash capture_nav_map.sh）"; exit 1; }

echo "=== 0. 清场 ==="
pkill -f 'ros2 launch nav2_bringup' 2>/dev/null
pkill -f 'gz sim' 2>/dev/null
pkill -f 'rviz2' 2>/dev/null
pkill -f 'parameter_bridge' 2>/dev/null
pkill -f 'component_container' 2>/dev/null
pkill -f 'robot_state_publisher' 2>/dev/null
sleep 3
echo "残留: $(pgrep -fc 'gz sim|ros2 launch|rviz2' 2>/dev/null || echo 0)"

echo "=== 1. 拉起 launch（headless=$HL use_rviz=$RV）==="
: > "$LOG"
setsid nohup ros2 launch nav2_bringup tb3_simulation_launch.py \
  world:="$WORLD" map:="$MAP" params_file:="$PARAMS" \
  headless:="$HL" use_rviz:="$RV" use_simulator:=True use_robot_state_pub:=True \
  x_pose:=0.0 y_pose:=0.0 z_pose:=0.05 yaw:=0.0 \
  > "$LOG" 2>&1 < /dev/null &
echo "日志: $LOG"

echo "=== 2. 等 /clock 与 /scan ==="
ready=0
for i in $(seq 1 90); do
  if timeout 3 ros2 topic list 2>/dev/null | grep -qx '/clock' \
     && timeout 3 ros2 topic list 2>/dev/null | grep -qx '/scan'; then
    ready=1; echo "话题就绪 ${i}s"; break
  fi
  sleep 1
done
[ "$ready" = 1 ] || { echo "致命: 仿真话题未就绪"; tail -25 "$LOG"; exit 1; }
sleep 4

echo "=== 3. 发初始位姿 (0,0) ==="
POSE='{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0.25,0,0,0, 0,0,0,0.0685,0,0, 0,0,0,0,0.0685,0, 0,0,0,0,0,0.0685]}}'
for i in 1 2 3; do
  timeout 10 ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "$POSE" >/dev/null 2>&1 \
    && echo "  第 ${i} 次已发布"
  sleep 2
done

echo "=== 4. 等 map->odom ==="
got=0
for i in $(seq 1 30); do
  if timeout 6 ros2 run tf2_ros tf2_echo map odom 2>/dev/null | grep -q 'Translation'; then
    got=1; echo "map->odom 可用 (${i}s)"; break
  fi
  sleep 1
done
[ "$got" = 1 ] || echo "警告: map->odom 仍不可用"

echo "=== 5. 生命周期 ==="
for n in /amcl /map_server /planner_server /controller_server /bt_navigator /behavior_server /velocity_smoother; do
  printf "  %-24s " "$n"
  timeout 5 ros2 lifecycle get "$n" 2>&1 | head -1
done
echo "=== 6. 近期报错 ==="
grep -iE 'Failed to|Aborting bringup|died' "$LOG" | grep -v 'Timed out waiting' | tail -8 || true
echo "OK: 栈已拉起，用 python3 verify_nav2_field.py 跑导航验收；停止用 stop_nav2_field.sh"