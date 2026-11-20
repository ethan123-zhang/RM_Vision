#!/usr/bin/env bash
# 起 "Nav2 + RMUL2027 3V3 场地（翻转后）"（默认 headless）。
#   bash nav2_field.sh            # 无 GUI（验收用）
#   bash nav2_field.sh --gui      # 带 Gazebo GUI + RViz（看画面用）
# 2026-11-20 更新：场地已翻转（结构面朝上，地面 z=0，外墙内侧面 ±6.00/±4.00），
#   旧世界/旧地图（围挡变体、自由区=整块矩形）已失效，这里指向
#   worlds/rmul2027_3v3_up_nav2.sdf.xacro + maps/rmul2027_3v3_up.yaml。
#   地图由 capture_nav_map_up.sh / make_nav_map_up.py 从 48 块碰撞 box 生成。
# 关键点：
#   - 场地世界是 xacro（nav2 的 launch 会用 xacro 展成临时 SDF）
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

WORLD="$ROOT/worlds/rmul2027_3v3_up_nav2.sdf.xacro"
MAP="$ROOT/maps/rmul2027_3v3_up.yaml"
PARAMS=/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml
LOG="$ROOT/logs/nav2_field.log"
mkdir -p "$ROOT/logs"
[ -f "$WORLD" ] || { echo "缺 world: $WORLD"; exit 1; }
[ -f "$MAP" ]    || { echo "缺 map: $MAP（先跑 bash capture_nav_map.sh）"; exit 1; }

echo "=== 0. 清场 ==="
# 先 TERM 再 KILL：gz sim 有时不响应 SIGTERM，留着旧进程会变成两个仿真器同时发
# /clock /scan /tf（症状：local_costmap 一直报 "Invalid frame ID odom"、话题频率 0）。
for pat in 'ros2 launch nav2_bringup' 'gz sim' 'rviz2' 'parameter_bridge' \
           'component_container' 'robot_state_publisher'; do
  pkill -f "$pat" 2>/dev/null
done
sleep 3
for pat in 'ros2 launch nav2_bringup' 'gz sim' 'parameter_bridge' 'component_container'; do
  pkill -9 -f "$pat" 2>/dev/null
done
sleep 2
# 清掉上一个 participant 留下的 FastRTPS 共享内存端口文件（否则新栈可能抢不到端口）
rm -f /dev/shm/fastrtps_port* /dev/shm/fastrtps_* 2>/dev/null
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

echo "=== 5. 生命周期（直接调 /<node>/get_state 服务）==="
# 不要用 `ros2 lifecycle get`：它走 ros2 daemon 的**节点列表**，而本机 WSL 上 daemon 经常看不到
# composable 容器里的节点（`ros2 node list` 为空）→ 一律误报 "Node not found"，
# 但同一个节点的服务/动作是能发现的。所以直接调生命周期服务，取 current_state.id：
#   1=unconfigured  2=inactive  3=active  4=finalized
for n in /amcl /map_server /planner_server /controller_server /bt_navigator /behavior_server /velocity_smoother; do
  printf "  %-24s " "$n"
  out=""
  for i in $(seq 1 10); do
    id=$(timeout 6 ros2 service call "${n}/get_state" lifecycle_msgs/srv/GetState "{}" 2>/dev/null \
         | grep -oE 'id=[0-9]+' | head -1 | cut -d= -f2)
    if [ -n "$id" ]; then
      case "$id" in
        1) out="unconfigured" ;;
        2) out="inactive" ;;
        3) out="ACTIVE" ;;
        4) out="finalized" ;;
        *) out="id=$id" ;;
      esac
      break
    fi
    sleep 2
  done
  echo "${out:-无响应}"
done

echo "=== 5b. 等导航 action server（/navigate_to_pose）==="
NAVOK=0
for i in $(seq 1 30); do
  if timeout 5 ros2 action list 2>/dev/null | grep -qx '/navigate_to_pose'; then
    NAVOK=1; echo "  action server 就绪 (${i}x2s)"; break
  fi
  sleep 2
done
[ "$NAVOK" = 1 ] || echo "  警告: 60 s 内没看到 /navigate_to_pose —— 先别跑验收，看上面的生命周期与日志"

echo "=== 6. 近期报错 ==="
grep -iE 'Failed to|Aborting bringup|died' "$LOG" | grep -v 'Timed out waiting' | tail -8 || true
echo "OK: 栈已拉起。跑验收：ROS_DOMAIN_ID=42 python3 verify_nav2_field.py（脚本内部也默认 42）；停止用 stop_nav2_field.sh"