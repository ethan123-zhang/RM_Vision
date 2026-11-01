#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# 一键 bringup：拉起 Nav2 官方 TurtleBot3 示例 → 发布初始位姿 → 等导航栈全 active
# 日志: logs/nav2_tb3_gui.log
# ---------------------------------------------------------------------------
WS=/mnt/e/RM/tb3_gazebo_nav2
set +u
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export DISPLAY="${DISPLAY:-:0}"
cd "$WS" || exit 1
mkdir -p logs
LOG="$WS/logs/nav2_tb3_gui.log"

# ---- 0. 清场 ----
pkill -f 'ros2 launch nav2_bringup' 2>/dev/null
pkill -f 'gz sim' 2>/dev/null
pkill -f 'rviz2' 2>/dev/null
pkill -f 'parameter_bridge' 2>/dev/null
pkill -f 'component_container' 2>/dev/null
pkill -f 'robot_state_publisher' 2>/dev/null
sleep 3
echo "[bringup] 残留进程: $(pgrep -fc 'gz sim|ros2 launch|rviz2' || true)"

# ---- 1. 启动 ----
: > "$LOG"
setsid nohup bash "$WS/run_nav2_tb3.sh" headless:=False use_rviz:=True > "$LOG" 2>&1 < /dev/null &
echo "[bringup] 已拉起 launch (日志 $LOG)"

# ---- 2. 等仿真话题就绪（/clock 与 /scan 有数据）----
ready=0
for i in $(seq 1 60); do
  if timeout 3 ros2 topic list 2>/dev/null | grep -qx '/scan' \
     && timeout 3 ros2 topic list 2>/dev/null | grep -qx '/clock'; then
    ready=1; echo "[bringup] 仿真话题就绪，用时 ${i}s"; break
  fi
  sleep 1
done
[ "$ready" = 1 ] || { echo "[bringup] 致命: 仿真话题未就绪"; tail -20 "$LOG"; exit 1; }
sleep 4

# ---- 3. 发布初始位姿（等价 RViz 的 2D Pose Estimate）----
#    机器人 spawn 在 (-2.0, -0.5)，朝向 +x
POSE='{header: {frame_id: map}, pose: {pose: {position: {x: -2.0, y: -0.5, z: 0.0}, orientation: {z: 0.0, w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0.25,0,0,0, 0,0,0,0.0685,0,0, 0,0,0,0,0.0685,0, 0,0,0,0,0,0.0685]}}'
for i in 1 2 3; do
  timeout 10 ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "$POSE" >/dev/null 2>&1 \
    && echo "[bringup] 已发布 /initialpose (第 ${i} 次)"
  sleep 2
done

# ---- 4. 等 map -> odom TF ----
echo "[bringup] 等待 map->odom 变换..."
for i in $(seq 1 30); do
  if timeout 6 ros2 run tf2_ros tf2_echo map odom 2>/dev/null | grep -q 'Translation'; then
    echo "[bringup] map->odom 已可用 (${i}s)"; break
  fi
  sleep 1
done

# ---- 5. 状态汇总 ----
echo
echo "=================== 生命周期 ==================="
for n in /amcl /map_server /planner_server /controller_server /bt_navigator /behavior_server /smoother_server /velocity_smoother /waypoint_follower /lifecycle_manager_navigation /lifecycle_manager_localization; do
  printf "  %-28s " "$n"
  timeout 5 ros2 lifecycle get "$n" 2>&1 | head -1
done
echo
echo "=================== bringup 报错（若有） ==================="
grep -iE 'Failed to|Aborting bringup|died' "$LOG" | grep -v 'Timed out waiting' | tail -10 || true