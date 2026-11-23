#!/usr/bin/env bash
# S1b: one-shot start/stop for the sentry simulation + Nav2 (no AMCL).
#   ./run_sentry_nav.sh [gui:=true] [map:=/abs/other.yaml]   start, wait for OK
#   ./run_sentry_nav.sh stop                                  stop this domain
#   ./run_sentry_nav.sh status                                lifecycle / actions
# Same ROS_DOMAIN_ID discipline as run_sentry_sim.sh: only processes whose
# /proc/<pid>/environ says this domain are touched.
set -uo pipefail

WS="${HOME}/sentry_ws"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-45}"
LOG="${SENTRY_NAV_LOG:-/tmp/sentry_nav/nav.log}"

PATTERN='gz sim|gz-sim-server|ros2 launch sentry|parameter_bridge|ros_gz_sim|odin1_sim_bridge|pointcloud_to_laserscan|robot_state_publisher|map_click\.py|nav2_|map_server|controller_server|planner_server|bt_navigator|behavior_server|smoother_server|waypoint_follower|velocity_smoother|collision_monitor|route_server|docking_server|lifecycle_manager'

domain_pids() {
  local p d
  for p in $(pgrep -f "$PATTERN" 2>/dev/null); do
    [ "$p" = "$$" ] && continue
    d=$(tr '\0' '\n' < "/proc/$p/environ" 2>/dev/null | sed -n 's/^ROS_DOMAIN_ID=//p' | head -1)
    [ -z "$d" ] && continue
    [ "$d" = "$ROS_DOMAIN_ID" ] && echo "$p"
  done
}

cleanup() {
  local pids sig
  pids=$(domain_pids)
  if [ -n "$pids" ]; then
    echo "[run_sentry_nav] TERM domain ${ROS_DOMAIN_ID}: $(echo "$pids" | tr '\n' ' ')"
    for sig in TERM KILL; do
      for p in $pids; do kill -"$sig" "$p" 2>/dev/null || true; done
      sleep 3
      pids=$(domain_pids)
      [ -z "$pids" ] && break
    done
  fi
  sleep 1
  echo "[run_sentry_nav] remaining in domain ${ROS_DOMAIN_ID}: $(domain_pids | tr '\n' ' ')"
}

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "${HOME}/lidar_ws/install/setup.bash"
# shellcheck disable=SC1091
source "${WS}/install/setup.bash"
set -u

if [ "${1:-}" = "stop" ]; then
  cleanup
  exit 0
fi

if [ "${1:-}" = "status" ]; then
  echo "[run_sentry_nav] domain ${ROS_DOMAIN_ID} nodes: $(ros2 node list 2>/dev/null | wc -l)"
  ros2 action list -t 2>/dev/null | grep navigate_to_pose || echo '  /navigate_to_pose MISSING'
  for n in map_server controller_server planner_server smoother_server \
           behavior_server bt_navigator waypoint_follower velocity_smoother \
           collision_monitor route_server docking_server; do
    printf '  %-18s %s\n' "$n" "$(ros2 lifecycle get "/$n" 2>&1 | head -1)"
  done
  exit 0
fi

cleanup
mkdir -p "$(dirname "$LOG")"

echo "[run_sentry_nav] ROS_DOMAIN_ID=${ROS_DOMAIN_ID} launching sentry_nav2 (args: ${*:-none})"
setsid nohup ros2 launch sentry_bringup sentry_nav2.launch.py "$@" \
  > "$LOG" 2>&1 < /dev/null &

ok=0
for _ in $(seq 1 90); do
  if ros2 action list -t 2>/dev/null | grep -q 'navigate_to_pose'; then
    ok=1
    break
  fi
  sleep 2
done

if [ "$ok" != 1 ]; then
  echo "[run_sentry_nav] FAIL: /navigate_to_pose not ready in 180 s; tail of $LOG:"
  tail -40 "$LOG"
  exit 1
fi

MAP_RES=$(timeout 15 ros2 topic echo --once /map --field info.resolution 2>/dev/null | head -1)
TF_OK=$(timeout 12 ros2 run tf2_ros tf2_echo map base_footprint 2>/dev/null | \
        grep -c 'Translation' || true)
echo "[run_sentry_nav] OK  action=/navigate_to_pose  map_resolution=${MAP_RES:-none}  " \
     "map->base_footprint_echo_lines=${TF_OK}  log=$LOG"
tail -3 "$LOG"
exit 0