#!/usr/bin/env bash
# One-shot start/stop for the RM sentry S1a simulation (no Nav2).
#   ./run_sentry_sim.sh [gui:=true]   start (cleans this ROS domain first)
#   ./run_sentry_sim.sh stop          stop only
set -uo pipefail

WS="${HOME}/sentry_ws"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-45}"

PATTERN='gz sim|gz-sim-server|ros2 launch sentry|parameter_bridge|ros_gz_sim/create|ros_gz_sim create|odin1_sim_bridge|pointcloud_to_laserscan|robot_state_publisher|sentry_ws/world\.sdf'

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
    echo "[run_sentry] TERM domain ${ROS_DOMAIN_ID}: $(echo "$pids" | tr '\n' ' ')"
    for sig in TERM KILL; do
      for p in $pids; do kill -"$sig" "$p" 2>/dev/null || true; done
      sleep 3
      pids=$(domain_pids)
      [ -z "$pids" ] && break
    done
  fi
  pkill -f "gz-sim-server.*sentry_ws" 2>/dev/null || true
  sleep 1
  echo "[run_sentry] remaining in domain ${ROS_DOMAIN_ID}: $(domain_pids | tr '\n' ' ')"
}

if [ "${1:-}" = "stop" ]; then
  cleanup
  exit 0
fi

cleanup

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "${HOME}/lidar_ws/install/setup.bash"
# shellcheck disable=SC1091
source "${WS}/install/setup.bash"
set -u

echo "[run_sentry] ROS_DOMAIN_ID=${ROS_DOMAIN_ID} launching sentry_sim (args: ${*:-none})"
exec ros2 launch sentry_bringup sentry_sim.launch.py "$@"