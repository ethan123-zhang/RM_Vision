#!/usr/bin/env bash
# 只起"地图点选"窗口（栈已经在跑时单独用；整栈用 ./run_sentry_nav.sh gui:=true map_click:=true）
#   ./tools/run_map_click.sh [--ppm 70] [--min-clear 0.45] [--click X Y]
set -uo pipefail
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-45}"
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u
MAP="${MAP_YAML:-/mnt/e/RM/tb3_gazebo_nav2/rm_field/maps/rmul2027_3v3_up.yaml}"
PPM="${MAP_CLICK_PPM:-70}"
exec python3 "$(dirname "$0")/map_click.py" --map "$MAP" --ppm "$PPM" "$@"