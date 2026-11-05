#!/usr/bin/env bash
# 保留回起点/起始对齐/终点收口，增加实时感知与局部绕行。
# 用法：bash drive_lap_obstacles.sh <地图目录>，必须先 start_relocal_obstacles.sh。
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export NAV_PARAMS_FILE="${OBSTACLE_CONFIG:-$SCRIPT_DIR/../config/obstacle_avoidance.yaml}"
export NAV_MODE="实时点云避障：回起点 → 跟踪 → 遇障局部绕行 → 接回原路线。"
export ACTUAL_NAME="${ACTUAL_NAME:-lap_actual_obstacles.yaml}"
export VMAX="${VMAX:-0.15}"
export W_MAX="${W_MAX:-0.35}"
export RETURN_W_MAX="${RETURN_W_MAX:-0.30}"
export BASE_OFFSET_X="${BASE_OFFSET_X:-0.2626}"
export BASE_OFFSET_Y="${BASE_OFFSET_Y:--0.0471}"
echo "== 车身和感知参数: $NAV_PARAMS_FILE"
echo "== Odin1 前向约 129°：侧后方盲区保持空旷；无路/点云失效会停车。"
exec bash "$SCRIPT_DIR/drive_lap.sh" "$@"
