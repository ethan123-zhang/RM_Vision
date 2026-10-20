#!/usr/bin/env bash
# 走圈：照着 path.yaml 用纯追踪跑一圈。
#
# 用法：  bash drive_lap.sh                 # 最近一次建图的 path.yaml
#         bash drive_lap.sh <path.yaml>
#         VMAX=0.25 bash drive_lap.sh      # 临时降速（正式跑之前一定先慢速试）
#
# 三条前置检查（脚本会先验，不满足直接退出，不会带着车乱跑）：
#   1. car_bridge 在跑（/car/status 有数据）
#   2. 重定位成功（map -> imu TF 通）
#   3. path.yaml 存在
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

ARG="${1:-$(current_map_dir)}"
if [ -z "$ARG" ]; then
  echo "✗ 没给轨迹：先跑 start_mapping.sh 录 path.yaml"
  exit 1
fi
if [ -d "$ARG" ]; then
  PATH_FILE="$ARG/path.yaml"
  MAP_DIR="$ARG"
else
  PATH_FILE="$ARG"
  MAP_DIR="$(dirname "$ARG")"
fi
require_file "$PATH_FILE"

echo "== 轨迹: $PATH_FILE"
head -4 "$PATH_FILE"

echo "== 检查 car_bridge"
if ! pgrep -f "[b]ridge_node" >/dev/null; then
  echo "✗ car_bridge 没在跑。先起：ros2 run car_bridge bridge_node &"
  exit 1
fi
if ! ros2 topic list 2>/dev/null | grep -qx "/car/status"; then
  echo "✗ 没有 /car/status：桥接起来了但没连上 C8T6（拔线了？波特率？）"
  exit 1
fi

echo "== 检查重定位（map -> imu）"
if ! wait_for_tf map imu 5; then
  echo "✗ 查不到 map -> imu 的 TF：先跑 start_relocal.sh"
  exit 1
fi
echo "✓ 定位可用"

echo "== 5 s 后开始，Ctrl-C 随时停"
VMAX="${VMAX:-0.30}"
ros2 run car_bridge follow_path --ros-args \
  -p path_file:="$PATH_FILE" \
  -p v_max:="$VMAX" \
  -p save_actual:="$MAP_DIR/lap_actual.yaml" \
  -p start_delay_s:=5.0

echo
echo "== 实际轨迹: $MAP_DIR/lap_actual.yaml"
echo "== 想对比看两条线，用 CloudCompare 打开 map.pcd，或直接看 follow_path 打印的横向误差"
