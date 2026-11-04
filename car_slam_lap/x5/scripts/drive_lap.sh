#!/usr/bin/env bash
# 走圈：严格校验 route_map.yaml + manifest + 地图后，用纯追踪跑一圈。
#
# 用法：  bash drive_lap.sh                    # 最近建图目录下的 route_map.yaml
#         bash drive_lap.sh <地图目录>
#         bash drive_lap.sh <route_map.yaml>
#         VMAX=0.25 bash drive_lap.sh         # 临时降速（正式跑之前一定先慢速试）
#
# 前置检查（不满足直接退出，不会带着车乱跑）：
#   1. route_map.yaml 存在，且与地图文件有有效 bundle（内容 SHA256 配套）
#   2. car_bridge 在跑（/car/status 有数据）；本脚本不会自动启动底盘桥接
#   3. 重定位成功（map -> base_frame TF 通）
#
# 安全提示：先回起点再跟踪；限起点附近空旷通道，无自动绕障。
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

BASE_FRAME="${BASE_FRAME:-imu}"
ARG="${1:-$(current_map_dir)}"
if [ -z "$ARG" ]; then
  echo "✗ 没给路线：先跑 teach_map_route.sh 生成 route_map.yaml"
  exit 1
fi

if [ -d "$ARG" ]; then
  MAP_DIR="$(cd "$ARG" && pwd)"
  PATH_FILE="$MAP_DIR/route_map.yaml"
else
  PATH_FILE="$ARG"
  MAP_DIR="$(cd "$(dirname "$ARG")" && pwd)"
fi
require_file "$PATH_FILE"

select_map() {
  local dir="$1"
  local list_file="$dir/map_path.txt"
  if [ -f "$list_file" ]; then
    local candidate
    candidate="$(cat "$list_file")"
    case "$candidate" in
      /*) ;;
      *) candidate="$dir/$candidate" ;;
    esac
    if [ -f "$candidate" ]; then
      MAP_BIN="$candidate"
      return 0
    fi
    echo "✗ map_path.txt 指向的地图不存在: $candidate" >&2
    return 1
  fi

  local bins=()
  while IFS= read -r -d '' f; do
    bins+=("$f")
  done < <(find "$dir" -maxdepth 1 -type f -name '*.bin' -print0)
  if [ "${#bins[@]}" -eq 0 ]; then
    echo "✗ $dir 下没有 .bin 地图，也没有 map_path.txt" >&2
    return 1
  fi
  if [ "${#bins[@]}" -gt 1 ]; then
    echo "✗ $dir 下有多个 .bin，拒绝随便挑一个：" >&2
    printf '   %s\n' "${bins[@]}" >&2
    echo "   请先在 $dir/map_path.txt 写明要用的地图" >&2
    return 1
  fi
  MAP_BIN="${bins[0]}"
}

select_map "$MAP_DIR" || exit 1
require_file "$MAP_BIN"

echo "== 路线: $PATH_FILE"
sed -n '1,5p' "$PATH_FILE"
echo "== 地图: $MAP_BIN"

echo "== 校验地图/路线 bundle 内容..."
if ! ros2 run car_bridge route_bundle validate \
    --path "$PATH_FILE" --map "$MAP_BIN" --base-frame "$BASE_FRAME"; then
  echo "✗ bundle 校验失败：拒绝运行。"
  echo "  旧 path.yaml 或无 manifest 的路线不能直接走；请用 teach_map_route.sh 重新示教。"
  exit 1
fi

echo "== 检查 car_bridge"
if ! pgrep -f "[b]ridge_node" >/dev/null; then
  echo "✗ car_bridge 没在跑。先起：ros2 run car_bridge bridge_node &"
  exit 1
fi
if ! ros2 topic list 2>/dev/null | grep -qx "/car/status"; then
  echo "✗ 没有 /car/status：桥接起来了但没连上 C8T6（拔线了？波特率？）"
  exit 1
fi

echo "== 检查重定位（map -> $BASE_FRAME）"
if ! wait_for_tf map "$BASE_FRAME" 5; then
  echo "✗ 查不到 map -> $BASE_FRAME 的 TF：先跑 start_relocal.sh"
  exit 1
fi
echo "✓ 定位可用（TF 只是必要条件，原始里程计新鲜度由 follow_path 继续检查）"

cat <<EOF

== 5 s 后开始，Ctrl-C 随时停 ==
   先回起点再跟踪；限起点附近空旷通道，无自动绕障。
   手工起 bridge 的方式不变，本脚本不会自动启动底盘。

EOF
VMAX="${VMAX:-0.30}"
ros2 run car_bridge follow_path --ros-args \
  -p path_file:="$PATH_FILE" \
  -p map_file:="$MAP_BIN" \
  -p base_frame:="$BASE_FRAME" \
  -p v_max:="$VMAX" \
  -p save_actual:="$MAP_DIR/lap_actual.yaml" \
  -p start_delay_s:=5.0

echo
echo "== 实际轨迹: $MAP_DIR/lap_actual.yaml"
echo "== 想对比看两条线，用 CloudCompare 打开 map.pcd，或直接看 follow_path 打印的横向误差"
