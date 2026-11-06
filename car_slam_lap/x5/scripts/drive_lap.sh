#!/usr/bin/env bash
# 走圈：严格校验 route_map.yaml + manifest + 地图后，用纯追踪跑一圈。
#
# 用法：  bash drive_lap.sh                    # 最近建图目录下的 route_map.yaml
#         bash drive_lap.sh <地图目录>
#         bash drive_lap.sh <route_map.yaml>
#         VMAX=0.25 bash drive_lap.sh         # 临时降速（正式跑之前一定先慢速试）
#
# 可选环境变量（都直接透传给 follow_path）：
#   VMAX=0.15              目标速度上限（默认 0.30）
#   W_MAX=0.35             角速度上限（rad/s，默认 0.35；follow_path 内建默认 1.2 太快）
#   RETURN_W_MAX=0.30      回起点/起始对齐的角速度上限（rad/s，默认 0.30）
#   BASE_OFFSET_X/Y=...    力臂补偿（m，车体系；标定见 calibrate_lever_arm）
#   START_MAX_OFFSET_M=... 允许距离起点多远（m，默认 0.5；放大时同步放大下面的超时）
#   RETURN_TIMEOUT_S=...   回起点+起始对齐总超时（s，默认 30）
#   RETURN_REVERSE=1        回起点改用倒车三拍（车尾对准→倒车→转正），默认 0
#   RETURN_W_ACCEL=...     起始阶段角速度变化率上限 rad/s^2（默认 0=不限）
#   RETURN_REVERSE_V_MAX=... 倒车速度 m/s（默认 0.06）
#   MAX_TIME_S=...         总时限（s，默认 0 = 不限）
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

# 力臂补偿：base_frame（imu/雷达）相对转心的偏置，车体系、有符号、单位 m。
# 用 `ros2 run car_bridge calibrate_lever_arm` 标定；默认 0 表示不补偿（旧行为）。
OFFX="${BASE_OFFSET_X:-0.0}"
OFFY="${BASE_OFFSET_Y:-0.0}"
# 角速度上限（rad/s）。follow_path 的默认值是 1.2，急弯原地对齐时看起来会"猛转"，
# 演示/低速场景务必显式给个小值（0.30~0.40 ≈ 17~23 deg/s）。
WMAX="${W_MAX:-0.35}"
# 回起点/起始对齐阶段的角速度上限（rad/s）；比跟踪段更保守。
RWMAX="${RETURN_W_MAX:-0.30}"
# 起点范围（m）：车第一次位姿离路线首点超过它就直接拒跑，避免半路乱切路线。
SMO="${START_MAX_OFFSET_M:-0.5}"
# 回起点+起始对齐总超时（s）：范围放大时必须同步放大（实测 1.5 m 约需 54 s）。
RTO="${RETURN_TIMEOUT_S:-30.0}"
RREV="${RETURN_REVERSE:-0}"
RVMAX="${RETURN_REVERSE_V_MAX:-0.06}"
RWACC="${RETURN_W_ACCEL:-0.0}"
# 总时限（s），0 = 不限。14 m 路线 @0.15 m/s 约需 100 s 跟踪 + 起点阶段。
MX="${MAX_TIME_S:-0.0}"

# ROS 2 从命令行值推断参数类型：写 280 会被当成 INTEGER，而 follow_path 声明的
# max_time_s 是 DOUBLE，直接抛 InvalidParameterTypeException。所以统一补成浮点。
num_float() {
  local v="$1" name="$2"
  case "$v" in
    ''|*[!0-9.eE+-]*) echo "✗ $name 不是数字: '$v'" >&2; exit 1 ;;
  esac
  case "$v" in
    *.*|*e*|*E*) printf '%s' "$v" ;;
    *) printf '%s.0' "$v" ;;
  esac
}
VMAX="$(num_float "$VMAX" VMAX)"
WMAX="$(num_float "$WMAX" W_MAX)"
RWMAX="$(num_float "$RWMAX" RETURN_W_MAX)"
OFFX="$(num_float "$OFFX" BASE_OFFSET_X)"
OFFY="$(num_float "$OFFY" BASE_OFFSET_Y)"
SMO="$(num_float "$SMO" START_MAX_OFFSET_M)"
RTO="$(num_float "$RTO" RETURN_TIMEOUT_S)"
RWACC="$(num_float "$RWACC" RETURN_W_ACCEL)"
RVMIN="$(num_float "$RVMAX" RETURN_REVERSE_V_MAX)"
# RETURN_TO_START=0：跳过回起点与起始对齐（车已人工摆正时用，避免原地转 180° 打滑）
if [ "${RETURN_TO_START:-1}" = "0" ]; then RTS=false; else RTS=true; fi
if [ "$RREV" = "0" ]; then RREV_B=false; else RREV_B=true; fi
# 回起点的位置容差：放宽到 0.2 m 可减少"对齐→补位→再对齐"的来回抽搐
SPT="${START_POS_TOL_M:-0.10}"
SPT="$(num_float "$SPT" START_POS_TOL_M)"
MX="$(num_float "$MX" MAX_TIME_S)"

# 独立避障入口通过参数文件启用；未指定时沿用原有跟踪行为。
EXTRA_ROS_ARGS=()
if [ -n "${NAV_PARAMS_FILE:-}" ]; then
  require_file "$NAV_PARAMS_FILE"
  EXTRA_ROS_ARGS+=(--params-file "$NAV_PARAMS_FILE")
fi
NAV_MODE="${NAV_MODE:-先回起点再跟踪；限起点附近空旷通道，无自动绕障。}"
SAVE_ACTUAL="$MAP_DIR/${ACTUAL_NAME:-lap_actual.yaml}"

cat <<EOF

== 5 s 后开始，Ctrl-C 随时停 ==
   $NAV_MODE
   手工起 bridge 的方式不变，本脚本不会自动启动底盘。
   v_max=$VMAX  w_max=$WMAX  return_w_max=$RWMAX
   base_offset=($OFFX, $OFFY)  起点范围=${SMO} m  回起点超时=${RTO} s  总时限=$MX

EOF
ros2 run car_bridge follow_path --ros-args \
  "${EXTRA_ROS_ARGS[@]}" \
  -p path_file:="$PATH_FILE" \
  -p map_file:="$MAP_BIN" \
  -p base_frame:="$BASE_FRAME" \
  -p v_max:="$VMAX" \
  -p w_max:="$WMAX" \
  -p return_w_max:="$RWMAX" \
  -p base_offset_x:="$OFFX" \
  -p base_offset_y:="$OFFY" \
  -p start_max_offset_m:="$SMO" \
  -p return_timeout_s:="$RTO" \
  -p return_reverse:="$RREV_B" \
  -p return_reverse_v_max:="$RVMIN" \
  -p return_w_accel:="$RWACC" \
  -p max_time_s:="$MX" \
  -p return_to_start:="$RTS" \
  -p start_pos_tol_m:="$SPT" \
  -p save_actual:="$SAVE_ACTUAL" \
  -p start_delay_s:=5.0

echo
echo "== 实际轨迹: $SAVE_ACTUAL"
echo "== 想对比看两条线，用 CloudCompare 打开 map.pcd，或直接看 follow_path 打印的横向误差"
