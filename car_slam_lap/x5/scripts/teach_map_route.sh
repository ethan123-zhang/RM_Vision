#!/usr/bin/env bash
# 严格定位示教：把当前旧地图下的示教轨迹录成 route_map.yaml + manifest。
#
# 本脚本只启动严格模式 record_path：
#   - 不启动 bridge_node / follow_path / 建图脚本；
#   - 不自动杀任何进程；
#   - 不编造静态 TF。
#
# 用法：
#   bash teach_map_route.sh                 # 最近建图目录
#   bash teach_map_route.sh <地图目录>
#   OUT_PATH=/path/route_map.yaml bash teach_map_route.sh <地图目录>
#
# 前置：
#   1. 先用 start_relocal.sh 手动启动旧地图重定位；
#   2. 用 DT7 遥控器示教（不要同时让 bridge_node 转发 /cmd_vel）；
#   3. 录制器等待 map<-base 与 odom<-map 都有效才开始录点。
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

BASE_FRAME="${BASE_FRAME:-imu}"
ARG="${1:-$(current_map_dir)}"
if [ -z "$ARG" ]; then
  echo "✗ 没给地图：先跑 start_mapping.sh + save_map.sh，或传地图目录"
  exit 1
fi

if [ -d "$ARG" ]; then
  MAP_DIR="$(cd "$ARG" && pwd)"
elif [ -f "$ARG" ] && [ "${ARG##*.}" = "bin" ]; then
  MAP_BIN="$(cd "$(dirname "$ARG")" && pwd)/$(basename "$ARG")"
  MAP_DIR="$(dirname "$MAP_BIN")"
else
  echo "✗ 参数既不是地图目录也不是 .bin: $ARG"
  exit 1
fi

OUT="${OUT_PATH:-$MAP_DIR/route_map.yaml}"
MANIFEST="$OUT.manifest.json"
if [ -e "$OUT" ] || [ -e "$MANIFEST" ]; then
  echo "✗ 输出或 manifest 已存在，拒绝覆盖："
  [ -e "$OUT" ] && echo "   $OUT"
  [ -e "$MANIFEST" ] && echo "   $MANIFEST"
  echo "   保留历史示教数据；请换目录或先人工确认后再处理。"
  exit 1
fi

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

if [ -z "${MAP_BIN:-}" ]; then
  select_map "$MAP_DIR" || exit 1
fi
if [ ! -f "$MAP_BIN" ]; then
  echo "✗ 选中的地图不存在: $MAP_BIN"
  exit 1
fi
echo "== 地图: $MAP_BIN"
echo "== 输出: $OUT"

# ---------------------------------------------------------------- 控制权检查
if pgrep -f "[b]ridge_node" >/dev/null; then
  echo "✗ bridge_node 正在运行：它可能把 /cmd_vel 转给底盘，和 DT7 抢控制权。"
  echo "  本脚本不会自动杀进程；请先手动停掉 bridge_node，再用 DT7 示教。"
  exit 1
fi
if pgrep -f "[r]ecord_path" >/dev/null; then
  echo "✗ record_path 已在运行；先停掉旧的录制器，避免两个进程抢输出。"
  exit 1
fi

# ---------------------------------------------------------------- 驱动配置核对
CFG="$ODIN_DRIVER_CFG"
if [ ! -f "$CFG" ]; then
  echo "✗ 找不到 Odin1 驱动实际配置: $CFG"
  echo "  先按 README 部署 ~/odin_ws，并用 start_relocal.sh 启动旧地图重定位。"
  exit 1
fi
MODE="$(sed -n 's/^[[:space:]]*custom_map_mode:[[:space:]]*\([0-9][0-9]*\).*/\1/p' "$CFG" | tail -1)"
if [ "$MODE" != "2" ]; then
  echo "✗ 驱动配置不是重定位模式（custom_map_mode=$MODE，期望 2）。"
  echo "  先跑 start_relocal.sh <地图目录>，它会安装正确的配置。"
  exit 1
fi
RELOC="$(sed -n 's/^[[:space:]]*relocalization_map_abs_path:[[:space:]]*//p' "$CFG" | tail -1)"
RELOC="$(printf '%s' "$RELOC" | sed -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//' \
  -e 's/^"//' -e 's/"$//' -e "s/^'//" -e "s/'$//")"
if [ -z "$RELOC" ]; then
  echo "✗ 驱动配置缺少 relocalization_map_abs_path。"
  exit 1
fi
RELOC_REAL="$(readlink -f "$RELOC" 2>/dev/null || true)"
MAP_REAL="$(readlink -f "$MAP_BIN" 2>/dev/null || true)"
if [ -z "$RELOC_REAL" ] || [ ! -f "$RELOC_REAL" ]; then
  echo "✗ 配置里的地图文件不存在: $RELOC"
  exit 1
fi
if [ -z "$MAP_REAL" ] || [ ! -f "$MAP_REAL" ]; then
  echo "✗ 选中的地图文件不存在: $MAP_BIN"
  exit 1
fi
if ! cmp -s "$MAP_REAL" "$RELOC_REAL"; then
  echo "✗ 驱动配置指向的地图内容与选中地图不同："
  echo "   配置: $RELOC_REAL"
  echo "   选中: $MAP_REAL"
  echo "   先重新跑 start_relocal.sh 选中同一张地图；本检查只证明内容一致，不证明定位精度。"
  exit 1
fi
echo "✓ 配置核对：custom_map_mode=2，地图内容与选中地图一致"
echo "  （这只是配套检查，不等于定位精度或整车导航验收）"

cat <<EOF

== 严格定位示教启动 ==
   只在 map<-base 和 odom<-map 都有效后才录点；地图 TF 最多等 30 s。
   若定位/原始里程计中途故障，录制器会中止并返回非 0，且不写正式 bundle。
   现在用 DT7 开车示教，结束后 Ctrl-C（正常结束才会写 bundle）。
   本脚本不会启动 bridge/follow/mapping，也不会自动杀其他进程。

EOF

if ! ros2 run car_bridge record_path --ros-args \
    -p map_file:="$MAP_BIN" \
    -p out_path:="$OUT" \
    -p base_frame:="$BASE_FRAME" \
    -p world_frame:=map \
    -p "fallback_frame:=''" \
    -p use_odom_pose:=false; then
  echo "✗ 录制器异常退出，未形成可正式运行的 bundle。"
  exit 1
fi

if [ ! -f "$MANIFEST" ]; then
  echo "✗ 录制器正常结束但没有生成 manifest，不能用于正式走圈。"
  exit 1
fi

echo
echo "✓ 严格示教完成"
echo "  路线:   $OUT"
echo "  manifest: $MANIFEST"
echo "  下一步: bash $(dirname "${BASH_SOURCE[0]}")/drive_lap.sh $(printf '%q' "$MAP_DIR")"
