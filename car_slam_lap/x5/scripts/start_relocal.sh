#!/usr/bin/env bash
# 走圈第一步：用建好的地图起 Odin1 重定位，等 map->odom 出现。
#
# 用法：  bash start_relocal.sh                    # 用最近一次建图目录
#         bash start_relocal.sh <地图目录|map.bin>
#
# 重定位成功前 map 帧不在 TF 树里，所以"等 TF map<-imu 通"就是成功的判据。
# 成功率的关键：车停在建图那趟的起点附近（1 m 内、朝向差 <10°）。
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

ARG="${1:-$(current_map_dir)}"
if [ -z "$ARG" ]; then
  echo "✗ 没给地图：先跑过 start_mapping.sh + save_map.sh，或直接传 map.bin 路径"
  exit 1
fi

if [ -f "$ARG" ] && [ "${ARG##*.}" = "bin" ]; then
  MAP_BIN="$ARG"
  MAP_DIR="$(dirname "$ARG")"
else
  MAP_DIR="$ARG"
  if [ -f "$MAP_DIR/map_path.txt" ]; then
    MAP_BIN="$(cat "$MAP_DIR/map_path.txt")"
  else
    MAP_BIN="$(find "$MAP_DIR" -type f -name '*.bin' | head -1 || true)"
  fi
fi

require_file "$MAP_BIN"
echo "== 地图: $MAP_BIN"
ls -lh "$MAP_BIN"

# ---------------------------------------------------------------- 换配置
GEN="$CAR_CONFIG/generated"
mkdir -p "$GEN"
CFG="$GEN/control_command.relocal.yaml"
sed "s|__MAP_ABS_PATH__|$MAP_BIN|" "$CAR_CONFIG/control_command.relocal.yaml" > "$CFG"
if grep -q "__MAP_ABS_PATH__" "$CFG"; then
  echo "✗ 替换地图路径失败（模板里 __MAP_ABS_PATH__ 没被替换）"
  exit 1
fi
echo "== 生成配置: $CFG"
grep -E "custom_map_mode|relocalization_map_abs_path" "$CFG"

# ---------------------------------------------------------------- 重启驱动
if pgrep -x host_sdk_sample >/dev/null; then
  echo "== 先停掉正在跑的驱动（要换重定位配置）..."
  pkill -INT -x host_sdk_sample || true
  for _ in $(seq 1 30); do
    pgrep -x host_sdk_sample >/dev/null || break
    sleep 0.5
  done
  sleep 1
fi

echo "== 启动 Odin1（重定位模式）..."
setsid nohup ros2 run odin_ros_driver host_sdk_sample \
    --ros-args -p config_file:="$CFG" \
    > "$MAP_DIR/odin_reloc.log" 2>&1 < /dev/null &

echo "== 等重定位（最多 90 s）..."
if wait_for_tf map imu 90; then
  echo "✓ 重定位成功：map -> odom 已发布，TF 链通了"
else
  echo "✗ 90 s 内 map 帧没出现，重定位没成功。"
  echo "  1) 车是不是停在建图起点 1 m 内、朝向大致相同？"
  echo "  2) 场景变化大吗（家具挪动/关灯）？"
  echo "  3) 日志: $MAP_DIR/odin_reloc.log"
  echo "  4) README 说重定位失败时会先退化成 SLAM 模式继续尝试，轻轻晃一下设备可能有帮助"
  exit 1
fi

cat <<EOF

------------------------------ 可以走圈了 ------------------------------
  bash $(dirname "${BASH_SOURCE[0]}")/drive_lap.sh
  （前提是 car_bridge 在跑；没跑的话先起：
    ros2 run car_bridge bridge_node & ）
-------------------------------------------------------------------------
EOF
