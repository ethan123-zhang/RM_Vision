#!/usr/bin/env bash
# 把 Odin1 这次建图的结果存盘，并拷进本次建图目录。
#
# 设备存图是异步的：往 /tmp/odin_command.txt 写 `set save_map 1` 只是"下单"，驱动会在
# 后台让设备生成 .bin 再回传，SDK 最多给 120 s。所以不能"等 8 s 就拷"，要等驱动日志
# 出现 "Map transfer completed"，否则拷到的可能是半个文件（用它重定位必然失败）。
#
# 还有一点：驱动把图存到哪个目录，取决于它启动时 COLCON_PREFIX_PATH 里第一个
# "/install"（odin_ros_driver/src/host_sdk_sample.cpp）。所以这里不写死路径：
# 优先相信驱动日志里打印的真实路径，其次再扫三个候选目录。
# 2026-10-26 的坑：car_ws 排在 odin_ws 前面时，图存进了
# ~/car_ws/src/odin_ros_driver/map，旧版脚本只认 ~/odin_ws/...，于是报"目录是空的"。
#
# 用法：  bash save_map.sh                  # 存最近一次 start_mapping.sh 的目录
#         bash save_map.sh <目录>
#         SAVE_TIMEOUT=180 bash save_map.sh # 改最长等待秒数（默认 150）
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

MAP_DIR="${1:-$(current_map_dir)}"
if [ -z "$MAP_DIR" ] || [ ! -d "$MAP_DIR" ]; then
  echo "✗ 找不到建图目录（先跑 start_mapping.sh，或把目录作为参数传进来）"
  exit 1
fi

if ! pgrep -x host_sdk_sample >/dev/null; then
  echo "✗ Odin1 驱动没在跑，存不了图"
  exit 1
fi

LOG="$MAP_DIR/odin.log"
WAIT_S="${SAVE_TIMEOUT:-150}"
BEFORE=0
if [ -f "$LOG" ]; then
  BEFORE="$(wc -l < "$LOG")"
fi

echo "== 触发存图: set save_map 1"
echo "set save_map 1" > /tmp/odin_command.txt

echo "== 等设备生成并回传（最多 ${WAIT_S}s，日志 $LOG）..."
MAP_BIN=""
KIND=""
NEW=""
for _ in $(seq 1 "$WAIT_S"); do
  sleep 1
  NEW=""
  if [ -f "$LOG" ]; then
    NEW="$(tail -n "+$((BEFORE + 1))" "$LOG" 2>/dev/null || true)"
  fi
  if [ -n "$NEW" ] && grep -q "Map transfer completed: " <<<"$NEW"; then
    MAP_BIN="$(grep -o "Map transfer completed: .*" <<<"$NEW" | tail -1 \
      | sed 's/^Map transfer completed: //')"
    KIND="ok"
    break
  fi
  if [ -n "$NEW" ] && grep -qE "Map transfer failed|Map transfer skipped" <<<"$NEW"; then
    KIND="failed"
    break
  fi
done

if [ "$KIND" = "failed" ]; then
  echo "✗ 驱动报存图失败/被跳过，日志相关行："
  tail -n "+$((BEFORE + 1))" "$LOG" 2>/dev/null \
    | grep -iE "map save|map transfer|error" | tail -5 || true
  exit 1
fi

if [ -z "$MAP_BIN" ] && [ -n "$NEW" ] && grep -q "Map save triggered" <<<"$NEW"; then
  echo "✗ ${WAIT_S}s 内没等到 'Map transfer completed'（还在传，或传挂了）。看："
  echo "  tail -n 50 $LOG | grep -iE 'map save|map transfer'"
  exit 1
fi

if [ -z "$MAP_BIN" ]; then
  echo "!! 日志里没有本次存图记录（驱动可能不是 start_mapping.sh 起的），改扫候选目录..."
  MAP_BIN="$(find \
      "$HOME/car_ws/src/odin_ros_driver/map" \
      "$HOME/odin_ws/src/odin_ros_driver/map" \
      "$HOME/odin_ws/install/odin_ros_driver/share/odin_ros_driver/map" \
      -type f -name '*.bin' -printf '%T@ %p\n' 2>/dev/null \
    | sort -nr | head -1 | cut -d' ' -f2- || true)"
fi

if [ -z "$MAP_BIN" ] || [ ! -f "$MAP_BIN" ]; then
  echo "✗ 没拿到 .bin。排查顺序："
  echo "  1) 是不是建图模式？  grep 'Custom map mode' $LOG | tail -1        # 要 1"
  echo "  2) 存图有没有触发？  grep -E 'Map save triggered|Map transfer' $LOG | tail -5"
  echo "  3) 设备侧目录：      ls -1dt ~/car_ws/src/odin_ros_driver/map/* ~/odin_ws/src/odin_ros_driver/map/* 2>/dev/null"
  exit 1
fi

DEST="$MAP_DIR/odin_map"
mkdir -p "$DEST"
cp -a "$MAP_BIN" "$DEST/"
DEST_BIN="$DEST/$(basename "$MAP_BIN")"
echo "$DEST_BIN" > "$MAP_DIR/map_path.txt"

echo
echo "== 地图已存"
ls -lh "$DEST_BIN"
echo "  设备侧路径: $MAP_BIN"
echo "  地图文件  : $DEST_BIN"
echo "  路径文件  : $MAP_DIR/map_path.txt"
echo "  轨迹文件  : $MAP_DIR/path.yaml"
echo
echo "下一步：stop_mapping.sh 收尾 -> start_relocal.sh 走圈"
