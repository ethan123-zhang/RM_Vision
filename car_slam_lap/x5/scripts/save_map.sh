#!/usr/bin/env bash
# 把 Odin1 这次建图的结果存盘，并拷进本次建图目录。
#
# Odin1 的存图是运行时命令：驱动会轮询 /tmp/odin_command.txt（等价于驱动源码目录
# 里的 ./set_param.sh save_map 1）。存的是设备专有 .bin，走圈重定位要用它。
#
# 用法：  bash save_map.sh            # 存最近一次 start_mapping.sh 的目录
#         bash save_map.sh <目录>
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

echo "== 触发存图: set save_map 1"
echo "set save_map 1" > /tmp/odin_command.txt
echo "== 等 8 s（README 要求两次存图至少间隔 5 s）..."
sleep 8

# 默认存到 {ws}/src/odin_ros_driver/map/{驱动启动时间}/，取最新的那个目录
DRV_MAP="$HOME/odin_ws/src/odin_ros_driver/map"
if [ ! -d "$DRV_MAP" ]; then
  echo "✗ 没有 $DRV_MAP：驱动是不是没按建图模式起？（control_command.mapping.yaml）"
  exit 1
fi
SRC_DIR="$(ls -1dt "$DRV_MAP"/* 2>/dev/null | head -1 || true)"
if [ -z "$SRC_DIR" ]; then
  echo "✗ $DRV_MAP 是空的：存图命令没生效，看 $MAP_DIR/odin.log 里有没有 save_map 相关报错"
  exit 1
fi

DEST="$MAP_DIR/odin_map"
mkdir -p "$DEST"
cp -a "$SRC_DIR"/. "$DEST"/
MAP_BIN="$(find "$DEST" -type f -name '*.bin' | head -1 || true)"
if [ -z "$MAP_BIN" ]; then
  echo "✗ 拷进来但没有 .bin："
  ls -lhR "$DEST"
  exit 1
fi
echo "$MAP_BIN" > "$MAP_DIR/map_path.txt"

echo
echo "== 地图已存"
ls -lh "$DEST"
echo "  地图文件: $MAP_BIN"
echo "  路径文件: $MAP_DIR/path.yaml"
echo
echo "下一步：stop_mapping.sh 收尾 -> start_relocal.sh 走圈"
