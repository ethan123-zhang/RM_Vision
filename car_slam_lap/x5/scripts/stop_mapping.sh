#!/usr/bin/env bash
# 建图收尾：按正确顺序停服务（先让 record_path / map_from_cloud 落盘，再停驱动）。
#
# 顺序错了会丢数据：直接 pkill -9 或先停驱动，map.pcd / path.yaml 就没了。
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

MAP_DIR="$(current_map_dir)"
echo "== 目录: ${MAP_DIR:-（没有记录，未跑过 start_mapping.sh）}"

stop_pat() {
  local pat="$1" name="$2"
  if pgrep -f "$pat" >/dev/null; then
    pkill -INT -f "$pat" || true
    for _ in $(seq 1 30); do
      pgrep -f "$pat" >/dev/null || break
      sleep 0.3
    done
    echo "  已停 $name"
  else
    echo "  $name 未运行"
  fi
}

echo "== 停上层节点"
stop_pat "[t]eleop_keys" teleop_keys
stop_pat "[r]ecord_path" record_path
stop_pat "[m]ap_from_cloud" map_from_cloud
stop_pat "[b]ridge_node" car_bridge

echo "== 停 Odin1 驱动"
if pgrep -x host_sdk_sample >/dev/null; then
  pkill -INT -x host_sdk_sample || true
  for _ in $(seq 1 30); do
    pgrep -x host_sdk_sample >/dev/null || break
    sleep 0.3
  done
  echo "  已停 host_sdk_sample"
else
  echo "  驱动未运行"
fi

if [ -n "$MAP_DIR" ] && [ -d "$MAP_DIR" ]; then
  echo
  echo "== 产物"
  ls -lh "$MAP_DIR" 2>/dev/null || true
  for f in path.yaml map.pcd map_bev.png map_info.txt odin_map; do
    [ -e "$MAP_DIR/$f" ] || echo "  !! 缺 $f"
  done
fi
echo
echo "下一步：确认 path.yaml / map.pcd 都在，再 start_relocal.sh 走圈。"
