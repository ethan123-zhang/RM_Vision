#!/usr/bin/env bash
# 把上一次失败的建图/示教清干净，好重新开始。
#
# 核心原则：先让进程自己收尾（SIGINT），再动文件。反过来做，
# record_path 会把半截轨迹写回 yaml、map_from_cloud 会写出残缺的 map.pcd。
#
# 用法：
#   bash reset_run.sh                                    # 只停进程，不动任何文件（先看一眼）
#   bash reset_run.sh <地图目录> --teach-only            # 地图留着，只删示教产物（重示教）
#   bash reset_run.sh <地图目录> --full                  # 整个目录改名备份，清驱动暂存，可重新建图
#   bash reset_run.sh <地图目录> --purge                 # 整个目录直接删掉（确认不要了）
#
# 例：
#   bash reset_run.sh ~/car_ws/maps/demo-1106 --full
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAP_DIR=""
MODE="stop"

for a in "$@"; do
  case "$a" in
    --stop)       MODE="stop" ;;
    --teach-only) MODE="teach" ;;
    --full)       MODE="full" ;;
    --purge)      MODE="purge" ;;
    -h|--help)    sed -n '2,20p' "${BASH_SOURCE[0]}"; exit 0 ;;
    -*)           echo "未知参数: $a"; exit 2 ;;
    *)            MAP_DIR="$a" ;;
  esac
done

case "$MAP_DIR" in
  "") [ "$MODE" = "stop" ] || { echo "用法错误：这个模式必须给地图目录"; exit 2; } ;;
  "$HOME"/car_ws/maps/*) ;;
  *) echo "拒绝操作：$MAP_DIR 不在 ~/car_ws/maps 下"; exit 2 ;;
esac

if [ -n "$MAP_DIR" ]; then
  MAP_DIR="${MAP_DIR/#\~/$HOME}"
  [ -d "$MAP_DIR" ] || { echo "目录不存在: $MAP_DIR"; exit 2; }
fi

echo "== 1/4 让录制器自己收尾（只发一次 SIGINT，别连按）"
for pat in "record_path[.]py\$" "record_odin_rgb[.]py\$"; do
  if pkill -INT -f "$pat" 2>/dev/null; then echo "   已停 $pat"; fi
done
sleep 2

echo "== 2/4 停点云导出 / 桥接 / 跟随"
for pat in "lib/car_bridge/map_from_cloud\$" "lib/car_bridge/bridge_node\$" "lib/car_bridge/follow_path\$"; do
  if pkill -f "$pat" 2>/dev/null; then echo "   已停 $pat"; fi
done

echo "== 3/4 停驱动并恢复厂商配置"
bash "$HERE/stop_mapping.sh" >/dev/null 2>&1 || true

LEFT=$(ps -ef | grep -E "[h]ost_sdk_sample|[b]ridge_node|[f]ollow_path|[r]ecord_path|[r]ecord_odin_rgb|[m]ap_from_cloud" | wc -l)
echo "   残留进程: $LEFT"
[ "$LEFT" -eq 0 ] || { echo "!! 还有进程没退，先别删文件：ps -ef | grep -E 'host_sdk|bridge_node|record_'"; exit 1; }

echo "== 4/4 清理（模式: $MODE）"
case "$MODE" in
  stop)
    echo "   只停了进程，文件没动。要清文件加 --teach-only / --full / --purge"
    ;;
  teach)
    rm -f "$MAP_DIR"/route_map.yaml "$MAP_DIR"/route_map.yaml.manifest.json \
          "$MAP_DIR"/route_*.yaml "$MAP_DIR"/lap_actual*.yaml
    echo "   已删示教产物；地图 .bin 保留："
    ls -lh "$MAP_DIR"/odin_map/*.bin 2>/dev/null || echo "   !! 没有 .bin，地图不可用，得 --full 重新建图"
    ;;
  full)
    BAK="${MAP_DIR}-aborted-$(date +%H%M%S)"
    mv "$MAP_DIR" "$BAK"
    echo "   目录已改名备份: $BAK"
    rm -rf "$HOME"/odin_ws/src/odin_ros_driver/map/* 2>/dev/null
    rm -f "$HOME"/car_ws/.current_map_dir
    echo "   已清驱动暂存地图 + 当前地图指针"
    ;;
  purge)
    rm -rf "$MAP_DIR"
    rm -rf "$HOME"/odin_ws/src/odin_ros_driver/map/* 2>/dev/null
    rm -f "$HOME"/car_ws/.current_map_dir
    echo "   已删除 $MAP_DIR（含驱动暂存副本）"
    ;;
esac

echo
echo "== 现在可以直接重新开始 =="
echo "   重新建图: MAP=~/car_ws/maps/demo-\$(date +%m%d-%H%M)"
echo "             VIDEO=1 VIDEO_STRIDE=2 DT7=1 bash $HERE/start_mapping.sh \"\$MAP\""
echo "   重示教:   先 bash $HERE/start_relocal.sh \"$MAP_DIR\""
echo "             再 bash $HERE/teach_map_route.sh \"$MAP_DIR\""
echo "   走完示教务必按一次 Ctrl-C，验收: ls -l \"$MAP_DIR\"/route_map.yaml*"
df -h "$HOME" | tail -1