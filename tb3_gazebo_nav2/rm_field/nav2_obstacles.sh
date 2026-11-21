#!/usr/bin/env bash
# 绕障验收：起"翻转后 3V3 场地 + 静态/动态障碍物 + Nav2"。
#   bash nav2_obstacles.sh                 # 静态 2 箱 + 动态 1 箱（默认档位 slow）
#   bash nav2_obstacles.sh --tier fast     # 动态箱换档位（slow 0.3 / fast 0.6 / faster 1.0 m/s）
#   bash nav2_obstacles.sh --static-only   # 只静态箱（动态驱动不拉）
#   bash nav2_obstacles.sh --gui           # 带 Gazebo GUI / RViz 看画面
# 前置：障碍物布局与档位在 config/avoid_obstacles.json（几何唯一源）；
#       本脚本每次都重新生成世界并做同源校核 + xacro 展开自检，不会用旧世界跑验收。
#       世界里的动态箱初始位置 = 该档位的 start，驱动脚本也读同一档位。
# 验收：python3 verify_nav2_obstacles.py --tier <档位>
#       → results/report_nav2_obstacles[_<档位>].json
# 停止：bash stop_nav2_field.sh（已包含动态障碍物驱动）
set +u
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
cd "$ROOT" || exit 1
source /opt/ros/jazzy/setup.bash 2>/dev/null

GUI_ARG=""
DYN=1
TIER=""
while [ $# -gt 0 ]; do
  case "$1" in
    --gui) GUI_ARG="--gui" ;;
    --static-only|--no-dynamic) DYN=0 ;;
    --tier) TIER="${2:-}"; [ -n "$TIER" ] || { echo "--tier 需要一个档位名"; exit 2; }; shift ;;
    --tier=*) TIER="${1#--tier=}" ;;
    *) echo "未知参数: $1"; exit 2 ;;
  esac
  shift
done
TierArg=()
[ -n "$TIER" ] && TierArg=(--tier "$TIER")

echo "=== 0. 生成障碍物世界（world 与验收同源，档位=${TIER:-<config default>}）==="
python3 make_obstacles_world.py --only obstacles "${TierArg[@]}" \
  || { echo "致命: 障碍物世界生成/校核失败"; exit 1; }
TMP=/tmp/obstacles_world_check.sdf
xacro worlds/rmul2027_3v3_up_nav2_obstacles.sdf.xacro headless:=true > "$TMP" 2>/tmp/xacro_err.txt \
  || { echo "致命: xacro 展开失败"; cat /tmp/xacro_err.txt; exit 1; }
for name in $(python3 -c "import json;c=json.load(open('config/avoid_obstacles.json'));print(' '.join([o['name'] for o in c['static']]+[c['dynamic']['name']]))"); do
  grep -q "name=\"$name\"" "$TMP" || { echo "致命: xacro 展开后缺 $name"; exit 1; }
done
echo "  xacro 展开 OK（障碍物齐全）"

export NAV2_WORLD="$ROOT/worlds/rmul2027_3v3_up_nav2_obstacles.sdf.xacro"
export RM_DYNAMIC="$DYN"
export RM_DYN_TIER="$TIER"
echo "=== 1. 起栈（世界=$(basename "$NAV2_WORLD")，动态=$DYN，档位=${TIER:-<config default>}）==="
bash "$ROOT/nav2_field.sh" $GUI_ARG || exit 1
if [ "$DYN" = 1 ]; then
  echo "=== 2. 验收命令 ==="
  echo "  python3 verify_nav2_obstacles.py --tier ${TIER:-slow}"
fi