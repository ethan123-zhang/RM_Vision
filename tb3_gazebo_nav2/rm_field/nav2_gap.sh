#!/usr/bin/env bash
# 窄通道（贴边）验收：起"翻转场地 + 0.80 m 门洞 + Nav2"。
#   bash nav2_gap.sh         # headless
#   bash nav2_gap.sh --gui
# 之后：python3 verify_narrow_gap.py  → results/report_narrow_gap.json
# 停止：bash stop_nav2_field.sh
set +u
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
cd "$ROOT" || exit 1
source /opt/ros/jazzy/setup.bash 2>/dev/null

GUI_ARG=""
[ "${1:-}" = "--gui" ] && GUI_ARG="--gui"

echo "=== 0. 生成窄通道世界（同源校核 + xacro 展开自检）==="
python3 make_obstacles_world.py --only gap || { echo "致命: 窄通道世界生成/校核失败"; exit 1; }
TMP=/tmp/gap_world_check.sdf
xacro worlds/rmul2027_3v3_up_nav2_gap.sdf.xacro headless:=true > "$TMP" 2>/tmp/xacro_err.txt \
  || { echo "致命: xacro 展开失败"; cat /tmp/xacro_err.txt; exit 1; }
grep -q 'name="gate_west"' "$TMP" || { echo "致命: 展开后没有 gate_west"; exit 1; }
echo "  xacro 展开 OK"

# 出生点：从门南侧 2.5 m 处起步（缺省出生点 (0,0) 贴着门，没有助跑距离）
SPAWN=$(python3 -c "import json;print(' '.join(str(v) for v in json.load(open('config/avoid_obstacles.json'))['gate']['spawn']))")
export NAV2_WORLD="$ROOT/worlds/rmul2027_3v3_up_nav2_gap.sdf.xacro"
export RM_DYNAMIC=0
export RM_SPAWN_X="${SPAWN% *}"
export RM_SPAWN_Y="${SPAWN#* }"
echo "=== 1. 起栈（世界=$(basename "$NAV2_WORLD")，出生点 ($RM_SPAWN_X,$RM_SPAWN_Y)）==="
exec bash "$ROOT/nav2_field.sh" $GUI_ARG