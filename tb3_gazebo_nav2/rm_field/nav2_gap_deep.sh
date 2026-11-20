#!/bin/bash
# 窄通道 + 定位压力回归：门洞 0.80 m 不变，但两块箱子放大到 1.0×1.2×1.0 m。
#   bash nav2_gap_deep.sh         # headless
#   bash nav2_gap_deep.sh --gui
# 之后：python3 verify_narrow_gap.py --scenario gate_deep
#       → results/report_narrow_gap_deep.json
# 为什么单独做一档：0.6×0.5 的小门在 AMCL 里不触发漂移（实测 max 0.38 m）；
# 放大到 GUI 默认箱量级后，未建图回波能占 in-range 束的 70~90%，修复前实测
# 车钻出门后定位漂 2.9 m、nav2 ABORTED。这一档用来回归 AMCL 的
# likelihood_field_prob + do_beamskip 修复（见 make_nav2_params.py）。
# 停止：bash stop_nav2_field.sh
set +u
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
cd "$ROOT" || exit 1
source /opt/ros/jazzy/setup.bash 2>/dev/null

GUI_ARG=""
[ "${1:-}" = "--gui" ] && GUI_ARG="--gui"

echo "=== 0. 生成放大门世界（同源校核 + xacro 展开自检）==="
python3 make_obstacles_world.py --only gate_deep || { echo "致命: 放大门世界生成/校核失败"; exit 1; }
TMP=/tmp/gate_deep_world_check.sdf
xacro worlds/rmul2027_3v3_up_nav2_gap_deep.sdf.xacro headless:=true > "$TMP" 2>/tmp/xacro_err.txt \
  || { echo "致命: xacro 展开失败"; cat /tmp/xacro_err.txt; exit 1; }
grep -q 'name="gate_west"' "$TMP" || { echo "致命: 展开后没有 gate_west"; exit 1; }
echo "  xacro 展开 OK"

# 出生点：从门南侧 2.0 m 处起步（与窄通道验收同一起跑线，方便对比）
SPAWN=$(python3 -c "import json;print(' '.join(str(v) for v in json.load(open('config/avoid_obstacles.json'))['gate_deep']['spawn']))")
export NAV2_WORLD="$ROOT/worlds/rmul2027_3v3_up_nav2_gap_deep.sdf.xacro"
export RM_DYNAMIC=0
export RM_SPAWN_X="${SPAWN% *}"
export RM_SPAWN_Y="${SPAWN#* }"
echo "=== 1. 起栈（世界=$(basename "$NAV2_WORLD")，出生点 ($RM_SPAWN_X,$RM_SPAWN_Y)）==="
exec bash "$ROOT/nav2_field.sh" $GUI_ARG