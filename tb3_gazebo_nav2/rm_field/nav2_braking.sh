#!/usr/bin/env bash
# 刹车验收：起"翻转场地 + 弹出挡板 + Nav2"（安全层 collision_monitor 验收用）。
#   bash nav2_braking.sh         # headless（验收）
#   bash nav2_braking.sh --gui   # 带 Gazebo GUI / RViz
# 之后：python3 verify_braking.py  → results/report_collision_monitor.json
# 停止：bash stop_nav2_field.sh（已包含挡板驱动）
set +u
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
cd "$ROOT" || exit 1
source /opt/ros/jazzy/setup.bash 2>/dev/null

GUI_ARG=""
[ "${1:-}" = "--gui" ] && GUI_ARG="--gui"

echo "=== 0. 生成刹车世界（同源校核 + xacro 展开自检）==="
python3 make_obstacles_world.py --only braking || { echo "致命: 刹车世界生成/校核失败"; exit 1; }
TMP=/tmp/braking_world_check.sdf
xacro worlds/rmul2027_3v3_up_nav2_braking.sdf.xacro headless:=true > "$TMP" 2>/tmp/xacro_err.txt \
  || { echo "致命: xacro 展开失败"; cat /tmp/xacro_err.txt; exit 1; }
grep -q 'name="popup_barrier"' "$TMP" || { echo "致命: 展开后没有 popup_barrier"; exit 1; }
echo "  xacro 展开 OK"

export NAV2_WORLD="$ROOT/worlds/rmul2027_3v3_up_nav2_braking.sdf.xacro"
export RM_DYNAMIC=0
echo "=== 1. 起栈（世界=$(basename "$NAV2_WORLD")）==="
bash "$ROOT/nav2_field.sh" $GUI_ARG || exit 1

echo "=== 2. 弹出挡板驱动（先藏起来，等机器人靠近）==="
bash "$ROOT/start_popup.sh"