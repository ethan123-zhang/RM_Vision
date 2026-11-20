#!/usr/bin/env bash
# 绕障验收：起"翻转后 3V3 场地 + 静态/动态障碍物 + Nav2"。
#   bash nav2_obstacles.sh                 # 静态 2 箱 + 动态 1 箱（验收默认）
#   bash nav2_obstacles.sh --static-only   # 只静态箱（动态驱动不拉）
#   bash nav2_obstacles.sh --gui           # 带 Gazebo GUI / RViz 看画面
# 前置：障碍物布局在 config/avoid_obstacles.json（几何唯一源）；
#       本脚本每次都重新生成世界并做同源校核 + xacro 展开自检，不会用旧世界跑验收。
# 验收：python3 verify_nav2_obstacles.py → results/report_nav2_obstacles.json
# 停止：bash stop_nav2_field.sh（已包含动态障碍物驱动）
set +u
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
cd "$ROOT" || exit 1
source /opt/ros/jazzy/setup.bash 2>/dev/null

GUI_ARG=""
DYN=1
for a in "$@"; do
  case "$a" in
    --gui) GUI_ARG="--gui" ;;
    --static-only|--no-dynamic) DYN=0 ;;
    *) echo "未知参数: $a"; exit 2 ;;
  esac
done

echo "=== 0. 生成障碍物世界（world 与验收同源）==="
python3 make_obstacles_world.py || { echo "致命: 障碍物世界生成/校核失败"; exit 1; }
TMP=/tmp/obstacles_world_check.sdf
xacro worlds/rmul2027_3v3_up_nav2_obstacles.sdf.xacro headless:=true > "$TMP" 2>/tmp/xacro_err.txt \
  || { echo "致命: xacro 展开失败"; cat /tmp/xacro_err.txt; exit 1; }
for name in $(python3 -c "import json;c=json.load(open('config/avoid_obstacles.json'));print(' '.join([o['name'] for o in c['static']]+[c['dynamic']['name']]))"); do
  grep -q "name=\"$name\"" "$TMP" || { echo "致命: xacro 展开后缺 $name"; exit 1; }
done
echo "  xacro 展开 OK（障碍物齐全）"

export NAV2_WORLD="$ROOT/worlds/rmul2027_3v3_up_nav2_obstacles.sdf.xacro"
export RM_DYNAMIC="$DYN"
echo "=== 1. 起栈（世界=$(basename "$NAV2_WORLD")，动态=$DYN）==="
exec bash "$ROOT/nav2_field.sh" $GUI_ARG