#!/usr/bin/env bash
# 超窄门（0.62 m，车宽 0.44）验收：默认用 **2.5 cm 碰撞栅格变体**（模型/世界/地图都带 _25mm），
# 加 --grid 5cm 则用 5 cm 基线做对照——用来回答 README §17.6/§18.3 的欠账
# "门洞 <0.70 m 时，5 cm 栅格够不够、要不要 2.5 cm 变体"。
#   bash nav2_gap_tight.sh                 # 2.5 cm 变体（默认）
#   bash nav2_gap_tight.sh --grid 5cm      # 5 cm 基线对照
#   bash nav2_gap_tight.sh --gui
# 之后：
#   python3 verify_narrow_gap.py --scenario gate_tight --grid 25mm
#   python3 verify_narrow_gap.py --scenario gate_tight --grid 5cm --tag 5cm
#   → results/report_narrow_gap_tight[_5cm].json
# 停止：bash stop_nav2_field.sh
# 前置（2.5 cm 变体）：先跑 bash make_25mm_variant.sh 生成模型/基世界/地图。
set +u
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
cd "$ROOT" || exit 1
source /opt/ros/jazzy/setup.bash 2>/dev/null

GUI_ARG=""
GRID="25mm"
while [ $# -gt 0 ]; do
  case "$1" in
    --gui) GUI_ARG="--gui" ;;
    --grid) GRID="${2:-}"; shift ;;
    --grid=*) GRID="${1#--grid=}" ;;
    *) echo "未知参数: $1"; exit 2 ;;
  esac
  shift
done

case "$GRID" in
  25mm)
    VARIANT="gate_tight_25mm"
    WORLD="$ROOT/worlds/rmul2027_3v3_up_nav2_gate_tight_25mm.sdf.xacro"
    MAP="$ROOT/maps/rmul2027_3v3_up_25mm.yaml"
    TAG="25mm"
    ;;
  5cm|5)
    GRID="5cm"
    VARIANT="gate_tight"
    WORLD="$ROOT/worlds/rmul2027_3v3_up_nav2_gate_tight.sdf.xacro"
    MAP="$ROOT/maps/rmul2027_3v3_up.yaml"
    TAG="5cm"
    ;;
  *) echo "未知 grid: $GRID（可选 25mm / 5cm）"; exit 2 ;;
esac

echo "=== 0. 生成超窄门世界（$GRID 栅格，world 与验收同源）==="
if [ ! -f "$MAP" ]; then
  echo "致命: 缺地图 $MAP"
  [ "$TAG" = "25mm" ] && echo "      先跑 bash make_25mm_variant.sh"
  exit 1
fi
python3 make_obstacles_world.py --only "$VARIANT" || { echo "致命: 超窄门世界生成/校核失败"; exit 1; }
TMP=/tmp/gate_tight_world_check.sdf
xacro "$WORLD" headless:=true > "$TMP" 2>/tmp/xacro_err.txt \
  || { echo "致命: xacro 展开失败"; cat /tmp/xacro_err.txt; exit 1; }
for name in gate_west gate_east; do
  grep -q "name=\"$name\"" "$TMP" || { echo "致命: 展开后缺 $name"; exit 1; }
done
echo "  xacro 展开 OK（世界=$(basename "$WORLD")）"

SPAWN=$(python3 -c "import json;print(' '.join(str(v) for v in json.load(open('config/avoid_obstacles.json'))['gate_tight']['spawn']))")
export NAV2_WORLD="$WORLD"
export NAV2_MAP="$MAP"
export RM_DYNAMIC=0
export RM_SPAWN_X="${SPAWN% *}"
export RM_SPAWN_Y="${SPAWN#* }"
echo "=== 1. 起栈（栅格=$GRID，地图=$(basename "$MAP")，出生点 ($RM_SPAWN_X,$RM_SPAWN_Y)）==="
bash "$ROOT/nav2_field.sh" $GUI_ARG || exit 1
echo "=== 2. 验收命令 ==="
if [ "$TAG" = "25mm" ]; then
  echo "  python3 verify_narrow_gap.py --scenario gate_tight --grid 25mm"
else
  echo "  python3 verify_narrow_gap.py --scenario gate_tight --grid 5cm --tag 5cm"
fi