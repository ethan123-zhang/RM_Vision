#!/usr/bin/env bash
# 生成"2.5 cm 碰撞栅格"变体（不碰 5 cm 基线）：
#   models/rmul2027_3v3_field_up_25mm/         只换碰撞 box（块数约 4×），视觉网格仍引用原模型
#   debug/box_collisions_25mm.json             2.5 cm 高度图生成的 box 列表（地图脚本的几何源）
#   worlds/rmul2027_3v3_up_nav2_25mm.sdf.xacro 基世界，include 变体模型
#   maps/rmul2027_3v3_up_25mm.pgm/.yaml        由变体 box 生成（0.025 m/px）+ 独立校核
#   results/report_nav_map_up_25mm.json        地图校核报告
# 为什么单独开变体而不是改基线：0.80 m 门洞已用 5 cm 栅格实测通过（README §18），
# 基线不能动；2.5 cm 只服务 <0.70 m 门洞（README §17.6 的欠账）。
# 用法：bash make_25mm_variant.sh
set +u
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
cd "$ROOT" || exit 1
source /opt/ros/jazzy/setup.bash 2>/dev/null
BASE_MODEL="$ROOT/models/rmul2027_3v3_field_up"
VAR_MODEL="$ROOT/models/rmul2027_3v3_field_up_25mm"
BASE_WORLD="$ROOT/worlds/rmul2027_3v3_up_nav2.sdf.xacro"
VAR_WORLD="$ROOT/worlds/rmul2027_3v3_up_nav2_25mm.sdf.xacro"
BOXES_JSON="$ROOT/debug/box_collisions_25mm.json"

[ -f "$BASE_MODEL/model.sdf" ] || { echo "致命: 缺基线模型 $BASE_MODEL/model.sdf"; exit 1; }
[ -f "$BASE_WORLD" ] || { echo "致命: 缺基线世界 $BASE_WORLD"; exit 1; }
mkdir -p "$VAR_MODEL" "$ROOT/maps" "$ROOT/results" "$ROOT/shots"

echo "=== 0. 变体模型目录（首次创建：复制 model.sdf 并改模型名；视觉网格仍指向原模型）==="
if [ ! -f "$VAR_MODEL/model.sdf" ]; then
  cp "$BASE_MODEL/model.sdf" "$VAR_MODEL/model.sdf"
  python3 - "$VAR_MODEL/model.sdf" <<'PY'
import sys
p = sys.argv[1]
s = open(p, encoding="utf-8").read()
old = '<model name="rmul2027_3v3_field_up">'
new = '<model name="rmul2027_3v3_field_up_25mm">'
if old not in s:
    raise SystemExit("model.sdf 里找不到基线模型名")
open(p, "w", encoding="utf-8").write(s.replace(old, new, 1))
PY
  echo "  已创建 $VAR_MODEL/model.sdf（模型名已改为 _25mm）"
else
  echo "  已存在 $VAR_MODEL/model.sdf（复用；下面会重写碰撞块）"
fi
if [ ! -f "$VAR_MODEL/model.config" ]; then
  cat > "$VAR_MODEL/model.config" <<'XML'
<?xml version="1.0"?>
<model>
  <name>rmul2027_3v3_field_up_25mm</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author>lead</author>
  <description>RMUL2027 3V3 场地（翻转版）的 2.5 cm 碰撞栅格变体：
  碰撞用 2.5 cm 高度图生成的 box 拼块（其余与 rmul2027_3v3_field_up 相同，
  视觉网格仍引用 rmul2027_3v3_field_up/meshes）。只服务 &lt;0.70 m 门洞验收。</description>
</model>
XML
  echo "  已创建 $VAR_MODEL/model.config"
fi

echo
echo "=== 1. 2.5 cm 碰撞 box（写回变体 model.sdf + $BOXES_JSON）==="
python3 "$ROOT/debug/make_box_collisions.py" --cell 0.025 \
  --model-dir "$VAR_MODEL" --src "$BASE_MODEL/meshes/structures.stl" \
  --out-json "$BOXES_JSON" || { echo "致命: 2.5 cm 碰撞体生成失败"; exit 1; }

echo
echo "=== 2. 变体基世界（include 换成 _25mm 模型，world 名保持 rmul2027_3v3）==="
python3 - "$BASE_WORLD" "$VAR_WORLD" <<'PY'
import sys
src, dst = sys.argv[1], sys.argv[2]
s = open(src, encoding="utf-8").read()
s = s.replace("model://rmul2027_3v3_field_up</uri>",
              "model://rmul2027_3v3_field_up_25mm</uri>")
s = s.replace("<name>rmul2027_3v3_field_up</name>",
              "<name>rmul2027_3v3_field_up_25mm</name>")
if "field_up_25mm" not in s:
    raise SystemExit("世界替换失败：找不到基模型 include")
open(dst, "w", encoding="utf-8").write(s)
print("  写:", dst)
PY

echo
echo "=== 3. 地图（2.5 cm 碰撞体 → 0.025 m/px，含独立校核）==="
python3 "$ROOT/make_nav_map_up.py" \
  --boxes "$BOXES_JSON" --tag rmul2027_3v3_up_25mm \
  --report "$ROOT/results/report_nav_map_up_25mm.json" \
  --overlay "$ROOT/shots/nav_map_up_25mm_overlay.png"
RC=$?

echo
echo "=== 落盘文件 ==="
ls -l "$VAR_MODEL/model.sdf" "$VAR_MODEL/model.config" "$BOXES_JSON" "$VAR_WORLD" \
      "$ROOT/maps/rmul2027_3v3_up_25mm.yaml" "$ROOT/maps/rmul2027_3v3_up_25mm.pgm" 2>/dev/null
exit $RC