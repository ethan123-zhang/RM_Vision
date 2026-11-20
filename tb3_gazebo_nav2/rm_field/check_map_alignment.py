#!/usr/bin/env python3
"""核对"地图坐标系"和"场地方块几何"是否严格对齐（离线，无需仿真）。

地图是由 make_nav_map_up.py 从 debug/box_collisions.json 的 48 个方块投影生成的，
所以理论上逐格一致；这个脚本就是验证这一点，并顺便搜索是否存在系统性平移/旋转。

判据：把每个方块按 yaml 的 resolution/origin 投影到地图栅格，统计这些格子里
"地图标为占据"的比例（覆盖率）；覆盖率接近 100% = 对齐良好。
"""
import json
import math
import os

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
PGM = os.path.join(HERE, "maps", "rmul2027_3v3_up.pgm")
BOXES = os.path.join(HERE, "debug", "box_collisions.json")
RES, OX, OY = 0.025, -6.4, -4.4

a = np.array(Image.open(PGM))
H, W = a.shape
occ = a < 80


def cells_for_box(cx, cy, sx, sy):
    x0, x1 = cx - sx / 2, cx + sx / 2
    y0, y1 = cy - sy / 2, cy + sy / 2
    c0 = int(math.floor((x0 - OX) / RES))
    c1 = int(math.ceil((x1 - OX) / RES))
    r0 = H - 1 - int(math.ceil((y1 - OY) / RES))
    r1 = H - 1 - int(math.floor((y0 - OY) / RES))
    return max(0, r0), min(H, r1 + 1), max(0, c0), min(W, c1 + 1)


with open(BOXES, "r", encoding="utf-8") as f:
    boxes = json.load(f)["boxes"]

print(f"地图 {W}x{H} @ {RES} m，origin ({OX},{OY})；方块 {len(boxes)} 个")
bad = []
for b in boxes:
    cx, cy, _ = b["center"]
    sx, sy, _ = b["size"]
    r0, r1, c0, c1 = cells_for_box(cx, cy, sx, sy)
    sub = occ[r0:r1, c0:c1]
    cover = sub.mean() if sub.size else 0.0
    if cover < 0.9:
        bad.append((b["name"], round(cover, 3), (round(cx, 2), round(cy, 2))))
print(f"逐方块覆盖率 ≥90% 的: {len(boxes) - len(bad)}/{len(boxes)}")
for name, cover, c in bad:
    print(f"  ! {name} 覆盖率 {cover:.2f} @ {c}")

# 搜索系统性平移（±0.3 m，1 格 = 2.5 cm）：看整体覆盖率能否明显提升
best = (0, 0, None)
all_cells = []
for b in boxes:
    cx, cy, _ = b["center"]
    sx, sy, _ = b["size"]
    all_cells.append(cells_for_box(cx, cy, sx, sy))


def cover_at(dc, dr):
    tot = ok = 0
    for r0, r1, c0, c1 in all_cells:
        sub = occ[max(0, r0 + dr):min(H, r1 + dr), max(0, c0 + dc):min(W, c1 + dc)]
        ok += int(sub.sum())
        tot += sub.size
    return ok / tot if tot else 0.0


base = cover_at(0, 0)
for dc in range(-8, 9):
    for dr in range(-8, 9):
        cv = cover_at(dc, dr)
        if cv > best[0]:
            best = (cv, (dc, dr), None)
print(f"整体覆盖率（无平移）{base*100:.1f}%；最优点是平移 ({best[1][0]*RES:+.3f},"
      f"{best[1][1]*RES:+.3f}) m → {best[0]*100:.1f}%")
if base > 0.95 and best[0] - base < 0.02:
    print("结论：地图与场地方块几何对齐（没有系统性平移/旋转）")
else:
    print("结论：地图与方块几何存在系统性偏差，需要检查 make_nav_map_up.py 的坐标/翻转")