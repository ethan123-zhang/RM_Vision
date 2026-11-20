#!/usr/bin/env python3
"""列出与给定矩形区域重叠的场地方块（来自 debug/box_collisions.json）。

用法：
  python3 list_boxes.py --x 1.6 3.4 --y 2.6 4.0
"""
import argparse
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument("--x", nargs=2, type=float, required=True)
ap.add_argument("--y", nargs=2, type=float, required=True)
args = ap.parse_args()

with open(os.path.join(HERE, "debug", "box_collisions.json"), "r", encoding="utf-8") as f:
    boxes = json.load(f)["boxes"]

sel = []
for b in boxes:
    cx, cy, _ = b["center"]
    sx, sy, _ = b["size"]
    if (cx + sx / 2 >= args.x[0] and cx - sx / 2 <= args.x[1]
            and cy + sy / 2 >= args.y[0] and cy - sy / 2 <= args.y[1]):
        sel.append(b)
sel.sort(key=lambda b: (-b["center"][1], b["center"][0]))
print(f"与 x[{args.x[0]},{args.x[1]}] y[{args.y[0]},{args.y[1]}] 重叠的方块 {len(sel)} 个：")
for b in sel:
    cx, cy, cz = b["center"]
    sx, sy, sz = b["size"]
    print(f"  {b['name']:>12}  x[{cx-sx/2:6.3f},{cx+sx/2:6.3f}] "
          f"y[{cy-sy/2:6.3f},{cy+sy/2:6.3f}] z_center={cz:.2f} size=({sx:.2f},{sy:.2f},{sz:.2f}) "
          f"top={b.get('top', 0):.2f}")