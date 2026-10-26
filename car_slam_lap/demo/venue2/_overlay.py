"""把 record_path 录的轨迹叠到 map_from_cloud 的俯视图上（诊断用，不依赖 ROS）。

输入：同目录下 path.yaml + map.pcd
输出：path_on_bev.png（复现 map_from_cloud 的坐标系/分辨率，再画红/绿/蓝）
"""
import math
import os
import re

import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
PCD = os.path.join(HERE, "map.pcd")
PATHY = os.path.join(HERE, "path.yaml")
OUT = os.path.join(HERE, "path_on_bev.png")

Z_MIN, Z_MAX = -0.30, 2.00
RES = 0.05
MARGIN = 0.5

# ---------------- 读点云 ----------------
with open(PCD, "r", encoding="ascii", errors="ignore") as fh:
    lines = fh.readlines()
start = next(i for i, ln in enumerate(lines) if ln.strip().upper().startswith("DATA"))
pts = np.loadtxt(lines[start + 1:], dtype=np.float64)
print(f"pcd: {len(pts)} 点")

band = pts[(pts[:, 2] >= Z_MIN) & (pts[:, 2] <= Z_MAX)]
print(f"z 带内: {len(band)} 点")

lo = band[:, :2].min(axis=0) - MARGIN
hi = band[:, :2].max(axis=0) + MARGIN
w = int(math.ceil((hi[0] - lo[0]) / RES))
h = int(math.ceil((hi[1] - lo[1]) / RES))

img = np.full((h, w, 3), 255, dtype=np.uint8)
cx = ((band[:, 0] - lo[0]) / RES).astype(np.int32)
cy = (h - 1 - (band[:, 1] - lo[1]) / RES).astype(np.int32)
ok = (cx >= 0) & (cx < w - 1) & (cy >= 0) & (cy < h - 1)
cx, cy = cx[ok], cy[ok]
for dx in (0, 1):
    for dy in (0, 1):
        img[cy + dy, cx + dx] = 0

# ---------------- 读轨迹 ----------------
pat = re.compile(r"x:\s*(-?[\d.]+),\s*y:\s*(-?[\d.]+),\s*yaw:\s*(-?[\d.]+),\s*t:\s*(-?[\d.]+)")
tp = []
with open(PATHY, "r", encoding="utf-8") as fh:
    for ln in fh:
        m = pat.search(ln)
        if m:
            tp.append((float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))))
tp = np.array(tp)
print(f"path: {len(tp)} 点, 长度 {np.hypot(np.diff(tp[:, 0]), np.diff(tp[:, 1])).sum():.2f} m")

def to_px(x, y):
    return int(round((x - lo[0]) / RES)), int(round(h - 1 - (y - lo[1]) / RES))

im = Image.fromarray(img)
dr = ImageDraw.Draw(im)
for a, b in zip(tp[:-1], tp[1:]):
    dr.line([to_px(a[0], a[1]), to_px(b[0], b[1])], fill=(220, 30, 30), width=2)
sx, sy = to_px(tp[0][0], tp[0][1])
ex, ey = to_px(tp[-1][0], tp[-1][1])
dr.ellipse([sx - 6, sy - 6, sx + 6, sy + 6], outline=(0, 140, 0), width=3)
dr.ellipse([ex - 6, ey - 6, ex + 6, ey + 6], outline=(0, 80, 220), width=3)
font = ImageFont.load_default()
dr.text((6, 6), f"map {w * RES:.1f} x {h * RES:.1f} m | path {len(tp)} pts "
                f"{np.hypot(np.diff(tp[:, 0]), np.diff(tp[:, 1])).sum():.1f} m",
        fill=(0, 0, 0), font=font)
dr.text((6, h - 16), "red = recorded path   green = start   blue = end", fill=(0, 0, 0), font=font)
im.save(OUT)
print(f"wrote {OUT} ({w}x{h} px)")

# ---------------- 量化：轨迹有多少落在已建图区域 ----------------
occ = np.zeros((h, w), dtype=bool)
occ[cy, cx] = True
# 把占据栅格膨胀 0.25 m（5 格），容忍点云稀疏
k = 5
dil = occ.copy()
for d in range(1, k + 1):
    dil[d:, :] |= occ[:-d, :]
    dil[:-d, :] |= occ[d:, :]
    dil[:, d:] |= occ[:, :-d]
    dil[:, :-d] |= occ[:, d:]
hits = 0
for x, y in tp[:, :2]:
    px, py = to_px(x, y)
    if 0 <= px < w and 0 <= py < h and dil[py, px]:
        hits += 1
print(f"轨迹点落在已建图区域(0.25 m 容差)内的比例: {hits}/{len(tp)} = {100.0 * hits / len(tp):.1f}%")
