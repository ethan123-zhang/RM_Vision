#!/usr/bin/env python3
"""把场地网格的"可行驶顶面"栅格化成高度图：找出台阶/凹陷/凸起平台。

对每个 (x,y) 网格点取网格在该处的最高面 z，输出覆盖率、高度分层面积和 ASCII 图。
用法: python3 map_surface.py <field_zup.stl> [cell_m]
"""
import struct
import sys

import numpy as np

path = sys.argv[1]
cell = float(sys.argv[2]) if len(sys.argv) > 2 else 0.05

with open(path, "rb") as f:
    data = f.read()
n = struct.unpack("<I", data[80:84])[0]
rec = np.frombuffer(data[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
tris = rec[:, 12:48].copy().view("<f4").reshape(n, 3, 3).astype(np.float64)
print(f"{path}: {n} triangles")

x0, y0 = tris[:, :, 0].min(), tris[:, :, 1].min()
x1, y1 = tris[:, :, 0].max(), tris[:, :, 1].max()
nx = int(np.ceil((x1 - x0) / cell)) + 1
ny = int(np.ceil((y1 - y0) / cell)) + 1
xs = x0 + np.arange(nx) * cell
ys = y0 + np.arange(ny) * cell
height = np.full((ny, nx), np.nan)
print(f"grid {nx}x{ny} = {nx*ny} cells @ {cell} m")

for t in tris:
    (ax, ay, az), (bx, by, bz), (cx, cy, cz) = t
    # 只处理可能成为顶面的三角形
    if max(az, bz, cz) < -0.05:
        continue
    i0 = max(0, int((min(ax, bx, cx) - x0) / cell) - 1)
    i1 = min(nx - 1, int((max(ax, bx, cx) - x0) / cell) + 1)
    j0 = max(0, int((min(ay, by, cy) - y0) / cell) - 1)
    j1 = min(ny - 1, int((max(ay, by, cy) - y0) / cell) + 1)
    if i1 < i0 or j1 < j0:
        continue
    gx = xs[i0:i1 + 1][None, :]
    gy = ys[j0:j1 + 1][:, None]
    det = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
    if abs(det) < 1e-12:
        continue
    l1 = ((by - cy) * (gx - cx) + (cx - bx) * (gy - cy)) / det
    l2 = ((cy - ay) * (gx - cx) + (ax - cx) * (gy - cy)) / det
    l3 = 1.0 - l1 - l2
    inside = (l1 >= -1e-9) & (l2 >= -1e-9) & (l3 >= -1e-9)
    if not inside.any():
        continue
    z = l1 * az + l2 * bz + l3 * cz
    sub = height[j0:j1 + 1, i0:i1 + 1]
    upd = inside & (np.isnan(sub) | (z > sub))
    sub[upd] = z[upd]
    height[j0:j1 + 1, i0:i1 + 1] = sub

cov = ~np.isnan(height)
print(f"覆盖率: {100*cov.mean():.2f}%  ({cov.sum()} / {cov.size} cells)")
h = height[cov]
print(f"高度范围: {h.min():+.4f} .. {h.max():+.4f} m")

# 按 5 mm 分层统计面积
levels = np.round(h / 0.005) * 0.005
uniq, cnt = np.unique(levels, return_counts=True)
area = cell * cell
print("\n高度分层（面积占比 >0.05%）:")
for lv, c in zip(uniq, cnt):
    if c * area / (h.size * area) > 0.0005:
        print(f"  z={lv:+.3f} m  {c*area:8.2f} m^2  {100.0*c/len(h):6.2f}%")

# ASCII 图（只画覆盖到的）
step = max(1, nx // 84)
print("\n顶面高度 ASCII（' '=无覆盖  '.'=0  '1'..'9'=低于 0 的台阶，单位 -20 mm）:")
for j in range(0, ny, step * 2):
    line = []
    for i in range(0, nx, step):
        v = height[j, i]
        if np.isnan(v):
            line.append(" ")
        elif abs(v) < 0.002:
            line.append(".")
        else:
            level = int(round(-v / 0.02))
            line.append(str(min(9, max(1, level))))
    print("  " + "".join(line))
print(f"\n  图例: '.' = z≈0（台面）；数字 = 比台面低 n×20 mm；行间距 {step*2*cell:.2f} m")