#!/usr/bin/env python3
"""把场地网格"背面"的水平面按深度画成 ASCII 地形图，看看那一面到底是什么形状。

对每个 0.15 m 网格，记录该格里出现的水平面高度（-0.10 / -0.11 / -0.30 / -0.48 / -0.50），
用不同字符打印，形成背面"浮雕"的平面图。
"""
import struct

import numpy as np

P = "/mnt/e/RM/tb3_gazebo_nav2/rm_field/models/rmul2027_3v3_field/meshes/field_zup.stl"
d = open(P, "rb").read()
n = struct.unpack("<I", d[80:84])[0]
dt = np.dtype([("n", "<f4", (3,)), ("v", "<f4", (3, 3)), ("a", "<u2")])
t = np.frombuffer(d[84:84 + n * 50], dtype=dt)
v = t["v"].astype(float)
a = v[:, 1] - v[:, 0]
b = v[:, 2] - v[:, 0]
nr = np.cross(a, b)
ln = np.linalg.norm(nr, axis=1)
ln[ln == 0] = 1
nz = nr[:, 2] / ln
cz = v[:, :, 2].mean(axis=1)
mx = v[:, :, 0].mean(axis=1)
my = v[:, :, 1].mean(axis=1)

h = np.abs(nz) > 0.7
levels = [(-0.10, "a"), (-0.11, "b"), (-0.30, "c"), (-0.48, "d"), (-0.50, "e")]
NX, NY = 84, 58
xs = np.linspace(-6.3, 6.3, NX + 1)
ys = np.linspace(-4.3, 4.3, NY + 1)
ix = np.clip(np.digitize(mx, xs) - 1, 0, NX - 1)
iy = np.clip(np.digitize(my, ys) - 1, 0, NY - 1)

grid = np.full((NY, NX), ".", dtype="<U1")
for lz, ch in levels:
    m = h & (np.abs(cz - lz) < 0.01)
    # 同一格取"最靠外（最低）"的层，先粗后细覆盖
    grid[iy[m], ix[m]] = ch

print("背面水平面地形图（a=-0.10  b=-0.11  c=-0.30  d=-0.48  e=-0.50   .=无水平面）")
print("    " + "".join(str((i // 10) % 10) if i % 10 == 0 else " " for i in range(NX)))
for j in range(NY - 1, -1, -1):
    y = 0.5 * (ys[j] + ys[j + 1])
    print("%+5.1f %s" % (y, "".join(grid[j])))