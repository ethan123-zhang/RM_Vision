#!/usr/bin/env python3
"""按高度汇总场地网格的水平面面积，用来判断"哪一面是台面、哪一面是结构"。"""
import collections
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
area = 0.5 * ln
cz = v[:, :, 2].mean(axis=1)

h = np.abs(nz) > 0.7
acc = collections.Counter()
for zz, ar, u in zip(np.round(cz[h], 2), area[h], nz[h] > 0):
    acc[(float(zz), bool(u))] += float(ar)

print("水平面按高度汇总（面积 > 1 m2 的）:")
for (zz, u), ar in sorted(acc.items()):
    if ar > 1.0:
        print("  z=%+.2f  %s  %8.1f m2" % (zz, "朝上" if u else "朝下", ar))

print()
print("朝上的水平面合计 %.1f m2 / 朝下合计 %.1f m2"
      % (sum(a for (z, u), a in acc.items() if u), sum(a for (z, u), a in acc.items() if not u)))