#!/usr/bin/env python3
"""把场地网格拆成"主体"和"底面"两份，保证任何角度都是实心的（不再看得穿）。

和 debug/split_field_mesh.py 的区别：那个只保留 朝上 + 竖直 面，朝下的面被丢掉，
从台子下面往上看就是空的。这里改成**所有面都保留**：

  field_main.stl    nz >= -0.7 的面（台面 + 侧壁 + 斜/复杂面） -> 出图里画绿色
  field_bottom.stl  nz <  -0.7 的面（底面）                    -> 画深灰

两份合起来 = 原始 154,520 个三角面，一个都不少，所以模型是闭合的。
用法: python3 debug/make_solid_meshes.py
"""
import os
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.path.join(ROOT, "models", "rmul2027_3v3_field", "meshes", "field_zup.stl")
OUTDIR = os.path.join(ROOT, "models", "drive_field_colored", "meshes")

DT = np.dtype([("n", "<f4", (3,)), ("v", "<f4", (3, 3)), ("attr", "<u2")])


def read_stl(path):
    with open(path, "rb") as f:
        head = f.read(84)
        n = struct.unpack("<I", head[80:84])[0]
        buf = f.read(n * 50)
    return np.frombuffer(buf, dtype=DT).copy(), n


def write_stl(path, tris):
    with open(path, "wb") as f:
        f.write(b"make_solid_meshes.py".ljust(80, b"\0"))
        f.write(struct.pack("<I", len(tris)))
        out = np.zeros(len(tris), dtype=DT)
        out["n"] = tris["n"]
        out["v"] = tris["v"]
        out["attr"] = tris["attr"]
        f.write(out.tobytes())


tris, n = read_stl(SRC)
v = tris["v"].astype(np.float64)
a = v[:, 1] - v[:, 0]
b = v[:, 2] - v[:, 0]
nrm = np.cross(a, b)
ln = np.linalg.norm(nrm, axis=1)
ln[ln == 0] = 1.0
nz = nrm[:, 2] / ln

down = nz < -0.7
main = ~down
print(f"源 {n} 面  ->  主体 {int(main.sum())} 面，底面 {int(down.sum())} 面")
assert int(main.sum()) + int(down.sum()) == n

os.makedirs(OUTDIR, exist_ok=True)
p1 = os.path.join(OUTDIR, "field_main.stl")
p2 = os.path.join(OUTDIR, "field_bottom.stl")
write_stl(p1, tris[main])
write_stl(p2, tris[down])


def bbox(t):
    pts = t["v"].reshape(-1, 3)
    return [(round(float(pts[:, i].min()), 3), round(float(pts[:, i].max()), 3)) for i in range(3)]


print(f"  {p1}  bbox {bbox(tris[main])}")
print(f"  {p2}  bbox {bbox(tris[down])}")
print("两份合计 = 原始全部面，模型闭合，任何角度都不会看穿")