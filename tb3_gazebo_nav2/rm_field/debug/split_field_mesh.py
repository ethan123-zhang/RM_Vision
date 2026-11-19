#!/usr/bin/env python3
"""把场地的 field_zup.stl 按面法向拆成两个网格，供"颜色标注"出图用：

  field_top.stl    水平朝上的面（z≈0 的台面）       -> 出图里画成浅灰
  field_walls.stl  竖直面（|nz|<0.3，即 0.503 m 侧壁）-> 出图里画成橙色（"凸起的部分"）

几何本身一个顶点都不改，只是按朝向分类后写成两个文件。
用法: python3 debug/split_field_mesh.py [原STL] [输出目录]
"""
import os
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    ROOT, "models", "rmul2027_3v3_field", "meshes", "field_zup.stl")
OUTDIR = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
    ROOT, "models", "debug_field_colored", "meshes")

DT = np.dtype([("n", "<f4", (3,)), ("v", "<f4", (3, 3)), ("attr", "<u2")])


def read_stl(path):
    with open(path, "rb") as f:
        head = f.read(84)
        if len(head) < 84:
            raise SystemExit("文件太短，不像二进制 STL")
        n = struct.unpack("<I", head[80:84])[0]
        buf = f.read(n * 50)
    if len(buf) != n * 50:
        raise SystemExit("二进制 STL 长度与面数不符（可能是 ASCII STL）")
    return np.frombuffer(buf, dtype=DT).copy(), n


def write_stl(path, tris):
    with open(path, "wb") as f:
        f.write(b"split by debug/split_field_mesh.py".ljust(80, b"\0"))
        f.write(struct.pack("<I", len(tris)))
        out = np.zeros(len(tris), dtype=DT)
        out["n"] = tris["n"]
        out["v"] = tris["v"]
        out["attr"] = tris["attr"]
        f.write(out.tobytes())


tris, n = read_stl(SRC)
v = tris["v"].astype(np.float64)          # (n, 3 顶点, 3 坐标)
# 用顶点重算法向，比文件里存的法向可靠
a = v[:, 1] - v[:, 0]
b = v[:, 2] - v[:, 0]
nrm = np.cross(a, b)
ln = np.linalg.norm(nrm, axis=1)
ln[ln == 0] = 1.0
nz = (nrm[:, 2] / ln)

top = tris[nz > 0.7]
walls = tris[np.abs(nz) < 0.3]
other = tris.shape[0] - len(top) - len(walls)

os.makedirs(OUTDIR, exist_ok=True)
p_top = os.path.join(OUTDIR, "field_top.stl")
p_wall = os.path.join(OUTDIR, "field_walls.stl")
write_stl(p_top, top)
write_stl(p_wall, walls)


def bbox(t):
    pts = t["v"].reshape(-1, 3)
    return [(float(pts[:, i].min()), float(pts[:, i].max())) for i in range(3)]


print(f"源网格 {SRC}")
print(f"  三角面 {n}，bbox {bbox(tris)}")
print(f"台面(朝上) {len(top)} 面 -> {p_top}  bbox {bbox(top)}")
print(f"侧壁(竖直) {len(walls)} 面 -> {p_wall} bbox {bbox(walls)}")
print(f"其余(朝下等) {other} 面：不参与出图")