#!/usr/bin/env python3
"""把场地网格上下翻转（z -> -z，并反转三角形绕序保持法向朝外），
用来验证"这份 CAD 是不是上下装反了"。

产出: models/probe_field_flipped/meshes/field_flipped.stl
用法: python3 debug/flip_field_mesh.py
"""
import os
import struct

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.path.join(ROOT, "models", "rmul2027_3v3_field", "meshes", "field_zup.stl")
OUTDIR = os.path.join(ROOT, "models", "probe_field_flipped", "meshes")
OUT = os.path.join(OUTDIR, "field_flipped.stl")

DT = np.dtype([("n", "<f4", (3,)), ("v", "<f4", (3, 3)), ("attr", "<u2")])

d = open(SRC, "rb").read()
n = struct.unpack("<I", d[80:84])[0]
t = np.frombuffer(d[84:84 + n * 50], dtype=DT).copy()

v = t["v"].copy()
v[:, :, 2] *= -1.0     # 上下翻转
v[:, [1, 2]] = v[:, [2, 1]]  # 反转绕序，法向保持朝外
t["v"] = v
# 记录用的法向也翻转 z
t["n"] = t["n"] * np.array([1, 1, -1], dtype=np.float32)

os.makedirs(OUTDIR, exist_ok=True)
with open(OUT, "wb") as f:
    f.write(b"flipped for orientation check".ljust(80, b"\0"))
    f.write(struct.pack("<I", n))
    f.write(t.tobytes())

pts = t["v"].reshape(-1, 3)
print("flipped mesh ->", OUT)
print("bbox", [(round(float(pts[:, i].min()), 3), round(float(pts[:, i].max()), 3)) for i in range(3)])