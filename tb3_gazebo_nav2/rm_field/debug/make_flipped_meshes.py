#!/usr/bin/env python3
"""把场地模型上下翻转，并按"地面 / 内部凸起 / 外围墙"拆成几份网格分色：

  floor.stl     平整的场地地面（翻转后 z≈0）                 -> 浅灰（可行驶）
  inner.stl     内部凸起的平台/结构（抬高 0.19 / 0.37 m）    -> 绿色（凸起部分）
  wall.stl      外围墙墙体（外围 0.3 m 宽一圈，到 +0.40）     -> 红色（围栏）
  wall_top.stl  外围墙顶沿（z 最高那一圈）                    -> 黄色（顶沿）
  structures.stl 除地面外的全部面（给碰撞用）

翻转: z -> -z，反转三角形绕序，再整体 -0.10 m，使场地地面落在 z = 0。
"""
import os
import struct

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.path.join(ROOT, "models", "rmul2027_3v3_field", "meshes", "field_zup.stl")
OUT = os.path.join(ROOT, "models", "rmul2027_3v3_field_up", "meshes")

DT = np.dtype([("n", "<f4", (3,)), ("v", "<f4", (3, 3)), ("attr", "<u2")])


def read_stl(path):
    d = open(path, "rb").read()
    n = struct.unpack("<I", d[80:84])[0]
    return np.frombuffer(d[84:84 + n * 50], dtype=DT).copy(), n


def write_stl(path, tris):
    with open(path, "wb") as f:
        f.write(b"flipped + color split".ljust(80, b"\0"))
        f.write(struct.pack("<I", len(tris)))
        out = np.zeros(len(tris), dtype=DT)
        out["n"] = tris["n"]
        out["v"] = tris["v"]
        out["attr"] = tris["attr"]
        f.write(out.tobytes())


t, n = read_stl(SRC)
v = t["v"].copy()
v[:, :, 2] *= -1.0
v[:, [1, 2]] = v[:, [2, 1]]
v[:, :, 2] -= 0.10
t["v"] = v
t["n"] = np.stack([t["n"][:, 0], -t["n"][:, 2], -t["n"][:, 1]], axis=1)

a = v[:, 1] - v[:, 0]
b = v[:, 2] - v[:, 0]
nr = np.cross(a, b)
ln = np.linalg.norm(nr, axis=1)
ln[ln == 0] = 1
nz = nr[:, 2] / ln
cz = v[:, :, 2].mean(axis=1)
cx = v[:, :, 0].mean(axis=1)
cy = v[:, :, 1].mean(axis=1)

is_floor = (nz > 0.7) & (np.abs(cz) < 0.03)
is_base = (nz < -0.7) & (cz < -0.05)          # 台体底板（朝下，z≈-0.10）
outer = (np.abs(cx) > 6.00) | (np.abs(cy) > 4.00)
is_wall = (~is_floor) & (~is_base) & outer
is_wall_top = is_wall & (cz > 0.35)
is_wall_side = is_wall & (~is_wall_top)
is_inner = (~is_floor) & (~is_base) & (~is_wall)
is_struct = ~is_floor

os.makedirs(OUT, exist_ok=True)
for name, mask in [("floor", is_floor), ("inner", is_inner), ("wall", is_wall_side),
                   ("wall_top", is_wall_top), ("base", is_base), ("structures", is_struct)]:
    write_stl(os.path.join(OUT, name + ".stl"), t[mask])
    print("  %-11s %6d 面  面积 %6.1f m2" % (name, int(mask.sum()), float((0.5 * ln)[mask].sum())))

print("翻转 + 下移 0.10 m 后 bbox:",
      [(round(float(v[:, :, i].min()), 3), round(float(v[:, :, i].max()), 3)) for i in range(3)])