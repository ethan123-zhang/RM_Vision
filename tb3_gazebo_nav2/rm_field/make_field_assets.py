#!/usr/bin/env python3
"""把 field.stl 转成 Gazebo 用的 Z-up 米制网格，并做网格/碰撞体体检。

输入（模型坐标，Y 向上，单位米）:  build/field.stl
输出: models/rmul2027_3v3_field/meshes/field_zup.stl   (Z 向上，场地台面 z=0)
      results/report_geometry.json

变换: (x, y, z)_model -> (x, -z, y - 0.1)
  * 模型 Y(向上) -> 世界 Z；场地台面 y=+0.100 m -> z=0
  * 模型 Z(8.6 方向) -> 世界 -Y；场地位于原点对称，符号不影响
"""
import json
import os
import struct
import sys

import numpy as np

ROOT = os.environ.get("FIELD_ROOT") or os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(ROOT, "build", "field.stl")
MESH_DIR = os.path.join(ROOT, "models", "rmul2027_3v3_field", "meshes")
DST = os.path.join(MESH_DIR, "field_zup.stl")
OUT = os.path.join(ROOT, "results")
os.makedirs(MESH_DIR, exist_ok=True)
os.makedirs(OUT, exist_ok=True)


def read_stl(path):
    with open(path, "rb") as f:
        f.seek(80)
        n = struct.unpack("<I", f.read(4))[0]
        raw = np.frombuffer(f.read(n * 50), dtype=np.uint8).reshape(n, 50)
    tri = raw[:, 12:48].copy().view("<f4").reshape(n, 3, 3).astype(np.float64)
    return tri


def write_stl(path, tri):
    tri32 = tri.astype("<f4")
    nrm = np.cross(tri32[:, 1] - tri32[:, 0], tri32[:, 2] - tri32[:, 0])
    nl = np.linalg.norm(nrm, axis=1, keepdims=True)
    nrm = np.divide(nrm, np.where(nl == 0, 1, nl))
    rec = np.zeros((len(tri32), 12), dtype="<f4")
    rec[:, 0:3] = nrm
    rec[:, 3:12] = tri32.reshape(-1, 9)
    data = np.zeros((len(tri32), 50), dtype=np.uint8)
    data[:, 0:48] = rec.view(np.uint8).reshape(len(tri32), 48)
    with open(path, "wb") as f:
        f.write(b"RMUL2027 3V3 field, Z-up, meters, top surface z=0".ljust(80, b" "))
        f.write(struct.pack("<I", len(tri32)))
        f.write(data.tobytes())


tri = read_stl(SRC)
src_bbox = [[float(tri[:, :, i].min()), float(tri[:, :, i].max())] for i in range(3)]

# 变换到 Z-up，并把台面移到 z=0
out = np.empty_like(tri)
out[:, :, 0] = tri[:, :, 0]
out[:, :, 1] = -tri[:, :, 2]
out[:, :, 2] = tri[:, :, 1] - 0.100
write_stl(DST, out)

bb = [[float(out[:, :, i].min()), float(out[:, :, i].max())] for i in range(3)]
size = [round(bb[i][1] - bb[i][0], 4) for i in range(3)]

# ---- 网格体检 ----
a, b, c = out[:, 0], out[:, 1], out[:, 2]
nrm = np.cross(b - a, c - a)
nl = np.linalg.norm(nrm, axis=1)
area = 0.5 * nl
degenerate = int((area < 1e-12).sum())

# 边统计：封闭性 / 非流形
vkey = [np.round(out[:, i], 6) for i in range(3)]
pts = np.stack(vkey, axis=1).reshape(-1, 3)
uniq, inv = np.unique(pts, axis=0, return_inverse=True)
f = inv.reshape(-1, 3)
e = np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]], axis=0)
e.sort(axis=1)
ue, cnt = np.unique(e, axis=0, return_counts=True)
open_edges = int((cnt == 1).sum())
nonmanifold = int((cnt > 2).sum())
# 有向边一致性（法线朝外/朝内）
de = np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]], axis=0)
dkey = {}
for p, q in de:
    dkey[(p, q)] = dkey.get((p, q), 0) + 1
inconsistent = int(sum(1 for (p, q), k in dkey.items() if (q, p) in dkey and k > 0 and dkey[(q, p)] > 0))
# 闭合网格的体积（有向四面体求和）
vol = float(np.sum(np.einsum("ij,ij->i", a, np.cross(b, c))) / 6.0)

# 水平面（Z-up 下法线 |nz|≈1）与竖直面
nz = np.abs(nrm[:, 2] / np.maximum(nl, 1e-12))
hor_area = float(area[nz > 0.99].sum())
ver_area = float(area[nz < 0.01].sum())
top_tris = out[nz > 0.99]
top_z = np.round(top_tris[:, :, 2].mean(axis=1), 4)

rep = {
    "source_stl": SRC,
    "source_bbox_m": [[round(v, 4) for v in r] for r in src_bbox],
    "output_stl": DST,
    "output_bytes": os.path.getsize(DST),
    "bbox_m": [[round(v, 4) for v in r] for r in bb],
    "size_m": size,
    "triangles": int(len(out)),
    "unique_vertices": int(len(uniq)),
    "surface_area_m2": round(float(area.sum()), 3),
    "horizontal_area_m2": round(hor_area, 3),
    "vertical_area_m2": round(ver_area, 3),
    "closed_volume_m3": round(vol, 4),
    "degenerate_triangles": degenerate,
    "open_edges": open_edges,
    "nonmanifold_edges": nonmanifold,
    "edge_direction_conflicts": inconsistent,
    "top_surface_levels_m": {f"{z:+.3f}": int((np.abs(top_z - z) < 1e-4).sum())
                             for z in np.unique(top_z)},
}
with open(os.path.join(OUT, "report_geometry.json"), "w") as fh:
    json.dump(rep, fh, indent=2, ensure_ascii=False)
print(json.dumps(rep, indent=2, ensure_ascii=False))