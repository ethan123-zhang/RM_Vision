#!/usr/bin/env python3
"""STEP(RMUL2027 3V3 场地) -> 网格 + 几何体检 + 三视图渲染。

用法:  python3 step2mesh.py <in.stp> <outdir>
产出:  field.stl(米制)、report.json、topview.png、frontview.png、sideview.png
"""
import collections
import json
import os
import struct
import sys

import gmsh
import numpy as np
from PIL import Image, ImageDraw

STEP = sys.argv[1]
OUT = sys.argv[2]
os.makedirs(OUT, exist_ok=True)
rep = {"step": STEP, "out": OUT}

gmsh.initialize()
gmsh.option.setNumber("General.Terminal", 1)
gmsh.option.setNumber("General.Verbosity", 2)
gmsh.model.add("field")

gmsh.model.occ.importShapes(STEP)
gmsh.model.occ.synchronize()

bb_mm = gmsh.model.getBoundingBox(-1, -1)
rep["bbox_mm"] = [round(v, 3) for v in bb_mm]
rep["size_mm"] = [round(bb_mm[3] - bb_mm[0], 3),
                  round(bb_mm[4] - bb_mm[1], 3),
                  round(bb_mm[5] - bb_mm[2], 3)]
rep["entities"] = {str(d): len(gmsh.model.getEntities(d)) for d in range(4)}
print("BBOX(mm):", rep["bbox_mm"])
print("SIZE(mm):", rep["size_mm"])
print("ENTITIES:", rep["entities"])

vols = gmsh.model.getEntities(3)
if vols:
    v = gmsh.model.occ.getMass(3, vols[0][1])
    c = gmsh.model.occ.getCenterOfMass(3, vols[0][1])
    rep["volume_mm3"] = round(v, 1)
    rep["center_mm"] = [round(x, 2) for x in c]
    print("VOLUME mm3:", rep["volume_mm3"], "center:", rep["center_mm"])
    gmsh.model.occ.dilate(vols, 0, 0, 0, 1e-3, 1e-3, 1e-3)
else:
    surfs = gmsh.model.getEntities(2)
    gmsh.model.occ.dilate(surfs, 0, 0, 0, 1e-3, 1e-3, 1e-3)
gmsh.model.occ.synchronize()

bb = gmsh.model.getBoundingBox(-1, -1)
rep["bbox_m"] = [round(v, 5) for v in bb]
rep["size_m"] = [round(bb[3] - bb[0], 5), round(bb[4] - bb[1], 5), round(bb[5] - bb[2], 5)]
print("BBOX(m):", rep["bbox_m"], "SIZE(m):", rep["size_m"])

# 网格：模型以平面多边形为主，粗尺寸避免面数爆炸
for k, val in (("Mesh.MeshSizeMin", 0.002), ("Mesh.MeshSizeMax", 0.08),
               ("Mesh.MeshSizeFromCurvature", 0), ("Mesh.Algorithm", 6),
               ("Mesh.SaveAll", 1)):
    gmsh.option.setNumber(k, val)

print("meshing...")
gmsh.model.mesh.generate(2)   # 只要表面网格：渲染/碰撞都用外壳，3D 体网格徒增面数
nodeTags, nodeCoords, _ = gmsh.model.mesh.getNodes()
rep["nodes"] = len(nodeTags)
elemTypes, elemTags, _ = gmsh.model.mesh.getElements(2)
rep["triangles"] = int(sum(len(t) // 3 for t in elemTags))
print("NODES:", rep["nodes"], "TRIS:", rep["triangles"])

stl = os.path.join(OUT, "field.stl")
gmsh.option.setNumber("Mesh.Binary", 1)
gmsh.write(stl)
rep["stl_bytes"] = os.path.getsize(stl)
gmsh.finalize()


# ---------- 读回 STL 做统计与三视图 ----------
def read_stl(path):
    with open(path, "rb") as f:
        f.seek(80)
        n = struct.unpack("<I", f.read(4))[0]
        data = np.frombuffer(f.read(n * 50), dtype=np.uint8).reshape(n, 50)
    return data[:, 12:48].copy().view("<f4").reshape(n, 3, 3).astype(np.float64)


tri = read_stl(stl)
rep["stl_triangles"] = int(len(tri))
rep["stl_bbox_m"] = [[round(float(tri[:, :, i].min()), 4), round(float(tri[:, :, i].max()), 4)]
                     for i in range(3)]

# 面积按高度分层（判断行走面高度）
v1 = tri[:, 1, :] - tri[:, 0, :]
v2 = tri[:, 2, :] - tri[:, 0, :]
area = 0.5 * np.linalg.norm(np.cross(v1, v2), axis=1)
ymid = tri[:, :, 1].mean(axis=1)
bins = collections.defaultdict(float)
for y, a in zip(np.round(ymid, 2), area):
    bins[float(y)] += float(a)
rep["area_by_y_m2"] = {f"{k:+.2f}": round(v, 3) for k, v in sorted(bins.items(), key=lambda kv: -kv[1])[:20]}
print("AREA by Y:", json.dumps(rep["area_by_y_m2"], ensure_ascii=False))

normals = np.cross(v1, v2)
nl = np.linalg.norm(normals, axis=1)
nz = np.abs(normals[:, 1] / np.maximum(nl, 1e-12))   # |ny|，Y 是模型里的"上"
rep["area_horizontal_ratio"] = round(float(area[nz > 0.99].sum() / area.sum()), 4)
rep["area_vertical_ratio"] = round(float(area[nz < 0.01].sum() / area.sum()), 4)
print("horizontal(面积占比):", rep["area_horizontal_ratio"], "vertical:", rep["area_vertical_ratio"])


def render(proj, size_px, up_idx, title):
    """proj: (i,j) 平面索引；用 up 轴高度着色。"""
    i, j = proj
    lo = [float(tri[:, :, i].min()), float(tri[:, :, j].min())]
    hi = [float(tri[:, :, i].max()), float(tri[:, :, j].max())]
    w = int(size_px)
    h = max(1, int(size_px * (hi[1] - lo[1]) / max(1e-9, (hi[0] - lo[0]))))
    img = Image.new("RGB", (w, h), (12, 12, 14))
    d = ImageDraw.Draw(img)
    ymin, ymax = float(tri[:, :, up_idx].min()), float(tri[:, :, up_idx].max())

    def hcol(y):
        t = (y - ymin) / max(1e-9, ymax - ymin)
        r = int(np.clip(1.5 - abs(4 * t - 3), 0, 1) * 255)
        g = int(np.clip(1.5 - abs(4 * t - 2), 0, 1) * 255)
        b = int(np.clip(1.5 - abs(4 * t - 1), 0, 1) * 255)
        return (r, g, b)

    order = np.argsort(tri[:, :, up_idx].mean(axis=1))
    sx = (w - 1) / max(1e-9, hi[0] - lo[0])
    sy = (h - 1) / max(1e-9, hi[1] - lo[1])
    for k in order:
        t = tri[k]
        pts = [((t[q, i] - lo[0]) * sx, h - 1 - (t[q, j] - lo[1]) * sy) for q in range(3)]
        d.polygon(pts, fill=hcol(float(t[:, up_idx].mean())))
    img.save(os.path.join(OUT, title))
    return title


render((0, 2), 1260, 1, "topview.png")    # X-Z 俯视（Y 为高度）
render((0, 1), 1260, 2, "frontview.png")  # X-Y 立面
render((2, 1), 860, 0, "sideview.png")    # Z-Y 侧视
print("views written")

with open(os.path.join(OUT, "report.json"), "w") as f:
    json.dump(rep, f, indent=2, ensure_ascii=False)
print("DONE", stl, rep["stl_bytes"], "bytes")