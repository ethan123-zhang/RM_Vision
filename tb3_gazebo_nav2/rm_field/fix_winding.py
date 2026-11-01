#!/usr/bin/env python3
"""修正 STL 三角面朝向（winding），让所有面法线一致朝外。

问题：Creo/OCCT 导出的 STEP 里各平面朝向不一致，gmsh 转 STL 后三角形绕序混乱，
Gazebo 按绕序算面法线 -> 场地表面出现暗色斜纹/接缝。

做法：顶点焊接 -> 按共享边 BFS 传播一致朝向 -> 用有符号体积判断整体朝内/朝外。
用法: python3 fix_winding.py <in.stl> [out.stl]
"""
import struct
import sys
from collections import defaultdict

import numpy as np

src = sys.argv[1]
dst = sys.argv[2] if len(sys.argv) > 2 else src

with open(src, "rb") as f:
    data = f.read()
n = struct.unpack("<I", data[80:84])[0]
print(f"{src}: {n} triangles, {len(data)} bytes")
rec = np.frombuffer(data[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
tris = rec[:, 12:48].copy().view("<f4").reshape(n, 3, 3).astype(np.float64)
attrs = rec[:, 48:50].copy()

# 顶点焊接（1e-6 m 容差）
key = np.round(tris.reshape(-1, 3), 6)
uniq, inv = np.unique(key, axis=0, return_inverse=True)
vid = inv.reshape(n, 3)
print(f"welded vertices: {len(uniq)}")

# 边 -> [(tri, dir)]  dir=+1 表示该三角形按 (lo,hi) 方向走过这条边
edges = defaultdict(list)
for t in range(n):
    a, b, c = vid[t]
    for u, v in ((a, b), (b, c), (c, a)):
        if u == v:
            continue
        lo, hi = (u, v) if u < v else (v, u)
        edges[(lo, hi)].append((t, 1 if u == lo else -1))

flip = np.zeros(n, dtype=bool)
seen = np.zeros(n, dtype=bool)
stack = [0]
seen[0] = True
conflicts = 0
while stack:
    t = stack.pop()
    a, b, c = vid[t]
    for u, v in ((a, b), (b, c), (c, a)):
        if u == v:
            continue
        lo, hi = (u, v) if u < v else (v, u)
        d_self = 1 if u == lo else -1
        for (t2, d2) in edges[(lo, hi)]:
            if t2 == t:
                continue
            # 一致朝向要求两个三角形以相反方向走过共享边
            eff2 = d2 * (-1 if flip[t2] else 1)
            need = -d_self * (-1 if flip[t] else 1)
            if not seen[t2]:
                if eff2 != need:
                    flip[t2] = True
                seen[t2] = True
                stack.append(t2)
            elif eff2 != need:
                conflicts += 1

print(f"visited {seen.sum()}/{n} triangles, unresolved {n - seen.sum()}, conflicts {conflicts}")

out = tris.copy()
out[flip] = out[flip][:, ::-1]   # 翻转绕序


def signed_volume(t):
    v0, v1, v2 = t[:, 0], t[:, 1], t[:, 2]
    return float(np.sum(np.einsum("ij,ij->i", v0, np.cross(v1, v2))) / 6.0)


vol_before, vol_after = signed_volume(tris), signed_volume(out)
print(f"signed volume: before={vol_before:.4f} after={vol_after:.4f} m^3")
if vol_after < 0:
    out = out[:, ::-1]
    vol_after = signed_volume(out)
    print(f"flipped whole mesh -> volume {vol_after:.4f}")

# 法线一致性统计：相邻面法线夹角
normals = np.cross(out[:, 1] - out[:, 0], out[:, 2] - out[:, 0])
ln = np.linalg.norm(normals, axis=1, keepdims=True)
ln[ln == 0] = 1
normals /= ln
dots = []
for (lo, hi), lst in list(edges.items())[:200000]:
    if len(lst) == 2:
        t1, t2 = lst[0][0], lst[1][0]
        dots.append(float(np.dot(normals[t1], normals[t2])))
dots = np.array(dots)
if dots.size:
    print(f"adjacent normal dots: min={dots.min():.3f} mean={dots.mean():.3f} "
          f"flipped(<0)={(dots < 0).sum()}")

# 写回二进制 STL
head = data[:80]
with open(dst, "wb") as f:
    f.write(head)
    f.write(struct.pack("<I", n))
    buf = bytearray()
    for i in range(n):
        buf += b"\x00\x00\x00\x00"
        buf += out[i].astype("<f4").tobytes()
        buf += attrs[i].tobytes()
    f.write(bytes(buf))
print(f"wrote {dst}")