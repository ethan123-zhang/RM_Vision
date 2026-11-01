#!/usr/bin/env python3
"""对 field.stl 做结构体检：水平面分层、竖直面分布、ASCII 高度图。

用法: python3 analyze_mesh.py <field.stl>
坐标约定：模型 Y 轴向上，X 为场地长边(12.6)，Z 为短边(8.6)。
"""
import sys
import numpy as np

stl = sys.argv[1]
with open(stl, "rb") as f:
    f.seek(80)
    n = int(np.frombuffer(f.read(4), "<I")[0])
    raw = np.frombuffer(f.read(n * 50), dtype=np.uint8).reshape(n, 50)
tri = raw[:, 12:48].copy().view("<f4").reshape(n, 3, 3).astype(np.float64)
print(f"triangles: {len(tri)}")

a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
nrm = np.cross(b - a, c - a)
nl = np.linalg.norm(nrm, axis=1)
area = 0.5 * nl
unit = nrm / np.maximum(nl, 1e-12)[:, None]

# 1) 水平面（模型里 Y 是上）：法线 |ny| ~ 1
hor = np.abs(unit[:, 1]) > 0.99
ys = tri[hor][:, :, 1].mean(axis=1)
yh = np.round(ys, 3)
print("\n== 水平面按高度分层 (mm, 面积 m^2, 面片数, X范围, Z范围) ==")
for y in np.unique(yh):
    m = yh == y
    sel = tri[hor][m]
    a_m = area[hor][m].sum()
    print(f"  y={y*1000:8.1f}  area={a_m:8.3f}  tris={m.sum():6d}  "
          f"X[{sel[:,:,0].min():7.3f},{sel[:,:,0].max():7.3f}] "
          f"Z[{sel[:,:,2].min():7.3f},{sel[:,:,2].max():7.3f}]  "
          f"up={(unit[hor][m][:,1]>0).sum()} down={(unit[hor][m][:,1]<0).sum()}")

# 2) 竖直面（法线在水平面内）
ver = np.abs(unit[:, 1]) < 0.01
print("\n== 竖直面 ==")
print(f"  area={area[ver].sum():.3f} m^2, tris={ver.sum()}, "
      f"y range [{tri[ver][:,:,1].min():.3f},{tri[ver][:,:,1].max():.3f}]")

# 3) ASCII 高度图：每 0.2m 单元内最高水平面
cell = 0.2
x0, z0 = -6.3, -4.3
nx, nz = int(round(12.6 / cell)), int(round(8.6 / cell))
top = np.full((nz, nx), -9.9)
for t in tri[hor]:
    xm, zm, ym = t[:, 0].mean(), t[:, 2].mean(), t[:, 1].mean()
    ix = int((xm - x0) / cell)
    iz = int((zm - z0) / cell)
    if 0 <= ix < nx and 0 <= iz < nz:
        top[iz, ix] = max(top[iz, ix], ym)
ys_u = np.unique(np.round(top, 3))
print("\n== ASCII 俯视高度图（每格 0.2m，行=Z 从 +4.3 往下，列=X 从 -6.3 往右） ==")
print("  高度字符映射:", ", ".join(f"{y*1000:+.0f}mm->{chr(48+i)}" for i, y in enumerate(ys_u)))
chars = " .:-=+*#%@"
for iz in range(nz - 1, -1, -1):
    line = "".join(chars[min(len(chars)-1, int((top[iz, ix] - ys_u[0]) / max(1e-9, (ys_u[-1]-ys_u[0])) * (len(chars)-1)))]
                   if top[iz, ix] > -9 else "?" for ix in range(nx))
    print(f"{z0 + (iz+0.5)*cell:+5.2f} |{line}|")
print("        " + "".join(str(int(abs(x0 + (i+0.5)*cell))) % 10 for i in range(nx)))