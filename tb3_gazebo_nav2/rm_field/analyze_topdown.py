#!/usr/bin/env python3
"""正俯视出图的比例校验：用像素尺寸反推场地米数，和图纸 12.6x8.6 对比。

相机: (0,0,20) 正俯视, hfov=1.2 rad, 1280x720 -> 20/(640/tan(0.6)) m/px
"""
import os
import sys

import numpy as np
from PIL import Image

CAM_H = 20.0
W, H = 1280, 720
HFOV = 1.2
F = (W / 2.0) / np.tan(HFOV / 2.0)
M_PER_PX = CAM_H / F
print(f"相机模型: f={F:.2f}px  距离={CAM_H}m  -> {M_PER_PX*1000:.3f} mm/px")

path = sys.argv[1]
img = np.asarray(Image.open(path).convert("RGB")).astype(np.float64)
lum = 0.299 * img[:, :, 0] + 0.587 * img[:, :, 1] + 0.114 * img[:, :, 2]

# 场地台面是亮面（≈185），地面是暗面（≈108）；阈值取 145
field = lum > 145
ys, xs = np.nonzero(field)
if len(xs) == 0:
    print("!! 没找到亮面（场地）")
    sys.exit(2)
x0, x1, y0, y1 = xs.min(), xs.max(), ys.min(), ys.max()
pw, ph = x1 - x0 + 1, y1 - y0 + 1
print(f"\n场地亮面 bbox: x[{x0},{x1}] y[{y0},{y1}]  = {pw} x {ph} px")
print(f"反推尺寸: {pw*M_PER_PX:.3f} m x {ph*M_PER_PX:.3f} m  "
      f"(图纸 12.600 x 8.600)")
print(f"相对误差: X {abs(pw*M_PER_PX-12.6)/12.6:.3%}  Y {abs(ph*M_PER_PX-8.6)/8.6:.3%}")
print(f"长老比: 实测 {pw/ph:.4f}  图纸 {12.6/8.6:.4f}")
print(f"覆盖率: {field.mean():.1%}  图像中心: ({W/2:.0f},{H/2:.0f})  "
      f"亮面中心: ({(x0+x1)/2:.0f},{(y0+y1)/2:.0f})")

# 场内的暗色物体（机器人）像素尺寸
inner = np.zeros_like(field)
inner[y0:y1 + 1, x0:x1 + 1] = True
dark = (lum < 145) & inner
dys, dxs = np.nonzero(dark)
if len(dxs) > 20:
    dw = dxs.max() - dxs.min() + 1
    dh = dys.max() - dys.min() + 1
    print(f"\n场内暗色物体: bbox {dw} x {dh} px -> {dw*M_PER_PX:.3f} m x "
          f"{dh*M_PER_PX:.3f} m (TB3 waffle 底盘 0.281 x 0.306 m)")
else:
    print("\n场内没有暗色物体（空场地）")