#!/usr/bin/env python3
"""统计渲染图里"亮面内部出现的暗色条纹"像素数，用于对比法线修正前后的视觉质量。

判定: 像素亮度 < 165，且上下左右各 12px 处都是亮面(>=170)。
用法: python3 seam_check.py <png> [bright] [dark]
"""
import sys

import numpy as np
from PIL import Image

path = sys.argv[1]
BRIGHT = float(sys.argv[2]) if len(sys.argv) > 2 else 170.0
DARK = float(sys.argv[3]) if len(sys.argv) > 3 else 165.0
img = np.asarray(Image.open(path).convert("RGB")).astype(float)
lum = 0.299 * img[:, :, 0] + 0.587 * img[:, :, 1] + 0.114 * img[:, :, 2]
b = lum >= BRIGHT
k = 12
up = np.zeros_like(b)
dn = np.zeros_like(b)
lf = np.zeros_like(b)
rt = np.zeros_like(b)
up[k:, :] = b[:-k, :]
dn[:-k, :] = b[k:, :]
lf[:, k:] = b[:, :-k]
rt[:, :-k] = b[:, k:]
interior = up & dn & lf & rt
seam = (lum < DARK) & interior
print(f"{path}: 亮面像素 {b.sum()}  内部嵌入暗像素 (=条纹/接缝) {seam.sum()}  "
      f"占亮面 {100.0*seam.sum()/max(1,b.sum()):.3f}%")
ys, xs = np.nonzero(seam)
if len(xs):
    print(f"  条纹 bbox x[{xs.min()},{xs.max()}] y[{ys.min()},{ys.max()}]")
    print(f"  条纹像素亮度: min={lum[seam].min():.0f} mean={lum[seam].mean():.0f}")