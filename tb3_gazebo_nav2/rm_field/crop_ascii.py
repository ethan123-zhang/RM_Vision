#!/usr/bin/env python3
"""裁剪窗口区域并输出 ASCII 亮度图：无视觉引擎时判断 Gazebo 里画了什么。

用法: python3 crop_ascii.py <png> [x y w h] [cols]
"""
import sys

import numpy as np
from PIL import Image

path = sys.argv[1]
img = Image.open(path).convert("RGB")
W, H = img.size
if len(sys.argv) > 5:
    x, y, w, h = (int(v) for v in sys.argv[2:6])
    img = img.crop((x, y, x + w, y + h))
else:
    x, y, w, h = 0, 0, W, H
cols = int(sys.argv[6]) if len(sys.argv) > 6 else 90

a = np.asarray(img).astype(np.float64)
lum = 0.299 * a[:, :, 0] + 0.587 * a[:, :, 1] + 0.114 * a[:, :, 2]
h, w = lum.shape
sx = max(1, w // cols)
sy = max(1, int(sx * 2.1))   # 字符高宽比补偿
small = lum[::sy, ::sx]
print(f"{path} crop=({x},{y},{w},{h}) -> {small.shape[1]}x{small.shape[0]} chars "
      f"mean_lum={lum.mean():.1f} max={lum.max():.0f}")
chars = " .:-=+*#%@"
for row in small:
    print("".join(chars[min(9, int(v / 25.6))] for v in row))
# 粗略统计"灰色场地"占比
print("亮度直方图:", {f"{i*32}-{i*32+31}": int(((lum >= i * 32) & (lum < i * 32 + 32)).sum())
                     for i in range(8)})