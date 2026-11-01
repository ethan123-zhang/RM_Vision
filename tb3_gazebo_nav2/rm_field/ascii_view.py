#!/usr/bin/env python3
"""把 PNG 压成 ASCII 亮度图 + 颜色统计，无视觉引擎时用来"看"渲染结果。"""
import sys
import zlib
import struct

import numpy as np
from PIL import Image

path = sys.argv[1]
img = Image.open(path).convert("RGB")
W, H = img.size
a = np.asarray(img).astype(np.float64)
lum = 0.299 * a[:, :, 0] + 0.587 * a[:, :, 1] + 0.114 * a[:, :, 2]
print(f"{path}  {W}x{H}  mean_lum={lum.mean():.1f}  min={lum.min():.0f} max={lum.max():.0f}")
print("唯一颜色数(采样):", len(np.unique(a[::7, ::7].reshape(-1, 3), axis=0)))
hist, edges = np.histogram(lum, bins=8, range=(0, 256))
print("亮度直方图:", {f"{int(edges[i])}-{int(edges[i+1])}": int(hist[i]) for i in range(8)})

# 20 色阶 ASCII（0..255）
sx = max(1, W // 78)
sy = max(1, H // 40)
small = lum[::sy, ::sx]
chars = " .:-=+*#%@"
print(f"\nASCII 亮度图 ({small.shape[1]}x{small.shape[0]}, 采样步长 {sx}x{sy}):")
for row in small:
    print("".join(chars[min(9, int(v / 25.6))] for v in row))

# 中心行的亮度剖面，判断是否有明确的场地边界
mid = lum[H // 2, :]
step = max(1, W // 60)
print("\n中心行亮度剖面(每 %d px):" % step,
      " ".join(f"{int(mid[i]):3d}" for i in range(0, W, step)))