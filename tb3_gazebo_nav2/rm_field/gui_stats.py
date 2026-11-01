#!/usr/bin/env python3
"""统计 GUI 视口截图里的内容构成：判断 3D 视图是否框住了场地。

用法: python3 gui_stats.py <png> [x y w h]
"""
import sys

import numpy as np
from PIL import Image

path = sys.argv[1]
img = Image.open(path).convert("RGB")
if len(sys.argv) > 5:
    x, y, w, h = (int(v) for v in sys.argv[2:6])
    img = img.crop((x, y, x + w, y + h))
else:
    # 默认只取左 58%（3D 视口区，右侧是面板）
    w, h = img.size
    img = img.crop((0, int(h * 0.10), int(w * 0.58), int(h * 0.92)))
a = np.asarray(img).astype(float)
lum = 0.299 * a[:, :, 0] + 0.587 * a[:, :, 1] + 0.114 * a[:, :, 2]
n = lum.size
print(f"{path}  视口区 {img.size[0]}x{img.size[1]}  平均亮度={lum.mean():.1f}")
hist = {f"{i*32}-{i*32+31}": int(((lum >= i * 32) & (lum < i * 32 + 32)).sum())
        for i in range(8)}
print("亮度直方图:", hist)
for th in (60, 90, 120, 140):
    print(f"  >{th}: {100.0*(lum > th).mean():.1f}%")
# 场地亮面在 GUI 里通常明显亮于地面/背景，取高分位做参考
print(f"分位: p10={np.percentile(lum,10):.0f} p50={np.percentile(lum,50):.0f} "
      f"p90={np.percentile(lum,90):.0f} p99={np.percentile(lum,99):.0f}")