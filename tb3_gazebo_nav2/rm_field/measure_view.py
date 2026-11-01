#!/usr/bin/env python3
"""分析 Gazebo GUI 窗口截图：自动排除右侧面板/白底，只统计 3D 视口内容。

用法: python3 measure_view.py <png> [x y w h]
分类: 场地亮面 160..235  地面 95..135  背景 <60  面板/其它
"""
import sys

import numpy as np
from PIL import Image

path = sys.argv[1]
img = Image.open(path).convert("RGB")
W, H = img.size
if len(sys.argv) > 5:
    x, y, w, h = (int(v) for v in sys.argv[2:6])
else:
    x, y, w, h = 0, 0, W, H
img = img.crop((x, y, x + w, y + h))
a = np.asarray(img).astype(float)
lum = 0.299 * a[:, :, 0] + 0.587 * a[:, :, 1] + 0.114 * a[:, :, 2]
h, w = lum.shape

panel = lum > 236                      # GUI 面板 / 白底
field = (lum >= 160) & (lum <= 236)
ground = (lum >= 95) & (lum < 135)
bg = lum < 60
scene = ~panel

print(f"{path} {w}x{h}")
print(f"  面板/白底 {100*panel.mean():5.1f}%  视口 {100*scene.mean():5.1f}%")
den = max(1, scene.sum())
print(f"  视口内: 场地亮面 {100*field.sum()/den:5.1f}%  地面 {100*ground.sum()/den:5.1f}%"
      f"  背景(暗) {100*bg.sum()/den:5.1f}%  其它 {100*(scene & ~field & ~ground & ~bg).sum()/den:5.1f}%")
ys, xs = np.nonzero(field)
if len(xs):
    print(f"  亮面 bbox x[{xs.min()},{xs.max()}] y[{ys.min()},{ys.max()}]  "
          f"{xs.max()-xs.min()+1}x{ys.max()-ys.min()+1} px")
    print(f"  亮面中心 ({(xs.min()+xs.max())//2},{(ys.min()+ys.max())//2})  图像中心 ({w//2},{h//2})")

cols = 76
sx = max(1, w // cols)
sy = max(1, int(sx * 2.1))
print("\n  '#'=场地亮面  '='=地面  '.'=背景暗  'P'=面板   ' '=其它")
for r in range(0, h, sy):
    line = []
    for c in range(0, w, sx):
        blk = lum[r:r + sy, c:c + sx]
        m = blk.mean()
        if m > 236:
            line.append("P")
        elif m >= 160:
            line.append("#")
        elif 95 <= m < 135:
            line.append("=")
        elif m < 60:
            line.append(".")
        else:
            line.append(" ")
    print("  " + "".join(line))