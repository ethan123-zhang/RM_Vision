#!/usr/bin/env python3
"""量化 Gazebo GUI 窗口截图里"3D 视口到底画了什么"。

用途：回答"为什么看着像一块平板"以及"现在能不能看见 0.503 m 侧壁"。
只做像素分类，不依赖视觉模型：

  panel  lum >= 236        右侧 GUI 面板 / 顶部工具条（白底）
  top   150 <= lum < 236   台面顶面（材质 0.55 灰，受光最亮）
  wall   70 <= lum < 150   侧壁（前立面受光中等）/ 机器人
  dark   lum < 70          背景天空与 -0.503 m 地面

用法: python3 gui_view_report.py <png> [--viewport x y w h]
"""
import sys

import numpy as np
from PIL import Image


def main():
    path = sys.argv[1]
    img = Image.open(path).convert("RGB")
    a = np.asarray(img).astype(float)
    lum = 0.299 * a[:, :, 0] + 0.587 * a[:, :, 1] + 0.114 * a[:, :, 2]
    H, W = lum.shape

    if "--viewport" in sys.argv:
        i = sys.argv.index("--viewport")
        x, y, w, h = (int(v) for v in sys.argv[i + 1:i + 5])
    else:
        # 右侧面板左边界 = 从左侧数第一列"白像素 > 60%"
        white = lum > 236
        colwhite = white.mean(axis=0)
        panel_start = W
        for c in range(W):
            if colwhite[c] > 0.60:
                panel_start = c
                break
        # 顶部那条全白的是 GUI 工具条，切掉
        rowwhite = (lum[:, :max(1, panel_start)] > 236).mean(axis=1)
        top_cut = 0
        for r in range(H):
            if rowwhite[r] < 0.30:
                top_cut = r
                break
        x, y, w, h = 0, top_cut, max(1, panel_start), H - top_cut

    vp = lum[y:y + h, x:x + w]
    panel = vp > 236
    bright = (vp >= 150) & (vp < 236)
    wall = (vp >= 70) & (vp < 150)
    dark = vp < 70
    scene = ~panel
    den = max(1, int(scene.sum()))

    print(f"{path}  {W}x{H}")
    print(f"3D 视口 = x[{x},{x+w}) y[{y},{y+h})  {w}x{h} px  "
          f"(占窗口 {100*w*h/(W*H):.0f}%)")
    print(f"  视口内:  台面亮面 {100*bright.sum()/den:5.1f}%   侧壁/机器人 {100*wall.sum()/den:5.1f}%"
          f"   背景+地面 {100*dark.sum()/den:5.1f}%   面板残留 {100*panel.sum()/den:5.1f}%")

    # 逐行分类比例，找"台面亮带"和它下面的"侧壁带"
    fb = bright.mean(axis=1)
    fw = wall.mean(axis=1)
    fd = dark.mean(axis=1)
    rows_b = np.nonzero(fb > 0.05)[0]
    print(f"\n  行剖面（视口内每 3% 高度：亮面/侧壁/暗 占比）:")
    step = max(1, h // 33)
    for r in range(0, h, step):
        print(f"  y={r:4d}  bright {fb[r]:5.1%}  wall {fw[r]:5.1%}  dark {fd[r]:5.1%}"
              f" |{'B' * int(fb[r] * 40)}")

    ok_platform = len(rows_b) > 0 and fb.max() > 0.15
    # 亮带下缘之后的 8% 视口高度内应出现侧壁（mid 亮度）行
    wall_ok = False
    wall_px = 0.0
    if len(rows_b):
        r0 = rows_b.max()
        seg = fw[r0 + 1:min(h, r0 + 1 + int(h * 0.08))]
        if seg.size:
            wall_ok = seg.max() > 0.10
            wall_px = (seg > 0.10).sum() * (h / max(1, h))
    print(f"\n  台面亮带行范围: y[{rows_b.min()},{rows_b.max()}] = "
          f"{len(rows_b)} px" if len(rows_b) else "\n  台面亮带: 无")
    print(f"  [{'OK ' if ok_platform else 'FAIL'}] 视口里存在台面亮带（峰值行占比 "
          f"{fb.max():.1%} > 15%）")
    print(f"  [{'OK ' if wall_ok else 'FAIL'}] 台面下缘紧邻存在侧壁带（下 8% 视口高度内 "
          f"wall 行占比 > 10%，实测最大 {fw[rows_b.max()+1:min(h, rows_b.max()+1+int(h*0.08))].max() if len(rows_b) and rows_b.max()+1 < h else 0:.1%}）")
    return 0 if (ok_platform and wall_ok) else 1


if __name__ == "__main__":
    sys.exit(main())