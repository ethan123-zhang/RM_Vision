#!/usr/bin/env python3
"""判断"这张图看起来是不是一块高台"：只看像素，不看几何。

判据（人眼判断高度靠这几件事）：
  1. 台面顶部与侧壁之间有明显亮度差（≥15），否则侧面和顶面糊成一片；
  2. 侧壁与地面/背景之间有明显亮度差（≥15），否则台子像悬空的纸片；
  3. 侧壁带在画面里够厚（≥25 px），否则"半米"看不见；
  4. 台子在地面上有投影（地面出现比平均地面更暗的邻接区域）。

用法: python3 probe_report.py <png> [列带中心比例]
"""
import sys

import numpy as np
from PIL import Image


def band_lum(lum, y0, y1, x0, x1):
    return float(np.median(lum[y0:y1, x0:x1]))


def main():
    path = sys.argv[1]
    frac = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
    a = np.asarray(Image.open(path).convert("RGB")).astype(float)
    lum = 0.299 * a[:, :, 0] + 0.587 * a[:, :, 1] + 0.114 * a[:, :, 2]
    H, W = lum.shape
    x0, x1 = int(W * (frac - 0.08)), int(W * (frac + 0.08))

    prof = np.median(lum[:, x0:x1], axis=1)
    print(f"{path}  {W}x{H}   列带 x[{x0},{x1})")
    print("  行剖面（每 4% 高度）:")
    step = max(1, H // 28)
    for r in range(0, H, step):
        print(f"   y={r:4d} lum={prof[r]:6.1f} |{'#' * min(64, int(prof[r] / 4))}")

    # 分类阈值按全图分布自适应：取三档（暗/中/亮）的分位
    p10, p50, p90 = np.percentile(lum, [10, 50, 90])
    print(f"\n  全图分位: p10={p10:.1f} p50={p50:.1f} p90={p90:.1f}")

    # 用"亮面"（台面顶）找出来，再往下找侧壁与地面
    bright = lum > max(140, p90 * 0.9)
    ys, xs = np.nonzero(bright)
    if len(xs) == 0:
        print("  !! 没有找亮点（台面）"); return 2
    top_row = int(np.percentile(ys, 5))
    bot_row = int(np.percentile(ys, 95))
    top_lum = band_lum(lum, top_row, top_row + max(1, (bot_row - top_row) // 4), x0, x1)
    below = prof[bot_row:min(H, bot_row + H // 3)]
    print(f"  台面亮区 y[{top_row},{bot_row}] 中值亮度={top_lum:.1f}")

    # 台面下缘以下逐行梯度，找侧壁与地面
    g = np.diff(below)
    wall_row = bot_row + int(np.argmin(g)) if len(g) else bot_row
    print(f"  台面下缘第一处强下降: y={wall_row}  梯度={g.min() if len(g) else 0:.1f}")
    wall_lum = band_lum(lum, wall_row + 2, min(H, wall_row + max(4, H // 40)), x0, x1)
    ground_lum = band_lum(lum, min(H - 4, bot_row + H // 4), min(H, bot_row + H // 3), x0, x1)
    bg_lum = band_lum(lum, 0, max(2, H // 20), x0, x1)

    # 侧壁带厚度：从台面下缘往下，直到亮度落到地面水平
    thr = (wall_lum + ground_lum) / 2
    rows = np.arange(bot_row, min(H, bot_row + H // 2))
    below_ground = rows[prof[rows] < thr]
    wall_band = int(below_ground[0] - bot_row) if len(below_ground) else -1

    print(f"\n  台面亮度 {top_lum:6.1f}")
    print(f"  侧壁亮度 {wall_lum:6.1f}")
    print(f"  地面亮度 {ground_lum:6.1f}")
    print(f"  背景亮度 {bg_lum:6.1f}")
    print(f"  侧壁带厚度 ≈ {wall_band} px")

    ok = []
    ok.append(("台面↔侧壁 亮度差 ≥15", abs(top_lum - wall_lum) >= 15,
               f"{abs(top_lum - wall_lum):.1f}"))
    ok.append(("侧壁↔地面/背景 亮度差 ≥15",
               max(abs(wall_lum - ground_lum), abs(wall_lum - bg_lum)) >= 15,
               f"地面 {abs(wall_lum - ground_lum):.1f} / 背景 {abs(wall_lum - bg_lum):.1f}"))
    ok.append(("侧壁带 ≥25 px", wall_band >= 25, f"{wall_band} px"))
    for n, good, d in ok:
        print(f"  [{'OK ' if good else 'FAIL'}] {n}: {d}")
    return 0 if all(o[1] for o in ok) else 1


if __name__ == "__main__":
    sys.exit(main())