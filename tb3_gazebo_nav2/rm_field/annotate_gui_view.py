#!/usr/bin/env python3
"""给 GUI 视角渲染图做标注：把"围栏顶 / 台面边 / 地面对应的高度线"按几何预测画上去，
并把预测行与实测剖面比对 —— 证明 GUI 里看到的确实是 0.503 m 台阶 + 0.30 m 围栏。

用法: python3 annotate_gui_view.py
输入: shots/gui_fenced_view.png（worlds/gui_capture_fenced.sdf 的 /gui_view/image）
输出: shots/gui_fenced_view_annotated.png
"""
import json
import os
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import verify_height as VH  # noqa: E402

ROOT = os.path.dirname(os.path.abspath(__file__))
SHOTS = os.path.join(ROOT, "shots")
WORLD = os.path.join(ROOT, "worlds", "gui_capture_fenced.sdf")

FIELD_X, FIELD_Y, FIELD_H = 12.6, 8.6, 0.503
FENCE_H = 0.30


def font(sz):
    try:
        return ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", sz)
    except OSError:
        return ImageFont.load_default()


def label(d, xy, txt, f, color=(255, 235, 60)):
    x, y = xy
    for dx, dy in ((-2, 0), (2, 0), (0, -2), (0, 2)):
        d.text((x + dx, y + dy), txt, font=f, fill=(0, 0, 0))
    d.text((x, y), txt, font=f, fill=color)


def main():
    cams = VH.load_cameras(WORLD)
    tag = "gui_view"
    if tag not in cams:
        print("找不到 gui_view 相机"); return 2
    cam = VH.Cam(cams[tag])
    p = os.path.join(SHOTS, "gui_fenced_view.png")
    img = Image.open(p).convert("RGB")
    d = ImageDraw.Draw(img)
    lum = VH.load_lum(p)

    # 相机在 +y 侧，看到的是 y = +4.30 这面（平台外沿）。
    # 注意：这条外沿在画面里是斜的（从右上到左下），所以线必须沿边逐点投影，不能画水平线。
    y_edge = FIELD_Y / 2
    z_fence_top, z_top, z_bot = FENCE_H, 0.0, -FIELD_H
    xs = np.linspace(-FIELD_X / 2, FIELD_X / 2, 61)
    for name, z in (("fence_top", z_fence_top), ("platform_top", z_top), ("ground", z_bot)):
        pts = [cam.project((x, y_edge, z)) for x in xs]
        d.line([(u, v) for u, v, _ in pts], fill=(255, 60, 60), width=3)

    # 在同一列（x = x_ann 处的外沿）上量实测分界，和预测比
    x_ann = 0.6
    ann = {k: cam.project((x_ann, y_edge, z)) for k, z in
           (("fence_top", z_fence_top), ("platform_top", z_top), ("ground", z_bot))}
    u_ann = ann["platform_top"][0]
    rows = {k: v[1] for k, v in ann.items()}
    prof = VH.column_profile(lum, u_ann - 10, u_ann + 10)
    meas = {}
    # 搜索窗收窄：围栏顶 / 平台边 / 地面三者在 25~40 px 内相邻，窗太大就会互相抢
    for name, win in (("fence_top", 18), ("platform_top", 12), ("ground", 22)):
        r, _, _ = VH.find_edge(prof, rows[name], win=win)
        meas[name] = r

    # 竖向箭头：围栏 0.30 m，平台 0.503 m
    arrow = (255, 235, 60)
    for a, b in (("fence_top", "platform_top"), ("platform_top", "ground")):
        d.line([(u_ann, rows[a]), (u_ann, rows[b])], fill=arrow, width=3)
        for k in (a, b):
            d.line([(u_ann - 9, rows[k]), (u_ann + 9, rows[k])], fill=arrow, width=3)

    f = font(30)
    label(d, (u_ann + 16, (rows["fence_top"] + rows["platform_top"]) / 2 - 34),
          f"围栏 {FENCE_H:.2f} m", f)
    label(d, (u_ann + 16, (rows["platform_top"] + rows["ground"]) / 2 - 6),
          f"平台高 {FIELD_H:.3f} m", f, color=(120, 255, 140))
    label(d, (22, 18), "GUI 相机 (-8.5, 7.5, 2.6) / hfov 1.15 / 1000x845  ——  离线等价渲染", font(26),
          color=(180, 230, 255))

    out = os.path.join(SHOTS, "gui_fenced_view_annotated.png")
    img.save(out)

    print(f"预测行: " + "  ".join(f"{k}={v:.1f}" for k, v in rows.items()))
    print(f"实测行: " + "  ".join(f"{k}={meas[k]:.1f}" for k in meas))
    for k in ("fence_top", "platform_top", "ground"):
        print(f"  [{ 'OK ' if abs(meas[k]-rows[k])<=6 else 'FAIL'}] {k}: 偏差 {meas[k]-rows[k]:+.1f} px")
    print(f"  围栏带 {rows['platform_top']-rows['fence_top']:.1f} px, "
          f"侧壁带 {rows['ground']-rows['platform_top']:.1f} px, 视口高 {cam.h}")
    print(f"图: {out}")
    json.dump({"predicted_rows": rows, "measured_rows": meas},
              open(os.path.join(ROOT, "results", "report_gui_view.json"), "w"), indent=2)


if __name__ == "__main__":
    sys.exit(main())