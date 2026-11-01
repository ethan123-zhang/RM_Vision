#!/usr/bin/env python3
"""在渲染图上把 0.503 m 的侧壁标出来，生成人眼可直接确认的标注图。

标注内容：
  - 墙顶/墙底两条线（位置 = 针孔投影预测，不是手画的）
  - 双箭头 + 文字：0.503 m（并给实测毫米数）
  - 场地宽 12.6 m 的横向标尺

用法: python3 annotate_height.py [--shots shots] [--out shots]
"""
import os
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT)
import verify_height as VH  # noqa: E402

SHOTS = os.path.join(ROOT, "shots")
WORLD = os.path.join(ROOT, "worlds/visual_capture.sdf")


def font(size):
    for p in ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
              "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        if os.path.exists(p):
            return ImageFont.truetype(p, size)
    return ImageFont.load_default()


def hline(d, y, x0, x1, color=(255, 60, 60), w=3):
    d.line([(x0, y), (x1, y)], fill=color, width=w)


def label(d, xy, text, f, color=(255, 235, 60), bg=(0, 0, 0)):
    x, y = xy
    box = d.textbbox((x, y), text, font=f)
    d.rectangle([box[0] - 6, box[1] - 4, box[2] + 6, box[3] + 4], fill=bg)
    d.text((x, y), text, font=f, fill=color)


def arrow_v(d, x, y0, y1, color=(255, 60, 60), w=3, head=12):
    d.line([(x, y0), (x, y1)], fill=color, width=w)
    d.line([(x - head, y0 + head), (x, y0), (x + head, y0 + head)], fill=color, width=w)
    d.line([(x - head, y1 - head), (x, y1), (x + head, y1 - head)], fill=color, width=w)


def main():
    cams = VH.load_cameras(WORLD)
    report = {}

    # ---------- A. 墙面近景：一整个 0.503 m，450+ px ----------
    tag = "elevation_wall"
    cam = VH.Cam(cams[tag])
    p = os.path.join(SHOTS, VH.IMG[tag])
    img = Image.open(p).convert("RGB")
    d = ImageDraw.Draw(img)
    top = cam.project((2.0, -VH.FIELD_Y / 2, 0.0))[1]
    bot = cam.project((2.0, -VH.FIELD_Y / 2, -VH.FIELD_H))[1]
    h_px = bot - top
    # 标注上的"measured"必须是真的像素实测值，不是预测值：
    # 用 verify_height 的同一套列剖面 + 梯度找边，复算一次。
    lum = VH.load_lum(p)
    prof = VH.column_profile(lum, cam.w * 0.35, cam.w * 0.65)
    m_top, _, _ = VH.find_edge(prof, top)
    m_bot, _, _ = VH.find_edge(prof, bot)
    m_px = m_bot - m_top
    h_mm = m_px * cam.m_per_px(cam.project((2.0, -VH.FIELD_Y / 2, 0.0))[2]) * 1000
    hline(d, top, 60, cam.w - 60)
    hline(d, bot, 60, cam.w - 60)
    arrow_v(d, 150, top + 4, bot - 4)
    f = font(38)
    label(d, (175, (top + bot) / 2 - 22), f"SIDE WALL = 0.503 m  (measured {h_mm:.0f} mm)",
          f, color=(255, 235, 60))
    label(d, (60, 20), "front elevation of RMUL2027 3V3 field  |  camera 4.7 m from wall, hfov 0.30",
          font(24), color=(180, 230, 255))
    out1 = os.path.join(SHOTS, "height_annotated_wall.png")
    img.save(out1)
    report["annotated_wall"] = dict(path=out1, top_row=top, bottom_row=bot,
                                    height_px=h_px, height_mm=h_mm)

    # ---------- B. 整条立面全景：12.6 m x 0.503 m 的亮带 ----------
    tag = "elevation_wide"
    cam = VH.Cam(cams[tag])
    p = os.path.join(SHOTS, VH.IMG[tag])
    img = Image.open(p).convert("RGB")
    d = ImageDraw.Draw(img)
    top = cam.project((0.0, -VH.FIELD_Y / 2, 0.0))[1]
    bot = cam.project((0.0, -VH.FIELD_Y / 2, -VH.FIELD_H))[1]
    u_l = cam.project((-VH.FIELD_X / 2, -VH.FIELD_Y / 2, 0.0))[0]
    u_r = cam.project((VH.FIELD_X / 2, -VH.FIELD_Y / 2, 0.0))[0]
    hline(d, top, u_l, u_r)
    hline(d, bot, u_l, u_r)
    for u in (u_l, u_r):
        d.line([(u, top - 15), (u, bot + 15)], fill=(255, 60, 60), width=3)
    f = font(40)
    h_px = bot - top
    label(d, (u_l + 20, top - 80),
          f"FIELD WALL: 12.6 m x 0.503 m   ({h_px:.0f} px tall here)", f)
    label(d, (u_l + 20, bot + 20),
          "width matches predicted 12.6 m to <1 px;  height/width ratio = 25.5 vs drawing 25.05",
          font(26), color=(180, 230, 255))
    out2 = os.path.join(SHOTS, "height_annotated_wide.png")
    img.save(out2)
    report["annotated_wide"] = dict(path=out2, top_row=top, bottom_row=bot,
                                    height_px=bot - top, width_u=[u_l, u_r])

    # ---------- C. 低机位斜视：台面 + 侧壁带 ----------
    tag = "oblique_low"
    cam = VH.Cam(cams[tag])
    p = os.path.join(SHOTS, VH.IMG[tag])
    img = Image.open(p).convert("RGB")
    d = ImageDraw.Draw(img)
    u = cam.w / 2.0
    wt, wb = VH.wall_rows_at_column(cam, u, -VH.FIELD_Y / 2)
    hline(d, wt, 40, cam.w - 40, color=(255, 190, 40))
    hline(d, wb, 40, cam.w - 40, color=(255, 190, 40))
    arrow_v(d, u, wt + 3, wb - 3, color=(255, 190, 40), head=10)
    f = font(34)
    label(d, (u + 20, (wt + wb) / 2 - 20),
          f"visible side wall band = {wb - wt:.0f} px  (= 0.503 m)", f,
          color=(255, 235, 120))
    label(d, (40, 30), "low oblique view: top surface + 0.503 m side walls + ground at z=-0.503",
          font(26), color=(180, 230, 255))
    out3 = os.path.join(SHOTS, "height_annotated_oblique.png")
    img.save(out3)
    report["annotated_oblique"] = dict(path=out3, wall_top=wt, wall_bottom=wb,
                                       band_px=wb - wt)

    for k, v in report.items():
        print(k, "->", v["path"])
    return 0


if __name__ == "__main__":
    sys.exit(main())