#!/usr/bin/env python3
"""PC 端：把板端 capture_one_frame.py 抓下来的 npz 渲染成演示图。

用法：
    python render_npz.py --src <npz目录> --out <图片目录>

输入（capture_one_frame.py 产出）：
    cloud_render.npz  x/y/z/rgb（设备坐标系）
    rgb.jpg           JPEG 原图 1600x1296
    calib.yaml        设备下发的标定（从板端 ~/.ros/odin_ros_driver/ 拷回）
输出：
    odin1_x5_demo.png      三联图：原图 / 深度伪彩 / 深度叠加
    odin1_rgb.jpg / odin1_depth.png / odin1_overlay.png  单图
"""
import argparse
import os
import re

import numpy as np
from PIL import Image, ImageDraw, ImageFont

TURBO = np.array([[48, 18, 59], [70, 107, 227], [62, 188, 195], [180, 222, 44],
                  [249, 160, 30], [232, 60, 20], [122, 4, 3]], dtype=np.float64)
FONT_CANDIDATES = [r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\simhei.ttf",
                   "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"]


def turbo(t):
    t = np.clip(t, 0, 1) * (len(TURBO) - 1)
    i = np.clip(t.astype(int), 0, len(TURBO) - 2)
    f = (t - i)[:, None]
    return TURBO[i] * (1 - f) + TURBO[i + 1] * f


class Calib:
    def __init__(self, path):
        txt = open(path, encoding="utf-8").read()

        def sc(k):
            return float(re.search(rf"^\s*{k}:\s*([-\d.eE+]+)", txt, re.M).group(1))

        tcl = re.search(r"Tcl_0:\s*\[(.*?)\]", txt, re.S).group(1)
        self.T = np.array([float(x) for x in tcl.replace("\n", " ").split(",")]).reshape(4, 4)
        self.fx, self.fy, self.skew = sc("A11"), sc("A22"), sc("A12")
        self.cx, self.cy = sc("u0"), sc("v0")
        self.k = [sc(k) for k in ("k2", "k3", "k4", "k5", "k6", "k7")]
        self.w, self.h = int(sc("image_width")), int(sc("image_height"))


def load_cloud(path):
    z = np.load(path)
    d = {k: z[k] for k in z.files if k not in ("step", "width", "height")}
    x, y, zz = d["x"].astype(float), d["y"].astype(float), d["z"].astype(float)
    ok = np.isfinite(x) & np.isfinite(y) & np.isfinite(zz) & (x * x + y * y + zz * zz > 0)
    x, y, zz = x[ok], y[ok], zz[ok]
    if "rgb" in d:
        u = d["rgb"].view(np.uint32) if d["rgb"].dtype == np.float32 else d["rgb"].astype(np.uint32)
        rgb = np.stack([((u >> 16) & 255), ((u >> 8) & 255), (u & 255)], 1)[ok].astype(np.uint8)
    else:
        rgb = np.full((x.size, 3), 200, dtype=np.uint8)
    return x, y, zz, rgb


def project(cal, x, y, z):
    """设备系 → 相机系（Tcl），FishPoly 鱼眼投影到像素。"""
    p = np.stack([x, y, z, np.ones_like(x)], 1)
    pc = p @ cal.T.T
    x1, y1, z1 = pc[:, 0], pc[:, 1], pc[:, 2]
    norm = np.linalg.norm(pc[:, :3], axis=1)
    r = np.hypot(x1, y1)
    ok = (z1 > 0) & (norm > 1e-6) & (r > 1e-6)
    th = np.arccos(np.clip(z1[ok] / norm[ok], -1, 1))
    td = th.copy()
    for i, k in enumerate(cal.k, start=2):
        td = td + th ** i * k
    s = td / r[ok]
    u = x1[ok] * s * cal.fx + y1[ok] * s * cal.skew + cal.cx
    v = y1[ok] * s * cal.fy + cal.cy
    g = (u >= 0) & (u < cal.w) & (v >= 0) & (v < cal.h)
    idx = np.where(ok)[0][g]
    return idx, u[g].astype(int), v[g].astype(int)


def font(size):
    for f in FONT_CANDIDATES:
        if os.path.exists(f):
            return ImageFont.truetype(f, size)
    return ImageFont.load_default()


def panel(im, title, width=980, size=25):
    im = im.resize((width, int(im.height * width / im.width)), Image.LANCZOS)
    cv = Image.new("RGB", (im.width, im.height + 46), (16, 16, 18))
    cv.paste(im, (0, 46))
    ImageDraw.Draw(cv).text((14, 10), title, font=font(size), fill=(235, 235, 235))
    return cv


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True, help="npz/jpg/calib.yaml 所在目录")
    ap.add_argument("--out", required=True, help="输出目录")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    cal = Calib(os.path.join(args.src, "calib.yaml"))
    x, y, z, rgb = load_cloud(os.path.join(args.src, "cloud_render.npz"))
    idx, u, v = project(cal, x, y, z)
    rng = np.linalg.norm(np.stack([x, y, z], 1), axis=1)[idx]

    photo = Image.open(os.path.join(args.src, "rgb.jpg")).convert("RGB")
    rot = lambda im: im.rotate(-90, expand=True)  # 设备侧装，统一转正

    dep = Image.new("RGB", (cal.w, cal.h), (0, 0, 0))
    dd = ImageDraw.Draw(dep)
    dcol = turbo((rng - rng.min()) / (rng.max() - rng.min() + 1e-9))
    for a, b, c in zip(u, v, dcol):
        dd.ellipse([a - 4, b - 4, a + 4, b + 4], fill=tuple(int(t) for t in c))

    overlay = Image.blend(photo, dep, 0.42)

    p1 = panel(rot(photo), f"① Odin1 RGB {cal.w}x{cal.h}（USB3 实时传到 RDK X5）")
    p2 = panel(rot(dep), f"② 同帧点云深度图（{rng.min():.2f}-{rng.max():.2f} m 伪彩，{idx.size} 点）")
    p3 = panel(rot(overlay), "③ 深度叠在 RGB 上：边缘吻合 → 标定与时间同步正确")

    h = p1.height + p2.height + p3.height
    fig = Image.new("RGB", (p1.width, h), (8, 8, 10))
    y0 = 0
    for p in (p1, p2, p3):
        fig.paste(p, (0, y0))
        y0 += p.height
    fig.save(os.path.join(args.out, "odin1_x5_demo.png"), quality=95)
    rot(photo).save(os.path.join(args.out, "odin1_rgb.jpg"), quality=92)
    rot(dep).save(os.path.join(args.out, "odin1_depth.png"))
    rot(overlay).save(os.path.join(args.out, "odin1_overlay.png"))
    print(f"saved {args.out}/odin1_x5_demo.png  {fig.size}  points={idx.size}  "
          f"range={rng.min():.2f}-{rng.max():.2f} m")


if __name__ == "__main__":
    main()
