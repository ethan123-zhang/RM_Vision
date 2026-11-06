#!/usr/bin/env python3
"""顶视渲染图 → Nav2 地图（pgm + yaml），并用渲染图做独立校核。

思路（为什么不是"把渲染图二值化了直接用"）：
  - 地图的**权威来源是图纸几何**：场地 12.600 x 8.600 m，围挡内侧面在 ±6.250 / ±4.250。
    渲染图有阴影、有 5 cm 围挡的透视斜面，直接二值化会把边界量偏 10 cm 量级；
  - 所以地图按几何生成（自由区 = 围挡内侧面以内），渲染图用来**独立校核**：
      1) 场地亮面剪影 bbox → 应等于 12.60 x 8.60（台面边界在 z=0，正俯视无透视误差）
      2) 围挡红顶沿 bbox → 应等于 12.62 x 8.62（红顶沿 7 cm 宽、中心线 ±6.275/±4.275，
         在 z=+0.31 上，需按针孔模型做透视改正）
      3) 地图自由区 → 应 12.50 x 8.50 m²（= 围挡内侧面以内），且边界与红顶沿内缘一致

用法: python3 make_nav_map.py [--render shots/nav_topdown.png] [--out maps/]
产出: maps/rmul2027_3v3_fenced.pgm + .yaml + _preview.png
      shots/nav_map_overlay.png（渲染图 + 地图边界叠加）
      results/report_nav_map.json（全部读数与断言）
"""
import argparse
import json
import math
import os
import struct
import sys

import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from verify_height import load_cameras  # noqa: E402

# ---- 图纸几何（与 report_geometry.json / rm_field/README.md §0 一致）
FIELD_LEN, FIELD_WID = 12.600, 8.600
FENCE_T = 0.05
FENCE_H = 0.30
RAIL_H = 0.31          # 红顶沿顶面在 z=+0.31（fence link 0.15 高 0.30 + 顶沿中心 0.31 / 厚 0.04 → 顶 0.33）
RAIL_TOP_Z = 0.33      # 红顶沿**顶面**：link pose z=0.31、厚 0.04 → 顶面 0.33
RAIL_W = 0.07
RAIL_CENTER_X = FIELD_LEN / 2 - FENCE_T / 2      # 6.275
RAIL_CENTER_Y = FIELD_WID / 2 - FENCE_T / 2      # 4.275
INNER_X = RAIL_CENTER_X - FENCE_T / 2            # 6.250
INNER_Y = RAIL_CENTER_Y - FENCE_T / 2            # 4.250

# ---- 地图参数
# 0.025 m/px 让自由区正好是整数像素：12.5/0.025=500、8.5/0.025=340，
# 像素中心与围挡内侧面严格对齐（面积可精确断言，不留半像素量化误差）。
RES = 0.025                  # m/px
MAP_HALF_X = FIELD_LEN / 2 + 0.10     # 6.40（四周留 10 cm 边，界外一律算障碍）
MAP_HALF_Y = FIELD_WID / 2 + 0.10     # 4.40
ORIGIN = (round(-MAP_HALF_X, 4), round(-MAP_HALF_Y, 4))

TOL_FIELD = 0.03
TOL_RAIL = 0.03
TOL_RAIL_CENTER = 0.02
TOL_AREA = 0.05
# 台面亮度 ≈ 99 分位的 0.78：实拍台面 215、围挡灰面 157、台面阴影 140、地面 90，
# 0.78*215=168 把"受光台面"单独切出来（容差 ~10 灰阶，不会被阴影/围挡面误判）。
LIT_RATIO = 0.78


# ---------------------------------------------------------------- 图像 ↔ 世界
def cam_params(world_path):
    cams = load_cameras(world_path)
    name = "nav_topdown"
    if name not in cams:
        raise SystemExit(f"!! {world_path} 里没有 {name} 相机")
    c = cams[name]
    f = (c["w"] / 2.0) / math.tan(c["hfov"] / 2.0)
    return {
        "w": c["w"], "h": c["h"], "hfov": c["hfov"], "f": f,
        "cam_z": c["pose"][2],
    }


def px_to_world(cam, u, v, z):
    """正俯视针孔反投影：给定世界高度 z，把像素坐标换成世界 X/Y。"""
    k = (cam["cam_z"] - z) / cam["f"]
    x = (u - cam["w"] / 2.0) * k
    y = -(v - cam["h"] / 2.0) * k
    return x, y


def world_to_px(cam, x, y, z):
    k = cam["f"] / (cam["cam_z"] - z)
    return cam["w"] / 2.0 + x * k, cam["h"] / 2.0 - y * k


def otsu(gray):
    hist, _ = np.histogram(gray, bins=256, range=(0, 256))
    hist = hist.astype(float)
    total = hist.sum()
    if total == 0:
        return 128
    p = hist / total
    omega = np.cumsum(p)
    mu = np.cumsum(p * np.arange(256))
    mu_t = mu[-1]
    denom = omega * (1.0 - omega)
    denom[denom == 0] = 1e-12
    sigma_b = (mu_t * omega - mu) ** 2 / denom
    return int(np.argmax(sigma_b))


def bbox_of(mask):
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        return None
    return int(xs.min()), int(xs.max()), int(ys.min()), int(ys.max())


# ---------------------------------------------------------------- 地图
def build_map():
    w = int(round(2 * MAP_HALF_X / RES))
    h = int(round(2 * MAP_HALF_Y / RES))
    img = np.full((h, w), 0, dtype=np.uint8)          # 默认全障碍（界外=0，别让车出去）
    cols = (np.arange(w) + 0.5) * RES + ORIGIN[0]     # 每个像素中心的世界 x
    rows = (np.arange(h) + 0.5) * RES + ORIGIN[1]
    free_cols = np.abs(cols) < INNER_X
    free_rows = np.abs(rows) < INNER_Y
    img[np.ix_(free_rows, free_cols)] = 254
    return img


def write_pgm(path, img):
    h, w = img.shape
    with open(path, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (w, h))
        f.write(img.tobytes())


def write_yaml(path, pgm_name):
    with open(path, "w", encoding="utf-8") as f:
        f.write("# RMUL2027 3V3 场地（带 0.30 m 围挡）的 Nav2 地图，由 make_nav_map.py 生成\n")
        f.write(f"image: {pgm_name}\n")
        f.write(f"resolution: {RES}\n")
        f.write(f"origin: [{ORIGIN[0]}, {ORIGIN[1]}, 0.0]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--render", default=os.path.join(HERE, "shots", "nav_topdown.png"))
    ap.add_argument("--world", default=os.path.join(HERE, "worlds", "nav_map_capture.sdf"))
    ap.add_argument("--out", default=os.path.join(HERE, "maps"))
    ap.add_argument("--report", default=os.path.join(HERE, "results", "report_nav_map.json"))
    ap.add_argument("--overlay", default=os.path.join(HERE, "shots", "nav_map_overlay.png"))
    args = ap.parse_args()

    for p in (args.render, args.world):
        if not os.path.exists(p):
            raise SystemExit(f"!! 缺文件: {p}")
    os.makedirs(args.out, exist_ok=True)

    cam = cam_params(args.world)
    rgb = np.asarray(Image.open(args.render).convert("RGB"), dtype=np.float32)
    h, w, _ = rgb.shape
    if (w, h) != (cam["w"], cam["h"]):
        raise SystemExit(f"!! 渲染图 {w}x{h} 与 SDF 相机 {cam['w']}x{cam['h']} 不一致")
    lum = 0.299 * rgb[:, :, 0] + 0.587 * rgb[:, :, 1] + 0.114 * rgb[:, :, 2]
    thr = otsu(lum)
    p_peak = float(np.percentile(lum, 99))
    thr_lit = LIT_RATIO * p_peak
    print(f"渲染图 {w}x{h}  f={cam['f']:.1f} px  1px={20.0/cam['f']:.4f} m")
    print(f"  亮度: Otsu={thr}  99分位(台面)={p_peak:.1f}  '受光台面'阈值={thr_lit:.1f}")

    # ---- 1) 受光台面：亮度在台面档、且不是红顶沿。
    # 围挡踩在台面内侧（x∈[6.250,6.300]），所以台面受光区的边界**就是围挡内侧面**，
    # 即 12.50 x 8.50 m —— 正好是地图自由区应该覆盖的范围。
    # 台面边缘的阴影（东/南两侧约 0.15 m）亮度 140，低于阈值，被自然排除。
    r, g, b = rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2]
    red = (r > 140) & ((r - g) > 50) & ((r - b) > 50)
    lit = (lum > thr_lit) & ~red
    lb = bbox_of(lit)
    if lb is None:
        raise SystemExit("!! 渲染图里找不到受光台面（相机没对准？）")
    lx0, lx1, ly0, ly1 = lb
    (la_x, la_y) = px_to_world(cam, lx0, ly0, 0.0)
    (lb_x, lb_y) = px_to_world(cam, lx1, ly1, 0.0)
    px_m = (cam["cam_z"]) / cam["f"]
    # bbox 首尾像素中心之间少算 1 px，补回来（边缘像素本身也算进尺寸）
    lit_w_m = abs(lb_x - la_x) + px_m
    lit_h_m = abs(lb_y - la_y) + px_m
    lit_long_m, lit_short_m = max(lit_w_m, lit_h_m), min(lit_w_m, lit_h_m)

    # ---- 2) 围挡红顶沿：取过图像中心的**行/列**上的红色连续段（避免四角红顶沿端头
#         把全局 bbox 撑大 —— 南北顶沿长 12.66 m，端头比顶沿外缘还靠外 2 cm）。
    #         红顶沿顶面在 z=+0.33，必须按该高度做透视改正，否则中心线会偏 ~8 cm。
    def runs_1d(mask):
        idx = np.nonzero(mask)[0]
        out = []
        if len(idx) == 0:
            return out
        s = p = int(idx[0])
        for i in idx[1:]:
            i = int(i)
            if i != p + 1:
                out.append((s, p))
                s = i
            p = i
        out.append((s, p))
        return out

    row_runs = runs_1d(red[h // 2])
    col_runs = runs_1d(red[:, w // 2])
    m_px_rail = (cam["cam_z"] - RAIL_TOP_Z) / cam["f"]
    rail = None
    if len(row_runs) >= 2 and len(col_runs) >= 2:
        def run_center(r):
            return (r[0] + r[1]) / 2.0

        u_l = run_center(row_runs[0])
        u_r = run_center(row_runs[-1])
        v_t = run_center(col_runs[0])
        v_b = run_center(col_runs[-1])
        x_l = (u_l - cam["w"] / 2.0) * m_px_rail
        x_r = (u_r - cam["w"] / 2.0) * m_px_rail
        y_t = -(v_t - cam["h"] / 2.0) * m_px_rail
        y_b = -(v_b - cam["h"] / 2.0) * m_px_rail
        rail = {
            "runs_row": row_runs, "runs_col": col_runs,
            "center_x_m": [round(x_l, 4), round(x_r, 4)],
            "center_y_m": [round(y_t, 4), round(y_b, 4)],
            "sep_x_m": round(abs(x_r - x_l), 4),          # 中心线间距，应 2*6.275
            "sep_y_m": round(abs(y_t - y_b), 4),          # 应 2*4.275
            "ring_center_m": [round((x_l + x_r) / 2.0, 4), round((y_t + y_b) / 2.0, 4)],
            "width_x_m": round((row_runs[-1][1] - row_runs[-1][0] + 1) * m_px_rail, 4),
            "width_y_m": round((col_runs[-1][1] - col_runs[-1][0] + 1) * m_px_rail, 4),
        }
    if rail is None:
        raise SystemExit("!! 中心行/列上没有找到两段红色顶沿（围挡没渲染出来？）")
    rail_half_x = rail["sep_x_m"] / 2.0
    rail_half_y = rail["sep_y_m"] / 2.0

    # ---- 3) 地图（按几何生成）
    img = build_map()
    mh, mw = img.shape
    free_px = int((img == 254).sum())
    free_area = free_px * RES * RES
    exp_area = (2 * INNER_X) * (2 * INNER_Y)
    pgm_name = "rmul2027_3v3_fenced.pgm"
    pgm_path = os.path.join(args.out, pgm_name)
    yaml_path = os.path.join(args.out, "rmul2027_3v3_fenced.yaml")
    write_pgm(pgm_path, img)
    write_yaml(yaml_path, pgm_name)
    Image.fromarray(np.where(img > 0, 254, 0).astype(np.uint8)).save(
        os.path.join(args.out, "rmul2027_3v3_fenced_preview.png"))

    # 自由区边界（像素→米）实测：从掩码反推，验证像素离散化没跑偏
    ys, xs = np.nonzero(img == 254)
    free_x = (xs + 0.5) * RES + ORIGIN[0]
    free_y = (ys + 0.5) * RES + ORIGIN[1]
    free_rect = (free_x.max() - free_x.min() + RES, free_y.max() - free_y.min() + RES)

    # ---- 4) 叠加图：渲染图上画地图自由区边界 + 围挡内侧面
    ov = Image.open(args.render).convert("RGB")
    dr = ImageDraw.Draw(ov)
    def rect(px_half_x, px_half_y, color, width=2):
        a = world_to_px(cam, -px_half_x, px_half_y, 0.0)
        c = world_to_px(cam, px_half_x, -px_half_y, 0.0)
        dr.rectangle([min(a[0], c[0]), min(a[1], c[1]),
                      max(a[0], c[0]), max(a[1], c[1])], outline=color, width=width)
    rect(INNER_X, INNER_Y, (0, 255, 0), 2)                 # 绿 = 地图自由区边界（围挡内侧面）
    rect(FIELD_LEN / 2, FIELD_WID / 2, (0, 255, 255), 2)   # 青 = 图纸场地边界 12.6x8.6
    ov.save(args.overlay)

    # ---- 5) 断言
    checks = {
        "render_lit_top_m": {
            "_what": "受光台面 bbox（边界 = 围挡内侧面）→ 应 12.50 x 8.50 m",
            "measured": [round(lit_long_m, 3), round(lit_short_m, 3)],
            "expected": [round(2 * INNER_X, 3), round(2 * INNER_Y, 3)],
            "dev": [round(abs(lit_long_m - 2 * INNER_X), 3),
                    round(abs(lit_short_m - 2 * INNER_Y), 3)],
            "tol": TOL_FIELD,
            "ok": (abs(lit_long_m - 2 * INNER_X) <= TOL_FIELD
                   and abs(lit_short_m - 2 * INNER_Y) <= TOL_FIELD),
        },
        "render_rail_sep_m": {
            "_what": "红顶沿中心线间距（z=+0.33 透视改正）→ 应 2*6.275 / 2*4.275",
            "measured": [rail["sep_x_m"], rail["sep_y_m"]],
            "expected": [round(2 * RAIL_CENTER_X, 3), round(2 * RAIL_CENTER_Y, 3)],
            "dev": [round(abs(rail["sep_x_m"] - 2 * RAIL_CENTER_X), 3),
                    round(abs(rail["sep_y_m"] - 2 * RAIL_CENTER_Y), 3)],
            "tol": TOL_RAIL,
            "ok": (abs(rail["sep_x_m"] - 2 * RAIL_CENTER_X) <= TOL_RAIL
                   and abs(rail["sep_y_m"] - 2 * RAIL_CENTER_Y) <= TOL_RAIL),
        },
        "render_rail_ring_center_m": {
            "_what": "红顶沿环的中心 → 应 (0,0)（场地与相机都对中）",
            "measured": rail["ring_center_m"], "expected": [0.0, 0.0],
            "tol": TOL_RAIL_CENTER,
            "ok": (abs(rail["ring_center_m"][0]) <= TOL_RAIL_CENTER
                   and abs(rail["ring_center_m"][1]) <= TOL_RAIL_CENTER),
        },
        "render_rail_width_m": {
            "_what": "红顶沿在画面里的宽度 → 应 0.070 m（模型 12.66/8.6 长的顶沿，宽 7 cm）",
            "measured": [rail["width_x_m"], rail["width_y_m"]],
            "expected": [RAIL_W, RAIL_W],
            "tol": 0.02,
            "ok": (abs(rail["width_x_m"] - RAIL_W) <= 0.02
                   and abs(rail["width_y_m"] - RAIL_W) <= 0.02),
        },
        "map_free_area_m2": {
            "_what": "地图自由区像素数换算成面积（= 围挡内侧面以内）",
            "measured": round(free_area, 3), "expected": round(exp_area, 3),
            "dev": round(abs(free_area - exp_area), 3), "tol": TOL_AREA,
            "ok": abs(free_area - exp_area) <= TOL_AREA,
        },
        "map_free_rect_m": {
            "_what": "地图自由区 bbox（像素离散化后）→ 应 12.50 x 8.50 m",
            "measured": [round(free_rect[0], 3), round(free_rect[1], 3)],
            "expected": [round(2 * INNER_X, 3), round(2 * INNER_Y, 3)],
            "dev": [round(abs(free_rect[0] - 2 * INNER_X), 3),
                    round(abs(free_rect[1] - 2 * INNER_Y), 3)],
            "tol": RES,
            "ok": (abs(free_rect[0] - 2 * INNER_X) <= RES
                   and abs(free_rect[1] - 2 * INNER_Y) <= RES),
        },
        "map_meta": {
            "_what": "地图尺寸/origin 与配置一致（12.80 x 8.80 m，origin -6.40/-4.40）",
            "image": pgm_name, "width_px": mw, "height_px": mh, "resolution": RES,
            "origin": list(ORIGIN),
            "extent_m": [round(mw * RES, 3), round(mh * RES, 3)],
            "expected_extent_m": [round(2 * MAP_HALF_X, 3), round(2 * MAP_HALF_Y, 3)],
            "ok": (abs(mw * RES - 2 * MAP_HALF_X) < 1e-9
                   and abs(mh * RES - 2 * MAP_HALF_Y) < 1e-9),
        },
        "map_vs_render_boundary": {
            "_what": "地图自由区边界 ↔ 渲染实测受光台面边界（同一件东西的两条独立证据）",
            "measured": [round(lit_long_m, 3), round(lit_short_m, 3)],
            "map_inner_half": [INNER_X, INNER_Y],
            "render_rail_inner_half": [round(rail_half_x - RAIL_W / 2.0, 4),
                                       round(rail_half_y - RAIL_W / 2.0, 4)],
            "tol": TOL_FIELD,
            "ok": bool(abs(lit_long_m - 2 * INNER_X) <= TOL_FIELD
                       and abs(lit_short_m - 2 * INNER_Y) <= TOL_FIELD
                       and abs(rail_half_x - RAIL_W / 2.0 - INNER_X) <= TOL_RAIL
                       and abs(rail_half_y - RAIL_W / 2.0 - INNER_Y) <= TOL_RAIL),
        },
    }
    ok = all(c.get("ok") for c in checks.values())
    report = {
        "render": os.path.abspath(args.render),
        "camera": {"width": cam["w"], "height": cam["h"], "hfov": cam["hfov"],
                   "focal_px": round(cam["f"], 2), "cam_z": cam["cam_z"],
                   "m_per_px_at_z0": round(cam["cam_z"] / cam["f"], 5)},
        "thresholds": {"otsu": thr, "p99_lum": round(p_peak, 2),
                       "lit_ratio": LIT_RATIO, "lit": round(thr_lit, 2)},
        "bbox_px": {"lit_top": list(lb)},
        "rail_measure": rail,
        "map": {"pgm": pgm_path, "yaml": yaml_path,
                "free_px": free_px, "occupied_px": int((img == 0).sum())},
        "checks": checks,
        "summary": {"ok": bool(ok)},
    }
    os.makedirs(os.path.dirname(args.report), exist_ok=True)
    with open(args.report, "w") as f:
        json.dump(report, f, indent=2, ensure_ascii=False, default=bool)

    print("\n===== 校核 =====")
    for k, c in checks.items():
        print(f"  [{'OK ' if c.get('ok') else 'FAIL'}] {k}: "
              f"measured={c.get('measured', c.get('extent_m'))} "
              f"expected={c.get('expected', c.get('expected_extent_m'))} "
              f"tol={c.get('tol', '-')}")
    print(f"\n地图: {pgm_path} ({mw}x{mh} @ {RES} m/px, origin {ORIGIN})")
    print(f"yaml: {yaml_path}")
    print(f"叠加图: {args.overlay}")
    print(f"报告: {args.report}")
    print(f"summary.ok = {ok}")
    return 0 if ok else 3


if __name__ == "__main__":
    raise SystemExit(main())