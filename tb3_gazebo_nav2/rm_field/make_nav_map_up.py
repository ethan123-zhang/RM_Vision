#!/usr/bin/env python3
"""翻转后 RMUL2027 3V3 场地 → Nav2 地图（pgm + yaml）+ 独立校核。

为什么不用 make_nav_map.py 了：那个脚本生成的自由区是**围挡内侧面以内的整块矩形**，
对应"翻转前、台面平整"的场地。2026-11-19 场地翻转成结构面朝上之后，地面 z=0 上多了
0.19~0.40 m 的凸起结构（平台 / 筋 / 外墙），自由区变成"室内矩形 − 凸起结构投影"，
旧图会把车规划到平台上去。

权威几何来源 = 碰撞体（debug/box_collisions.json，48 块 box，5 cm 栅格，与
models/rmul2027_3v3_field_up/model.sdf 的 BOX_COLLISION_BEGIN/END 同源）。
理由：机器人实际上是被这 48 块 box 挡住的；拿三角网格直接当几何反而会和物理对不上
（网格碰撞把车垫高 6 mm 那次事故就是这么来的）。5 cm 量化误差交给 Nav2 的
inflation（TB3 参数 robot_radius 0.22 / inflation_radius 0.55）吸收。

独立校核 = 正俯视渲染（worlds/nav_map_capture_up.sdf，1 px = 0.01 m，平光无阴影）：
  浅灰=地面 / 绿=凸起结构（含墙面内侧裙边）/ 黄=墙顶沿 / 深灰偏蓝=场外
  1) 渲染里"浅灰地面"的 bbox → 应 = 12.00 x 8.00 m。正俯视时地面的可见边界
     就是墙面与地面的交线（墙面在 ±6.00 之外），所以四个边正好量到 ±6.00 / ±4.00；
  2) 在室内缩边区域 (|x|<5.90, |y|<3.90) 内，比较 渲染地面面积 ↔ 地图自由区面积、
     渲染绿色面积 ↔ 地图室内障碍面积（面积、重心）；
  3) 去掉离分界 0.10 m 的边界后逐像素比较（平台顶面在 z≈0.2 上会被透视放大约 2%）；
  4) 连通性：从出生点 (0,0) 泛洪，必须覆盖全部自由格且能到每个验收目标点。

用法: python3 make_nav_map_up.py [--render shots/nav_topdown_up.png] [--out maps/]
产出: maps/rmul2027_3v3_up.pgm + .yaml + _preview.png
      shots/nav_map_up_overlay.png（渲染图 + 地图自由区边界叠加）
      results/report_nav_map_up.json（全部读数与断言）
"""
import argparse
import json
import math
import os
import sys
from collections import deque

import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from verify_height import load_cameras  # noqa: E402

# ---- 栅格几何：由碰撞体实测（debug/box_collisions.json → stats.test_geometry）
INNER_X = 6.00          # 墙面内侧面 x = ±6.00（翻转后场地自带 0.40 m 墙）
INNER_Y = 4.00          # 墙面内侧面 y = ±4.00
BOXES_JSON = os.path.join(HERE, "debug", "box_collisions.json")

# ---- 地图参数：分辨率/尺寸/origin 与翻转前那张图保持一致（RViz、参数文件都不用改）
RES = 0.025
MAP_HALF_X = 6.40
MAP_HALF_Y = 4.40
ORIGIN = (-MAP_HALF_X, -MAP_HALF_Y)

SPAWN = (0.0, 0.0)                       # nav2_field.sh 的 spawn 点 / AMCL 初始位姿
GOALS = [(4.0, 2.0), (4.0, -2.5), (0.0, 0.0)]

# 校核容差
TOL_FIELD = 0.05         # 渲染实测地面 bbox / 四边 与 12.00 x 8.00 的偏差（m）
TOL_AREA = 0.10          # 地图自由区面积 vs 几何解析面积（m²）
# 渲染面积 vs 地图面积：平台顶面在 z≈0.2 上按针孔投影放大 20/(20-0.2) ≈ +2%，
# 18.5 m² 的平台 ⇒ 地面可见面积少 ~0.37 m²。容差放到 0.60 覆盖这个已知系统偏差。
TOL_AREA_RENDER = 0.60
TOL_CENTROID = 0.10      # 绿区重心 vs 地图室内障碍重心（m）
# 面积/重心比较用的同一块区域：室内矩形再缩 0.10 m，避开墙面内侧的绿色裙边（|x|≈6.0~6.15）
RECT_X, RECT_Y = 5.90, 3.90
EDGE_COUNT_RATIO = 0.20  # 地面边缘：只认"该列/行地面像素数 ≥ 最多的 20%"的列/行（剔杂散像素）
CORE_M = 0.10            # 逐像素比较时剔掉的边界宽度（m）
CORE_MIN_AGREE = 0.995   # 核心区一致率下限

# 颜色判据（实测值见 report.json 的 classify.samples，渲染世界用平光无阴影）：
#   地面 (105,105,106) 中性灰；场外 (112,112,120) 带蓝（材料 diffuse 0.12/0.12/0.14）；
#   绿 (32,93,50)；黄墙顶 (116,108,32)；红墙身（很少，只有墙侧面露出来）。
SAT_MIN = 0.15           # 饱和度 ≥ 0.15 视为"有颜色"
FLOOR_MAX_BLUENESS = 0.04  # (b-r)/max(r,g,b)：地面 0.009、场外 0.067
FLOOR_MIN_V = 0.25       # 太黑的不算地面


# ---------------------------------------------------------------- 几何 ↔ 栅格
def load_boxes(path=BOXES_JSON):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    geo = data["stats"]["test_geometry"]
    boxes = data["boxes"]
    for b in boxes:
        cx, cy, _ = b["center"]
        sx, sy, _ = b["size"]
        b["x0"], b["x1"] = cx - sx / 2.0, cx + sx / 2.0
        b["y0"], b["y1"] = cy - sy / 2.0, cy + sy / 2.0
    return boxes, geo, data["stats"]


def box_area_in_rect(b, hx, hy):
    """box 投影与 |x|<hx, |y|<hy 的交集面积（解析值，用来核对栅格化误差）"""
    ax = max(0.0, min(b["x1"], hx) - max(b["x0"], -hx))
    ay = max(0.0, min(b["y1"], hy) - max(b["y0"], -hy))
    return ax * ay


def map_dims():
    return (int(round(2 * MAP_HALF_Y / RES)), int(round(2 * MAP_HALF_X / RES)))


def cell_cols_rows():
    mh, mw = map_dims()
    cols = (np.arange(mw) + 0.5) * RES + ORIGIN[0]
    rows = (np.arange(mh) + 0.5) * RES + ORIGIN[1]
    return cols, rows


def build_map(boxes):
    cols, rows = cell_cols_rows()
    free = ((np.abs(cols) < INNER_X)[None, :]
            & (np.abs(rows) < INNER_Y)[:, None]).copy()      # 室内矩形
    for b in boxes:
        inx = np.abs(cols - b["center"][0]) < (b["size"][0] / 2.0)
        iny = np.abs(rows - b["center"][1]) < (b["size"][1] / 2.0)
        free &= ~(iny[:, None] & inx[None, :])               # 挖掉凸起结构
    return np.where(free, 254, 0).astype(np.uint8)


def world_to_cell(x, y):
    mh, mw = map_dims()
    c = int(math.floor((x - ORIGIN[0]) / RES))
    r = int(math.floor((y - ORIGIN[1]) / RES))
    return (r, c) if (0 <= r < mh and 0 <= c < mw) else None


def flood_fill(free, start_rc):
    """4 邻接泛洪，返回可达掩码"""
    h, w = free.shape
    seen = np.zeros_like(free, dtype=bool)
    r0, c0 = start_rc
    if not free[r0, c0]:
        return seen
    dq = deque([(r0, c0)])
    seen[r0, c0] = True
    while dq:
        r, c = dq.popleft()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            rr, cc = r + dr, c + dc
            if 0 <= rr < h and 0 <= cc < w and free[rr, cc] and not seen[rr, cc]:
                seen[rr, cc] = True
                dq.append((rr, cc))
    return seen


# ---------------------------------------------------------------- 渲染图分类
def classify(rgb):
    """按色相/中性灰/蓝偏把正俯视渲染图分成 地面 / 场外 / 结构 / 墙，返回掩码。"""
    r, g, b = rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2]
    mx = rgb.max(axis=2)
    mn = rgb.min(axis=2)
    d = mx - mn
    sat = np.where(mx > 1e-6, d / np.maximum(mx, 1e-6), 0.0)
    blueness = np.where(mx > 1e-6, (b - r) / np.maximum(mx, 1e-6), 0.0)

    hue = np.zeros_like(mx)
    nz = d > 1e-6
    im = (mx == r) & nz
    hue[im] = ((g[im] - b[im]) / d[im]) % 6.0
    ig = (mx == g) & nz
    hue[ig] = (b[ig] - r[ig]) / d[ig] + 2.0
    ib = (mx == b) & nz
    hue[ib] = (r[ib] - g[ib]) / d[ib] + 4.0
    hue = hue / 6.0

    gray = sat < SAT_MIN
    floor = gray & (blueness < FLOOR_MAX_BLUENESS) & (mx >= FLOOR_MIN_V)   # 中性灰 = 地面
    outside = gray & ~floor                                                # 偏蓝深灰 = 场外
    colored = ~gray
    green = colored & (hue > 0.22) & (hue < 0.45)
    yellow = colored & (hue >= 0.10) & (hue <= 0.22)
    red = colored & ((hue < 0.06) | (hue > 0.94))
    other = colored & ~(green | yellow | red)
    return {"floor": floor, "outside": outside, "green": green,
            "yellow": yellow, "red": red, "other": other,
            "mx": mx, "sat": sat, "hue": hue, "blueness": blueness}


def px_to_world(cam, u, v, z):
    """正俯视针孔反投影（与 nav_map_capture_up.sdf 的相机朝向一致）。"""
    k = (cam["cam_z"] - z) / cam["f"]
    x = (u - cam["w"] / 2.0) * k
    y = -(v - cam["h"] / 2.0) * k
    return x, y


def world_to_px(cam, x, y, z):
    k = cam["f"] / (cam["cam_z"] - z)
    return cam["w"] / 2.0 + x * k, cam["h"] / 2.0 - y * k


def cam_params(world_path, name="nav_topdown"):
    cams = load_cameras(world_path)
    if name not in cams:
        raise SystemExit(f"!! {world_path} 里没有 {name} 相机")
    c = cams[name]
    f = (c["w"] / 2.0) / math.tan(c["hfov"] / 2.0)
    return {"w": c["w"], "h": c["h"], "hfov": c["hfov"], "f": f,
            "cam_z": c["pose"][2], "m_per_px": c["pose"][2] / f}


def erode(mask, n):
    """正方形结构元腐蚀 n 次（n px）"""
    m = mask.copy()
    for _ in range(n):
        e = m.copy()
        e[1:, :] &= m[:-1, :]
        e[:-1, :] &= m[1:, :]
        e[:, 1:] &= m[:, :-1]
        e[:, :-1] &= m[:, 1:]
        e[0, :] = False
        e[-1, :] = False
        e[:, 0] = False
        e[:, -1] = False
        m = e
    return m


def write_pgm(path, img):
    """写 PGM。

    方向约定：map_server 把**图像第 0 行当作地图最上面一行**（world y 最大），
    而 build_map 里第 0 行是 y 最小的一行 ⇒ 落盘前必须上下翻转。
    旧版 make_nav_map.py 漏了这一步，但当时自由区是一整块矩形，翻转肉眼看不出来；
    翻转后场内有平台/墙，不翻转会把障碍摆到镜像位置（AMCL 会锁到镜像位姿上）。
    读回校验见 occupancy_at()。
    """
    h, w = img.shape
    with open(path, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (w, h))
        f.write(np.ascontiguousarray(img[::-1]).tobytes())


def read_pgm(path):
    with open(path, "rb") as f:
        if f.readline().strip() != b"P5":
            raise SystemExit(f"!! {path} 不是 P5 PGM")
        w, h = [int(v) for v in f.readline().split()]
        if int(f.readline()) != 255:
            raise SystemExit(f"!! {path} 不是 8 bit PGM")
        data = np.frombuffer(f.read(), dtype=np.uint8)
    return data.reshape(h, w)


def occupancy_at(img, x, y):
    """按 map_server 读图约定取某世界坐标处的像素值（254=自由、0=障碍）。"""
    h, w = img.shape
    c = int((x - ORIGIN[0]) / RES)
    r = int((y - ORIGIN[1]) / RES)            # 地图行号（0 = 世界最低 y）
    if not (0 <= c < w and 0 <= r < h):
        return None
    return int(img[h - 1 - r, c])             # 图像行 = 上下翻转后的行


def write_yaml(path, pgm_name):
    with open(path, "w", encoding="utf-8") as f:
        f.write("# RMUL2027 3V3 场地（翻转后：地面 z=0，凸起结构最高 0.40 m）的 Nav2 地图\n")
        f.write("# 由 make_nav_map_up.py 生成：自由区 = 墙壁内侧面以内 − 48 块碰撞 box 的投影\n")
        f.write(f"image: {pgm_name}\n")
        f.write(f"resolution: {RES}\n")
        f.write(f"origin: [{ORIGIN[0]}, {ORIGIN[1]}, 0.0]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--render", default=os.path.join(HERE, "shots", "nav_topdown_up.png"))
    ap.add_argument("--world", default=os.path.join(HERE, "worlds", "nav_map_capture_up.sdf"))
    ap.add_argument("--out", default=os.path.join(HERE, "maps"))
    ap.add_argument("--report", default=os.path.join(HERE, "results", "report_nav_map_up.json"))
    ap.add_argument("--overlay", default=os.path.join(HERE, "shots", "nav_map_up_overlay.png"))
    args = ap.parse_args()

    for p in (args.render, args.world):
        if not os.path.exists(p):
            raise SystemExit(f"!! 缺文件: {p}")
    os.makedirs(args.out, exist_ok=True)

    boxes, geo, stats = load_boxes()
    if abs(geo["east_inner_x"] - INNER_X) > 1e-6 or abs(geo["north_inner_y"] - INNER_Y) > 1e-6:
        raise SystemExit(f"!! 碰撞几何的墙内侧面 {geo} 与脚本常量不一致，先同步常量")
    print(f"碰撞体: {len(boxes)} 块 box, {stats['cell_m']} m 栅格, "
          f"墙内侧面 x=±{geo['east_inner_x']} y=±{geo['north_inner_y']}")

    # ---- 1) 按几何生成地图
    img = build_map(boxes)
    mh, mw = img.shape
    free = img == 254
    n_free = int(free.sum())
    free_area = n_free * RES * RES
    interior = (2 * INNER_X) * (2 * INNER_Y)
    obst_area_analytic = sum(box_area_in_rect(b, INNER_X, INNER_Y) for b in boxes)
    exp_area = interior - obst_area_analytic
    print(f"地图 {mw}x{mh} @ {RES} m/px  自由区 {free_area:.3f} m²  "
          f"（解析值 {exp_area:.3f} = {interior:.1f} − 障碍 {obst_area_analytic:.3f}）")

    pgm_name = "rmul2027_3v3_up.pgm"
    pgm_path = os.path.join(args.out, pgm_name)
    yaml_path = os.path.join(args.out, "rmul2027_3v3_up.yaml")
    write_pgm(pgm_path, img)
    write_yaml(yaml_path, pgm_name)
    Image.fromarray(np.where(img[::-1] > 0, 254, 0).astype(np.uint8)).save(
        os.path.join(args.out, "rmul2027_3v3_up_preview.png"))

    # 自由区 bbox（像素离散化后）
    ys, xs = np.nonzero(free)
    free_x = (xs + 0.5) * RES + ORIGIN[0]
    free_y = (ys + 0.5) * RES + ORIGIN[1]
    free_rect = (free_x.max() - free_x.min() + RES, free_y.max() - free_y.min() + RES)
    spawn_cell = world_to_cell(*SPAWN)
    spawn_free = bool(spawn_cell and free[spawn_cell])

    # ---- 1b) 落盘后按 map_server 的读图约定回读，核对方向没写反
    #      （2026-11-20 踩过：PGM 上下翻转会让 AMCL 锁到镜像位姿上，整段导航跑飞）
    pgm_read = read_pgm(pgm_path)
    orient_pts = {
        "free_0_0": (0.0, 0.0, 254), "free_4_2": (4.0, 2.0, 254),
        "free_4_-2.5": (4.0, -2.5, 254), "free_-5.5_0": (-5.5, 0.0, 254),
        "obst_-2.5_-2.3": (-2.5, -2.3, 0), "obst_2.5_0.8": (2.5, 0.8, 0),
        "obst_2.5_2.3": (2.5, 2.3, 0), "obst_-2.5_0.5": (-2.5, 0.5, 0),
    }
    orient_meas = {k: occupancy_at(pgm_read, x, y) for k, (x, y, _) in orient_pts.items()}
    orient_bad = [k for k, (x, y, v) in orient_pts.items() if orient_meas[k] != v]

    # ---- 2) 连通性
    reach = flood_fill(free, spawn_cell) if spawn_free else np.zeros_like(free)
    unreached = int((free & ~reach).sum())
    goal_cells = {str(g): world_to_cell(*g) for g in GOALS}
    goals_reachable = {k: bool(rc and reach[rc]) for k, rc in goal_cells.items()}
    print(f"连通性: 出生点 {SPAWN} free={spawn_free}  未连通自由格={unreached}  "
          f"目标可达={goals_reachable}")

    # ---- 3) 渲染图独立校核
    cam = cam_params(args.world)
    rgb = np.asarray(Image.open(args.render).convert("RGB"), dtype=np.float32) / 255.0
    h, w, _ = rgb.shape
    if (w, h) != (cam["w"], cam["h"]):
        raise SystemExit(f"!! 渲染图 {w}x{h} 与 SDF 相机 {cam['w']}x{cam['h']} 不一致")
    print(f"渲染图 {w}x{h}  f={cam['f']:.1f} px  1px={cam['m_per_px']:.4f} m")
    cls = classify(rgb)
    counts = {k: int(cls[k].sum()) for k in ("floor", "outside", "green", "yellow", "red", "other")}
    print("  分类像素:", counts)

    # 3a) 地面（中性灰）的 bbox：正俯视下地面可见边界 = 墙面内侧面
    #     边缘用"该列/行地面像素数 ≥ 最多的 20%"来定，避开墙面接缝处少数中性灰杂散像素
    floor = cls["floor"]
    col_cnt = floor.sum(axis=0)
    row_cnt = floor.sum(axis=1)
    if col_cnt.max() == 0:
        raise SystemExit("!! 渲染图里找不到浅灰地面（光照/颜色变了？）")
    vcol = np.nonzero(col_cnt >= EDGE_COUNT_RATIO * col_cnt.max())[0]
    vrow = np.nonzero(row_cnt >= EDGE_COUNT_RATIO * row_cnt.max())[0]
    u0, u1 = int(vcol.min()), int(vcol.max())
    v0, v1 = int(vrow.min()), int(vrow.max())
    fl = px_to_world(cam, float(u0), float(v1), 0.0)     # 左下
    fr = px_to_world(cam, float(u1), float(v0), 0.0)     # 右上
    floor_w = fr[0] - fl[0] + cam["m_per_px"]
    floor_h = fr[1] - fl[1] + cam["m_per_px"]
    floor_edges = (float(fl[0]), float(fr[0]), float(fl[1]), float(fr[1]))   # x0 x1 y0 y1
    floor_stray = int(floor.sum() - floor[:, u0:u1 + 1][v0:v1 + 1].sum())

    # 3b) 同一块区域（室内矩形缩边，见 RECT_X/RECT_Y）内比较面积与重心
    uu, vv = np.meshgrid(np.arange(w), np.arange(h))
    wx = (uu - w / 2.0) * cam["m_per_px"]
    wy = -(vv - h / 2.0) * cam["m_per_px"]
    mc = np.clip(((wx - ORIGIN[0]) / RES).astype(int), 0, mw - 1)
    mr = np.clip(((wy - ORIGIN[1]) / RES).astype(int), 0, mh - 1)
    map_free_px = free[mr, mc]
    in_rect = (np.abs(wx) < RECT_X) & (np.abs(wy) < RECT_Y)

    cols, rows = cell_cols_rows()
    cell_in_rect = ((np.abs(cols) < RECT_X)[None, :]
                    & (np.abs(rows) < RECT_Y)[:, None])
    map_obst_cells = (~free) & (np.abs(cols) < INNER_X)[None, :] \
                            & (np.abs(rows) < INNER_Y)[:, None]     # 室内障碍 = 平台投影
    obst_area_map = float((map_obst_cells & cell_in_rect).sum()) * RES * RES
    free_area_rect = float((free & cell_in_rect).sum()) * RES * RES

    floor_area_render = float((floor & in_rect).sum()) * cam["m_per_px"] ** 2
    green = cls["green"] & in_rect
    green_area = float(green.sum()) * cam["m_per_px"] ** 2
    gy, gx = np.nonzero(green)
    gw = px_to_world(cam, float(gx.mean()), float(gy.mean()), 0.0) if len(gx) else (0.0, 0.0)
    oy, ox = np.nonzero(map_obst_cells & cell_in_rect)
    oc = (((ox + 0.5) * RES + ORIGIN[0]).mean(), ((oy + 0.5) * RES + ORIGIN[1]).mean())
    centroid_dev = math.hypot(gw[0] - oc[0], gw[1] - oc[1])

    # 3c) 核心区逐像素比较（离自由/障碍分界 ≥ CORE_M）
    er = int(round(CORE_M / cam["m_per_px"]))
    free_core = erode(map_free_px, er)
    obs_core = erode(~map_free_px, er)
    free_core_total = int(free_core.sum())
    obs_core_total = int(obs_core.sum())
    free_core_agree = int((free_core & floor).sum())
    obs_core_agree = int((obs_core & ~floor).sum())
    rate_free = free_core_agree / max(1, free_core_total)
    rate_obs = obs_core_agree / max(1, obs_core_total)

    # 3d) 颜色判据的实测证据（取几处已知世界的点）
    rgb8 = np.asarray(Image.open(args.render).convert("RGB"), dtype=np.float32)

    def sample(x, y, r=2):
        u = int(round(cam["w"] / 2.0 + x / cam["m_per_px"]))
        v = int(round(cam["h"] / 2.0 - y / cam["m_per_px"]))
        return [int(t) for t in
                rgb8[v - r:v + r + 1, u - r:u + r + 1].reshape(-1, 3).mean(axis=0)]

    samples = {"floor_0_3.5": sample(0.0, 3.5), "outside_7.5_0": sample(7.5, 0.0),
               "green_-2.5_-2.3": sample(-2.5, -2.3), "walltop_6.25_0": sample(6.25, 0.0)}

    # ---- 4) 叠加图：渲染图上画自由区边界（墙面内侧面）
    ov = Image.open(args.render).convert("RGB")
    dr = ImageDraw.Draw(ov)
    a = world_to_px(cam, -INNER_X, INNER_Y, 0.0)
    c2 = world_to_px(cam, INNER_X, -INNER_Y, 0.0)
    dr.rectangle([min(a[0], c2[0]), min(a[1], c2[1]),
                  max(a[0], c2[0]), max(a[1], c2[1])], outline=(0, 255, 0), width=2)
    ov.save(args.overlay)

    # ---- 5) 断言
    checks = {
        "map_free_area_m2": {
            "_what": "地图自由区面积 vs 解析值（室内矩形 − 48 块 box 投影）",
            "measured": round(free_area, 3), "expected": round(exp_area, 3),
            "dev": round(abs(free_area - exp_area), 3), "tol": TOL_AREA,
            "ok": abs(free_area - exp_area) <= TOL_AREA,
        },
        "map_free_rect_m": {
            "_what": "地图自由区 bbox → 应 12.00 x 8.00 m（= 墙面内侧面以内）",
            "measured": [round(free_rect[0], 3), round(free_rect[1], 3)],
            "expected": [round(2 * INNER_X, 3), round(2 * INNER_Y, 3)],
            "dev": [round(abs(free_rect[0] - 2 * INNER_X), 3),
                    round(abs(free_rect[1] - 2 * INNER_Y), 3)],
            "tol": RES, "ok": (abs(free_rect[0] - 2 * INNER_X) <= RES
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
        "pgm_orientation": {
            "_what": "落盘 PGM 按 map_server 读图约定回读：已知自由点/平台点必须对得上",
            "measured": orient_meas, "bad_points": orient_bad,
            "ok": len(orient_bad) == 0,
        },
        "spawn_free": {
            "_what": "AMCL 初始位姿 / 出生点在地图里必须是自由格",
            "spawn": list(SPAWN), "cell": list(spawn_cell) if spawn_cell else None,
            "ok": spawn_free,
        },
        "map_connected": {
            "_what": "从出生点泛洪，自由区必须单一连通且到达每个验收目标",
            "unreached_free_cells": unreached, "goals_reachable": goals_reachable,
            "tol_cells": 0,
            "ok": spawn_free and unreached == 0 and all(goals_reachable.values()),
        },
        "render_floor_rect_m": {
            "_what": "渲染中性灰地面 bbox → 应 12.00 x 8.00 m（地面可见边界 = 墙面）",
            "measured": [round(floor_w, 3), round(floor_h, 3)],
            "expected": [round(2 * INNER_X, 3), round(2 * INNER_Y, 3)],
            "dev": [round(abs(floor_w - 2 * INNER_X), 3), round(abs(floor_h - 2 * INNER_Y), 3)],
            "tol": TOL_FIELD, "stray_px": floor_stray,
            "ok": abs(floor_w - 2 * INNER_X) <= TOL_FIELD and abs(floor_h - 2 * INNER_Y) <= TOL_FIELD,
        },
        "render_floor_edges_m": {
            "_what": "渲染地面四条边 → 应正好在 -6.00/+6.00/-4.00/+4.00",
            "measured_xyxy": [round(v, 3) for v in floor_edges],
            "expected_xyxy": [-INNER_X, INNER_X, -INNER_Y, INNER_Y],
            "dev": [round(abs(floor_edges[0] + INNER_X), 3), round(abs(floor_edges[1] - INNER_X), 3),
                    round(abs(floor_edges[2] + INNER_Y), 3), round(abs(floor_edges[3] - INNER_Y), 3)],
            "tol": TOL_FIELD,
            "ok": all(d <= TOL_FIELD for d in
                      (abs(floor_edges[0] + INNER_X), abs(floor_edges[1] - INNER_X),
                       abs(floor_edges[2] + INNER_Y), abs(floor_edges[3] - INNER_Y))),
        },
        "render_free_area_vs_map": {
            "_what": f"室内缩边区域 (|x|<{RECT_X}, |y|<{RECT_Y}) 内：渲染地面面积 vs 地图自由区面积"
                     "（平台顶面 2% 透视放大使渲染地面略小）",
            "render_m2": round(floor_area_render, 3), "map_m2": round(free_area_rect, 3),
            "dev": round(abs(floor_area_render - free_area_rect), 3), "tol": TOL_AREA_RENDER,
            "ok": abs(floor_area_render - free_area_rect) <= TOL_AREA_RENDER,
        },
        "render_green_vs_map_obstacles": {
            "_what": f"室内缩边区域 (|x|<{RECT_X}, |y|<{RECT_Y}) 内：渲染绿色（凸起结构）面积/重心 vs 地图室内障碍",
            "render_m2": round(green_area, 3), "map_m2": round(obst_area_map, 3),
            "area_dev": round(abs(green_area - obst_area_map), 3), "area_tol": TOL_AREA_RENDER,
            "centroid_render": [round(gw[0], 3), round(gw[1], 3)],
            "centroid_map": [round(oc[0], 3), round(oc[1], 3)],
            "centroid_dev_m": round(centroid_dev, 3), "centroid_tol": TOL_CENTROID,
            "ok": (abs(green_area - obst_area_map) <= TOL_AREA_RENDER
                   and centroid_dev <= TOL_CENTROID),
        },
        "pixel_agreement_core": {
            "_what": f"核心区（离分界 ≥{CORE_M} m）逐像素：地图 free/障碍 与渲染 地面/非地面 一致率",
            "free_core_px": free_core_total, "free_agree_px": free_core_agree,
            "obs_core_px": obs_core_total, "obs_agree_px": obs_core_agree,
            "rate_free": round(rate_free, 5), "rate_obs": round(rate_obs, 5),
            "min_rate": CORE_MIN_AGREE,
            "ok": rate_free >= CORE_MIN_AGREE and rate_obs >= CORE_MIN_AGREE,
        },
    }
    ok = all(c.get("ok") for c in checks.values())
    report = {
        "render": os.path.abspath(args.render),
        "camera": {"width": cam["w"], "height": cam["h"], "hfov": cam["hfov"],
                   "focal_px": round(cam["f"], 2), "cam_z": cam["cam_z"],
                   "m_per_px_at_z0": round(cam["m_per_px"], 5)},
        "geometry": {"source": os.path.abspath(BOXES_JSON), "n_boxes": len(boxes),
                     "cell_m": stats["cell_m"], "inner_half": [INNER_X, INNER_Y],
                     "interior_area_m2": round(interior, 3),
                     "obstacle_area_analytic_m2": round(obst_area_analytic, 3)},
        "classify": {"sat_min": SAT_MIN, "floor_max_blueness": FLOOR_MAX_BLUENESS,
                     "pixel_counts": counts, "samples_rgb": samples},
        "map": {"pgm": pgm_path, "yaml": yaml_path, "free_px": n_free,
                "occupied_px": int((img == 0).sum())},
        "checks": checks,
        "summary": {"ok": bool(ok)},
    }
    os.makedirs(os.path.dirname(args.report), exist_ok=True)
    with open(args.report, "w") as f:
        json.dump(report, f, indent=2, ensure_ascii=False, default=bool)

    print("\n===== 校核 =====")
    for k, c in checks.items():
        print(f"  [{'OK ' if c.get('ok') else 'FAIL'}] {k}")
    print(f"\n地图: {pgm_path} ({mw}x{mh} @ {RES} m/px, origin {ORIGIN})")
    print(f"yaml: {yaml_path}")
    print(f"叠加图: {args.overlay}")
    print(f"报告: {args.report}")
    print(f"summary.ok = {ok}")
    return 0 if ok else 3


if __name__ == "__main__":
    raise SystemExit(main())