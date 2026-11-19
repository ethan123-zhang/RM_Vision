#!/usr/bin/env python3
"""把翻转后场地的**碰撞**从 113k 面三角网格换成 box 拼块。

背景（2026-11-19）：`structures.stl` 当碰撞体时机器人被垫高 6 mm、轮子空转车不走。
根因是 dartsim 三角网格接触；box 接触稳（地面 box 一直很正常）。

做法：
  1. 把 structures.stl 光栅化成 0.05 m 高度图（水平面用"重心坐标判格心在三角形内"取值，
     竖直面用包围盒覆盖、取三角形最大 z —— 偏保守，但不会漏掉薄墙）；
  2. 按高度聚类成若干层（同一层内高度差 < 2 cm），每层用一个 box 从 z=-0.10 顶到该层高度；
  3. 每层做"行内游程 + 同游程纵向合并"压成矩形，矩形即一个 box；
  4. 写回 model.sdf 的 BOX_COLLISION_BEGIN/END 之间，并输出 debug/box_collisions.json
     （含 box 列表、层高、覆盖率-误差统计、墙内侧面、候选平台）。

用法（WSL）：
    python3 debug/make_box_collisions.py            # 生成并写回 model.sdf
    python3 debug/make_box_collisions.py --dry-run  # 只打印统计，不改 model.sdf
"""
import argparse
import json
import math
import os
import struct

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
MODEL = os.path.join(ROOT, "models", "rmul2027_3v3_field_up")
SRC = os.path.join(MODEL, "meshes", "structures.stl")
MODEL_SDF = os.path.join(MODEL, "model.sdf")
OUT_JSON = os.path.join(HERE, "box_collisions.json")

CELL = 0.05            # 栅格边长（m）
BOTTOM = -0.10         # box 底面（与台体底板同高，顶住地面 box 不悬空）
MIN_LEVEL_AREA = 0.50  # 高度直方图里面积小于此值的层并进相邻层（m^2）
MERGE_LEVEL_TOL = 0.02 # 层高差小于此值的峰合并（m）
TOP_MIN = 0.02         # 高于地面 2 cm 才认为"有凸起"

DT = np.dtype([("n", "<f4", (3,)), ("v", "<f4", (3, 3)), ("attr", "<u2")])


def read_stl(path):
    d = open(path, "rb").read()
    n = struct.unpack("<I", d[80:84])[0]
    body = d[84:84 + n * 50]
    if len(body) != n * 50:
        raise SystemExit("STL 长度不对: %s" % path)
    return np.frombuffer(body, dtype=DT).copy()


def rasterize(v, cell=CELL):
    """返回 (H 高度图 ny×nx, gx0, gy0)。H 为每格顶面最高 z。"""
    gx0 = math.floor(float(v[:, :, 0].min()) / cell + 1e-6) * cell
    gy0 = math.floor(float(v[:, :, 1].min()) / cell + 1e-6) * cell
    nx = int(math.ceil((float(v[:, :, 0].max()) - gx0) / cell))
    ny = int(math.ceil((float(v[:, :, 1].max()) - gy0) / cell))
    H = np.full((ny, nx), -np.inf, dtype=np.float32)

    p0, p1, p2 = v[:, 0], v[:, 1], v[:, 2]
    nrm = np.cross(p1 - p0, p2 - p0)
    ln = np.linalg.norm(nrm, axis=1)
    ln[ln == 0] = 1.0
    nz = nrm[:, 2] / ln

    xmn, xmx = v[:, :, 0].min(1), v[:, :, 0].max(1)
    ymn, ymx = v[:, :, 1].min(1), v[:, :, 1].max(1)
    zx = v[:, :, 2].max(1)

    i0 = np.floor((xmn - gx0) / cell).astype(int)
    i1 = np.ceil((xmx - gx0) / cell).astype(int) - 1
    j0 = np.floor((ymn - gy0) / cell).astype(int)
    j1 = np.ceil((ymx - gy0) / cell).astype(int) - 1
    i0 = np.clip(i0, 0, nx - 1); i1 = np.clip(i1, 0, nx - 1)
    j0 = np.clip(j0, 0, ny - 1); j1 = np.clip(j1, 0, ny - 1)

    for k in range(len(v)):
        a, b = j0[k], j1[k]
        c, d = i0[k], i1[k]
        if b < a or d < c:          # 投影为一条线（退化）时至少占一格
            continue
        if abs(nz[k]) > 0.5:        # 近水平面：按平面求 z，只填中心落在三角形内的格子
            cx = gx0 + (np.arange(c, d + 1) + 0.5) * cell
            cy = gy0 + (np.arange(a, b + 1) + 0.5) * cell
            X, Y = np.meshgrid(cx, cy)
            x0, y0, z0 = p0[k]
            x1, y1, _ = p1[k]
            x2, y2, _ = p2[k]
            area = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
            if abs(area) < 1e-12:
                continue
            s = 1.0 if area > 0 else -1.0
            e0 = (x1 - x0) * (Y - y0) - (y1 - y0) * (X - x0)
            e1 = (x2 - x1) * (Y - y1) - (y2 - y1) * (X - x1)
            e2 = (x0 - x2) * (Y - y2) - (y0 - y2) * (X - x2)
            inside = ((s * e0) >= -1e-9) & ((s * e1) >= -1e-9) & ((s * e2) >= -1e-9)
            if not inside.any():
                continue
            zz = z0 - (nrm[k, 0] * (X - x0) + nrm[k, 1] * (Y - y0)) / nrm[k, 2]
            block = H[a:b + 1, c:d + 1]
            np.maximum(block, np.where(inside, zz, -np.inf), out=block)
        else:                       # 近竖直面：包围盒覆盖，取最大 z（保守）
            block = H[a:b + 1, c:d + 1]
            np.maximum(block, zx[k], out=block)
    H[~np.isfinite(H)] = 0.0
    return H, gx0, gy0


def cluster_levels(H, cell=CELL):
    occ = H > TOP_MIN
    vals = np.round(H[occ], 2)
    uniq, cnt = np.unique(vals, return_counts=True)
    area = cnt * cell * cell
    kept = [(float(u), float(a)) for u, a in zip(uniq, area) if a >= MIN_LEVEL_AREA]
    levels = []
    for u, _a in kept:
        if levels and abs(u - levels[-1]) <= MERGE_LEVEL_TOL:
            continue
        levels.append(u)
    return occ, levels


def rectangles(mask):
    """把布尔掩码压成矩形列表 [(c0, c1, r0, r1)]，c1/r1 为开区间。"""
    ny, nx = mask.shape
    rects = []
    active = {}                      # (c0, c1) -> r0
    for r in range(ny):
        row = mask[r].astype(np.int8)
        d = np.diff(np.r_[0, row, 0])
        starts = np.flatnonzero(d == 1)
        ends = np.flatnonzero(d == -1)
        cur = {}
        for s, e in zip(starts, ends):
            key = (int(s), int(e))
            if key in active:
                cur[key] = active[key]
            else:
                cur[key] = r
        for key, r0 in active.items():
            if key not in cur:
                rects.append((key[0], key[1], r0, r))
        active = cur
    for key, r0 in active.items():
        rects.append((key[0], key[1], r0, ny))
    return rects


def build_boxes(H, levels, gx0, gy0, cell=CELL):
    ny, nx = H.shape
    occ = H > TOP_MIN
    # 每格归到最近层
    tops = np.zeros_like(H, dtype=np.float32)
    assigned = np.zeros_like(H, dtype=np.int16)
    for li, lv in enumerate(levels):
        mask = occ & (assigned == 0) & (np.abs(H - lv) <= 0.03)
        tops[mask] = lv
        assigned[mask] = li + 1
    # 剩下的（层之间）归最近的层
    rest = occ & (assigned == 0)
    if rest.any():
        lv_arr = np.asarray(levels, dtype=np.float32)
        for (r, c) in zip(*np.nonzero(rest)):
            li = int(np.argmin(np.abs(lv_arr - H[r, c])))
            tops[r, c] = lv_arr[li]
            assigned[r, c] = li + 1

    err = tops[occ] - H[occ]
    boxes = []
    seq = 0
    for li, lv in enumerate(levels):
        m = assigned == (li + 1)
        for (c0, c1, r0, r1) in rectangles(m):
            sx = (c1 - c0) * cell
            sy = (r1 - r0) * cell
            cx = gx0 + (c0 + c1) * 0.5 * cell
            cy = gy0 + (r0 + r1) * 0.5 * cell
            sz = float(lv) - BOTTOM
            cz = (float(lv) + BOTTOM) * 0.5
            seq += 1
            boxes.append({
                "name": "box_%03d" % seq,
                "level": round(float(lv), 4),
                "center": [round(cx, 4), round(cy, 4), round(cz, 4)],
                "size": [round(sx, 4), round(sy, 4), round(sz, 4)],
                "top": round(float(lv), 4),
                "area": round(sx * sy, 4),
            })
    stats = {
        "cell_m": cell,
        "grid": [int(nx), int(ny)],
        "origin": [round(gx0, 4), round(gy0, 4)],
        "occupied_cells": int(occ.sum()),
        "occupied_area_m2": round(float(occ.sum()) * cell * cell, 3),
        "levels": levels,
        "n_boxes": len(boxes),
        "height_err_m": {
            "max_abs": round(float(np.abs(err).max()), 4) if len(err) else 0.0,
            "rms": round(float(np.sqrt((err ** 2).mean())), 4) if len(err) else 0.0,
        },
    }
    return boxes, stats


def derive_test_geometry(boxes, levels, H, gx0, gy0, cell=CELL):
    """给验收脚本用：东/北墙内侧面、候选平台"南侧逼近面"（含车道净空检查）。"""
    tall = max(levels)
    walls = [b for b in boxes if b["top"] >= tall - 0.01]

    def covers_y(b, y, pad=0.0):
        return b["center"][1] - b["size"][1] / 2 - pad <= y <= b["center"][1] + b["size"][1] / 2 + pad

    def covers_x(b, x, pad=0.0):
        return b["center"][0] - b["size"][0] / 2 - pad <= x <= b["center"][0] + b["size"][0] / 2 + pad

    east = [b["center"][0] - b["size"][0] / 2 for b in walls if covers_y(b, 0.0) and b["center"][0] > 5.0]
    west = [b["center"][0] + b["size"][0] / 2 for b in walls if covers_y(b, 0.0) and b["center"][0] < -5.0]
    north = [b["center"][1] - b["size"][1] / 2 for b in walls if covers_x(b, 0.0) and b["center"][1] > 3.0]
    south = [b["center"][1] + b["size"][1] / 2 for b in walls if covers_x(b, 0.0) and b["center"][1] < -3.0]

    out = {
        "wall_top": round(float(tall), 4),
        "east_inner_x": round(min(east), 4) if east else None,
        "west_inner_x": round(max(west), 4) if west else None,
        "north_inner_y": round(min(north), 4) if north else None,
        "south_inner_y": round(max(south), 4) if south else None,
    }

    # 平台：取面积最大的 0.15~0.30 m 层盒子，检查从南侧 1.2 m 外逼近的车道是否干净
    def floor_clear(xc, y0, y1, half_width=0.25):
        j0 = int(round((y0 - gy0) / cell))
        j1 = int(round((y1 - gy0) / cell))
        i0 = int(round((xc - half_width - gx0) / cell))
        i1 = int(round((xc + half_width - gx0) / cell))
        patch = H[max(0, j0):j1, max(0, i0):i1 + 1]
        return bool(patch.size) and float(patch.max()) <= TOP_MIN

    inner_levels = [lv for lv in levels if TOP_MIN < lv < tall - 0.05]
    plats = [b for b in boxes if any(abs(b["top"] - lv) < 1e-3 for lv in inner_levels)]
    plats.sort(key=lambda b: -b["area"])
    for b in plats:
        cx = b["center"][0]
        face = b["center"][1] - b["size"][1] / 2
        spawn_y = face - 1.2
        if floor_clear(cx, spawn_y, face - 0.05):
            out["platform"] = {
                "level": b["top"], "lane_x": round(cx, 4),
                "south_face_y": round(face, 4),
                "spawn_y": round(spawn_y, 4),
                "spawn_x": round(cx, 4),
                "box": b,
            }
            break
    return out


SDF_TMPL = """\
      <collision name="{name}">
        <pose>{cx} {cy} {cz} 0 0 0</pose>
        <geometry>
          <box>
            <size>{sx} {sy} {sz}</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.9</mu>
              <mu2>0.9</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
"""


def render_sdf(boxes):
    return "".join(
        SDF_TMPL.format(name=b["name"], cx=b["center"][0], cy=b["center"][1],
                        cz=b["center"][2], sx=b["size"][0], sy=b["size"][1],
                        sz=b["size"][2])
        for b in boxes)


def patch_model_sdf(boxes):
    txt = open(MODEL_SDF, encoding="utf-8").read()
    beg = "<!-- BOX_COLLISION_BEGIN"
    end = "<!-- BOX_COLLISION_END -->"
    i = txt.find(beg)
    j = txt.find(end)
    if i < 0 or j < 0 or j < i:
        raise SystemExit("model.sdf 里找不到 BOX_COLLISION 标记，先补标记")
    i = txt.find("\n", i) + 1          # 保留 BEGIN 行
    new = txt[:i] + render_sdf(boxes) + txt[j:]
    with open(MODEL_SDF, "w", encoding="utf-8") as f:
        f.write(new)


def ascii_map(H, step=0.2):
    """粗看俯视高度（每格代表 step m，取最大值）。"""
    k = max(1, int(round(step / CELL)))
    ny, nx = H.shape
    hh = H[:ny - ny % k, :nx - nx % k].reshape(ny // k, k, nx // k, k).max(axis=(1, 3))
    chars = " .:-=+*#%@"
    lines = []
    for row in hh[::-1]:
        s = "".join(chars[min(9, int(max(0.0, z) / 0.045))] for z in row)
        lines.append(s)
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--cell", type=float, default=CELL)
    ap.add_argument("--ascii", action="store_true", help="打印 0.2 m 粗俯视高度图")
    args = ap.parse_args()

    t = read_stl(SRC)
    print("structures.stl: %d 面" % len(t))
    H, gx0, gy0 = rasterize(t["v"], args.cell)

    occ, levels = cluster_levels(H, args.cell)
    print("高层聚类:", levels)
    boxes, stats = build_boxes(H, levels, gx0, gy0, args.cell)
    stats["test_geometry"] = derive_test_geometry(boxes, levels, H, gx0, gy0, args.cell)
    print("box 数: %d  | 占据面积 %.2f m2 | 高度误差 max %.3f / rms %.3f m"
          % (stats["n_boxes"], stats["occupied_area_m2"],
             stats["height_err_m"]["max_abs"], stats["height_err_m"]["rms"]))
    tg = stats["test_geometry"]
    print("墙内侧面: east=%s west=%s north=%s south=%s"
          % (tg.get("east_inner_x"), tg.get("west_inner_x"),
             tg.get("north_inner_y"), tg.get("south_inner_y")))
    if tg.get("platform"):
        p = tg["platform"]
        print("候选平台: top=%.3f m  车道 x=%.2f  y %.2f -> 南侧面 y=%.3f"
              % (p["level"], p["lane_x"], p["spawn_y"], p["south_face_y"]))
    print("原点(0,0) 所在格高度: %.3f m" % float(H[int((0 - gy0) / args.cell),
                                                    int((0 - gx0) / args.cell)]))
    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump({"stats": stats, "boxes": boxes}, f, indent=1, ensure_ascii=False)
    print("写:", OUT_JSON)
    if args.ascii:
        print(ascii_map(H))
    if args.dry_run:
        print("--dry-run：不改 model.sdf")
        return 0
    patch_model_sdf(boxes)
    print("已写回:", MODEL_SDF)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())