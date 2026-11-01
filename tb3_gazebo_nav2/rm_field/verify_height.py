#!/usr/bin/env python3
"""验收"台面高度 = 0.503 m 且看得出是一块高台"：

做法不是"看着像"，而是两条独立证据互相对上：
  1) **几何预测**：从 SDF 里读相机位姿/hfov/分辨率，用针孔模型把场地的
     已知三维点（±6.3, ±4.3, z=0 / z=-0.503）投影成像素坐标；
  2) **渲染实测**：在真实渲染图里用列中值亮度剖面找边缘/亮面 bbox。

两边对得上，就证明"渲染出来的墙确实是 0.503 m 高、且比例没被拉伸"。

相机约定（SDFornat：相机光轴 = 局部 +X，图像右 = -Y，图像上 = +Z）：
    xc = (P-C)·X_cam, yc = (P-C)·Y_cam, zc = (P-C)·Z_cam
    u  = W/2 - f*yc/xc,  v = H/2 - f*zc/xc,  f = (W/2)/tan(hfov/2)

用法: python3 verify_height.py [worlds/visual_capture.sdf] [shots_dir] [--json out.json]
"""
import json
import math
import os
import re
import sys

import numpy as np
from PIL import Image
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.abspath(__file__))
FIELD_X, FIELD_Y, FIELD_H = 12.6, 8.6, 0.503

# 每台相机的实测判据：图像文件名 + 允许误差
IMG = {
    "elevation_wide": "height_elevation_wide.png",
    "elevation_wall": "height_elevation_wall.png",
    "oblique_low": "height_oblique_low.png",
}


# ---------------------------------------------------------------- 相机与投影
def load_cameras(path):
    # 世界文件的注释里含 "--"（如 --headless-rendering），严格 XML 解析器会报
    # not well-formed；gz 自己的解析器容忍。先把注释整段剥掉再解析。
    with open(path, "r", encoding="utf-8") as f:
        txt = f.read()
    txt = re.sub(r"<!--.*?-->", "", txt, flags=re.S)
    root = ET.fromstring(txt)
    cams = {}
    for model in root.iter("model"):
        for sensor in model.iter("sensor"):
            if sensor.get("type") != "camera":
                continue
            pose = [float(v) for v in model.find("pose").text.split()]
            cam = sensor.find("camera")
            img = cam.find("image")
            cams[sensor.get("name")] = {
                "pose": pose,
                "hfov": float(cam.find("horizontal_fov").text),
                "w": int(img.find("width").text),
                "h": int(img.find("height").text),
                "topic": sensor.find("topic").text,
            }
    return cams


def rot_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


class Cam:
    def __init__(self, spec):
        x, y, z, roll, pitch, yaw = spec["pose"]
        self.C = np.array([x, y, z], dtype=float)
        R = rot_rpy(roll, pitch, yaw)
        self.X = R[:, 0]          # 光轴
        self.Y = R[:, 1]          # 左
        self.Z = R[:, 2]          # 上
        self.w, self.h = spec["w"], spec["h"]
        self.hfov = spec["hfov"]
        self.f = (self.w / 2.0) / math.tan(self.hfov / 2.0)
        self.spec = spec

    def project(self, P):
        d = np.asarray(P, dtype=float) - self.C
        xc, yc, zc = d @ self.X, d @ self.Y, d @ self.Z
        return self.w / 2.0 - self.f * yc / xc, self.h / 2.0 - self.f * zc / xc, xc

    def m_per_px(self, depth_m):
        """该深度平面上的横向米/像素"""
        return depth_m / self.f


def box_corners(z_top=0.0, z_bot=-FIELD_H):
    pts = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            pts.append((sx * FIELD_X / 2, sy * FIELD_Y / 2, z_top))
            pts.append((sx * FIELD_X / 2, sy * FIELD_Y / 2, z_bot))
    return pts


def proj_bbox(cam, pts):
    us, vs = [], []
    for p in pts:
        u, v, depth = cam.project(p)
        if depth <= 0.05:
            return None
        us.append(u)
        vs.append(v)
    return dict(u0=min(us), u1=max(us), v0=min(vs), v1=max(vs))


# ---------------------------------------------------------------- 图像测量
def load_lum(path):
    a = np.asarray(Image.open(path).convert("RGB")).astype(np.float64)
    return 0.299 * a[:, :, 0] + 0.587 * a[:, :, 1] + 0.114 * a[:, :, 2]


def mask_bbox(lum, thr):
    m = lum > thr
    ys, xs = np.nonzero(m)
    if len(xs) == 0:
        return None
    return dict(u0=int(xs.min()), u1=int(xs.max()),
                v0=int(ys.min()), v1=int(ys.max()),
                coverage=float(m.mean()))


def column_profile(lum, u0, u1):
    band = lum[:, max(0, int(u0)):int(u1)]
    return np.median(band, axis=1)


def find_edge(prof, row_pred, win=30):
    """在预测行附近找最大竖向梯度，返回亚像素行号与该处梯度强度"""
    lo = max(2, int(row_pred) - win)
    hi = min(len(prof) - 2, int(row_pred) + win)
    g = np.diff(prof[lo:hi + 1])
    i = int(np.argmax(np.abs(g)))
    return lo + i + 0.5, float(g[i]), prof[lo:hi + 1]


def column_direction(cam, u):
    """图像列 u 对应的射线方向（未归一化）：X + a*Y + b*Z，a=(cx-u)/f"""
    a = (cam.w / 2.0 - u) / cam.f
    return cam.X + a * cam.Y, a


def wall_rows_at_column(cam, u, y_wall, z_top=0.0, z_bot=-FIELD_H):
    """求"图像第 u 列"视线落在墙面 y=y_wall 上 z=z_top / z=z_bot 的行号。

    射线 C + s*(X + a*Y + b*Z)，未知 s、b；对 y、z 两个约束解 2x2 线性方程。
    这样得到的是该列真正看得见的墙面上下沿，而不是随便取某个世界点。
    """
    base, a = column_direction(cam, u)
    A = np.array([[base[1], cam.Z[1]],
                  [base[2], cam.Z[2]]])
    out = []
    for z_t in (z_top, z_bot):
        rhs = np.array([y_wall - cam.C[1], z_t - cam.C[2]])
        s, m = np.linalg.solve(A, rhs)
        b = m / s
        out.append(cam.h / 2.0 - cam.f * b)
    return out[0], out[1]


# ---------------------------------------------------------------- 各相机验收
def check_elevation(cam, lum, tag, thr, tol_edge, tol_h):
    """正立面：亮带 = 前立面（同一深度平面），用像素量高度"""
    res = {}
    pred_top = cam.project((-3.0, -FIELD_Y / 2, 0.0))
    pred_bot = cam.project((3.0, -FIELD_Y / 2, -FIELD_H))
    res["pred_top_row"] = pred_top[1]
    res["pred_bot_row"] = pred_bot[1]
    res["pred_height_px"] = pred_bot[1] - pred_top[1]
    res["depth_m"] = pred_top[2]
    res["m_per_px"] = cam.m_per_px(pred_top[2])

    prof = column_profile(lum, cam.w * 0.35, cam.w * 0.65)
    res["profile_median"] = float(np.median(prof))
    top_row, top_grad, _ = find_edge(prof, pred_top[1])
    bot_row, bot_grad, _ = find_edge(prof, pred_bot[1])
    res["meas_top_row"], res["meas_bot_row"] = top_row, bot_row
    res["meas_top_grad"], res["meas_bot_grad"] = top_grad, bot_grad
    res["meas_height_px"] = bot_row - top_row
    res["edge_err_px"] = {"top": abs(top_row - pred_top[1]),
                          "bottom": abs(bot_row - pred_bot[1])}

    h_from_f = res["meas_height_px"] * res["m_per_px"]
    res["height_m_from_focal"] = h_from_f

    bbox = mask_bbox(lum, thr)
    res["bright_bbox"] = bbox
    res["mask_thr"] = thr
    checks = []
    checks.append(("顶边位置", res["edge_err_px"]["top"] <= tol_edge,
                   f'{res["edge_err_px"]["top"]:.1f} px'))
    checks.append(("底边位置", res["edge_err_px"]["bottom"] <= tol_edge,
                   f'{res["edge_err_px"]["bottom"]:.1f} px'))
    checks.append(("墙高(焦距换算)", abs(h_from_f - FIELD_H) / FIELD_H <= tol_h,
                   f'{h_from_f * 1000:.1f} mm vs 503.0 mm'))
    res["checks"] = [dict(name=n, ok=bool(ok), detail=d) for n, ok, d in checks]
    return res


def verify(world, shots, out_json):
    cams = load_cameras(world)
    report = {"world": world, "field": {"x": FIELD_X, "y": FIELD_Y, "h": FIELD_H},
              "cameras": {}, "summary": {"ok": True, "failed": []}}

    def add(tag, res):
        report["cameras"][tag] = res
        bad = [c["name"] for c in res.get("checks", []) if not c["ok"]]
        report["summary"]["ok"] = report["summary"]["ok"] and not bad
        report["summary"]["failed"].extend(f"{tag}:{b}" for b in bad)

    # ---------- 1) 正立面（全景 1920x1080，亮带完整宽度） ----------
    tag = "elevation_wide"
    cam = Cam(cams[tag])
    path = os.path.join(shots, IMG[tag])
    if os.path.exists(path):
        lum = load_lum(path)
        res = check_elevation(cam, lum, tag, thr=115, tol_edge=4, tol_h=0.02)
        pred_w = cam.project((-FIELD_X / 2, -FIELD_Y / 2, 0))[0], \
            cam.project((FIELD_X / 2, -FIELD_Y / 2, 0))[0]
        res["pred_width_px"] = abs(pred_w[1] - pred_w[0])
        bb = res["bright_bbox"]
        if bb:
            w_px = bb["u1"] - bb["u0"] + 1
            h_px = bb["v1"] - bb["v0"] + 1
            res["meas_width_px"] = w_px
            res["meas_height_px_from_bbox"] = h_px
            res["aspect_meas"] = w_px / h_px
            res["aspect_drawing"] = FIELD_X / FIELD_H
            res["height_m_from_width_ref"] = FIELD_X * h_px / w_px
            res["aspect_err"] = abs(h_px / w_px - FIELD_H / FIELD_X) / (FIELD_H / FIELD_X)
            res["checks"].append(dict(name="亮带宽度 ≈ 12.6 m",
                                      ok=abs(w_px - res["pred_width_px"]) <= 6,
                                      detail=f'{w_px} px vs 预测 {res["pred_width_px"]:.1f} px'))
            res["checks"].append(dict(name="宽高比 = 12.6/0.503（不依赖焦距）",
                                      ok=res["aspect_err"] <= 0.03,
                                      detail=f'{res["aspect_meas"]:.2f} vs {res["aspect_drawing"]:.2f}'))
            res["checks"].append(dict(name="墙高(用宽度做标尺)",
                                      ok=abs(res["height_m_from_width_ref"] - FIELD_H) / FIELD_H <= 0.03,
                                      detail=f'{res["height_m_from_width_ref"]*1000:.1f} mm'))
        res["checks"] = [dict(name=c["name"], ok=bool(c["ok"]), detail=c["detail"])
                         for c in res["checks"]]
        add(tag, res)
    else:
        add(tag, {"error": f"缺图 {path}", "checks": [dict(name="出图", ok=False, detail=path)]})

    # ---------- 2) 墙面近景（1280x720） ----------
    tag = "elevation_wall"
    cam = Cam(cams[tag])
    path = os.path.join(shots, IMG[tag])
    if os.path.exists(path):
        lum = load_lum(path)
        res = check_elevation(cam, lum, tag, thr=115, tol_edge=4, tol_h=0.02)
        res["wall_px_of_frame"] = res["meas_height_px"] / cam.h
        add(tag, res)
    else:
        add(tag, {"error": f"缺图 {path}", "checks": [dict(name="出图", ok=False, detail=path)]})

    # ---------- 3) 低机位斜视（整场 + 侧壁带） ----------
    tag = "oblique_low"
    cam = Cam(cams[tag])
    path = os.path.join(shots, IMG[tag])
    if os.path.exists(path):
        lum = load_lum(path)
        res = {"cam_pose": cam.spec["pose"], "w": cam.w, "h": cam.h}
        solid = proj_bbox(cam, box_corners(0.0, -FIELD_H))          # 有 0.503 m 厚度的实体
        flat = proj_bbox(cam, box_corners(0.0, 0.0))                # 反事实：一张零厚度平板
        res["pred_solid_bbox"] = solid
        res["pred_flat_bbox"] = flat
        res["pred_wall_band_px"] = solid["v1"] - flat["v1"] if solid and flat else None
        bb = mask_bbox(lum, thr=170)
        res["bright_bbox"] = bb
        res["mask_thr"] = 150
        checks = []
        if bb and solid and flat:
            res["d_bbox_vs_flat"] = {k: bb[k] - flat[k] for k in ("u0", "u1", "v0", "v1")}
            res["d_bbox_vs_solid"] = {k: bb[k] - solid[k] for k in ("u0", "u1", "v0", "v1")}
            # 亮面 = 台面顶面(z=0)：应与"零厚度平板"预测吻合，
            # 底边 = 顶面近边，实体再往下一整条墙带 → 证明墙是实体、不是画出来的平面
            checks.append(("亮面 bbox ≈ 台面顶面投影（±10 px）",
                           all(abs(res["d_bbox_vs_flat"][k]) <= 10 for k in ("u0", "u1", "v0", "v1")),
                           str(res["d_bbox_vs_flat"])))
            checks.append(("预测墙带厚度 > 5 px（不是零厚度）",
                           res["pred_wall_band_px"] > 5,
                           f'{res["pred_wall_band_px"]:.1f} px'))
        # 侧壁带实测：取图像中间列，视线打在 -Y 前立面上，
        # 预测"台面->侧壁"和"侧壁->地面"两条边，再到渲染剖面里找梯度核对
        prof = column_profile(lum, cam.w * 0.45, cam.w * 0.55)
        wall_top_row, wall_bot_row = wall_rows_at_column(cam, cam.w / 2.0, -FIELD_Y / 2)
        top_row, top_g, _ = find_edge(prof, wall_top_row, win=25)
        bot_row, bot_g, _ = find_edge(prof, wall_bot_row, win=25)
        res["wall_band_meas"] = {
            "pred_top_row": wall_top_row, "meas_top_row": top_row, "grad_top": top_g,
            "pred_bot_row": wall_bot_row, "meas_bot_row": bot_row, "grad_bot": bot_g,
            "meas_band_px": bot_row - top_row,
            "pred_band_px": wall_bot_row - wall_top_row,
        }
        checks.append(("侧壁带厚度与预测一致（±8 px）",
                       abs(res["wall_band_meas"]["meas_band_px"]
                           - res["wall_band_meas"]["pred_band_px"]) <= 8,
                       f'{res["wall_band_meas"]["meas_band_px"]:.1f} vs '
                       f'{res["wall_band_meas"]["pred_band_px"]:.1f} px'))
        res["checks"] = [dict(name=n, ok=bool(ok), detail=d) for n, ok, d in checks]
        add(tag, res)
    else:
        add(tag, {"error": f"缺图 {path}", "checks": [dict(name="出图", ok=False, detail=path)]})

    report["summary"]["ok"] = all(c["ok"] for cam in report["cameras"].values()
                                  for c in cam.get("checks", []))
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)

    # ---------------- 控制台摘要 ----------------
    print("=" * 78)
    print(f"台面高度验收（世界: {os.path.basename(world)}）  图纸: "
          f"{FIELD_X} x {FIELD_Y} x {FIELD_H} m")
    print("=" * 78)
    for tag, res in report["cameras"].items():
        print(f"\n[{tag}]")
        if "error" in res:
            print("  !!", res["error"])
            continue
        print(f"  相机位姿 {[round(v,4) for v in res.get('cam_pose', cams[tag]['pose'])]}"
              f"  hfov={cams[tag]['hfov']:.2f}  {cams[tag]['w']}x{cams[tag]['h']}")
        for k in ("pred_top_row", "meas_top_row", "pred_bot_row", "meas_bot_row",
                  "pred_height_px", "meas_height_px", "m_per_px",
                  "height_m_from_focal", "height_m_from_width_ref", "aspect_meas",
                  "aspect_drawing", "pred_width_px", "meas_width_px",
                  "bright_bbox", "pred_solid_bbox", "pred_flat_bbox",
                  "pred_wall_band_px", "wall_band_meas", "wall_px_of_frame"):
            if k in res:
                v = res[k]
                if isinstance(v, float):
                    print(f"  {k:26s} {v:10.3f}")
                else:
                    print(f"  {k:26s} {v}")
        for c in res["checks"]:
            print(f"  [{'OK ' if c['ok'] else 'FAIL'}] {c['name']}: {c['detail']}")
    print("\n结论:", "全部通过" if report["summary"]["ok"]
          else f"未通过 -> {report['summary']['failed']}")
    print(f"报告写入: {out_json}")
    return 0 if report["summary"]["ok"] else 1


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    world = args[0] if args else os.path.join(ROOT, "worlds/visual_capture.sdf")
    shots = args[1] if len(args) > 1 else os.path.join(ROOT, "shots")
    out = os.path.join(ROOT, "results/report_height.json")
    if "--json" in sys.argv:
        out = sys.argv[sys.argv.index("--json") + 1]
    os.makedirs(os.path.dirname(out), exist_ok=True)
    sys.exit(verify(world, shots, out))