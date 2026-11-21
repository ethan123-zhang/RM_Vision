#!/usr/bin/env python3
"""窄通道（贴边）验收：0.80 m 门洞（车宽 0.44 m）能不能过去、贴得有多近、有没有碰，
以及**过门全程 AMCL 有没有被未建图障碍物带偏**。

两档场景（几何都在 config/avoid_obstacles.json，由 make_obstacles_world.py 生成世界）：
  --scenario gate（默认）     两块 0.6×0.5×0.6 m，窄段深 0.5 m；`bash nav2_gap.sh`
  --scenario gate_deep        两块 1.0×1.2×1.0 m，窄段深 1.2 m（GUI 默认箱量级）；
                              `bash nav2_gap_deep.sh`。这是 AMCL 漂移的回归档：
                              修复前实测出门后定位漂 2.9 m、nav2 ABORTED。
  --scenario gate_tight       0.62 m 超窄门（<0.70 m 档，车宽 0.44）：`bash nav2_gap_tight.sh`。
                              --grid 25mm（默认 2.5 cm 碰撞栅格变体）或 5cm（基线对照）；
                              --tag 用来区分同场景的两次跑（报告名带后缀）。

判据（gate 5 项 + deep 同 5 项，另有公共第 6 项）：
  1. 导航成功、到位 <0.30 m；
  2. 真的穿门：真值轨迹在 y=门线±0.10 的样本里 |x| ≤ 门洞半宽 + 2 cm；
  3. 贴边安全：全程到两块门箱 AABB 的最小间距 ≥ 0.20 m；
  4. 不出界/不掉台/不爬台（同基类判据）；
  5. 定位一致：TF(map->base_footprint) 与 Gazebo 真值全程 max < 0.50 m；
     并单独给"门线 ±0.90 m 的穿门段"的 max，防止全局最大值被别处稀释。
输出 results/report_narrow_gap[_deep|_tight][_tag].json；全过返回 0，否则 3。
"""
import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("ROS_DOMAIN_ID", "42")

from verify_nav2_field import (                                    # noqa: E402
    RES, Verifier, INNER_X, INNER_Y, MARGIN, Z_MIN, Z_MAX, GOAL_TOL)
from verify_nav2_obstacles import aabb_gap                         # noqa: E402

import rclpy                                                       # noqa: E402

CFG = os.path.join(HERE, "config", "avoid_obstacles.json")
GATE_CLEAR_MIN = 0.20        # 车体最大半宽约 0.19 m + 1 cm 余量
GATE_HALF_WIN = 0.10         # 判定"在门线上"的 y 窗口
LOC_GATE_HALF_WIN = 0.90     # 穿门段定位判据的 y 窗口（从门口到门后 0.9 m）
LOC_ERR_MAX = 0.50           # 与 verify_nav2_field 同口径：TF vs 真值


def load_cfg():
    with open(CFG, "r", encoding="utf-8") as f:
        return json.load(f)


def loc_errors(v):
    """TF(map->base_footprint) vs Gazebo 真值，按 wall 时间取最近真值帧。

    返回 [(wall_t, err_m, truth_x, truth_y)]。与 verify_nav2_field 同算法。
    """
    out = []
    gt = v.gz_track
    if not gt:
        return out
    gi = 0
    for wall_t, _sim_t, x, y, _yaw in v.track:
        while gi + 1 < len(gt) and gt[gi + 1][0] <= wall_t:
            gi += 1
        g = gt[gi]
        if abs(g[0] - wall_t) <= 0.5:
            out.append((wall_t, math.hypot(x - g[1], y - g[2]), g[1], g[2]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", choices=["gate", "gate_deep", "gate_tight"], default="gate")
    ap.add_argument("--grid", choices=["5cm", "25mm"], default="5cm",
                    help="这次跑用的是哪套碰撞栅格（只记录进报告；世界/地图由起栈脚本决定）")
    ap.add_argument("--tag", default="",
                    help="报告名后缀（例如同一 gate_tight 分别跑 5cm/25mm 时用 --tag 5cm）")
    args = ap.parse_args()
    key = args.scenario
    gate = load_cfg()[key]
    goal_xy = gate["goal"]
    west, east = gate["boxes"]
    gate_y = west["center"][1]
    gap = ((east["center"][0] - east["size"][0] / 2.0)
           - (west["center"][0] + west["size"][0] / 2.0))
    # 门洞半宽 + 2 cm：center 越过半个门洞就算没"真的从门里过"；贴边接触由
    # gate_clearance（点-箱 AABB ≥0.20）兜底。
    gate_half_limit = gap / 2.0 + 0.02
    rclpy.init()
    v = Verifier()
    report = {"scenario": {"name": key, "gate_gap_m": round(gap, 3), "goal": goal_xy,
                           "spawn": gate.get("spawn"), "gate_clear_min_m": GATE_CLEAR_MIN,
                           "gate_depth_m": west["size"][1],
                           "field_grid": args.grid,
                           "grid_quantization_m": 0.0125 if args.grid == "25mm" else 0.025,
                           "tag": args.tag,
                           "boxes": gate["boxes"]},
              "ros_domain_id": os.environ.get("ROS_DOMAIN_ID")}
    print(f"[0] scenario={key} grid={args.grid} ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID')} "
          f"门洞 {gap:.2f} m 箱深 {west['size'][1]:.2f} m goal={goal_xy}")

    print("[1] 等 action server / sim clock / TF ...")
    if not v.client.wait_for_server(timeout_sec=90.0):
        report["error"] = "no action server"
        _save(report, key, args.tag)
        return 2
    report["sim_time_ok"] = v.wait_sim_time(30.0)
    report["tf_ready"] = v.spin_until(v.tf_ready, 30.0)
    v.spin_for(1.0)
    print(f"    sim_time={report['sim_time_ok']} tf={report['tf_ready']}")

    print("[2] 发目标并穿门 ...")
    r = v.goto(goal_xy[0], goal_xy[1], 0.0)
    report["nav_result"] = r
    print("    ->", {k: r.get(k) for k in
                    ("succeeded", "error_code", "path_len_m", "end_dist_to_goal_m")})
    v.spin_for(0.5)

    tr = v.track
    gztr = v.gz_track
    # 穿门：真值里 y 落在门线上的样本，|x| 必须都在门洞内
    on_gate = [p for p in gztr if abs(p[2] - gate_y) <= GATE_HALF_WIN]
    crossed = len(on_gate) > 0
    max_abs_x_on_gate = max((abs(p[1]) for p in on_gate), default=None)
    # 全程到两块门箱的最小间距（真值）
    gaps = {}
    worst = None
    for ob in gate["boxes"]:
        cx, cy = ob["center"]
        sx, sy = ob["size"][0], ob["size"][1]
        mn = 1e9
        for p in gztr:
            g = aabb_gap(p[1], p[2], cx, cy, sx, sy)
            if g < mn:
                mn = g
                if worst is None or g < worst["gap_m"]:
                    worst = {"wall_t": round(p[0], 2), "robot_xy": [round(p[1], 3), round(p[2], 3)],
                             "box": ob["name"], "gap_m": round(g, 3)}
        gaps[ob["name"]] = round(mn, 3) if mn < 1e9 else None
    min_gap = min((g for g in gaps.values() if g is not None), default=None)
    zs = [p[2] for p in v.gz.values()] or [0.0]
    max_x = max((abs(p[2]) for p in tr), default=0.0)
    max_y = max((abs(p[3]) for p in tr), default=0.0)

    # 定位一致：全局 + 穿门段
    loc = loc_errors(v)
    loc_err = [e for _, e, _, _ in loc]
    loc_gate = [e for _, e, _, ty in loc if abs(ty - gate_y) <= LOC_GATE_HALF_WIN]
    loc_max = max(loc_err) if loc_err else None
    loc_mean = (sum(loc_err) / len(loc_err)) if loc_err else None
    loc_gate_max = max(loc_gate) if loc_gate else None
    report["loc_err_m"] = {"max": round(loc_max, 3) if loc_max is not None else None,
                           "mean": round(loc_mean, 3) if loc_mean is not None else None,
                           "gate_segment_max": round(loc_gate_max, 3)
                           if loc_gate_max is not None else None,
                           "samples": len(loc_err), "gate_samples": len(loc_gate),
                           "gate_half_win_m": LOC_GATE_HALF_WIN}
    print(f"    定位误差: max {loc_max:.3f} m / mean {loc_mean:.3f} m（门段 max "
          f"{loc_gate_max if loc_gate_max is None else round(loc_gate_max, 3)} m）"
          if loc_err else "    定位误差: 样本不足")

    checks = {
        "goals_succeeded": {"ok": bool(r.get("succeeded")) and r.get("error_code") == 0,
                            "succeeded": r.get("succeeded"),
                            "error_code": r.get("error_code"), "error": r.get("error")},
        "reached_goal": {"ok": (r.get("end_dist_to_goal_m") is not None
                                and r["end_dist_to_goal_m"] < GOAL_TOL),
                         "end_dist_to_goal_m": r.get("end_dist_to_goal_m"), "tol": GOAL_TOL},
        "gate_passed": {"ok": bool(crossed and max_abs_x_on_gate is not None
                                   and max_abs_x_on_gate <= gate_half_limit),
                        "crossed": bool(crossed),
                        "samples_on_gate_line": len(on_gate),
                        "max_abs_x_on_gate_m": round(max_abs_x_on_gate, 3)
                        if max_abs_x_on_gate is not None else None,
                        "limit_m": round(gate_half_limit, 3),
                        "note": "门线 y=门中心±0.1 的样本 |x| 必须 ≤ 门洞半宽 + 2 cm"},
        "gate_clearance": {"ok": min_gap is not None and min_gap >= GATE_CLEAR_MIN,
                           "per_box_min_gap_m": gaps, "min_gap_m": min_gap,
                           "clear_min_m": GATE_CLEAR_MIN, "worst": worst},
        "safe_envelope": {
            "ok": (min(zs) > Z_MIN and max(zs) < Z_MAX
                   and max_x <= INNER_X - MARGIN and max_y <= INNER_Y - MARGIN),
            "gz_z_min": round(min(zs), 4), "gz_z_max": round(max(zs), 4),
            "max_abs_x": round(max_x, 3), "max_abs_y": round(max_y, 3)},
        "localization_consistent": {
            "ok": loc_max is not None and loc_max < LOC_ERR_MAX,
            "max_m": report["loc_err_m"]["max"], "mean_m": report["loc_err_m"]["mean"],
            "gate_segment_max_m": report["loc_err_m"]["gate_segment_max"],
            "samples": report["loc_err_m"]["samples"],
            "gate_samples": report["loc_err_m"]["gate_samples"],
            "tol": LOC_ERR_MAX,
            "note": "未建图障碍物回波会让 AMCL 漂（修复前 gate_deep 实测 2.9 m）；"
                    "本判据就是回归 likelihood_field_prob + do_beamskip 的修复"},
    }
    report.update({"tf_samples": len(tr), "gz_samples": len(gztr),
                   "checks": checks,
                   "summary": {"ok": all(c["ok"] for c in checks.values())}})
    _save(report, key, args.tag)
    print("\n===== 摘要 =====")
    print(json.dumps({"checks": {k: c["ok"] for k, c in checks.items()},
                      "details": checks, "summary": report["summary"]},
                     indent=2, ensure_ascii=False))
    rclpy.shutdown()
    return 0 if report["summary"]["ok"] else 3


def _save(report, key, tag=""):
    base = {"gate": "report_narrow_gap.json",
            "gate_deep": "report_narrow_gap_deep.json",
            "gate_tight": "report_narrow_gap_tight.json"}[key]
    if tag:
        base = base[:-len(".json")] + f"_{tag}.json"
    path = os.path.join(RES, base)
    with open(path, "w") as f:
        json.dump(report, f, indent=2, ensure_ascii=False, default=float)
    print("报告:", path)


if __name__ == "__main__":
    raise SystemExit(main())