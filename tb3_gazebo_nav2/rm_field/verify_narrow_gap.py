#!/usr/bin/env python3
"""窄通道（贴边）验收：0.80 m 门洞（车宽 0.44 m）能不能过去、贴得有多近、有没有碰。

场景（`worlds/rmul2027_3v3_up_nav2_gap.sdf.xacro`，几何来自 config/avoid_obstacles.json 的 gate）：
  - 门：两块 0.6×0.5 m 静态箱放在 (∓0.70, 0.50)，内侧面 ±0.40 → **门洞 0.80 m**；
    门两侧到走廊内侧面只剩 0.40 m（< 车宽 0.44 m）→ 机器人绕不过去，只能走门。
  - 出生点 (0,-2.0)，目标 (0,2.6)：直线穿门。
  - 5 cm 碰撞栅格的量化误差 ±0.025 m：门洞有效值在 0.75~0.85 m 之间。

判据：
  1. 导航成功、到位 <0.30 m；
  2. 真的穿门：真值轨迹在 y=0.5±0.10 的样本里 |x| ≤ 0.42（否则说明没从门洞过）；
  3. 贴边安全：全程到两块门箱 AABB 的最小间距 ≥ 0.20 m（车体最大半宽约 0.19 m）；
  4. 不出界/不掉台/不爬台（同基类判据）。
输出 results/report_narrow_gap.json；全过返回 0，否则 3。
"""
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
GATE_LINE_Y = 0.50           # 门中心线
GATE_HALF_WIN = 0.10         # 判定"在门线上"的 y 窗口


def load_cfg():
    with open(CFG, "r", encoding="utf-8") as f:
        return json.load(f)


def main():
    gate = load_cfg()["gate"]
    goal_xy = gate["goal"]
    west, east = gate["boxes"]
    gap = ((east["center"][0] - east["size"][0] / 2.0)
           - (west["center"][0] + west["size"][0] / 2.0))
    rclpy.init()
    v = Verifier()
    report = {"scenario": {"gate_gap_m": round(gap, 3), "goal": goal_xy,
                           "spawn": gate.get("spawn"), "gate_clear_min_m": GATE_CLEAR_MIN,
                           "grid_quantization_m": 0.025,
                           "boxes": gate["boxes"]},
              "ros_domain_id": os.environ.get("ROS_DOMAIN_ID")}
    print(f"[0] ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID')} 门洞 {gap:.2f} m goal={goal_xy}")

    print("[1] 等 action server / sim clock / TF ...")
    if not v.client.wait_for_server(timeout_sec=90.0):
        report["error"] = "no action server"
        _save(report)
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
    on_gate = [p for p in gztr if abs(p[2] - GATE_LINE_Y) <= GATE_HALF_WIN]
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

    checks = {
        "goals_succeeded": {"ok": bool(r.get("succeeded")) and r.get("error_code") == 0,
                            "succeeded": r.get("succeeded"),
                            "error_code": r.get("error_code"), "error": r.get("error")},
        "reached_goal": {"ok": (r.get("end_dist_to_goal_m") is not None
                                and r["end_dist_to_goal_m"] < GOAL_TOL),
                         "end_dist_to_goal_m": r.get("end_dist_to_goal_m"), "tol": GOAL_TOL},
        "gate_passed": {"ok": bool(crossed and max_abs_x_on_gate is not None
                                   and max_abs_x_on_gate <= 0.42),
                        "crossed": bool(crossed),
                        "samples_on_gate_line": len(on_gate),
                        "max_abs_x_on_gate_m": round(max_abs_x_on_gate, 3)
                        if max_abs_x_on_gate is not None else None,
                        "limit_m": 0.42,
                        "note": "门线 y=0.5±0.1 的样本 |x| 必须 ≤0.42（门洞半宽 0.40 + 2 cm 容差）"},
        "gate_clearance": {"ok": min_gap is not None and min_gap >= GATE_CLEAR_MIN,
                           "per_box_min_gap_m": gaps, "min_gap_m": min_gap,
                           "clear_min_m": GATE_CLEAR_MIN, "worst": worst},
        "safe_envelope": {
            "ok": (min(zs) > Z_MIN and max(zs) < Z_MAX
                   and max_x <= INNER_X - MARGIN and max_y <= INNER_Y - MARGIN),
            "gz_z_min": round(min(zs), 4), "gz_z_max": round(max(zs), 4),
            "max_abs_x": round(max_x, 3), "max_abs_y": round(max_y, 3)},
    }
    report.update({"tf_samples": len(tr), "gz_samples": len(gztr),
                   "checks": checks,
                   "summary": {"ok": all(c["ok"] for c in checks.values())}})
    _save(report)
    print("\n===== 摘要 =====")
    print(json.dumps({"checks": {k: c["ok"] for k, c in checks.items()},
                      "details": checks, "summary": report["summary"]},
                     indent=2, ensure_ascii=False))
    rclpy.shutdown()
    return 0 if report["summary"]["ok"] else 3


def _save(report):
    path = os.path.join(RES, "report_narrow_gap.json")
    with open(path, "w") as f:
        json.dump(report, f, indent=2, ensure_ascii=False, default=float)
    print("报告:", path)


if __name__ == "__main__":
    raise SystemExit(main())