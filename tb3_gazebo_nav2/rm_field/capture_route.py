#!/usr/bin/env python3
"""采集一次导航路线（三个验收目标）的完整 TF 轨迹，存成可复用的基线。

用途：
  1. 给"绕障验收"挑障碍位置——障碍物必须摆在基线真的会经过的走道上，否则测不到绕行；
  2. 做对照：绕障跑出来的路程/轨迹与无障基线逐点比较，证明"确实是绕开而不是碰巧"。

前置：bash nav2_field.sh 已把栈拉起来（或 nav2_obstacles.sh --no-obstacles 的纯基线）。
用法：
  python3 capture_route.py                      # → results/baseline_route.json
  python3 capture_route.py --out results/xxx.json

记录内容：
  - track:  TF(map->base_footprint) 采样 [wall_t, sim_t, x, y, yaw]，10 Hz
  - gz:     Gazebo 真值 [wall_t, x, y, z]（waffle 的 pose/info）
  - goals:  每个目标的 /navigate_to_pose 结果（含是否 SUCCEEDED）
判据：只打印结果，不做验收（验收在 verify_nav2_obstacles.py）。
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import rclpy                                             # noqa: E402
from verify_nav2_field import GOALS, RES, Verifier       # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(RES, "baseline_route.json"))
    args = ap.parse_args()

    rclpy.init()
    v = Verifier()
    out = {"goals": [list(g) for g in GOALS], "ros_domain_id": os.environ.get("ROS_DOMAIN_ID")}

    print("[1] 等 action server ...")
    if not v.client.wait_for_server(timeout_sec=90.0):
        print("!! 等不到 /navigate_to_pose")
        rclpy.shutdown()
        return 2
    print("[2] 等仿真时钟 / TF ...")
    out["sim_time_ok"] = v.wait_sim_time(30.0)
    out["tf_ready"] = v.spin_until(v.tf_ready, 30.0)
    v.spin_for(1.0)
    if not out["tf_ready"]:
        print("!! TF map->base_footprint 不可用")
        rclpy.shutdown()
        return 2

    results = []
    for i, (x, y, yaw) in enumerate(GOALS, 1):
        print(f"[3.{i}] 目标 ({x}, {y}) ...")
        r = v.goto(x, y, yaw)
        results.append(r)
        print("    ->", {k: r.get(k) for k in ("succeeded", "path_len_m", "end_dist_to_goal_m")})
    out["nav_results"] = results
    out["track"] = [[round(a, 3), round(b, 3), round(c, 4), round(d, 4), round(e, 4)]
                    for a, b, c, d, e in v.track]
    out["gz"] = [[round(a, 3), round(b, 4), round(c, 4), round(d, 4)] for a, b, c, d in v.gz_track]
    with open(args.out, "w") as f:
        json.dump(out, f, ensure_ascii=False)
    total = sum(r.get("path_len_m") or 0.0 for r in results)
    print(f"轨迹 {len(v.track)} 点 / 合计 {total:.3f} m → {args.out}")
    rclpy.shutdown()
    return 0 if all(r.get("succeeded") for r in results) else 3


if __name__ == "__main__":
    raise SystemExit(main())