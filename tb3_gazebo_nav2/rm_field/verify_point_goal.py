#!/usr/bin/env python3
"""验收"在地图上点坐标 → 机器人走过去"（前置：栈已起 + start_goal_bridge.sh 已起）。

模拟点击的方式：往 **/goal_pose** 发 PoseStamped（RViz 的 "2D Goal Pose" 工具发的就是这条消息），
经 goal_pose_bridge.py 转成 NavigateToPose action——与被测链路完全同一条。
（RViz 的 "Nav2 Goal" 工具走的是 action 直发，链路更短，用同样方式在 GUI 里验证。）

场景（无障碍的翻转场地，出生点 (0,0)）：
  1. 依次"点"三处：A(2.5,-2.0) → B(4.5,1.5) → C(0,0)，每处到点 <0.30 m；
  2. 换目标：先点 D(2.5,-2.0)，3 s 后（机器人已在走）再点 E(1.0,2.0)，
     最终必须停在 E（<0.30 m），且不能停在 D；
  3. 全程不出界/不掉台/不爬台/不压平台墙。

输出 results/report_point_goal.json；全过返回 0，否则 3。
"""
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("ROS_DOMAIN_ID", "42")

from verify_nav2_field import (                                    # noqa: E402
    RES, Verifier, box_penetration, load_obstacles,
    INNER_X, INNER_Y, MARGIN, OBS_MARGIN, Z_MIN, Z_MAX)

import rclpy                                                       # noqa: E402
from geometry_msgs.msg import PoseStamped                          # noqa: E402

CLICKS = [(2.5, -2.0, 0.0), (4.5, 1.5, 0.0), (0.0, 0.0, 0.0)]
RETARGET = ((2.5, -2.0, 0.0), (1.0, 2.0, 0.0))   # 先点前者，3 s 后点后者
GOAL_TOL = 0.30
ARRIVE_HOLD_S = 1.0
LEG_TIMEOUT = 150.0


class ClickVerifier(Verifier):
    def __init__(self):
        super().__init__()
        self.pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

    def click(self, x, y, yaw):
        m = PoseStamped()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.orientation.w = math.cos(yaw / 2.0)
        for _ in range(1):            # 点一次就发一次（桥有去抖：1 s 内同点重复只算一次）
            self.pub.publish(m)
            self.spin_for(0.30)
        print(f"    已点击 ({x}, {y})", flush=True)

    def pos(self):
        if not self.track:
            return None
        t = self.track[-1]
        return t[2], t[3]

    def wait_arrive(self, x, y, timeout=LEG_TIMEOUT):
        """等 TF 到位（<0.30 m 并保持 1 s）；返回 (reach, 最终距离, 用时)。"""
        t0 = time.time()
        hold = None
        dist = None
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            self._sample_tf()
            p = self.pos()
            if p is None:
                continue
            dist = math.hypot(p[0] - x, p[1] - y)
            if dist <= GOAL_TOL:
                if hold is None:
                    hold = time.time()
                elif time.time() - hold >= ARRIVE_HOLD_S:
                    return True, dist, time.time() - t0
            else:
                hold = None
        return False, dist, time.time() - t0


def main():
    boxes = load_obstacles()
    rclpy.init()
    v = ClickVerifier()
    report = {"clicks": [list(c) for c in CLICKS],
              "retarget": [list(p) for p in RETARGET],
              "tol_m": GOAL_TOL,
              "ros_domain_id": os.environ.get("ROS_DOMAIN_ID")}
    print(f"[0] ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID')}；链路：/goal_pose → 桥 → /navigate_to_pose")

    print("[1] 等 action server / sim clock / TF ...")
    if not v.client.wait_for_server(timeout_sec=90.0):
        report["error"] = "no action server"
        _save(report)
        return 2
    report["sim_time_ok"] = v.wait_sim_time(30.0)
    report["tf_ready"] = v.spin_until(v.tf_ready, 30.0)
    v.spin_for(1.0)
    print(f"    sim_time={report['sim_time_ok']} tf={report['tf_ready']}")

    legs = []
    for i, (x, y, yaw) in enumerate(CLICKS, 1):
        p0 = v.pos()
        t_click = time.time()
        print(f"[2.{i}] 点 ({x}, {y}) ...")
        v.click(x, y, yaw)
        reach, dist, dur = v.wait_arrive(x, y)
        p1 = v.pos()
        moved = math.hypot(p1[0] - p0[0], p1[1] - p0[1]) if (p0 and p1) else None
        legs.append({"click": [x, y, yaw], "reached": bool(reach),
                     "final_dist_m": round(dist, 3) if dist is not None else None,
                     "wall_s": round(dur, 2), "moved_m": round(moved, 3) if moved else None})
        print(f"    -> reached={reach} dist={dist} dur={dur:.1f}s")
    report["legs"] = legs

    # 换目标：先点 D，3 s 后再点 E，必须停在 E
    (dx, dy, dyaw), (ex, ey, eyaw) = RETARGET
    print(f"[3] 换目标：点 ({dx},{dy})，3 s 后改点 ({ex},{ey}) ...")
    v.click(dx, dy, dyaw)
    v.spin_for(3.0)
    v.click(ex, ey, eyaw)
    reach_e, dist_e, dur_e = v.wait_arrive(ex, ey)
    pe = v.pos()
    dist_d = math.hypot(pe[0] - dx, pe[1] - dy) if pe else None
    report["retarget_result"] = {"reached_B": bool(reach_e),
                                 "final_dist_to_B_m": round(dist_e, 3) if dist_e else None,
                                 "final_dist_to_A_m": round(dist_d, 3) if dist_d is not None else None,
                                 "wall_s": round(dur_e, 2)}
    print(f"    -> B reached={reach_e} distB={dist_e} distA={dist_d}")

    v.spin_for(0.5)
    tr = v.track
    zs = [p[2] for p in v.gz.values()] or [0.0]
    max_x = max((abs(p[2]) for p in tr), default=0.0)
    max_y = max((abs(p[3]) for p in tr), default=0.0)
    checks = {
        "sim_time": {"ok": bool(report["sim_time_ok"])},
        "tf_available": {"ok": bool(report["tf_ready"])},
        "clicks_reached": {
            "ok": all(l["reached"] and l["final_dist_m"] is not None
                      and l["final_dist_m"] <= GOAL_TOL for l in legs),
            "per_click": [{"click": l["click"], "reached": l["reached"],
                           "final_dist_m": l["final_dist_m"]} for l in legs],
            "tol_m": GOAL_TOL},
        "click_moves_robot": {
            "ok": all((l["moved_m"] or 0.0) > 0.30 for l in legs),
            "moved_m": [l["moved_m"] for l in legs],
            "note": "每次点击后机器人至少位移 0.30 m，证明 /goal_pose 真的驱动了导航"},
        "retarget_to_latest": {
            "ok": bool(reach_e)
                  and (dist_e or 1e9) <= GOAL_TOL
                  and (dist_d or 0.0) > GOAL_TOL,
            **report["retarget_result"],
            "note": "第二次点击必须取代第一次：停在 E、不在 D"},
        "inside_field": {"ok": max_x <= INNER_X - MARGIN and max_y <= INNER_Y - MARGIN,
                         "max_abs_x": round(max_x, 3), "max_abs_y": round(max_y, 3)},
        "no_fall": {"ok": min(zs) > Z_MIN, "gz_z_min": round(min(zs), 4)},
        "no_climb": {"ok": max(zs) < Z_MAX, "gz_z_max": round(max(zs), 4)},
        "no_map_overlap": {"ok": max((box_penetration(p[2], p[3], boxes) for p in tr),
                                     default=-1e9) <= OBS_MARGIN,
                           "max_box_penetration_m": round(
                               max((box_penetration(p[2], p[3], boxes) for p in tr),
                                   default=-1e9), 4)},
    }
    report.update({"tf_samples": len(tr), "checks": checks,
                   "summary": {"ok": all(c["ok"] for c in checks.values())}})
    _save(report)
    print("\n===== 摘要 =====")
    print(json.dumps({"checks": {k: c["ok"] for k, c in checks.items()},
                      "details": checks, "summary": report["summary"]},
                     indent=2, ensure_ascii=False))
    rclpy.shutdown()
    return 0 if report["summary"]["ok"] else 3


def _save(report):
    path = os.path.join(RES, "report_point_goal.json")
    with open(path, "w") as f:
        json.dump(report, f, indent=2, ensure_ascii=False, default=float)
    print("报告:", path)


if __name__ == "__main__":
    raise SystemExit(main())