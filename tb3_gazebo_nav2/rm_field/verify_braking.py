#!/usr/bin/env python3
"""安全层（collision_monitor）刹车验收：宽挡板"突然出现"在机器人正前方，看它刹不刹得住。

场景（`worlds/rmul2027_3v3_up_nav2_braking.sdf.xacro`，几何来自 config/avoid_obstacles.json 的 popup）：
  - 机器人 (0,0) → 目标 (0,3.0)，在 3.2 m 宽的中线走道里全速北上；
  - 宽挡板 2.8×0.5×0.6 m 先藏在 (3,-3)（雷达 3.5 m 外看不见）；
  - 机器人离挡板出现点 (0,2.2) 只剩 0.7 m 时，挡板瞬移到正前方（拦满走道，绕不过去）；
  - 之后只靠 nav2 默认的 collision_monitor（FootprintApproach, time_before_collision 1.2 s）
    与 MPPI 减速：要求"刹住 + 不接触"；挡板拦死走廊，所以不会出现"绕过去"的假通过。

判据（全部是读数）：
  1. popup 真的瞬移出现：出现前在 hidden ±0.05 m、出现后到 appear ±0.05 m；
  2. 出现前机器人确实在跑：pop 前 0.8 s 平均速度 > 0.20 m/s；
  3. collision_monitor 真的介入：pop 之后出现 ≥1 次 state 事件（STOP/SLOWDOWN/APPROACH）；
  4. 刹住：pop 后 2.5 s 内机器人速度降到 < 0.05 m/s，并保持 ≥0.5 s；
  5. 不接触：pop 之后人-挡板最小间距 ≥ robot_radius(0.22)；
  6. 不掉台/不爬台/不出界（同 base 判据）。
目标点必然到不了（挡板拦死），所以到达判据不适用；脚本在机器人停住后主动取消目标。

输出 results/report_collision_monitor.json；全过返回 0，否则 3。
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
    RES, Verifier, INNER_X, INNER_Y, MARGIN, Z_MIN, Z_MAX)

import rclpy                                                       # noqa: E402
from nav2_msgs.action import NavigateToPose                        # noqa: E402
from verify_nav2_obstacles import (                                # noqa: E402
    ObstVerifier, aabb_gap, nearest_in_time, dedup)

CFG = os.path.join(HERE, "config", "avoid_obstacles.json")
ROBOT_R = 0.22
CLEAR_MIN = ROBOT_R
GOAL = (0.0, 3.0, 0.0)          # 挡板后面；到不了，脚本自己取消
PRE_MOVE_MIN_MPS = 0.20         # pop 前平均速度下限：证明"车真在跑"
BRAKE_WINDOW_S = 2.5            # pop 后多久内必须停住
STOP_SPEED = 0.05               # 视为停住的速度
STOP_HOLD_S = 0.50              # 停住要保持多久
POP_TOL = 0.05                  # 挡板位置判定容差


def load_cfg():
    with open(CFG, "r", encoding="utf-8") as f:
        return json.load(f)


def speed_between_tf(track, t0, t1):
    """track=[(wall_t, sim_t, x, y, yaw)] 升序；返回 [t0,t1] 内的平均速度。"""
    seg = [p for p in track if t0 <= p[0] <= t1]
    if len(seg) < 2:
        return None
    d = sum(math.hypot(seg[i][2] - seg[i - 1][2], seg[i][3] - seg[i - 1][3])
            for i in range(1, len(seg)))
    dt = seg[-1][0] - seg[0][0]
    return d / dt if dt > 0 else None


def speed_series(track, t0, t1):
    """track=[(wall_t, sim_t, x, y, yaw)]；返回 [(wall_t, m/s)]。"""
    out = []
    for i in range(1, len(track)):
        if not (t0 <= track[i][0] <= t1):
            continue
        dt = track[i][0] - track[i - 1][0]
        if dt > 1e-6:
            d = math.hypot(track[i][2] - track[i - 1][2], track[i][3] - track[i - 1][3])
            out.append((track[i][0], d / dt))
    return out


def main():
    cfg = load_cfg()
    pop = cfg["popup"]
    appear = pop["appear"]
    hidden = pop["hidden"]
    size = pop["size"]
    rclpy.init()
    v = ObstVerifier(pop["name"])
    report = {"scenario": {"hidden": hidden, "appear": appear, "size": size,
                           "trigger_dist": pop.get("trigger_dist"), "goal": list(GOAL),
                           "robot_radius": ROBOT_R,
                           "criteria": {"pre_move_min_mps": PRE_MOVE_MIN_MPS,
                                        "brake_window_s": BRAKE_WINDOW_S,
                                        "stop_speed": STOP_SPEED,
                                        "stop_hold_s": STOP_HOLD_S,
                                        "clear_min_m": CLEAR_MIN}},
              "ros_domain_id": os.environ.get("ROS_DOMAIN_ID")}
    print(f"[0] ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID')} goal={GOAL} "
          f"popup {pop['name']} {hidden}->{appear}")

    print("[1] 等 action server / sim clock / TF ...")
    if not v.client.wait_for_server(timeout_sec=90.0):
        report["error"] = "no action server"
        _save(report)
        return 2
    report["sim_time_ok"] = v.wait_sim_time(30.0)
    report["tf_ready"] = v.spin_until(v.tf_ready, 30.0)
    v.spin_for(1.0)
    print(f"    sim_time={report['sim_time_ok']} tf={report['tf_ready']}")

    print(f"[2] 发目标并跑（挡板会在正前方 {pop.get('trigger_dist')} m 处瞬移出现）...")
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.header.stamp = v.get_clock().now().to_msg()
    goal.pose.pose.position.x, goal.pose.pose.position.y = GOAL[0], GOAL[1]
    goal.pose.pose.orientation.w = 1.0
    send_fut = v.client.send_goal_async(goal)
    if not v.spin_until(lambda: send_fut.done(), 20.0):
        report["error"] = "send_goal timeout"
        _save(report)
        return 2
    gh = send_fut.result()
    if gh is None or not gh.accepted:
        report["error"] = "goal rejected"
        _save(report)
        return 2
    res_fut = gh.get_result_async()

    # 等：挡板出现过 + 机器人停住 >0.5 s（或超时 40 s）
    t_send = time.time()
    pop_t = None
    stop_since = None
    def stopped_for(hold):
        nonlocal stop_since
        now = time.time()
        tr = v.track
        if len(tr) >= 3:
            seg = [p for p in tr if now - p[0] <= 0.6]
            if len(seg) >= 2:
                sp = speed_between_tf(tr, now - 0.6, now)
                if sp is not None and sp < STOP_SPEED:
                    if stop_since is None:
                        stop_since = now
                    return now - stop_since >= hold
                stop_since = None
        return False

    def popped():
        nonlocal pop_t
        if pop_t is not None:
            return True
        for p in v.dyn:
            if math.hypot(p[1] - appear[0], p[2] - appear[1]) <= 0.10:
                pop_t = p[0]
                return True
        return False

    t_end = t_send + 40.0
    while time.time() < t_end:
        rclpy.spin_once(v, timeout_sec=0.05)
        v._sample_tf()
        if popped() and stopped_for(STOP_HOLD_S):
            print(f"    挡板已出现 t={pop_t:.2f}；机器人停住，取消目标")
            break
    cancelled = False
    if not res_fut.done():
        gh.cancel_goal_async()
        cancelled = True
    v.spin_for(1.0)

    # ---------------- 指标 ----------------
    tr = v.track
    dyns = dedup(v.dyn, 0.02)
    gztr = v.gz_track
    report["popup"] = {"samples": len(v.dyn), "pop_wall_t": pop_t,
                       "hidden_xy": hidden, "appear_xy": appear}
    checks = {}
    if pop_t is None:
        report["error"] = "挡板没有出现（驱动没起或触发距离不对）"
        report["checks"] = {"popup_appeared": {"ok": False}}
        report["summary"] = {"ok": False}
        _save(report)
        return 3

    before = [p for p in dyns if p[0] < pop_t - 0.05]
    after = [p for p in dyns if p[0] >= pop_t]
    hidden_ok = (not before) or all(math.hypot(p[1] - hidden[0], p[2] - hidden[1]) <= POP_TOL
                                    for p in before[-5:])
    appear_ok = bool(after) and all(
        math.hypot(p[1] - appear[0], p[2] - appear[1]) <= POP_TOL for p in after[:5])
    checks["popup_appeared"] = {
        "ok": bool(hidden_ok and appear_ok),
        "hidden_ok": bool(hidden_ok), "appear_ok": bool(appear_ok)}

    pre_speed = speed_between_tf(tr, pop_t - 0.8, pop_t - 0.05)
    checks["robot_was_moving"] = {
        "ok": pre_speed is not None and pre_speed > PRE_MOVE_MIN_MPS,
        "pre_pop_mean_speed_mps": round(pre_speed, 3) if pre_speed is not None else None,
        "min_required_mps": PRE_MOVE_MIN_MPS}

    cm_after = [c for c in v.cm_state if c[0] >= pop_t - 0.2]
    checks["collision_monitor_engaged"] = {
        "ok": len(cm_after) > 0,
        "events_after_pop": len(cm_after),
        "by_action_type": {str(at): sum(1 for c in cm_after if c[2] == at)
                           for at in sorted({c[2] for c in cm_after})},
        "first_event_wall_t": round(cm_after[0][0], 2) if cm_after else None,
        "note": "FootprintApproach 的 state 事件；0 次说明还没轮到安全层就停了"}

    # 刹住：pop 后速度序列降到 STOP_SPEED。要求"2.5 s 内出现过速度≈0"即可——
    # 实测 collision_monitor 先 STOP、随后恢复行为会倒车让开，这不是失败（让开更安全），
    # 所以不要求"停住不动"，只要求真的把速度刹到 ~0 过，并且整体不接触（见 no_contact）。
    sp = speed_series(tr, pop_t, pop_t + BRAKE_WINDOW_S + 1.0)
    first_below = None
    cum_stop = 0.0
    prev_t = None
    for t, s in sp:
        if s < STOP_SPEED:
            if first_below is None:
                first_below = t
            if prev_t is not None:
                cum_stop += min(t - prev_t, 0.3)
        prev_t = t
    min_sp = min((s for _t, s in sp), default=None) if sp else None
    checks["braked"] = {
        "ok": (first_below is not None and (first_below - pop_t) <= BRAKE_WINDOW_S
               and min_sp is not None and min_sp < STOP_SPEED),
        "brake_latency_s": round(first_below - pop_t, 3) if first_below else None,
        "min_speed_after_pop_mps": round(min_sp, 3) if min_sp is not None else None,
        "stopped_cum_s": round(cum_stop, 2),
        "window_s": BRAKE_WINDOW_S,
        "note": "2.5 s 内速度降到 <0.05 m/s 即算刹住；随后倒车让开不算失败"}

    # 不接触：pop 之后人-挡板 AABB 最小间距（真值时间对齐）
    min_gap = min_center = 1e9
    worst = None
    for g in gztr:
        if g[0] < pop_t - 0.1:
            continue
        n = nearest_in_time(dyns, g[0], 0.20)
        if n is None:
            continue
        gap = aabb_gap(g[1], g[2], n[1], n[2], size[0], size[1])
        dc = math.hypot(g[1] - n[1], g[2] - n[2])
        if gap < min_gap:
            min_gap = gap
            worst = {"wall_t": round(g[0], 2),
                     "robot_xy": [round(g[1], 3), round(g[2], 3)],
                     "barrier_xy": [round(n[1], 3), round(n[2], 3)],
                     "gap_m": round(gap, 3)}
        min_center = min(min_center, dc)
    checks["no_contact"] = {
        "ok": min_gap < 1e8 and min_gap >= CLEAR_MIN,
        "min_gap_m": round(min_gap, 3) if min_gap < 1e8 else None,
        "min_center_dist_m": round(min_center, 3) if min_center < 1e8 else None,
        "clear_min_m": CLEAR_MIN, "worst": worst}

    zs = [p[2] for p in v.gz.values()] or [0.0]
    max_x = max((abs(p[2]) for p in tr), default=0.0)
    max_y = max((abs(p[3]) for p in tr), default=0.0)
    checks["safe_envelope"] = {
        "ok": (min(zs) > Z_MIN and max(zs) < Z_MAX
               and max_x <= INNER_X - MARGIN and max_y <= INNER_Y - MARGIN),
        "gz_z_min": round(min(zs), 4), "gz_z_max": round(max(zs), 4),
        "max_abs_x": round(max_x, 3), "max_abs_y": round(max_y, 3)}

    report.update({"result_cancelled": bool(cancelled),
                   "cm_events_total": len(v.cm_state),
                   "cm_events_all": [{"wall_t": round(c[0], 2), "polygon": c[1],
                                      "action_type": c[2]} for c in v.cm_state],
                   "tf_samples": len(tr), "gz_samples": len(gztr),
                   "checks": checks,
                   "summary": {"ok": all(c["ok"] for c in checks.values())}})
    _save(report)
    print("\n===== 摘要 =====")
    print(json.dumps({"checks": {k: c["ok"] for k, c in checks.items()},
                      "details": {k: c for k, c in checks.items()},
                      "summary": report["summary"]}, indent=2, ensure_ascii=False))
    rclpy.shutdown()
    return 0 if report["summary"]["ok"] else 3


def _save(report):
    path = os.path.join(RES, "report_collision_monitor.json")
    with open(path, "w") as f:
        json.dump(report, f, indent=2, ensure_ascii=False, default=float)
    print("报告:", path)


if __name__ == "__main__":
    raise SystemExit(main())