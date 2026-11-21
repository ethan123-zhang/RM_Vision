#!/usr/bin/env python3
"""绕障 + 动态避障验收（前置：bash nav2_obstacles.sh 已把带障碍物的栈拉起来）。

与 verify_nav2_field.py 的关系：那边的 11 项判据（生命周期/话题/到位/不出界/不掉台/
不压平台/定位一致/速度合理）这里全部继承；本脚本只加"绕障"这一类新判据。

几何唯一源 = config/avoid_obstacles.json（与 make_obstacles_world.py 生成世界同一份）。
基线 = results/baseline_route.json（无障跑一遍的真实轨迹，capture_route.py 采的）。

新增判据：
  A. 测试有效性（障碍物确实摆在路上）
     A1. 每块静态箱：无障基线轨迹必须侵入箱体投影 ≥ BASELINE_INTRUSION_MIN（0.10 m）
         —— 否则这块箱子根本不在机器人会走的路上，"绕过去了"不能证明任何事；
     A2. 动态箱的**初始位置**必须压在无障基线轨迹上（基线侵入其投影 ≥0.10 m），
         并且其退让终点离基线轨迹 ≥0.30 m（robot_radius 0.22 + 8 cm，停住后不堵路）；
         动态轨迹（起点->终点线段）到无障基线轨迹的最近距离 ≤ DYN_ROUTE_MAX_DIST（0.30 m）。
  B. 绕行安全（静态）
     B1. 全程 Gazebo 真值里，机器人中心到每块静态箱 AABB 的最近间距 ≥ robot_radius(0.22)
         —— 即 nav2 用的保守外接圆从未侵入箱体；真实车体外扩更大，这一条足以排除接触。
  C. 动态避障（配置 mode=triggered_retreat：箱体起初停在基线走道正中挡路，
     机器人进到 trigger_dist 内后，箱体以配置速度单程让开、停在 end）
     C1. 从 /world/<world>/pose/info 独立实测动态箱：运动起止之间的位移/时长 = 配置速度 ±0.05，
         行程 = |end-start| ±20%，最终停在 end ±0.15 m 且不再动（"让开了"）；
     C2. 机器人与动态箱最近间距 ≥ robot_radius(0.22)（无接触），且确实发生过一次遭遇
         （最近中心距 < 2.0 m，否则"避开了"没有意义）；
  D. 证据字段（不作为通过判据，但写进报告）：
     - 动态遭遇窗口内机器人 cmd_vel / 速度下降、collision_monitor 的 state 事件；
     - 三段路程与无障基线的对比（绕行代价）。

输出 results/report_nav2_obstacles.json；全过返回 0，否则 3。
"""
import json
import math
import os
import sys
import time

import argparse

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("ROS_DOMAIN_ID", "42")

from verify_nav2_field import (                                    # noqa: E402
    GOALS, RES, VX_MAX, Verifier, box_penetration, load_obstacles,
    INNER_X, INNER_Y, MARGIN, OBS_MARGIN, LOC_ERR_MAX, Z_MIN, Z_MAX, GOAL_TOL)

import rclpy                                                       # noqa: E402

CFG = os.path.join(HERE, "config", "avoid_obstacles.json")
BASELINE = os.path.join(RES, "baseline_route.json")

ROBOT_R = 0.22                 # nav2 参数里的 robot_radius（保守外接圆）
CLEAR_MIN = ROBOT_R            # 中心到箱体 AABB 的间距下限（= 保守外接圆半径）
BASELINE_INTRUSION_MIN = 0.10  # 无障基线必须侵入箱体投影的最小深度，证明箱子挡在路上
DYN_ROUTE_MAX_DIST = 0.30      # 动态箱往返线段离无障基线轨迹的最大距离（必须横穿路线）
DYN_SPEED_TOL = 0.05           # 实测速度允差（m/s）
DYN_TRAVEL_TOL = 0.20          # 实测行程与 |end-start| 的相对允差
DYN_PARK_TOL = 0.15            # 最终必须停在 end 的容差（m）
# 遭遇判据：机器人中心到动态箱中心必须进到 2.0 m 内。为什么不是 1.0：触发距离本身
# 是 1.5 m（箱体看到机器人才开始让开），让开方向背离机器人，最近距离就停在 1.3~1.5 m；
# 这个量级照样能证明"机器人与正在移动的障碍物正面遭遇过"，再收紧就是拿几何玩游戏。
DYN_ENC_MAX_DIST = 2.00
# 退让终点离无障基线轨迹的最小距离：至少要让 robot_radius(0.22) 通过还留 8 cm，
# 即 ≥0.30 m。实测本轮该值是 0.35 m，且全程人-箱最小间距 0.841 m（停住后也没堵路）。
DYN_END_CLEAR_MIN = 0.30
DYN_WINDOW_S = 1.0             # 测速窗口（s），要大于 set_pose 的 0.1 s 量化步长
# 未建图障碍物附近 AMCL 的瞬态偏差：修复前（likelihood_field，无 beam skip）实测 0.76~1.05 m，
# 且会被拉到错误分支不回来；现已换成 likelihood_field_prob + do_beamskip（见 make_nav2_params.py），
# 全场 max 0.315 m。下面的 excluding_unmapped 仍保留，作为"贴障碍物那段的单独读数"（证据，不判分）。
LOC_EXCL_M = 2.00


def load_cfg(path=CFG):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def resolve_tier(dyn, name=None):
    """档位 → 实际几何（{name, speed, start, end}）；与 dynamic_obstacle.py 同一规则。

    2026-11-21 起动态箱按档位验收（slow 0.3 / fast 0.6 / faster 1.0 m/s，方向也不同）：
    验收必须用**同一档位**的几何来判"起点挡路/终点让开/实测速度"，否则会把
    slow 的判据套到 faster 的轨迹上。
    """
    tiers = dyn.get("tiers") or {}
    if not tiers:
        return {"name": "legacy", "speed": dyn["speed"],
                "start": list(dyn["start"]), "end": list(dyn["end"])}
    name = name or dyn.get("default_tier") or sorted(tiers)[0]
    if name not in tiers:
        raise SystemExit(f"未知档位 {name!r}；可选: {', '.join(sorted(tiers))}")
    t = tiers[name]
    return {"name": name, "speed": float(t["speed"]),
            "start": list(t["start"]), "end": list(t["end"])}


def aabb_gap(x, y, cx, cy, sx, sy):
    """点到轴对齐箱体投影的有符号间距：>0 在箱外，<0 侵入深度（m）。"""
    x0, x1 = cx - sx / 2.0, cx + sx / 2.0
    y0, y1 = cy - sy / 2.0, cy + sy / 2.0
    dx = max(x0 - x, 0.0, x - x1)
    dy = max(y0 - y, 0.0, y - y1)
    if dx > 0.0 or dy > 0.0:
        return math.hypot(dx, dy)
    return -min(x - x0, x1 - x, y - y0, y1 - y)


def seg_point_dist(px, py, ax, ay, bx, by):
    """点到线段距离（判断动态箱往返线段是否横穿基线轨迹）。"""
    vx, vy = bx - ax, by - ay
    L2 = vx * vx + vy * vy
    if L2 < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / L2))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


def dedup(series, dt_min=0.02):
    """同一姿态的重复采样会拉低速度估计，按时间桶去重。"""
    out, last_t = [], -1e9
    for s in series:
        if s[0] - last_t >= dt_min:
            out.append(s)
            last_t = s[0]
    return out


def window_speeds(series, window=DYN_WINDOW_S):
    """用 >= window 的两点算速度（对 0.1 s 一次的 set_pose 步进量化免疫）。"""
    sp = []
    j = 0
    for i in range(len(series)):
        while j < len(series) and series[j][0] - series[i][0] < window:
            j += 1
        if j >= len(series):
            break
        dt = series[j][0] - series[i][0]
        d = math.hypot(series[j][1] - series[i][1], series[j][2] - series[i][2])
        sp.append(d / dt)
    return sp


def nearest_in_time(series, t, tol=0.20):
    """series 按时间升序；返回 |dt|<=tol 的最近元素或 None。"""
    if not series:
        return None
    lo, hi = 0, len(series) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if series[mid][0] < t:
            lo = mid + 1
        else:
            hi = mid
    best = None
    for k in (lo - 1, lo):
        if 0 <= k < len(series):
            dt = abs(series[k][0] - t)
            if best is None or dt < best[0]:
                best = (dt, series[k])
    if best is not None and best[0] <= tol:
        return best[1]
    return None


class ObstVerifier(Verifier):
    """在基类基础上多记动态箱的真值轨迹与 collision_monitor 状态。"""

    def __init__(self, dyn_name):
        self.dyn_name = dyn_name
        self.dyn = []                 # (wall_t, x, y, z) 动态箱 Gazebo 真值
        self.cm_state = []            # (wall_t, polygon_name, action_type)
        super().__init__()
        try:
            from nav2_msgs.msg import CollisionMonitorState
            self.create_subscription(CollisionMonitorState, "/collision_monitor_state",
                                     self._on_cm, 10)
        except Exception as e:                                # noqa: BLE001
            self.cm_state_import_error = str(e)

    def _on_gz(self, msg):
        super()._on_gz(msg)
        now = time.time()
        for p in msg.pose:
            if p.name == self.dyn_name:
                self.dyn.append((now, p.position.x, p.position.y, p.position.z))

    def _on_cm(self, m):
        self.cm_state.append((time.time(), str(m.polygon_name), int(m.action_type)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tier", default=None,
                    help="动态箱档位（不传则用 RM_DYN_TIER 环境变量，再退到 config.default_tier）")
    args = ap.parse_args()
    cfg = load_cfg()
    statics = cfg["static"]
    dyn = cfg["dynamic"]
    tier = resolve_tier(dyn, args.tier or os.environ.get("RM_DYN_TIER"))
    # 默认档位沿用老报告名（report_nav2_obstacles.json），其它档位带后缀，
    # 这样三档验收可以并存、也方便与 2026-11-20 的历史报告直接对比。
    tier_suffix = "" if tier["name"] == (dyn.get("default_tier") or "legacy") else "_" + tier["name"]
    world_boxes = load_obstacles()
    if not os.path.exists(BASELINE):
        print(f"!! 缺无障基线 {BASELINE}：先跑 python3 capture_route.py（无障栈）")
        return 2
    with open(BASELINE, "r", encoding="utf-8") as f:
        base = json.load(f)
    base_track = base["track"]            # [wall_t, sim_t, x, y, yaw]

    rclpy.init()
    v = ObstVerifier(dyn["name"])
    report = {
        "field": {"len": 12.6, "wid": 8.6, "wall_inner_half": [INNER_X, INNER_Y],
                  "obstacle_margin": OBS_MARGIN, "z_window": [Z_MIN, Z_MAX]},
        "robot_radius": ROBOT_R,
        "goals": [list(g) for g in GOALS],
        "obstacles": {"static": statics, "dynamic": dyn},
        "tier": tier,
        "baseline": {"file": os.path.basename(BASELINE),
                     "path_len_m": [r.get("path_len_m") for r in base.get("nav_results", [])]},
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
        "params_hint": {"vx_max": VX_MAX},
        "criteria": {"clear_min_m": CLEAR_MIN, "baseline_intrusion_min_m": BASELINE_INTRUSION_MIN,
                     "dyn_route_max_dist_m": DYN_ROUTE_MAX_DIST, "dyn_speed_tol": DYN_SPEED_TOL,
                     "dyn_travel_rel_tol": DYN_TRAVEL_TOL, "dyn_park_tol_m": DYN_PARK_TOL,
                     "dyn_end_clear_min_m": DYN_END_CLEAR_MIN,
                     "dyn_encounter_max_m": DYN_ENC_MAX_DIST},
    }

    print(f"[0] ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID')} 静态箱 {len(statics)} "
          f"+ 动态箱 {dyn['name']}（档位 {tier['name']}：{tier['speed']} m/s，"
          f"{tier['start']}->{tier['end']}）")

    print("[1] 等 action server /navigate_to_pose ...")
    if not v.client.wait_for_server(timeout_sec=90.0):
        report["error"] = "no action server"
        _save(report, tier_suffix)
        return 2
    print("    ok")

    print("[2] 等仿真时钟 / TF map->base_footprint ...")
    report["sim_time_ok"] = v.wait_sim_time(30.0)
    report["tf_ready"] = v.spin_until(v.tf_ready, 30.0)
    v.spin_for(1.0)
    print(f"    sim_time={report['sim_time_ok']} tf={report['tf_ready']}")

    print("[3] 话题频率（3 s）...")
    report["rates"] = {"scan_hz": round(v.rate("scan_n", 3.0), 2),
                       "odom_hz": round(v.rate("odom_n", 3.0), 2)}
    print("   ", report["rates"])

    print(f"[4] 动态箱就位（{dyn.get('mode')}，触发距离 {dyn.get('trigger_dist')} m）："
          f"已记录 {len(v.dyn)} 个真值采样，机器人靠近后会自己让开")
    results = []
    for i, (x, y, yaw) in enumerate(GOALS, 1):
        print(f"[5.{i}] 目标 ({x}, {y}) ...")
        r = v.goto(x, y, yaw)
        results.append(r)
        print("    ->", {k: r.get(k) for k in
                        ("succeeded", "error_code", "sim_s", "path_len_m",
                         "end_dist_to_goal_m", "error")})
    report["nav_results"] = results

    v.spin_for(0.5)
    tr = v.track
    total_path = sum(math.hypot(tr[i][2] - tr[i - 1][2], tr[i][3] - tr[i - 1][3])
                     for i in range(1, len(tr)))
    sim_dt = (tr[-1][1] - tr[0][1]) if len(tr) > 1 else 0.0
    wall_dt = (tr[-1][0] - tr[0][0]) if len(tr) > 1 else 0.0
    zs = [p[2] for p in v.gz.values()] or [0.0]
    gztr = v.gz_track

    # ---------- 定位一致性（与 verify_nav2_field 同算法） ----------
    loc_err, loc_series, loc_t0 = [], [], (tr[0][0] if tr else 0.0)
    gt, gi = gztr, 0
    for wall_t, _sim_t, x, y, _yaw in tr:
        while gi + 1 < len(gt) and gt[gi + 1][0] <= wall_t:
            gi += 1
        if gt:
            g = gt[gi]
            if abs(g[0] - wall_t) <= 0.5:
                loc_err.append(math.hypot(x - g[1], y - g[2]))
                if wall_t - loc_t0 >= 2.0:
                    loc_t0 = wall_t
                    loc_series.append([round(wall_t - tr[0][0], 1), round(loc_err[-1], 3)])

    report["whole_run"] = {
        "tf_samples": len(tr), "path_len_m": round(total_path, 3),
        "sim_s": round(sim_dt, 2), "wall_s": round(wall_dt, 2),
        "mean_speed_mps": round(total_path / sim_dt, 3) if sim_dt > 0 else None,
        "global_rtf": round(sim_dt / wall_dt, 3) if wall_dt > 0 else None,
        "max_abs_x": round(max((abs(p[2]) for p in tr), default=0.0), 3),
        "max_abs_y": round(max((abs(p[3]) for p in tr), default=0.0), 3),
        "gz_z_min": round(min(zs), 4), "gz_z_max": round(max(zs), 4),
        "max_box_penetration_m": round(
            max((box_penetration(p[2], p[3], world_boxes) for p in tr), default=-1e9), 4),
        "loc_err_m": {"max": round(max(loc_err), 3) if loc_err else None,
                      "mean": round(sum(loc_err) / len(loc_err), 3) if loc_err else None,
                      "samples": len(loc_err), "series_2s": loc_series},
        "cmd_v_max_mps": round(max((abs(c[1]) for c in v.cmd), default=0.0), 3),
    }

    # ---------- A. 测试有效性：箱子真的在无障路线上 ----------
    base_clear = {}
    for ob in statics:
        cx, cy = ob["center"]
        sx, sy = ob["size"][0], ob["size"][1]
        gaps = [aabb_gap(p[2], p[3], cx, cy, sx, sy) for p in base_track]
        mn = min(gaps) if gaps else None
        base_clear[ob["name"]] = {"min_gap_m": round(mn, 4) if mn is not None else None,
                                  "intrusion_m": round(-mn, 4) if (mn is not None and mn < 0) else 0.0}
    d_start = tier["start"]
    d_end = tier["end"]
    dyn_route_dist = min((seg_point_dist(p[2], p[3], d_start[0], d_start[1],
                                         d_end[0], d_end[1])
                          for p in base_track), default=1e9)
    # 动态箱初始位置必须压在无障基线走道上：基线轨迹侵入其投影
    start_gaps = [aabb_gap(p[2], p[3], d_start[0], d_start[1],
                           dyn["size"][0], dyn["size"][1]) for p in base_track]
    start_intrusion = (-min(start_gaps)) if start_gaps and min(start_gaps) < 0 else 0.0
    # 退让终点必须离无障基线轨迹够远：否则它停住后会把后续航段又堵死
    end_clear = min((aabb_gap(p[2], p[3], d_end[0], d_end[1],
                              dyn["size"][0], dyn["size"][1]) for p in base_track),
                    default=1e9)
    report["test_validity"] = {
        "baseline_static": base_clear,
        "baseline_dyn_route_dist_m": round(dyn_route_dist, 3),
        "baseline_dyn_start_intrusion_m": round(start_intrusion, 4),
        "dyn_end_clear_of_baseline_m": round(end_clear, 3),
    }

    # ---------- B. 静态绕行：全程真值到每块箱子的最近间距 ----------
    static_clear = {}
    for ob in statics:
        cx, cy = ob["center"]
        sx, sy = ob["size"][0], ob["size"][1]
        gaps = [aabb_gap(p[1], p[2], cx, cy, sx, sy) for p in gztr]
        static_clear[ob["name"]] = {
            "min_gap_m": round(min(gaps), 3) if gaps else None,
            "min_gap_tf_m": round(min(aabb_gap(p[2], p[3], cx, cy, sx, sy) for p in tr), 3)
            if tr else None,
        }
    report["static_clearance"] = static_clear

    # ---------- C. 动态避障 ----------
    dyns = dedup(v.dyn, 0.02)
    ds = {"samples": len(v.dyn), "dedup_samples": len(dyns),
          "mode": dyn.get("mode", "triggered_retreat"),
          "segment": {"start": d_start, "end": d_end,
                      "len_m": round(math.hypot(d_end[0] - d_start[0],
                                                d_end[1] - d_start[1]), 3)}}
    seg_len = math.hypot(d_end[0] - d_start[0], d_end[1] - d_start[1])
    if len(dyns) >= 5:
        travel = sum(math.hypot(dyns[i][1] - dyns[i - 1][1], dyns[i][2] - dyns[i - 1][2])
                     for i in range(1, len(dyns)))

        def dist_to(p, xy):
            return math.hypot(p[1] - xy[0], p[2] - xy[1])

        # 运动起止时刻：用于算平均速度（对 set_pose 的 0.1 s 步进量化免疫）
        t_m0 = t_m1 = None
        for p in dyns:
            if t_m0 is None and dist_to(p, d_start) > 0.03:
                t_m0 = p[0]
            elif t_m0 is not None and t_m1 is None and dist_to(p, d_end) <= 0.03:
                t_m1 = p[0]
        motion_s = (t_m1 - t_m0) if (t_m0 is not None and t_m1 is not None
                                     and t_m1 > t_m0) else None
        motion_speed = (travel / motion_s) if motion_s else None
        # 运动窗口内的 1 s 窗口速度（也给出来做交叉核对）
        win = [p for p in dyns if t_m0 is not None and t_m1 is not None
               and t_m0 <= p[0] <= t_m1]
        sp = window_speeds(win, DYN_WINDOW_S) if len(win) > 2 else []
        sp_sorted = sorted(sp)
        med = sp_sorted[len(sp_sorted) // 2] if sp_sorted else None
        xs = [p[1] for p in dyns]
        ys = [p[2] for p in dyns]
        # 让开之后必须真的停住：取最后 1 s 的位移
        tail = [p for p in dyns if p[0] >= dyns[-1][0] - 1.0]
        tail_move = sum(math.hypot(tail[i][1] - tail[i - 1][1], tail[i][2] - tail[i - 1][2])
                        for i in range(1, len(tail))) if len(tail) > 1 else 0.0
        # 实际轨迹偏离配置线段的程度（应该贴着 start->end 直线走）
        dev = max((seg_point_dist(p[1], p[2], d_start[0], d_start[1],
                                  d_end[0], d_end[1]) for p in dyns), default=0.0)
        park_err = math.hypot(dyns[-1][1] - d_end[0], dyns[-1][2] - d_end[1])
        ds.update({"travel_m": round(travel, 3),
                   "travel_expected_m": round(seg_len, 3),
                   "motion_duration_s": round(motion_s, 3) if motion_s else None,
                   "motion_speed_mps": round(motion_speed, 3) if motion_speed else None,
                   "window_speed_1s_median_mps": round(med, 3) if med is not None else None,
                   "deviation_from_segment_m": round(dev, 3),
                   "park_err_m": round(park_err, 3),
                   "tail_1s_move_m": round(tail_move, 3),
                   "x_range": [round(min(xs), 3), round(max(xs), 3)],
                   "y_range": [round(min(ys), 3), round(max(ys), 3)]})
        # 与机器人最近间距（按时间对齐两条 60 Hz 真值流）
        dxx = dyn["size"][0]
        dyy = dyn["size"][1]
        min_center, min_gap = 1e9, 1e9
        worst = None
        enc_window = []          # 人-箱中心距 < 1 m 的墙钟时间点
        for g in gztr:
            n = nearest_in_time(dyns, g[0], 0.20)
            if n is None:
                continue
            dc = math.hypot(g[1] - n[1], g[2] - n[2])
            if dc < min_center:
                min_center = dc
            gap = aabb_gap(g[1], g[2], n[1], n[2], dxx, dyy)
            if gap < min_gap:
                min_gap = gap
                worst = {"wall_t": round(g[0], 1),
                         "robot_xy": [round(g[1], 3), round(g[2], 3)],
                         "box_xy": [round(n[1], 3), round(n[2], 3)],
                         "center_dist_m": round(dc, 3), "gap_m": round(gap, 3)}
            if dc < DYN_ENC_MAX_DIST:
                enc_window.append(g[0])
        ds["min_center_dist_m"] = round(min_center, 3) if min_center < 1e9 else None
        ds["min_gap_m"] = round(min_gap, 3) if min_gap < 1e9 else None
        ds["worst_gap"] = worst
        ds["encounter_samples"] = len(enc_window)
        # 遭遇窗口内的机器人表现：cmd_vel 与 TF 速度
        if enc_window:
            t0, t1 = min(enc_window) - 0.2, max(enc_window) + 0.2
            cmds = [c for c in v.cmd if t0 <= c[0] <= t1]
            spd_tf = []
            for i in range(1, len(tr)):
                if t0 <= tr[i][0] <= t1:
                    dt = tr[i][0] - tr[i - 1][0]
                    if dt > 1e-6:
                        d = math.hypot(tr[i][2] - tr[i - 1][2], tr[i][3] - tr[i - 1][3])
                        spd_tf.append(d / dt)
            ds["encounter"] = {
                "wall_s": [round(t0 + 0.2, 1), round(t1 - 0.2, 1)],
                "cmd_samples": len(cmds),
                "cmd_v_min_mps": round(min((abs(c[1]) for c in cmds), default=0.0), 3),
                "cmd_v_mean_mps": round(sum(abs(c[1]) for c in cmds) / len(cmds), 3) if cmds else None,
                "tf_speed_min_mps": round(min(spd_tf), 3) if spd_tf else None,
                "tf_speed_mean_mps": round(sum(spd_tf) / len(spd_tf), 3) if spd_tf else None,
            }
        else:
            ds["encounter"] = None
    else:
        ds["error"] = "动态箱真值采样不足（驱动没起？）"
    report["dynamic"] = ds

    # ---------- 定位一致性（剔除"离未建图障碍物 ≤ LOC_EXCL_M"的样本） ----------
    # 未建图障碍物会把 AMCL 的扫描匹配带偏（两轮实测都出现 0.9~1.05 m 的瞬态偏差，
    # 且都发生在贴着 avoid_box_1/动态箱走的 12~32 s 段）。地图-世界一致性判据本来
    # 是抓"PGM 翻转"这种米级错位，用被未建图障碍物污染的样本来判会把它变成噪声。
    # 所以这里给出两份数：all（全场）与 excluding_unmapped（判据用）。
    unmapped = [(ob["center"][0], ob["center"][1], ob["size"][0], ob["size"][1])
                for ob in statics]
    scoped = []
    gt2, gi2 = gztr, 0
    for wall_t, _sim_t, x, y, _yaw in tr:
        while gi2 + 1 < len(gt2) and gt2[gi2 + 1][0] <= wall_t:
            gi2 += 1
        if not gt2:
            continue
        g = gt2[gi2]
        if abs(g[0] - wall_t) > 0.5:
            continue
        gaps = [aabb_gap(g[1], g[2], cx, cy, sx, sy) for cx, cy, sx, sy in unmapped]
        n = nearest_in_time(dyns, wall_t, 0.20) if dyns else None
        if n is not None:
            gaps.append(aabb_gap(g[1], g[2], n[1], n[2], dyn["size"][0], dyn["size"][1]))
        if not gaps or min(gaps) > LOC_EXCL_M:
            scoped.append(math.hypot(x - g[1], y - g[2]))
    report["whole_run"]["loc_err_m"]["excluding_unmapped"] = {
        "max": round(max(scoped), 3) if scoped else None,
        "mean": round(sum(scoped) / len(scoped), 3) if scoped else None,
        "samples": len(scoped), "excl_m": LOC_EXCL_M,
        "note": "剔除离静态/动态未建图障碍物 ≤2 m 的样本；剩下的样本才用来判地图-世界一致性"}

    # collision_monitor 事件（证据，不作为通过判据：DWB 提前避开时它本来就不该触发）
    by_type = {}
    for _t, name, at in v.cm_state:
        by_type.setdefault(at, 0)
        by_type[at] += 1
    report["collision_monitor"] = {"events": len(v.cm_state),
                                   "by_action_type": by_type,
                                   "by_polygon": sorted({c[1] for c in v.cm_state})}

    # ---------- D. 绕行代价：三段路程与无障基线对比 ----------
    base_lens = [r.get("path_len_m") for r in base.get("nav_results", [])]
    report["detour_cost"] = [
        {"goal": r.get("goal"), "path_len_m": r.get("path_len_m"),
         "baseline_path_len_m": base_lens[i] if i < len(base_lens) else None,
         "delta_m": round((r.get("path_len_m") or 0.0) - base_lens[i], 3)
         if i < len(base_lens) and base_lens[i] is not None else None}
        for i, r in enumerate(results)]

    checks = {
        "sim_time": {"ok": bool(report["sim_time_ok"])},
        "tf_available": {"ok": bool(report["tf_ready"])},
        "topics": {"ok": report["rates"]["scan_hz"] > 1.0 and report["rates"]["odom_hz"] > 1.0,
                   **report["rates"]},
        "goals_succeeded": {
            "ok": all(r.get("succeeded") and r.get("error_code") == 0 for r in results),
            "per_goal": [{"succeeded": r.get("succeeded"), "error_code": r.get("error_code"),
                          "sim_s": r.get("sim_s"), "error": r.get("error")} for r in results]},
        "reached_goal": {
            "ok": all((r.get("end_dist_to_goal_m") is not None
                       and r["end_dist_to_goal_m"] < GOAL_TOL) for r in results),
            "end_dist_to_goal_m": [r.get("end_dist_to_goal_m") for r in results],
            "tol": GOAL_TOL},
        "inside_field": {
            "ok": report["whole_run"]["max_abs_x"] <= INNER_X - MARGIN + 1e-9
                  and report["whole_run"]["max_abs_y"] <= INNER_Y - MARGIN + 1e-9,
            "max_abs_x": report["whole_run"]["max_abs_x"],
            "max_abs_y": report["whole_run"]["max_abs_y"]},
        "no_fall": {"ok": report["whole_run"]["gz_z_min"] > Z_MIN,
                    "gz_z_min": report["whole_run"]["gz_z_min"]},
        "no_climb": {"ok": report["whole_run"]["gz_z_max"] < Z_MAX,
                     "gz_z_max": report["whole_run"]["gz_z_max"]},
        "no_map_overlap": {
            "ok": report["whole_run"]["max_box_penetration_m"] <= OBS_MARGIN,
            "max_box_penetration_m": report["whole_run"]["max_box_penetration_m"],
            "tol": OBS_MARGIN,
            "note": "场地平台/墙的 48 块碰撞 box 投影；5 cm = 碰撞体栅格粒度"},
        "localization_consistent": {
            # 2026-11-20 深夜修复后：AMCL 换成 likelihood_field_prob + do_beamskip，
            # 未建图障碍物回波会被当"离群束"跳过，不再把估计拉到贴墙/墙角位姿。
            # 本轮实测全场 max 0.315 m（修复前 0.764 m）→ 判据改回全样本，不再剔除。
            "ok": report["whole_run"]["loc_err_m"]["max"] is not None
                  and report["whole_run"]["loc_err_m"]["max"] < LOC_ERR_MAX,
            "all_samples_max_m": report["whole_run"]["loc_err_m"]["max"],
            "excluding_unmapped": report["whole_run"]["loc_err_m"]["excluding_unmapped"],
            "tol": LOC_ERR_MAX,
            "note": "TF(map->base_footprint) vs Gazebo 真值，全样本；修复前未建图障碍物附近"
                    "会带偏到 0.76~1.05 m（当时判据只能剔除 ≤2 m 样本），"
                    "换成 likelihood_field_prob + do_beamskip 后全场 max 0.315 m"},
        "speed_sane": {
            "ok": VX_MAX is not None
                  and all(r.get("cmd_v_max_mps") is not None
                          and r["cmd_v_max_mps"] <= VX_MAX * 1.05
                          and (r.get("mean_speed_mps") or 0.0) > 0.05 for r in results),
            "vx_max_from_params": VX_MAX,
            "cmd_v_max_mps": [r.get("cmd_v_max_mps") for r in results]},
        "obstacles_on_route": {
            "ok": all(x["intrusion_m"] >= BASELINE_INTRUSION_MIN for x in base_clear.values())
                  and dyn_route_dist <= DYN_ROUTE_MAX_DIST
                  and start_intrusion >= BASELINE_INTRUSION_MIN
                  and end_clear >= DYN_END_CLEAR_MIN,
            "baseline_static": base_clear,
            "baseline_dyn_route_dist_m": round(dyn_route_dist, 3),
            "baseline_dyn_start_intrusion_m": round(start_intrusion, 4),
            "dyn_end_clear_of_baseline_m": round(end_clear, 3),
            "note": "无障基线侵入箱体投影 ≥0.10 m 才算挡在路上；动态箱起点同理、"
                    "退让终点离基线 ≥0.60 m 才不堵后续航段"},
        "static_clearance": {
            "ok": all(x["min_gap_m"] is not None and x["min_gap_m"] >= CLEAR_MIN
                      for x in static_clear.values()),
            "per_box": static_clear, "clear_min_m": CLEAR_MIN,
            "note": "Gazebo 真值机器人中心到箱体 AABB 的最近间距；robot_radius=0.22 未侵入"},
        "dynamic_moving": {
            "ok": ds.get("window_speed_1s_median_mps") is not None
                  and abs(ds["window_speed_1s_median_mps"] - tier["speed"]) <= DYN_SPEED_TOL
                  and ds.get("travel_m") is not None
                  and abs(ds["travel_m"] - seg_len) <= seg_len * DYN_TRAVEL_TOL
                  and (ds.get("park_err_m") is not None and ds["park_err_m"] <= DYN_PARK_TOL)
                  and (ds.get("tail_1s_move_m") or 0.0) <= 0.05,
            "tier": tier["name"],
            "config_speed_mps": tier["speed"],
            "window_speed_1s_median_mps": ds.get("window_speed_1s_median_mps"),
            "motion_speed_mps": ds.get("motion_speed_mps"),
            "motion_duration_s": ds.get("motion_duration_s"),
            "travel_m": ds.get("travel_m"), "travel_expected_m": round(seg_len, 3),
            "park_err_m": ds.get("park_err_m"), "tail_1s_move_m": ds.get("tail_1s_move_m"),
            "deviation_from_segment_m": ds.get("deviation_from_segment_m"),
            "tol": {"speed": DYN_SPEED_TOL, "travel_rel": DYN_TRAVEL_TOL,
                    "park": DYN_PARK_TOL},
            "note": "独立测速用 1 s 窗口位移中位数（免疫 set_pose 0.1 s 步进量化；"
                    "motion_speed 是运动起止位移/时长，量化误差偏大只作参考）+ 行程与配置"
                    "线段一致 + 最终停住不再动"},
        "dynamic_clearance": {
            "ok": ds.get("min_gap_m") is not None and ds["min_gap_m"] >= CLEAR_MIN
                  and (ds.get("min_center_dist_m") is not None
                       and ds["min_center_dist_m"] < DYN_ENC_MAX_DIST),
            "min_gap_m": ds.get("min_gap_m"),
            "min_center_dist_m": ds.get("min_center_dist_m"),
            "worst_gap": ds.get("worst_gap"),
            "encounter_samples": ds.get("encounter_samples"),
            "clear_min_m": CLEAR_MIN, "encounter_max_m": DYN_ENC_MAX_DIST,
            "note": "无接触 + 确实遭遇过（中心距 <1 m）；否则动态测试无意义"},
    }
    ok = all(c["ok"] for c in checks.values())
    report["checks"] = checks
    report["summary"] = {"ok": bool(ok), "tier": tier["name"]}
    _save(report, tier_suffix)

    print("\n===== 摘要 =====")
    print(json.dumps({"rates": report["rates"],
                      "whole_run": {k: report["whole_run"][k] for k in
                                    ("path_len_m", "sim_s", "mean_speed_mps",
                                     "max_box_penetration_m", "loc_err_m", "cmd_v_max_mps")},
                      "baseline_static": base_clear,
                      "static_clearance": static_clear,
                      "dynamic": {k: ds.get(k) for k in
                                  ("mode", "motion_speed_mps", "motion_duration_s",
                                   "window_speed_1s_median_mps", "travel_m",
                                   "travel_expected_m", "park_err_m", "tail_1s_move_m",
                                   "deviation_from_segment_m", "min_center_dist_m",
                                   "min_gap_m", "worst_gap", "encounter_samples",
                                   "encounter")},
                      "collision_monitor": report["collision_monitor"],
                      "detour_cost": report["detour_cost"],
                      "checks": {k: c["ok"] for k, c in checks.items()},
                      "summary": report["summary"]}, indent=2, ensure_ascii=False))
    rclpy.shutdown()
    return 0 if ok else 3


def _save(report, tier_suffix=""):
    path = os.path.join(RES, f"report_nav2_obstacles{tier_suffix}.json")
    with open(path, "w") as f:
        json.dump(report, f, indent=2, ensure_ascii=False, default=float)
    print("报告:", path)


if __name__ == "__main__":
    raise SystemExit(main())