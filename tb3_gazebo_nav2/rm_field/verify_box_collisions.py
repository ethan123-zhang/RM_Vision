#!/usr/bin/env python3
"""box 拼块碰撞验收（2026-11-19：替换掉 113k 面三角网格碰撞之后）。

前置：`bash verify_box_collisions.sh`（它负责起 drive 世界、跑本脚本、收尾），
或手动 `gz sim -s -r worlds/drive_colored.sdf` 后直接跑本脚本。

五项，全部是读数断言：

  A. 静置：机器人放在 (0,0)，z 应为轮半径 **0.0100**（网格碰撞时是 0.0158 —— 被垫高 6 mm 的 bug），
     水平漂移 < 1 cm。
  B. 直行：0.15 m/s × 2 s，位移应 ≈ 0.3 m 以上（证明"轮子转车也走"）。
  C. 撞东墙：0.3 m/s 顶上 x=+6.00 的墙，停车中心应 = 6.00 − 0.0685（车头外缘到中心），
     不得穿墙；z 全程恒定。
  D. 撞北墙：同上，y=+4.00，停车中心 3.9315。
  E. 撞绿平台（0.20 m 高）：从南侧逼近 y=−1.00 的台沿，停车中心 −1.0685；
     **不得爬上平台**（z 必须仍是 0.010，不是 0.21）。

墙/台沿坐标不是手写死的：从 debug/box_collisions.json（生成脚本的产物）里读，
所以重新生成碰撞后验收会跟着更新。
"""
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gz.msgs10.twist_pb2 as twist_pb2          # noqa: E402

from verify_field import Harness, TB3_SDF         # noqa: E402

WORLD = os.environ.get("GZ_WORLD", "drive")
RES = os.path.join(HERE, "results")
os.makedirs(RES, exist_ok=True)

GEO_JSON = os.path.join(HERE, "debug", "box_collisions.json")

# TB3 waffle 底盘碰撞盒：0.265 m 见方、中心 pose x=-0.064（turtlebot3_gazebo/model.sdf）
# → 车头外缘比中心前 0.0685 m；撞停时中心 = 墙面 − 0.0685
TB3_FRONT_FROM_CENTER = 0.0685
Z_FLOOR = 0.0100        # 轮半径（turtlebot3 waffle 轮子 r=0.033，被底盘拉扯后稳定在 0.010）
TOL_Z = 0.003
TOL_STOP = 0.020        # 停车位置容差（围栏验收时误差 0.4 mm，这里给 2 cm 余量）
TOL_PENETRATE = 0.010   # 允许贴到墙面前 1 cm 以内，但不许越界
TOL_DRIFT = 0.010
TOL_DRIFT_CONTACT = 0.050

PUSH_SPEED = 0.30
FORWARD_SPEED = 0.15
FORWARD_S = 2.0


def rm(h, name):
    import gz.msgs10.boolean_pb2 as bool_pb2
    import gz.msgs10.entity_pb2 as entity_pb2
    req = entity_pb2.Entity()
    req.name = name
    req.type = entity_pb2.Entity.MODEL
    try:
        ok, rep = h.node.request(f"/world/{h.world}/remove", req,
                                 entity_pb2.Entity, bool_pb2.Boolean, 15000)
        return bool(ok and rep is not None and rep.data)
    except Exception as e:                                   # noqa: BLE001
        print(f"  !! remove {name} 异常: {type(e).__name__}: {e}")
        return False


def spawn_robot(h, x, y, yaw, name="waffle"):
    rm(h, name)
    h.poses.pop(name, None)
    time.sleep(0.5)
    ok = h.create(name, sdf_filename=TB3_SDF, xyz=(x, y, 0.25), yaw=yaw)
    t0 = time.time()
    p = None
    while time.time() - t0 < 8.0:
        p = h.pose(name)
        if p is not None and abs(p["x"] - x) < 0.05 and abs(p["y"] - y) < 0.05:
            break
        time.sleep(0.1)
    return ok, p


def make_pub(h):
    topics = [t for t in h.node.topic_list() if t.endswith("/cmd_vel")]
    cmd_topic = "/cmd_vel" if "/cmd_vel" in topics else (topics[0] if topics else "/cmd_vel")
    return h.node.advertise(cmd_topic, twist_pb2.Twist), cmd_topic


def push(h, name, speed, duration, pub=None, rate_s=0.1):
    """持续发前进指令 duration 秒（每 0.1 s 重发一次），随后发零并等停稳。返回轨迹。"""
    if pub is None:
        pub, _ = make_pub(h)
        time.sleep(0.5)
    tw = twist_pb2.Twist()
    tw.linear.x = speed
    track, jumps, prev = [], [], None
    t0 = time.time()
    last_pub = 0.0
    while time.time() - t0 < duration:
        now = time.time() - t0
        if now - last_pub >= rate_s:
            pub.publish(tw)
            last_pub = now
        p = h.pose(name)
        if p is not None:
            row = (round(now, 3), p["x"], p["y"], p["z"])
            if prev is not None and (abs(row[1] - prev[1]) + abs(row[2] - prev[2])) > 0.15:
                jumps.append({"t": row[0],
                              "from": [round(prev[1], 3), round(prev[2], 3), round(prev[3], 3)],
                              "to": [round(row[1], 3), round(row[2], 3), round(row[3], 3)]})
            track.append(row)
            prev = row
        time.sleep(0.05)
    pub.publish(twist_pb2.Twist())
    time.sleep(1.2)
    return track, jumps


def stats_along(track, axis):
    idx = 1 if axis == "x" else 2
    vals = [t[idx] for t in track]
    clean = []
    for row in track:
        if clean and (abs(row[1] - clean[-1][1]) + abs(row[2] - clean[-1][2])) > 0.15:
            continue
        clean.append(row)
    drift = None
    if len(clean) >= 2:
        last = clean[-1][0]
        seg = [t for t in clean if t[0] >= last - 2.0]
        if len(seg) >= 2:
            a, b = seg[0], seg[-1]
            drift = round(math.hypot(b[1] - a[1], b[2] - a[2]), 4)
    return {
        "max_coord": round(max(vals), 4) if vals else None,
        "z_min": round(min(t[3] for t in track), 4) if track else None,
        "z_max": round(max(t[3] for t in track), 4) if track else None,
        "drift_last_2s_m": drift,
        "n_samples": len(track),
    }


def test_static(h):
    print("\n[A] 静置（0,0）...")
    ok, p0 = spawn_robot(h, 0.0, 0.0, 0.0)
    time.sleep(3.0)
    p = h.pose("waffle") or {}
    z = p.get("z")
    drift = math.hypot(p.get("x", 0) - 0.0, p.get("y", 0) - 0.0) if p else None
    res = {
        "spawn_ok": ok, "z": round(z, 4) if z is not None else None,
        "z_target": Z_FLOOR, "z_err": round(abs(z - Z_FLOOR), 4) if z is not None else None,
        "xy_drift_m": round(drift, 4) if drift is not None else None,
    }
    res["ok"] = bool(z is not None and abs(z - Z_FLOOR) <= TOL_Z
                     and drift is not None and drift <= TOL_DRIFT)
    print("  z=%.4f (目标 %.4f, 旧网格碰撞 bug 是 0.0158) 漂移=%.4f  ok=%s"
          % (res["z"], Z_FLOOR, res["xy_drift_m"], res["ok"]))
    rm(h, "waffle")
    return res


def test_forward(h):
    print("\n[B] 直行 0.15 m/s × 2 s ...")
    ok, p0 = spawn_robot(h, 0.0, 0.0, 0.0)
    time.sleep(1.0)
    a = dict(h.pose("waffle") or {})
    pub, topic = make_pub(h)
    track, jumps = push(h, "waffle", FORWARD_SPEED, FORWARD_S, pub=pub)
    b = h.pose("waffle") or {}
    dx = b.get("x", 0) - a.get("x", 0)
    dz = max(abs(t[3] - a.get("z", Z_FLOOR)) for t in track) if track else None
    res = {
        "cmd_topic": topic, "spawn_ok": ok,
        "x0": round(a.get("x", 0), 4), "x1": round(b.get("x", 0), 4),
        "dx_m": round(dx, 4), "z_max_dev": round(dz, 4) if dz is not None else None,
        "jumps": jumps,
    }
    # 2 s × 0.15 = 0.30 m 理论值；实测 0.276 m（起步/订阅延迟 ~0.16 s），
    # 判据只看"车真的走了、速度量级对"：平均 ≥ 0.12 m/s。旧网格碰撞时 2 s 只动 1.4e-5 m。
    res["ok"] = bool(dx >= 0.24 and dx <= 0.60
                     and dz is not None and dz <= TOL_Z)
    print("  x: %.4f -> %.4f  dx=%.4f m  z 最大偏移=%.4f  ok=%s"
          % (res["x0"], res["x1"], res["dx_m"], res["z_max_dev"], res["ok"]))
    rm(h, "waffle")
    return res


def test_wall(h, axis, face, spawn_xy, yaw, duration=7.0):
    print("\n[%s] 撞墙 %s=%.2f（从 %s 出发，%.1f m/s）..."
          % (axis.upper(), axis, face, spawn_xy, PUSH_SPEED))
    ok, p0 = spawn_robot(h, spawn_xy[0], spawn_xy[1], yaw)
    time.sleep(1.0)
    pub, _ = make_pub(h)
    track, jumps = push(h, "waffle", PUSH_SPEED, duration, pub=pub)
    final = h.pose("waffle") or {}
    st = stats_along(track, axis)
    expected = round(face - TB3_FRONT_FROM_CENTER, 4)
    stop_err = round(abs(st["max_coord"] - expected), 4) if st["max_coord"] is not None else None
    crossed = st["max_coord"] is not None and st["max_coord"] > face - TOL_PENETRATE
    res = {
        "spawn_ok": ok, "face": face, "expected_stop_center": expected,
        "final": {k: round(final.get(k, 0), 4) for k in ("x", "y", "z")},
        "jumps": jumps, **st,
        "stop_error_m": stop_err, "crossed_face": crossed,
    }
    res["ok"] = bool(
        st["max_coord"] is not None and stop_err is not None and stop_err <= TOL_STOP
        and not crossed
        and st["z_min"] is not None and abs(st["z_min"] - Z_FLOOR) <= TOL_Z
        and abs(st["z_max"] - Z_FLOOR) <= TOL_Z
        and (st["drift_last_2s_m"] is None or st["drift_last_2s_m"] <= TOL_DRIFT_CONTACT)
    )
    print("  max %s=%.4f  期望停 %.4f  误差=%.4f  越界=%s  z=[%.4f,%.4f]  贴住后 2 s 漂移=%s  ok=%s"
          % (axis, res["max_coord"], expected, stop_err, crossed,
             st["z_min"], st["z_max"], st["drift_last_2s_m"], res["ok"]))
    rm(h, "waffle")
    return res


def test_platform(h, geo):
    p = geo.get("platform")
    if not p:
        return {"ok": False, "error": "box_collisions.json 里没有候选平台"}
    face = p["south_face_y"]
    print("\n[E] 撞绿平台（%.2f m 高）：车道 x=%.2f，南侧面 y=%.3f ..."
          % (p["level"], p["lane_x"], face))
    ok, p0 = spawn_robot(h, p["spawn_x"], p["spawn_y"], math.pi / 2)
    time.sleep(1.0)
    pub, _ = make_pub(h)
    track, jumps = push(h, "waffle", PUSH_SPEED, 6.0, pub=pub)
    final = h.pose("waffle") or {}
    st = stats_along(track, "y")
    expected = round(face - TB3_FRONT_FROM_CENTER, 4)
    stop_err = round(abs(st["max_coord"] - expected), 4) if st["max_coord"] is not None else None
    crossed = st["max_coord"] is not None and st["max_coord"] > face - TOL_PENETRATE
    climbed = st["z_max"] is not None and st["z_max"] > Z_FLOOR + 0.05
    res = {
        "spawn_ok": ok, "level": p["level"], "face_y": face,
        "expected_stop_center": expected,
        "final": {k: round(final.get(k, 0), 4) for k in ("x", "y", "z")},
        "jumps": jumps, **st,
        "stop_error_m": stop_err, "crossed_face": crossed, "climbed_onto_platform": climbed,
    }
    res["ok"] = bool(
        st["max_coord"] is not None and stop_err is not None and stop_err <= TOL_STOP
        and not crossed and not climbed
        and st["z_min"] is not None and abs(st["z_min"] - Z_FLOOR) <= TOL_Z
    )
    print("  max y=%.4f  期望停 %.4f  误差=%.4f  越界=%s  爬上台=%s  z=[%.4f,%.4f]  ok=%s"
          % (res["max_coord"], expected, stop_err, crossed, climbed,
             st["z_min"], st["z_max"], res["ok"]))
    rm(h, "waffle")
    return res


def main():
    if not os.path.exists(TB3_SDF):
        print("!! 找不到 TB3 模型:", TB3_SDF)
        return 1
    if not os.path.exists(GEO_JSON):
        print("!! 缺 debug/box_collisions.json —— 先跑 python3 debug/make_box_collisions.py")
        return 1
    geo = json.load(open(GEO_JSON, encoding="utf-8"))["stats"]
    tg = geo["test_geometry"]
    print("场地几何（来自 box 生成脚本）:", json.dumps(tg, ensure_ascii=False)[:400])

    h = Harness()
    if not h.wait_service(f"/world/{WORLD}/create/blocking", 30):
        print("!! 等不到 Gazebo create 服务，服务器没起来？")
        return 1
    t0 = time.time()
    while time.time() - t0 < 30 and "rmul2027_3v3_field_up" not in h.poses:
        time.sleep(0.3)
    print("world ready:", WORLD, "| 场地模型在场:",
          "rmul2027_3v3_field_up" in h.poses)
    time.sleep(1.0)

    report = {
        "world": WORLD,
        "geometry": {
            "n_boxes": geo["n_boxes"],
            "occupied_area_m2": geo["occupied_area_m2"],
            "levels": [round(lv, 4) for lv in geo["levels"]],
            "height_err_m": geo["height_err_m"],
            "east_inner_x": tg["east_inner_x"],
            "north_inner_y": tg["north_inner_y"],
            "platform": tg.get("platform", {}).get("level"),
        },
        "checks": {},
    }
    report["checks"]["A_static"] = test_static(h)
    report["checks"]["B_forward"] = test_forward(h)
    report["checks"]["C_east_wall"] = test_wall(
        h, "x", tg["east_inner_x"], (5.0, 0.0), 0.0)
    report["checks"]["D_north_wall"] = test_wall(
        h, "y", tg["north_inner_y"], (0.0, 3.0), math.pi / 2)
    report["checks"]["E_platform"] = test_platform(h, tg)

    ok = all(c.get("ok") for c in report["checks"].values())
    report["summary"] = {"ok": bool(ok)}
    path = os.path.join(RES, "report_box_collisions.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    print("\n===== 摘要 =====")
    print(json.dumps({k: {kk: v.get(kk) for kk in ("ok", "stop_error_m", "z_min", "z_max")}
                      for k, v in report["checks"].items()}, indent=2, ensure_ascii=False))
    print("报告:", path)
    if not ok:
        print("\n!! 有判据未通过，逐项看上面日志")
    return 0 if ok else 3


if __name__ == "__main__":
    raise SystemExit(main())