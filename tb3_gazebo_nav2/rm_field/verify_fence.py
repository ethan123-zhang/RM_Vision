#!/usr/bin/env python3
"""围挡（0.30 m 高 / 5 cm 厚）碰撞验收。

前置：`bash start_server.sh $PWD/worlds/rmul2027_3v3_field_fenced.sdf fenced`
（带围挡世界；1:1 图纸世界没有围挡，本脚本对它没有意义）

三项，全部是读数断言，不是"看着像"：

  A. 落体探针落在围挡**顶上**：4 个探针从 z=0.70 落在四面围挡正中，
     落地后 surface_z 应 = +0.30 m（围挡顶面）。这证明围挡的 collision 真的存在，
     且位置/高度与 visual 一致（只有 visual 的话探针会穿过去落到台面 z=0）。

  B. 场外探针仍正常掉到地面：x=6.40 / y=4.40 的探针 surface_z 应 = -0.503 m。
     这证明围挡外侧没有"幽灵碰撞"，场外空间仍是空的。

  C. TB3 以 0.7 m/s 正撞东围挡、北围挡，各自**持续推 5 s**：
     - 机器人中心必须始终 < 围挡内侧面（x < 6.25 / y < 4.25）→ 没有越界、没有骑上围挡；
     - z 必须始终 > -0.05 → 没有从围挡上翻出去、没有掉下 0.503 m；
     - 撞上后继续推 5 s，中心位移应 < 5 cm（没有慢慢蹭过去/爬上去）。

注：这里**不改** verify_field.py 的边缘探针判据。1:1 世界（无围挡）仍是验收基线，
    它的"探针掉下去、落差 0.503 m"结论保持不变；围挡是图纸外的补丁，单独验收。
"""
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gz.msgs10.boolean_pb2 as bool_pb2          # noqa: E402
import gz.msgs10.entity_pb2 as entity_pb2        # noqa: E402
import gz.msgs10.twist_pb2 as twist_pb2          # noqa: E402

from verify_field import (Harness, TB3_SDF, WORLD, FIELD_LEN, FIELD_WID,  # noqa: E402
                          DROP_Z, probe_sdf)

RES = os.path.join(HERE, "results")
os.makedirs(RES, exist_ok=True)

FENCE_H = 0.30            # 围挡高度（台面以上）
FENCE_T = 0.05            # 围挡厚度
FENCE_X = FIELD_LEN / 2 - FENCE_T / 2       # 围挡中心线 6.275
FENCE_Y = FIELD_WID / 2 - FENCE_T / 2       # 4.275
FENCE_INNER_X = FENCE_X - FENCE_T / 2       # 内侧面 6.250
FENCE_INNER_Y = FENCE_Y - FENCE_T / 2       # 4.250
GROUND_Z = -0.503

PROBE_SZ = 0.03
PROBE_FALL_Z = 0.70       # 比围挡顶(0.30)高 0.40 m 落下
PUSH_SPEED = 0.7
PUSH_EXTRA_S = 5.0
# TB3 waffle 底盘碰撞盒（turtlebot3_gazebo/model.sdf）：0.265 m 见方、中心 pose x=-0.064
# → 车头外缘比中心前 0.265/2 - 0.064 = 0.0685 m。撞停时中心应在 inner_face - 0.0685。
TB3_BOX_LEN = 0.265
TB3_BOX_XOFF = -0.064
TB3_FRONT_FROM_CENTER = TB3_BOX_LEN / 2 + TB3_BOX_XOFF      # 0.0685 m
STOP_TOL = 0.02

TOL_TOP = 0.02            # 围挡顶面读数容差
TOL_GROUND = 0.02
TOL_CENTER = 0.01         # 越界判据留 1 cm（内侧面 6.250 → 上限 6.240）
TOL_DRIFT_AFTER_CONTACT = 0.05


def rm(h, name):
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
    h.poses.pop(name, None)          # 清掉旧实体的陈旧位姿，否则会读到上一轮的位置
    time.sleep(0.5)
    ok = h.create(name, sdf_filename=TB3_SDF, xyz=(x, y, 0.25), yaw=yaw)
    if not ok:
        h.create(name, sdf_filename=TB3_SDF, xyz=(x, y, 0.25), yaw=yaw)
    t0 = time.time()
    p = None
    while time.time() - t0 < 8.0:
        p = h.pose(name)
        if p is not None and abs(p["x"] - x) < 0.05 and abs(p["y"] - y) < 0.05:
            break
        time.sleep(0.1)
    return ok, p


# ---------------------------------------------------------------- A/B 探针
def run_fence_probes(h):
    pdir = os.path.join(RES, "probe_sdf")
    os.makedirs(pdir, exist_ok=True)
    # 围挡顶面正中（四面各 1 个）+ 场外各 1 个（证明外侧没有幽灵碰撞）
    plan = [
        ("pf_east",  FENCE_X, 0.0, "fence_top"),
        ("pf_west", -FENCE_X, 0.0, "fence_top"),
        ("pf_north", 0.0,  FENCE_Y, "fence_top"),
        ("pf_south", 0.0, -FENCE_Y, "fence_top"),
        ("po_east",  6.40, 0.0, "outside"),
        ("po_north", 0.0, 4.40, "outside"),
    ]
    print(f"\n[A/B] 围挡探针 {len(plan)} 个（围挡顶 4 + 场外 2）...")
    for name, x, y, kind in plan:
        path = os.path.join(pdir, name + ".sdf")
        if not os.path.exists(path):
            with open(path, "w") as f:
                f.write('<?xml version="1.0"?>\n' + probe_sdf(name) + "\n")
        h.create(name, sdf_filename=path, xyz=(x, y, PROBE_FALL_Z))
        time.sleep(0.15)
    time.sleep(4.0)

    rows = []
    for name, x, y, kind in plan:
        p = h.pose(name)
        if p is None:
            rows.append({"name": name, "kind": kind, "spawn": [x, y], "missing": True})
            continue
        surface = p["z"] - PROBE_SZ / 2.0
        rows.append({
            "name": name, "kind": kind, "spawn": [x, y],
            "final": [round(p["x"], 4), round(p["y"], 4), round(p["z"], 4)],
            "surface_z": round(surface, 4),
        })
    return rows


def evaluate_probes(rows):
    top = [r for r in rows if r["kind"] == "fence_top" and "surface_z" in r]
    out = [r for r in rows if r["kind"] == "outside" and "surface_z" in r]
    ev = {}
    if top:
        zs = [r["surface_z"] for r in top]
        ev["fence_top"] = {
            "n": len(zs), "target_z": FENCE_H,
            "measured_z": [round(z, 4) for z in zs],
            "max_dev": round(max(abs(z - FENCE_H) for z in zs), 4),
            "ok": all(abs(z - FENCE_H) <= TOL_TOP for z in zs),
        }
    if out:
        zs = [r["surface_z"] for r in out]
        ev["outside"] = {
            "n": len(zs), "target_z": GROUND_Z,
            "measured_z": [round(z, 4) for z in zs],
            "max_dev": round(max(abs(z - GROUND_Z) for z in zs), 4),
            "ok": all(abs(z - GROUND_Z) <= TOL_GROUND for z in zs),
        }
    return ev


# ---------------------------------------------------------------- C 撞墙
def drive_into_fence(h, axis):
    """axis='x' 撞东围挡；axis='y' 撞北围挡。返回 dict。"""
    inner = FENCE_INNER_X if axis == "x" else FENCE_INNER_Y
    yaw = 0.0 if axis == "x" else math.pi / 2
    ok, p0 = spawn_robot(h, 0.0, 0.0, yaw)
    print(f"\n[C-{axis}] 生成机器人 ok={ok} pose={p0}")
    if p0 is None:
        return {"error": "spawn failed", "axis": axis}

    topics = [t for t in h.node.topic_list() if t.endswith("/cmd_vel")]
    cmd_topic = "/cmd_vel" if "/cmd_vel" in topics else (topics[0] if topics else "/cmd_vel")
    pub = h.node.advertise(cmd_topic, twist_pb2.Twist)
    time.sleep(0.6)

    # 差速车只会沿车头方向前进：撞哪个方向的墙由 spawn 时的 yaw 决定
    tw = twist_pb2.Twist()
    tw.linear.x = PUSH_SPEED

    track = []
    jumps = []
    prev = None
    dur = (FIELD_LEN / 2) / PUSH_SPEED + PUSH_EXTRA_S + 2.0   # 到墙 + 多推 5 s
    t0 = time.time()
    pub.publish(tw)
    while time.time() - t0 < dur:
        p = h.pose("waffle")
        if p is not None:
            row = (round(time.time() - t0, 3), p["x"], p["y"], p["z"])
            if prev is not None and (abs(row[1] - prev[1]) + abs(row[2] - prev[2])) > 0.15:
                jumps.append({"t": row[0], "from": [round(prev[1], 3), round(prev[2], 3),
                                                   round(prev[3], 3)],
                              "to": [round(row[1], 3), round(row[2], 3), round(row[3], 3)]})
            track.append(row)
            prev = row
        time.sleep(0.05)
    pub.publish(twist_pb2.Twist())
    time.sleep(1.5)

    final = dict(h.pose("waffle"))
    coord = 1 if axis == "x" else 2          # track 元组里 x 是 idx1，y 是 idx2
    vals = [t[coord] for t in track]
    zs = [t[3] for t in track]
    cross = next((t[0] for t in track if t[coord] > inner), None)
    maxv = max(vals) if vals else None

    # "撞上后还在推"的证据：取轨迹最后 3 s（此时仍在发 0.7 m/s 指令）的位移。
    # 先剔掉姿态瞬时跳变样本（0.05 s 内位移 >0.15 m，对应速度 >3 m/s，物理上不可能），
    # 跳变已记录在 jumps 里，不隐藏证据。
    clean = []
    for row in track:
        if clean and (abs(row[1] - clean[-1][1]) + abs(row[2] - clean[-1][2])) > 0.15:
            continue
        clean.append(row)
    drift = None
    if len(clean) >= 2:
        last = clean[-1][0]
        seg = [t for t in clean if t[0] >= last - 3.0]
        if len(seg) >= 2:
            a, b = seg[0], seg[-1]
            drift = round(math.hypot(b[1] - a[1], b[2] - a[2]), 4)

    # 撞停位置应 = 围挡内侧面 - 车头外缘距中心（几何预测 ↔ 实测）
    expected_stop = round(inner - TB3_FRONT_FROM_CENTER, 4)
    stop_error = round(abs(maxv - expected_stop), 4) if maxv is not None else None
    gap_to_inner = round(inner - maxv, 4) if maxv is not None else None

    res = {
        "axis": axis, "spawn_ok": ok, "cmd_topic": cmd_topic,
        "fence_inner_limit": inner,
        "max_coord": round(maxv, 4) if maxv is not None else None,
        "crossed_inner": cross is not None,
        "expected_stop_center": expected_stop,
        "stop_error_m": stop_error,
        "gap_center_to_inner_face_m": gap_to_inner,
        "final": {k: round(final[k], 4) for k in ("x", "y", "z")},
        "z_min": round(min(zs), 4), "z_max": round(max(zs), 4),
        "drift_last_3s_while_pushing_m": drift,
        "jumps": jumps,
        "n_track": len(track),
        "track": [{"t": t[0], "x": round(t[1], 3), "y": round(t[2], 3), "z": round(t[3], 3)}
                  for t in track],
    }
    res["ok"] = bool(
        maxv is not None and maxv < inner - TOL_CENTER
        and min(zs) > -0.05 and final["z"] > -0.05
        and maxv > inner - 0.25            # 真的开到了围挡附近（碰撞确实发生过）
        and stop_error is not None and stop_error <= STOP_TOL
        and (drift is None or drift <= TOL_DRIFT_AFTER_CONTACT)
    )
    print(f"  [C-{axis}] {axis}_max={res['max_coord']:.4f} (limit {inner - TOL_CENTER:.3f}) "
          f"crossed={res['crossed_inner']} final=({final['x']:.3f},{final['y']:.3f},"
          f"{final['z']:.3f}) z_min={res['z_min']:.3f} drift(3s)={drift} "
          f"stop_err={stop_error} ok={res['ok']}")
    rm(h, "waffle")
    time.sleep(0.8)
    return res


def main():
    if not os.path.exists(TB3_SDF):
        print("!! 找不到 TB3 模型:", TB3_SDF)
        return 1
    h = Harness()
    if not h.wait_service(f"/world/{WORLD}/create/blocking", 25):
        print("!! 等不到 Gazebo create 服务，服务器没起来？")
        return 1
    t0 = time.time()
    while time.time() - t0 < 30:
        if "rmul2027_3v3_fence" in h.poses:
            break
        time.sleep(0.3)
    has_fence = "rmul2027_3v3_fence" in h.poses
    print("world ready:", WORLD, "| 围挡模型在场:", has_fence)
    if not has_fence:
        print("!! 这个世界没有围挡（应该是 1:1 世界）。请用带围挡世界起服务器：")
        print("   bash start_server.sh $PWD/worlds/rmul2027_3v3_field_fenced.sdf fenced")
        return 2
    time.sleep(1.0)

    report = {
        "world": WORLD,
        "fence": {"height": FENCE_H, "thickness": FENCE_T,
                  "center_line_x": FENCE_X, "center_line_y": FENCE_Y,
                  "inner_face_x": FENCE_INNER_X, "inner_face_y": FENCE_INNER_Y},
        "probes": run_fence_probes(h),
    }
    report["probe_eval"] = evaluate_probes(report["probes"])
    print("\n探针判断:", json.dumps(report["probe_eval"], ensure_ascii=False))

    tests = []
    for axis in ("x", "y"):
        try:
            tests.append(drive_into_fence(h, axis))
        except Exception as e:                                # noqa: BLE001
            tests.append({"axis": axis, "error": f"{type(e).__name__}: {e}"})
            print(f"  !! C-{axis} 异常:", e)
    report["push_tests"] = tests

    ok = (report["probe_eval"].get("fence_top", {}).get("ok")
          and report["probe_eval"].get("outside", {}).get("ok")
          and all(t.get("ok") for t in tests))
    report["summary"] = {"ok": bool(ok)}
    path = os.path.join(RES, "report_fence.json")
    with open(path, "w") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    print("\n===== 摘要 =====")
    print(json.dumps({"probe_eval": report["probe_eval"],
                      "push": [{k: t.get(k) for k in
                                ("axis", "max_coord", "crossed_inner", "z_min", "ok")}
                               for t in tests],
                      "summary": report["summary"]}, indent=2, ensure_ascii=False))
    print("\n报告:", path)
    return 0 if ok else 3


if __name__ == "__main__":
    raise SystemExit(main())