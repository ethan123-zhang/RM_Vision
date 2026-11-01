#!/usr/bin/env python3
"""RMUL2027 3V3 场地在 Gazebo 里的「比例 + 碰撞」验收。

前置：gz sim -s -r worlds/rmul2027_3v3_field.sdf 已在跑（start_server.sh）。

测试分两步：
  A. 落体探针（28 个 3 cm 立方体）：量出场地台面高度、场地边界、台边落差。
     -> 比例：12.6 x 8.6 m 是否真的落在 ±6.30 / ±4.30
     -> 碰撞：台面是否挡得住、台外是否真的掉下去
  B. TurtleBot3 waffle：静置 / 直行 / 冲出场地边缘
     -> 碰撞：会不会穿地、抖动、被看不见的东西挡住
     -> 比例：机器人行程与场地尺寸是否自洽

输出：results/report_field_test.json + 控制台摘要
"""
import json
import math
import os
import time

import gz.msgs10.boolean_pb2 as bool_pb2
import gz.msgs10.entity_factory_pb2 as ef_pb2
import gz.msgs10.odometry_pb2 as odom_pb2
import gz.msgs10.pose_v_pb2 as pose_v_pb2
import gz.msgs10.twist_pb2 as twist_pb2
from gz.transport13 import Node

WORLD = os.environ.get("GZ_WORLD", "rmul2027_3v3")
ROOT = os.environ.get("FIELD_ROOT", "/mnt/e/RM/tb3_gazebo_nav2/rm_field")
RES = os.path.join(ROOT, "results")
TAG = os.environ.get("REPORT_TAG", "")
os.makedirs(RES, exist_ok=True)

# 图纸实测值（由 STEP -> 网格 得到，见 report_geometry.json）
FIELD_LEN, FIELD_WID, FIELD_H = 12.600, 8.600, 0.503
TOP_Z = 0.0
GROUND_Z = -FIELD_H

PROBE_SZ = 0.03          # 探针立方体边长
PROBE_MASS = 0.01
DROP_Z = 0.30

TB3_SDF = ("/opt/ros/jazzy/share/turtlebot3_gazebo/models/"
           "turtlebot3_waffle/model.sdf")


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class Harness:
    def __init__(self, world=WORLD):
        self.world = world
        self.node = Node()
        self.poses = {}
        self.odom = None
        self.node.subscribe(pose_v_pb2.Pose_V,
                            f"/world/{world}/pose/info", self._on_pose)

    # ---------- 基础 ----------
    def _on_pose(self, msg):
        for p in msg.pose:
            self.poses[p.name] = {
                "x": p.position.x, "y": p.position.y, "z": p.position.z,
                "w": p.orientation.w, "xq": p.orientation.x,
                "yq": p.orientation.y, "zq": p.orientation.z,
            }

    def wait_service(self, name, timeout=20.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if name in self.node.service_list():
                return True
            time.sleep(0.2)
        return False

    def create(self, name, sdf="", sdf_filename="", xyz=(0, 0, 0), yaw=0.0,
               blocking=True, timeout=15000, retries=2):
        req = ef_pb2.EntityFactory()
        req.name = name
        req.sdf = sdf
        req.sdf_filename = sdf_filename
        req.allow_renaming = False
        req.pose.position.x = float(xyz[0])
        req.pose.position.y = float(xyz[1])
        req.pose.position.z = float(xyz[2])
        qx, qy, qz, qw = quat_from_yaw(yaw)
        req.pose.orientation.x = qx
        req.pose.orientation.y = qy
        req.pose.orientation.z = qz
        req.pose.orientation.w = qw
        svc = f"/world/{self.world}/create" + ("/blocking" if blocking else "")
        for attempt in range(retries + 1):
            ok, rep = self.node.request(svc, req, ef_pb2.EntityFactory,
                                        bool_pb2.Boolean, timeout)
            if ok and rep is not None and rep.data:
                return True
            # 服务偶发在"世界还在装载"时返回 false，但实体其实已排队生成：
            # 以 pose/info 是否出现为准，必要时重试一次。
            time.sleep(0.8)
            if name in self.poses:
                return True
        return False

    def pose(self, name):
        return self.poses.get(name)

    def wait_pose(self, name, timeout=5.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if name in self.poses:
                return self.poses[name]
            time.sleep(0.05)
        return None

    def settle(self, seconds):
        time.sleep(seconds)

    def sim_time(self):
        """用世界时钟（迭代计数）粗测，避免依赖 stats 插件。"""
        return self.poses


# ======================================================================
# A. 落体探针
# ======================================================================
def probe_sdf(name):
    return f"""<sdf version="1.9"><model name="{name}"><static>false</static>
  <link name="l"><inertial><mass>{PROBE_MASS}</mass>
    <inertia><ixx>1e-7</ixx><iyy>1e-7</iyy><izz>1e-7</izz></inertia></inertial>
    <collision name="c"><geometry><box><size>{PROBE_SZ} {PROBE_SZ} {PROBE_SZ}</size></box></geometry>
      <surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction>
        <bounce><restitution_coefficient>0.0</restitution_coefficient></bounce></surface>
    </collision>
    <visual name="v"><geometry><box><size>{PROBE_SZ} {PROBE_SZ} {PROBE_SZ}</size></box></geometry>
      <material><ambient>1 0.3 0.1 1</ambient><diffuse>1 0.3 0.1 1</diffuse></material></visual>
  </link></model></sdf>"""


PROBES = []


def add_probe(name, x, y):
    PROBES.append((name, x, y))


def build_probe_plan():
    # 1) 台面平整度：散布在场地各处（含中心 3x3 m 区域）
    flat_pts = [(0.0, 0.0), (3.0, 2.0), (-3.0, -2.0), (5.5, 3.5), (-5.5, -3.5),
                (2.0, -3.0), (-2.0, 3.0), (4.0, 3.9), (-4.0, -3.9), (0.0, 2.5),
                (0.0, -2.5), (1.2, 0.0), (-1.2, 0.0)]
    idx = 0
    for (x, y) in flat_pts:
        add_probe(f"p_flat_{idx:02d}", x, y)
        idx += 1
    # 2) X 向边界扫描：中心经过 x≈6.30 时是否掉下去；用不同 y 错开避免互相碰撞
    xs = [6.24, 6.26, 6.28, 6.29, 6.30, 6.31, 6.32, 6.34, 6.36]
    for i, x in enumerate(xs):
        add_probe(f"p_edgex_{i:02d}", x, -3.0 + i * 0.5)
    # 3) Y 向边界扫描：中心经过 y≈4.30
    ys = [4.24, 4.26, 4.28, 4.29, 4.30, 4.31, 4.32, 4.34, 4.36]
    for i, y in enumerate(ys):
        add_probe(f"p_edgey_{i:02d}", -3.0 + i * 0.5, y)


def run_probe_test(h):
    build_probe_plan()
    pdir = os.path.join(RES, "probe_sdf")
    os.makedirs(pdir, exist_ok=True)
    print(f"\n[A] 落体探针 {len(PROBES)} 个 ...")
    # 注意：gz.msgs10 的 Python 绑定里 EntityFactory.sdf(string) 字段序列化会丢
    # （实测赋值能读回、序列化后为空），所以统一落盘用 sdf_filename 生成。
    for name, x, y in PROBES:
        path = os.path.join(pdir, name + ".sdf")
        if not os.path.exists(path):
            with open(path, "w") as f:
                f.write('<?xml version="1.0"?>\n' + probe_sdf(name) + "\n")
        t0 = time.time()
        ok = h.create(name, sdf_filename=path, xyz=(x, y, DROP_Z))
        if not ok:
            print(f"  !! spawn failed: {name} ({time.time()-t0:.1f}s)")
            time.sleep(0.2)
    print("  等待 4 s 让全部探针落地 ...")
    time.sleep(4.0)

    rows = []
    for name, x, y in PROBES:
        p = h.pose(name)
        if p is None:
            rows.append({"name": name, "spawn": [x, y], "missing": True})
            continue
        p1 = dict(p)
        time.sleep(0.05)
        p2 = h.pose(name)
        rows.append({
            "name": name, "spawn": [x, y],
            "final": [round(p1["x"], 5), round(p1["y"], 5), round(p1["z"], 5)],
            "surface_z": round(p1["z"] - PROBE_SZ / 2.0, 5),
            "jitter": round(abs(p2["z"] - p1["z"]), 6) if p2 else None,
        })
    return rows


def evaluate_probes(rows):
    flat = [r for r in rows if r["name"].startswith("p_flat") and "surface_z" in r]
    ex = [r for r in rows if r["name"].startswith("p_edgex") and "surface_z" in r]
    ey = [r for r in rows if r["name"].startswith("p_edgey") and "surface_z" in r]
    out = {}
    if flat:
        zs = [r["surface_z"] for r in flat]
        out["flat"] = {
            "n": len(zs), "min": min(zs), "max": max(zs),
            "mean": round(sum(zs) / len(zs), 5),
            "max_dev_from_0": round(max(abs(z) for z in zs), 5),
        }
    for key, group, axis, limit in (("edge_x", ex, 0, FIELD_LEN / 2),
                                    ("edge_y", ey, 1, FIELD_WID / 2)):
        if not group:
            continue
        group = sorted(group, key=lambda r: r["spawn"][axis])
        on = [r for r in group if r["surface_z"] > -0.1]
        off = [r for r in group if r["surface_z"] <= -0.1]
        out[key] = {
            "limit_expected": limit,
            "on_field_last": on[-1]["spawn"][axis] if on else None,
            "off_field_first": off[0]["spawn"][axis] if off else None,
            "measured_boundary": (round((on[-1]["spawn"][axis] + off[0]["spawn"][axis]) / 2, 4)
                                  if on and off else None),
            "on_field_surface_z": round(max(r["surface_z"] for r in on), 4) if on else None,
            "off_field_surface_z": round(min(r["surface_z"] for r in off), 4) if off else None,
            "drop_height_measured": (round(max(r["surface_z"] for r in on)
                                           - min(r["surface_z"] for r in off), 4)
                                     if on and off else None),
            "samples": [{"at": r["spawn"][axis], "surface_z": r["surface_z"], "final": r["final"]}
                        for r in group],
        }
    return out


# ======================================================================
# B. 机器人测试
# ======================================================================
def run_robot_test(h):
    print("\n[B] TurtleBot3 waffle ...")
    ok = h.create("waffle", sdf_filename=TB3_SDF, xyz=(0.0, 0.0, 0.25))
    print("  spawn:", ok)
    time.sleep(0.5)
    print("  poses with 'waffle':", [n for n in h.poses if "waffle" in n][:8])
    robot = "waffle"
    if robot not in h.poses:
        cand = [n for n in h.poses if "waffle" in n and "::" not in n]
        robot = cand[0] if cand else robot
        h.wait_pose(robot, 5.0)
    print("  model name:", robot)

    topics = []
    for _ in range(30):
        topics = [t for t in h.node.topic_list() if t.endswith("/cmd_vel")]
        if topics:
            break
        time.sleep(0.2)
    cmd_topic = "/cmd_vel" if "/cmd_vel" in topics else (topics[0] if topics else "/cmd_vel")
    print("  cmd_vel candidates:", topics, "-> use", cmd_topic)
    pub = h.node.advertise(cmd_topic, twist_pb2.Twist)
    time.sleep(0.5)

    def drive(vx, wz, seconds, hz=20.0):
        """边发速度指令边记轨迹，返回 [(t, x, y, z), ...]"""
        tw = twist_pb2.Twist()
        tw.linear.x = float(vx)
        tw.angular.z = float(wz)
        track = []
        t0 = time.time()
        while time.time() - t0 < seconds:
            pub.publish(tw)
            p = h.pose(robot)
            if p:
                track.append((time.time() - t0, p["x"], p["y"], p["z"]))
            time.sleep(1.0 / hz)
        pub.publish(twist_pb2.Twist())
        return track

    # B1 静置：确认没有陷进场地、没有抖动
    time.sleep(2.0)
    rest = dict(h.pose(robot))
    time.sleep(1.0)
    rest2 = dict(h.pose(robot))
    drift = math.dist([rest["x"], rest["y"], rest["z"]],
                      [rest2["x"], rest2["y"], rest2["z"]])
    print(f"  B1 静置 pose z={rest['z']:.4f} 1s 漂移={drift*1000:.2f} mm")

    # B2 直行 3 s @0.3 m/s
    start = dict(h.pose(robot))
    track = drive(0.3, 0.0, 3.0)
    time.sleep(1.0)
    end = dict(h.pose(robot))
    dist = math.hypot(end["x"] - start["x"], end["y"] - start["y"])
    zs = [t[3] for t in track] or [start["z"]]
    print(f"  B2 直行 3s: 位移 {dist:.3f} m (expect ≈0.9), z范围 "
          f"[{min(zs):.4f},{max(zs):.4f}]")

    # B3 冲出场地边缘 @0.7 m/s
    track2 = drive(0.7, 0.0, 14.0)
    time.sleep(1.5)
    final = dict(h.pose(robot))
    zs2 = [t[3] for t in track2] or [final["z"]]
    cross_t = next((t for t, x, y, z in track2 if x > FIELD_LEN / 2), None)
    tip_t = next((t for t, x, y, z in track2 if z < 0.0), None)
    fall = None
    for t, x, y, z in track2:
        if z < -0.15:
            fall = {"t": round(t, 3), "x": round(x, 3), "z": round(z, 3)}
            break
    print(f"  B3 冲出边缘: 中心过 {FIELD_LEN/2:.2f} m @t={cross_t}, "
          f"开始下坠 @t={tip_t}, z<-0.15 @t={None if not fall else fall['t']}, "
          f"final=({final['x']:.2f},{final['z']:.3f}), min z={min(zs2):.3f}")

    return {
        "model": robot, "cmd_topic": cmd_topic,
        "spawn_ok": ok,
        "B1_rest": {"z": round(rest["z"], 5), "drift_1s_m": round(drift, 5),
                    "expected_z": 0.010},
        "B2_drive": {"distance_m": round(dist, 4), "expected_m": 0.9,
                     "mean_speed_mps": round(dist / 3.0, 3),
                     "z_min": round(min(zs), 5), "z_max": round(max(zs), 5)},
        "B3_edge": {"cross_boundary_t": None if cross_t is None else round(cross_t, 3),
                    "tip_t": None if tip_t is None else round(tip_t, 3),
                    "fall": fall,
                    "final": {"x": round(final["x"], 3),
                              "y": round(final["y"], 3),
                              "z": round(final["z"], 3)},
                    "min_z": round(min(zs2), 4),
                    "expected_final_z": round(GROUND_Z + 0.010, 3),
                    "track_after_x6": [{"x": round(t[1], 2), "z": round(t[3], 3)}
                                       for t in track2 if t[1] > 6.0]},
    }


def main():
    if not os.path.exists(TB3_SDF):
        print("!! 找不到 TB3 模型:", TB3_SDF)
        return 1
    h = Harness()
    if not h.wait_service(f"/world/{WORLD}/create/blocking", 20):
        print("!! 等不到 Gazebo create 服务，服务器没起来？")
        return 1
    print("Gazebo 世界已就绪:", WORLD)
    # 等世界真正装载完（场地模型出现在 pose/info 里），避免首个 spawn 撞上装载竞态
    t0 = time.time()
    while time.time() - t0 < 30:
        if "rmul2027_3v3_field" in h.poses:
            break
        time.sleep(0.3)
    print("  场地模型已在世界中:", "rmul2027_3v3_field" in h.poses,
          f"(等待 {time.time()-t0:.1f}s)")
    time.sleep(1.0)

    rows = []
    robot = {}
    # 先跑机器人（场地上没有探针，避免干扰直线/边缘测试），再撒探针测几何
    try:
        robot = run_robot_test(h)
    except Exception as e:  # noqa: BLE001
        robot = {"error": f"{type(e).__name__}: {e}"}
        print("!! 机器人测试异常:", robot["error"])
    try:
        rows = run_probe_test(h)
        ev = evaluate_probes(rows)
    except Exception as e:  # noqa: BLE001
        ev = {"error": f"{type(e).__name__}: {e}"}
        print("!! 探针测试异常:", ev["error"])

    report = {
        "world": WORLD,
        "variant": TAG or "box",
        "field_expected_m": [FIELD_LEN, FIELD_WID, FIELD_H],
        "probes": rows,
        "evaluation": ev,
        "robot": robot,
    }
    path = os.path.join(RES, f"report_field_test{TAG}.json")
    with open(path, "w") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    print("\n===== 摘要 =====")
    print(json.dumps({"evaluation": ev, "robot": robot}, indent=2, ensure_ascii=False)[:4000])
    print("\n报告:", path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())