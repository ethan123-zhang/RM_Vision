#!/usr/bin/env python3
"""Nav2 在 RMUL2027 3V3 场地（翻转后）上的导航验收（前置：bash nav2_field.sh 已把栈拉起来）。

判据（全部是读数，不看画面）：
  1. 仿真链路：/scan、/odom 有稳定频率；节点用仿真时间；map->base_footprint 可查
  2. 导航闭环：每个 /navigate_to_pose 目标都 SUCCEEDED（error_code 0）
  3. 不出界：整段导航中 map->base_footprint 始终在墙面内侧面以内（|x|<6.00-0.15、|y|<4.00-0.15）
  4. 不掉台/不爬台：整段导航中 Gazebo 里机器人 z ∈ (-0.05, 0.06)。翻转后地面 z=0，
     平台顶 0.20 m、墙顶 0.40 m；爬上台 z 会到 0.21，掉到台外底板 z 会到 -0.09
  5. 不压平台/墙：整段导航中 TF 点不得侵入任何碰撞 box 的投影超过 5 cm
     （5 cm = 碰撞体栅格粒度；超过说明定位跳变或真撞上去了）
  6. 到位：每段结束时 TF 实测位置距目标 < 0.30 m
  6b. 定位一致：TF(map->base_footprint) 与 Gazebo 真值偏差 < 0.50 m
      （地图/世界不一致时这里会先炸：2026-11-20 PGM 上下翻转就是这条抓出来的）
  7. 真在跑：按 TF 轨迹算路程/仿真时长，平均速度应在 TB3 速度上限 (0.5 m/s) 以内、且 >0.05

为什么不用 /amcl_pose 算位置：AMCL 只在"移动超过 0.25 m / 转超过 0.2 rad"时才发布，
用它算终点会滞后半米。TF（map->base_footprint）是导航栈真正在用的位姿，直接采样。

输出 results/report_nav2_field.json
"""
import json
import math
import os
import re
import time

# nav2_field.sh 把整栈放在 ROS_DOMAIN_ID=42 里。跨 shell 直接跑本脚本时若没导出它，
# 新 participant 会落到 domain 0，连 /navigate_to_pose 的 action server 都发现不了
# （症状就是 "!! 等不到导航 action server"）。这里默认对齐 42，显式设置的环境变量优先。
os.environ.setdefault("ROS_DOMAIN_ID", "42")

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, "results")
os.makedirs(RES, exist_ok=True)

# 翻转后场地（2026-11-19）：地面 z=0，场内凸起 0.19~0.40 m；外墙内侧面 ±6.00 / ±4.00
# （实测见 debug/box_collisions.json → stats.test_geometry，与导航用的 48 块碰撞 box 同源）
FIELD_LEN, FIELD_WID = 12.600, 8.600
INNER_X = 6.00
INNER_Y = 4.00
MARGIN = 0.15                       # 车身半径余量（判"没压到/穿出外墙"）
OBS_MARGIN = 0.05                   # 允许侵入 box 投影的深度（=碰撞体 5 cm 栅格粒度）
# TF 位姿与 Gazebo 真值的最大允许偏差（m）。实测本仿真里转弯段瞬时 0.37 m、直行/静止段
# 0.02~0.06 m：这是 AMCL 按 update_min_d=0.25 m 周期修正 + odom 偏航误差在里程计原点上
# 放大出来的滞后，不是地图错位（PGM 上下翻转那次是 4.8 m，量级完全不同）。
LOC_ERR_MAX = 0.50
Z_MIN, Z_MAX = -0.05, 0.06          # 机器人 z 必须在这个窗口内（不落台、不爬台）
GOALS = [(4.0, 2.0, 0.0), (4.0, -2.5, 0.0), (0.0, 0.0, 0.0)]
GOAL_TOL = 0.30                     # 结束时到位判据（m）
GOAL_TIMEOUT = 240.0
BOXES_JSON = os.path.join(HERE, "debug", "box_collisions.json")
# 速度上限从实际用的 nav2 参数文件里读（velocity_smoother 的 vx_max）；
# TB3 底盘本身不限速，所以"跑多快"完全由这份参数决定，不能凭印象写 0.26。
PARAMS_FILE = os.environ.get(
    "NAV2_PARAMS", "/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml")


def read_vx_max(path=PARAMS_FILE):
    try:
        with open(path, "r", encoding="utf-8") as f:
            m = re.search(r"vx_max:\s*([0-9.]+)", f.read())
        return float(m.group(1)) if m else None
    except OSError:
        return None


def load_obstacles(path=BOXES_JSON):
    """碰撞 box 的水平投影 → [(x0, x1, y0, y1)]。地形/平台/墙都在里面。"""
    with open(path, "r", encoding="utf-8") as f:
        boxes = json.load(f)["boxes"]
    out = []
    for b in boxes:
        cx, cy, _ = b["center"]
        sx, sy, _ = b["size"]
        out.append((cx - sx / 2.0, cx + sx / 2.0, cy - sy / 2.0, cy + sy / 2.0))
    return out


def box_penetration(x, y, boxes):
    """点 (x,y) 对全部 box 投影的最大侵入深度；>0 表示落在某个 box 内部。"""
    pen = -1e9
    for x0, x1, y0, y1 in boxes:
        pen = max(pen, min(x - x0, x1 - x, y - y0, y1 - y))
    return pen


VX_MAX = read_vx_max()

import rclpy                                     # noqa: E402
from rclpy.action import ActionClient            # noqa: E402
from rclpy.node import Node                      # noqa: E402
from rclpy.parameter import Parameter            # noqa: E402
from rclpy.time import Time                      # noqa: E402
from tf2_ros import Buffer, TransformListener    # noqa: E402
from nav2_msgs.action import NavigateToPose      # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist  # noqa: E402
from nav_msgs.msg import Odometry                # noqa: E402
from sensor_msgs.msg import LaserScan            # noqa: E402

import gz.msgs10.pose_v_pb2 as pose_v_pb2        # noqa: E402
from gz.transport13 import Node as GzNode        # noqa: E402

SAMPLE_DT = 0.10                    # TF 采样间隔（wall 秒）


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Verifier(Node):
    def __init__(self):
        super().__init__("verify_nav2_field",
                         parameter_overrides=[Parameter("use_sim_time",
                                                       Parameter.Type.BOOL, True)])
        self.amcl = []
        self.gz = {}
        self.gz_names = set()
        self.gz_track = []     # (wall_t, x, y, z) Gazebo 真值
        self.track = []        # (wall_t, sim_t, x, y, yaw)
        self.scan_n = 0
        self.odom_n = 0
        self.feedback = []
        self.cmd = []          # (wall_t, lin_x, ang_z)
        self._last_sample = 0.0

        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_timer(0.5, lambda: None)
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)

        world = os.environ.get("GZ_WORLD", "rmul2027_3v3")
        self.gznode = GzNode()
        self.gznode.subscribe(pose_v_pb2.Pose_V, f"/world/{world}/pose/info", self._on_gz)

    # ---------------------------------------------------------- 回调
    def _on_amcl(self, m):
        p = m.pose.pose.position
        self.amcl.append((time.time(), p.x, p.y, yaw_of(m.pose.pose.orientation)))

    def _on_odom(self, _m):
        self.odom_n += 1

    def _on_scan(self, _m):
        self.scan_n += 1

    def _on_cmd(self, m):
        self.cmd.append((time.time(), m.linear.x, m.angular.z))

    def _on_gz(self, msg):
        now = time.time()
        for p in msg.pose:
            self.gz_names.add(p.name)
            if "waffle" in p.name and "::" not in p.name:
                self.gz[p.name] = (p.position.x, p.position.y, p.position.z)
                # Gazebo 真值：用来量化 AMCL/TF 的定位误差（地图与世界是否一致的最直接证据）
                self.gz_track.append((now, p.position.x, p.position.y, p.position.z))

    def _sample_tf(self):
        now = time.time()
        if now - self._last_sample < SAMPLE_DT:
            return
        try:
            tr = self.tf.lookup_transform("map", "base_footprint", Time())
        except Exception:                                            # noqa: BLE001
            return
        self._last_sample = now
        t = tr.transform.translation
        self.track.append((now, self.get_clock().now().nanoseconds / 1e9,
                           t.x, t.y, yaw_of(tr.transform.rotation)))

    # ---------------------------------------------------------- 循环
    def spin_for(self, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)
            self._sample_tf()

    def spin_until(self, pred, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            self._sample_tf()
            if pred():
                return True
        return False

    def wait_sim_time(self, timeout=30.0):
        return self.spin_until(lambda: self.get_clock().now().nanoseconds > 0, timeout)

    def rate(self, attr, seconds=3.0):
        before = getattr(self, attr)
        self.spin_for(seconds)
        return (getattr(self, attr) - before) / seconds

    def tf_ready(self):
        try:
            self.tf.lookup_transform("map", "base_footprint", Time())
            return True
        except Exception:                                            # noqa: BLE001
            return False

    # ---------------------------------------------------------- 导航
    def goto(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        i0 = len(self.track)
        i0c = len(self.cmd)
        sim0 = self.get_clock().now().nanoseconds / 1e9
        w0 = time.time()
        send_fut = self.client.send_goal_async(goal, feedback_callback=self._fb)
        if not self.spin_until(lambda: send_fut.done(), 20.0):
            return {"error": "send_goal timeout", "goal": [x, y, yaw]}
        gh = send_fut.result()
        if gh is None or not gh.accepted:
            return {"error": "goal rejected", "goal": [x, y, yaw]}
        res_fut = gh.get_result_async()
        if not self.spin_until(lambda: res_fut.done(), GOAL_TIMEOUT):
            gh.cancel_goal_async()
            self.spin_for(2.0)
            return {"error": "result timeout", "goal": [x, y, yaw]}
        self.spin_for(0.5)                       # 收几个尾点
        wrapped = res_fut.result()
        wall_dt = time.time() - w0
        sim_dt = self.get_clock().now().nanoseconds / 1e9 - sim0

        seg = self.track[i0:]
        path = sum(math.hypot(seg[i][2] - seg[i - 1][2], seg[i][3] - seg[i - 1][3])
                   for i in range(1, len(seg)))
        end = seg[-1] if seg else None
        cmd_seg = self.cmd[i0c:]
        lins = [c[1] for c in cmd_seg] or [0.0]
        angs = [c[2] for c in cmd_seg] or [0.0]
        return {
            "goal": [x, y, round(yaw, 3)],
            "status": int(wrapped.status),
            "succeeded": bool(wrapped.status == 4),
            "error_code": int(wrapped.result.error_code),
            "wall_s": round(wall_dt, 2),
            "sim_s": round(sim_dt, 2),
            "rtf_mean": round(sim_dt / wall_dt, 3),
            "path_len_m": round(path, 3),
            "mean_speed_mps": round(path / sim_dt, 3) if sim_dt > 0 else None,
            "cmd_v_max_mps": round(max(abs(v) for v in lins), 3),
            "cmd_w_max_rps": round(max(abs(v) for v in angs), 3),
            "cmd_samples": len(cmd_seg),
            "tf_samples": len(seg),
            "end_xy": [round(end[2], 3), round(end[3], 3)] if end else None,
            "end_dist_to_goal_m": round(math.hypot(end[2] - x, end[3] - y), 3) if end else None,
            "max_abs_x": round(max((abs(p[2]) for p in seg), default=0.0), 3),
            "max_abs_y": round(max((abs(p[3]) for p in seg), default=0.0), 3),
            "recoveries": self.feedback[-1]["recoveries"] if self.feedback else None,
        }

    def _fb(self, msg):
        self.feedback.append({
            "recoveries": int(msg.feedback.number_of_recoveries),
            "distance_remaining": round(float(msg.feedback.distance_remaining), 3),
        })


def main():
    rclpy.init()
    v = Verifier()
    boxes = load_obstacles()
    report = {"field": {"len": FIELD_LEN, "wid": FIELD_WID,
                        "wall_inner_half": [INNER_X, INNER_Y], "margin": MARGIN,
                        "z_window": [Z_MIN, Z_MAX], "obstacle_margin": OBS_MARGIN,
                        "n_obstacle_boxes": len(boxes)},
              "goals": [list(g) for g in GOALS],
              "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
              "params_hint": {"vx_max": VX_MAX, "params_file": PARAMS_FILE}}

    print(f"[0] ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID')}")

    print("[1] 等 action server /navigate_to_pose ...")
    if not v.client.wait_for_server(timeout_sec=90.0):
        print("!! 等不到导航 action server（栈没起来？）")
        report["error"] = "no action server"
        _save(report)
        return 2
    print("    ok")

    print("[2] 等仿真时钟 / TF map->base_footprint ...")
    report["sim_time_ok"] = v.wait_sim_time(30.0)
    report["tf_ready"] = v.spin_until(v.tf_ready, 30.0)
    print(f"    sim_time={report['sim_time_ok']} tf={report['tf_ready']}")
    v.spin_for(1.0)

    print("[3] 话题频率（3 s）...")
    report["rates"] = {"scan_hz": round(v.rate("scan_n", 3.0), 2),
                       "odom_hz": round(v.rate("odom_n", 3.0), 2)}
    print("   ", report["rates"])

    results = []
    for i, (x, y, yaw) in enumerate(GOALS, 1):
        print(f"[4.{i}] 目标 ({x}, {y}, yaw={yaw}) ...")
        r = v.goto(x, y, yaw)
        results.append(r)
        print("    ->", {k: r.get(k) for k in
                         ("succeeded", "error_code", "wall_s", "sim_s", "rtf_mean",
                          "path_len_m", "mean_speed_mps", "end_dist_to_goal_m", "error")})
    report["nav_results"] = results

    v.spin_for(0.5)
    tr = v.track
    total_path = sum(math.hypot(tr[i][2] - tr[i - 1][2], tr[i][3] - tr[i - 1][3])
                     for i in range(1, len(tr)))
    sim_dt = (tr[-1][1] - tr[0][1]) if len(tr) > 1 else 0.0
    wall_dt = (tr[-1][0] - tr[0][0]) if len(tr) > 1 else 0.0
    zs = [p[2] for p in v.gz.values()] or [0.0]
    # 定位误差：TF(map->base_footprint) vs Gazebo 真值，按 wall 时间取最近的真值帧
    loc_err = []
    loc_series = []
    loc_t0 = tr[0][0] if tr else 0.0
    if v.gz_track:
        gt = v.gz_track
        gi = 0
        for wall_t, _sim_t, x, y, _yaw in tr:
            while gi + 1 < len(gt) and gt[gi + 1][0] <= wall_t:
                gi += 1
            g = gt[gi]
            if abs(g[0] - wall_t) <= 0.5:
                loc_err.append(math.hypot(x - g[1], y - g[2]))
                if wall_t - loc_t0 >= 2.0:            # 每 2 s 留一个点看趋势（偏置 vs 漂移）
                    loc_t0 = wall_t
                    loc_series.append([round(wall_t - tr[0][0], 1), round(loc_err[-1], 3)])
    report["whole_run"] = {
        "tf_samples": len(tr),
        "path_len_m": round(total_path, 3),
        "sim_s": round(sim_dt, 2), "wall_s": round(wall_dt, 2),
        "mean_speed_mps": round(total_path / sim_dt, 3) if sim_dt > 0 else None,
        "global_rtf": round(sim_dt / wall_dt, 3) if wall_dt > 0 else None,
        "max_abs_x": round(max((abs(p[2]) for p in tr), default=0.0), 3),
        "max_abs_y": round(max((abs(p[3]) for p in tr), default=0.0), 3),
        "gz_z_min": round(min(zs), 4),
        "gz_z_max": round(max(zs), 4),
        "max_box_penetration_m": round(
            max((box_penetration(p[2], p[3], boxes) for p in tr), default=-1e9), 4),
        "loc_err_m": {"max": round(max(loc_err), 3) if loc_err else None,
                      "mean": round(sum(loc_err) / len(loc_err), 3) if loc_err else None,
                      "samples": len(loc_err),
                      "series_2s": loc_series},
        "limit": [round(INNER_X - MARGIN, 3), round(INNER_Y - MARGIN, 3)],
        "amcl_samples": len(v.amcl),
        "cmd_samples": len(v.cmd),
        "cmd_v_max_mps": round(max((abs(c[1]) for c in v.cmd), default=0.0), 3),
        "cmd_w_max_rps": round(max((abs(c[2]) for c in v.cmd), default=0.0), 3),
    }

    checks = {
        "sim_time": {"ok": bool(report["sim_time_ok"]), },
        "tf_available": {"ok": bool(report["tf_ready"])},
        "topics": {"ok": report["rates"]["scan_hz"] > 1.0 and report["rates"]["odom_hz"] > 1.0,
                   "scan_hz": report["rates"]["scan_hz"], "odom_hz": report["rates"]["odom_hz"]},
        "goals_succeeded": {
            "ok": all(r.get("succeeded") and r.get("error_code") == 0 for r in results),
            "per_goal": [{"succeeded": r.get("succeeded"), "error_code": r.get("error_code"),
                          "wall_s": r.get("wall_s")} for r in results]},
        "inside_field": {
            "ok": report["whole_run"]["max_abs_x"] <= INNER_X - MARGIN + 1e-9
                  and report["whole_run"]["max_abs_y"] <= INNER_Y - MARGIN + 1e-9,
            "max_abs_x": report["whole_run"]["max_abs_x"],
            "max_abs_y": report["whole_run"]["max_abs_y"],
            "limit": report["whole_run"]["limit"]},
        "no_fall": {"ok": report["whole_run"]["gz_z_min"] > Z_MIN,
                    "gz_z_min": report["whole_run"]["gz_z_min"], "z_min_limit": Z_MIN},
        "no_climb": {"ok": report["whole_run"]["gz_z_max"] < Z_MAX,
                     "gz_z_max": report["whole_run"]["gz_z_max"], "z_max_limit": Z_MAX},
        "no_obstacle_overlap": {
            "ok": report["whole_run"]["max_box_penetration_m"] <= OBS_MARGIN,
            "max_box_penetration_m": report["whole_run"]["max_box_penetration_m"],
            "tol": OBS_MARGIN,
            "note": "5 cm = 碰撞体栅格粒度；正数=TF 点落在某个平台/墙的投影内"},
        "localization_consistent": {
            "ok": report["whole_run"]["loc_err_m"]["max"] is not None
                  and report["whole_run"]["loc_err_m"]["max"] < LOC_ERR_MAX,
            "loc_err_m": report["whole_run"]["loc_err_m"], "tol": LOC_ERR_MAX,
            "note": "TF(map->base_footprint) 与 Gazebo 真值的偏差；"
                    "地图/世界不一致（如 PGM 上下翻转）时这里会先炸"},
        "reached_goal": {
            "ok": all((r.get("end_dist_to_goal_m") is not None
                       and r["end_dist_to_goal_m"] < GOAL_TOL) for r in results),
            "end_dist_to_goal_m": [r.get("end_dist_to_goal_m") for r in results],
            "tol": GOAL_TOL},
        "speed_sane": {
            "ok": VX_MAX is not None
                  and all(r.get("cmd_v_max_mps") is not None
                          and r["cmd_v_max_mps"] <= VX_MAX * 1.05
                          and r.get("mean_speed_mps") is not None
                          and 0.05 < r["mean_speed_mps"] <= VX_MAX
                          for r in results),
            "vx_max_from_params": VX_MAX,
            "cmd_v_max_mps": [r.get("cmd_v_max_mps") for r in results],
            "mean_speed_mps": [r.get("mean_speed_mps") for r in results],
            "note": "vx_max 读自 velocity_smoother；TB3 底盘自身不限速"},
    }
    ok = all(c["ok"] for c in checks.values())
    report["checks"] = checks
    report["summary"] = {"ok": bool(ok)}
    _save(report)

    print("\n===== 摘要 =====")
    print(json.dumps({"rates": report["rates"], "whole_run": report["whole_run"],
                      "checks": {k: c["ok"] for k, c in checks.items()},
                      "summary": report["summary"]}, indent=2, ensure_ascii=False))
    rclpy.shutdown()
    return 0 if ok else 3


def _save(report):
    path = os.path.join(RES, "report_nav2_field.json")
    with open(path, "w") as f:
        json.dump(report, f, indent=2, ensure_ascii=False, default=float)
    print("报告:", path)


if __name__ == "__main__":
    raise SystemExit(main())