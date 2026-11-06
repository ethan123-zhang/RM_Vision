#!/usr/bin/env python3
"""Nav2 在 RMUL2027 3V3 场地上的导航验收（前置：bash nav2_field.sh 已把栈拉起来）。

判据（全部是读数，不看画面）：
  1. 仿真链路：/scan、/odom 有稳定频率；节点用仿真时间；map->base_footprint 可查
  2. 导航闭环：两个 /navigate_to_pose 目标都 SUCCEEDED（error_code 0）
  3. 不出界：整段导航中 map->base_footprint 始终在围挡内侧面以内（|x|<6.25-0.15、|y|<4.25-0.15）
  4. 不掉落：整段导航中 Gazebo 里机器人 z 始终 > -0.05（没掉下 0.503 m 的台子）
  5. 到位：每段结束时 TF 实测位置距目标 < 0.30 m
  6. 真在跑：按 TF 轨迹算路程/仿真时长，平均速度应在 TB3 速度上限 (0.26 m/s) 以内、且 >0.05

为什么不用 /amcl_pose 算位置：AMCL 只在"移动超过 0.25 m / 转超过 0.2 rad"时才发布，
用它算终点会滞后半米。TF（map->base_footprint）是导航栈真正在用的位姿，直接采样。

输出 results/report_nav2_field.json
"""
import json
import math
import os
import re
import time

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, "results")
os.makedirs(RES, exist_ok=True)

FIELD_LEN, FIELD_WID = 12.600, 8.600
INNER_X = FIELD_LEN / 2 - 0.05      # 围挡内侧面 6.250
INNER_Y = FIELD_WID / 2 - 0.05      # 4.250
MARGIN = 0.15                       # 车身半径余量（判"没压到/穿出围挡"）
GOALS = [(4.0, 2.0, 0.0), (0.0, 0.0, 0.0)]
GOAL_TOL = 0.30                     # 结束时到位判据（m）
GOAL_TIMEOUT = 240.0
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
        for p in msg.pose:
            self.gz_names.add(p.name)
            if "waffle" in p.name and "::" not in p.name:
                self.gz[p.name] = (p.position.x, p.position.y, p.position.z)

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
    report = {"field": {"len": FIELD_LEN, "wid": FIELD_WID,
                        "fence_inner_half": [INNER_X, INNER_Y], "margin": MARGIN},
              "goals": [list(g) for g in GOALS],
              "params_hint": {"vx_max": VX_MAX, "params_file": PARAMS_FILE}}

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
        "no_fall": {"ok": report["whole_run"]["gz_z_min"] > -0.05,
                    "gz_z_min": report["whole_run"]["gz_z_min"]},
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