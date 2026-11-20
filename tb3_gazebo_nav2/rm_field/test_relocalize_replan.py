#!/usr/bin/env python3
"""回归：远距离导航中定位被设错 → 重定位 → **必须重发目标** 才会按新位姿重规划。

复现用户现象：AMCL 被设到错误位置后，即使再重定位对了，nav2 仍在跟旧路径 → 路线"断掉"。
验证修复：`map_click.py` 在重定位后会取消并重发同一个目标（本脚本模拟同样的消息序列）。

流程：
  1. 发远处目标 G，等机器人走起来；
  2. 发一个**错误**的 /initialpose（真值 +2.5 m）→ 观察目标是否失败/路径断掉；
  3. 发**正确**的 /initialpose（真值），不重发目标 → 观察是否恢复（预期：不会按新位姿重规划）；
  4. 取消并重发 G（= map_click 的修复行为）→ 等到点。

用法：python3 test_relocalize_replan.py --goal 4.6 -2.0
"""
import argparse
import math
import os
import sys
import time

os.environ.setdefault("ROS_DOMAIN_ID", "42")

import rclpy                                                       # noqa: E402
from rclpy.action import ActionClient                               # noqa: E402
from rclpy.node import Node                                         # noqa: E402
from rclpy.parameter import Parameter                               # noqa: E402
from rclpy.time import Time                                         # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped             # noqa: E402
from tf2_ros import Buffer, TransformListener                       # noqa: E402
from nav2_msgs.action import NavigateToPose                         # noqa: E402
from gz.msgs10.pose_v_pb2 import Pose_V                             # noqa: E402
from gz.transport13 import Node as GzNode                           # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--goal", nargs=2, type=float, required=True)
ap.add_argument("--wrong-offset", type=float, default=2.5)
args = ap.parse_args()


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class N(Node):
    def __init__(self):
        super().__init__("test_relocalize_replan",
                         parameter_overrides=[Parameter("use_sim_time",
                                                       Parameter.Type.BOOL, True)])
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.truth = None
        self.gz = GzNode()
        self.gz.subscribe(Pose_V, "/world/rmul2027_3v3/pose/info", self._truth)
        self.spin_n(1.0)

    def _truth(self, msg):
        for p in msg.pose:
            if "waffle" in p.name and "wheel" not in p.name:
                self.truth = (p.position.x, p.position.y, yaw_of(p.orientation))

    def spin_n(self, secs):
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(self, timeout_sec=0.05)

    def est(self):
        try:
            tr = self.tf.lookup_transform("map", "base_footprint", Time())
        except Exception:                                            # noqa: BLE001
            return None
        t = tr.transform.translation
        return t.x, t.y, yaw_of(tr.transform.rotation)

    def init_pose(self, x, y, yaw=0.0):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z = math.sin(yaw / 2)
        m.pose.pose.orientation.w = math.cos(yaw / 2)
        m.pose.covariance = [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0,
                             0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0.0685, 0, 0,
                             0, 0, 0, 0, 0.0685, 0, 0, 0, 0, 0, 0, 0.0685]
        for _ in range(3):
            self.pub.publish(m)
            self.spin_n(0.15)

    def send(self, x, y):
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = "map"
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = x, y
        g.pose.pose.orientation.w = 1.0
        fut = self.client.send_goal_async(g)
        self._wait(fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print("!! 目标被拒绝")
            return None
        rf = gh.get_result_async()
        return gh, rf

    def _wait(self, fut, timeout=10.0):
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            self.spin_n(0.05)


def main():
    rclpy.init()
    n = N()
    print("等 action server ...", flush=True)
    if not n.client.wait_for_server(timeout_sec=90):
        print("!! 没有 action server")
        return 1
    t0 = time.time()
    while n.truth is None and time.time() - t0 < 10:
        n.spin_n(0.2)
    print(f"真值 ({n.truth[0]:.2f},{n.truth[1]:.2f})；目标 {args.goal}", flush=True)

    print("[1] 发目标并等机器人走起来", flush=True)
    r = n.send(*args.goal)
    if r is None:
        return 1
    gh, rf = r
    n.spin_n(6.0)
    e = n.est()
    print(f"    6s 后 map 估计 ({e[0]:.2f},{e[1]:.2f}) 真值 ({n.truth[0]:.2f},{n.truth[1]:.2f})")

    print("[2] 发**错误**的 initialpose（真值 +2.5 m）模拟跟丢", flush=True)
    n.init_pose(n.truth[0] + args.wrong_offset, n.truth[1], n.truth[2])
    n.spin_n(5.0)
    e = n.est()
    print(f"    5s 后 map 估计 ({e[0]:.2f},{e[1]:.2f})，目标状态 done={rf.done()}"
          f" status={int(rf.result().status) if rf.done() else '-'}")

    print("[3] 用真值重定位（**不重发目标**）——预期：旧路径不会按新位姿重算", flush=True)
    n.init_pose(*n.truth)
    n.spin_n(6.0)
    e = n.est()
    print(f"    6s 后 map 估计 ({e[0]:.2f},{e[1]:.2f})，目标状态 done={rf.done()}"
          f" status={int(rf.result().status) if rf.done() else '-'}")
    stuck = rf.done() or (n.est() is not None and
                          math.hypot(n.est()[0] - args.goal[0], n.est()[1] - args.goal[1]) > 1.0)
    print(f"    仍离目标 {math.hypot(n.est()[0]-args.goal[0], n.est()[1]-args.goal[1]):.2f} m"
          f"（stuck={stuck}）")

    print("[4] 取消并重发目标（= map_click 重定位后的修复行为）", flush=True)
    gh.cancel_goal_async()
    n.spin_n(1.0)
    r2 = n.send(*args.goal)
    if r2 is None:
        return 1
    gh2, rf2 = r2
    t0 = time.time()
    ok = False
    while time.time() - t0 < 120:
        n.spin_n(0.5)
        e = n.est()
        if e and math.hypot(e[0] - args.goal[0], e[1] - args.goal[1]) < 0.35:
            ok = True
            break
        if rf2.done():
            break
    e = n.est()
    print(f"    结果：到点={ok}，最终 map ({e[0]:.2f},{e[1]:.2f})，"
          f"距目标 {math.hypot(e[0]-args.goal[0], e[1]-args.goal[1]):.2f} m，"
          f"status={int(rf2.result().status) if rf2.done() else '-'}")
    rclpy.shutdown()
    return 0 if ok else 3


if __name__ == "__main__":
    sys.exit(main())