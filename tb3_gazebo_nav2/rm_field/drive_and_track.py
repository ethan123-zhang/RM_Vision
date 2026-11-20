#!/usr/bin/env python3
"""发一个目标，边走边对比 AMCL 估计与 Gazebo 真值，统计远距离导航中的定位误差。

用途：验证 AMCL 参数（config/nav2_params_rm.yaml）改完后，"远距离自动导航"还会不会跟丢。

用法：
  python3 drive_and_track.py --goal 4.6 2.1 [--timeout 180]
输出：采样数、起点/终点误差、最大误差、最大误差出现时的位置、是否到达。
"""
import argparse
import math
import os
import statistics
import sys
import time

os.environ.setdefault("ROS_DOMAIN_ID", "42")

import rclpy                                                       # noqa: E402
from rclpy.action import ActionClient                               # noqa: E402
from rclpy.node import Node                                         # noqa: E402
from rclpy.parameter import Parameter                               # noqa: E402
from rclpy.time import Time                                         # noqa: E402
from tf2_ros import Buffer, TransformListener                       # noqa: E402
from nav2_msgs.action import NavigateToPose                         # noqa: E402
from gz.msgs10.pose_v_pb2 import Pose_V                             # noqa: E402
from gz.transport13 import Node as GzNode                           # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--goal", nargs=2, type=float, required=True)
ap.add_argument("--timeout", type=float, default=180.0)
args = ap.parse_args()


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class N(Node):
    def __init__(self):
        super().__init__("drive_and_track",
                         parameter_overrides=[Parameter("use_sim_time",
                                                       Parameter.Type.BOOL, True)])
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.truth = None
        self.gz = GzNode()
        self.gz.subscribe(Pose_V, "/world/rmul2027_3v3/pose/info", self._truth)

    def _truth(self, msg):
        for p in msg.pose:
            if "waffle" in p.name and "wheel" not in p.name:
                self.truth = (p.position.x, p.position.y, yaw_of(p.orientation))

    def est(self):
        try:
            tr = self.tf.lookup_transform("map", "base_footprint", Time())
        except Exception:                                            # noqa: BLE001
            return None
        t = tr.transform.translation
        return t.x, t.y, yaw_of(tr.transform.rotation)


def main():
    rclpy.init()
    n = N()
    print("等 action server ...", flush=True)
    if not n.client.wait_for_server(timeout_sec=90):
        print("!! 没有 action server")
        return 1
    t0 = time.time()
    while n.truth is None and time.time() - t0 < 10:
        rclpy.spin_once(n, timeout_sec=0.1)

    g = NavigateToPose.Goal()
    g.pose.header.frame_id = "map"
    g.pose.header.stamp = n.get_clock().now().to_msg()
    g.pose.pose.position.x, g.pose.pose.position.y = args.goal
    g.pose.pose.orientation.w = 1.0
    print(f"发目标 ({args.goal[0]:.2f},{args.goal[1]:.2f})；开始采样误差 ...", flush=True)
    fut = n.client.send_goal_async(g)
    rclpy.spin_once(n, timeout_sec=0.1)
    while not fut.done():
        rclpy.spin_once(n, timeout_sec=0.1)
    gh = fut.result()
    if gh is None or not gh.accepted:
        print("!! 目标被拒绝")
        return 1
    rfut = gh.get_result_async()
    rclpy.spin_once(n, timeout_sec=0.1)

    errs = []
    t0 = time.time()
    while time.time() - t0 < args.timeout:
        rclpy.spin_once(n, timeout_sec=0.1)
        e = n.est()
        if e is not None and n.truth is not None:
            errs.append((time.time() - t0, math.hypot(e[0] - n.truth[0], e[1] - n.truth[1]),
                         e[0], e[1], n.truth[0], n.truth[1]))
        if rfut.done():
            break
    r = rfut.result() if rfut.done() else None
    st = int(r.status) if r is not None else None
    name = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}.get(st, f"STATUS_{st}")
    if not errs:
        print("!! 没采到误差样本")
        return 1
    d = [x[1] for x in errs]
    k = max(range(len(d)), key=lambda i: d[i])
    print(f"结果: {name}；样本 {len(errs)}；用时 {time.time()-t0:.1f}s")
    print(f"误差：起点 {d[0]:.3f} m，中位数 {statistics.median(d):.3f}，最大 {d[k]:.3f} m "
          f"(t={errs[k][0]:.1f}s at est ({errs[k][2]:.2f},{errs[k][3]:.2f}) "
          f"truth ({errs[k][4]:.2f},{errs[k][5]:.2f}))")
    print(f"终点误差 {d[-1]:.3f} m")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())