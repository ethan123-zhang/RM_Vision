#!/usr/bin/env python3
"""对比"定位估计"与"仿真真值"：map→base_footprint（AMCL 输出）vs Gazebo 真值。

用途：用户说"地图上的坐标和实际坐标不一致"时，先量化差多少、是系统性偏移还是漂移。

输出：
  - TF(map→base_footprint)：导航/显示用的位姿（AMCL）
  - TF(odom→base_footprint)：里程计位姿（未校正）
  - Gazebo pose/info 的 turtlebot3_waffle：真值
  - 误差 = 估计 - 真值（x/y 分量、距离、yaw 差）

用法：python3 check_localization.py [--secs 6]
"""
import argparse
import math
import os
import sys
import time

os.environ.setdefault("ROS_DOMAIN_ID", "42")

import rclpy                                                       # noqa: E402
from rclpy.node import Node                                        # noqa: E402
from rclpy.parameter import Parameter                              # noqa: E402
from rclpy.time import Time                                        # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped            # noqa: E402
from tf2_ros import Buffer, TransformListener                      # noqa: E402

from gz.msgs10.pose_v_pb2 import Pose_V                            # noqa: E402
from gz.transport13 import Node as GzNode                          # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--secs", type=float, default=6.0)
args = ap.parse_args()


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Node2(Node):
    def __init__(self):
        super().__init__("check_localization",
                         parameter_overrides=[Parameter("use_sim_time",
                                                       Parameter.Type.BOOL, True)])
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self.node if hasattr(self, "node") else self)
        self.amcl = None
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._amcl, 10)

    def _amcl(self, msg):
        p = msg.pose.pose.position
        self.amcl = (p.x, p.y, yaw_of(msg.pose.pose.orientation),
                     msg.pose.covariance)


def tf_pose(buf, target, source):
    try:
        tr = buf.lookup_transform(target, source, Time())
    except Exception:                                            # noqa: BLE001
        return None
    t = tr.transform.translation
    return t.x, t.y, yaw_of(tr.transform.rotation)


def main():
    rclpy.init()
    n = Node2()
    truth = {}

    def cb(msg):
        for p in msg.pose:
            if "waffle" in p.name and "wheel" not in p.name:
                truth["pose"] = (p.position.x, p.position.y,
                                 math.atan2(2 * (p.orientation.w * p.orientation.z
                                                 + p.orientation.x * p.orientation.y),
                                            1 - 2 * (p.orientation.y ** 2 + p.orientation.z ** 2)))

    gz = GzNode()
    gz.subscribe(Pose_V, "/world/rmul2027_3v3/pose/info", cb)

    rows = []
    t0 = time.time()
    while time.time() - t0 < args.secs:
        rclpy.spin_once(n, timeout_sec=0.1)
        if "pose" not in truth:
            continue
        mp = tf_pose(n.tf, "map", "base_footprint")
        op = tf_pose(n.tf, "odom", "base_footprint")
        if mp is None:
            continue
        tx, ty, tyaw = truth["pose"]
        dx, dy = mp[0] - tx, mp[1] - ty
        dyaw = math.atan2(math.sin(mp[2] - tyaw), math.cos(mp[2] - tyaw))
        odx = (op[0] - tx) if op else float("nan")
        ody = (op[1] - ty) if op else float("nan")
        rows.append((mp[0], mp[1], tx, ty, dx, dy, math.hypot(dx, dy), math.degrees(dyaw),
                     odx, ody))
        time.sleep(0.4)

    if not rows:
        print("!! 没采到数据（TF / Gazebo 真值 / 时钟 有一个没起来）")
        return 1
    mx = sum(r[4] for r in rows) / len(rows)
    my = sum(r[5] for r in rows) / len(rows)
    md = sum(r[6] for r in rows) / len(rows)
    print(f"样本 {len(rows)} 个（{args.secs:.0f}s）")
    print(f"最近一次：map 估计 ({rows[-1][0]:.3f},{rows[-1][1]:.3f})  "
          f"真值 ({rows[-1][2]:.3f},{rows[-1][3]:.3f})  "
          f"误差 ({rows[-1][4]:+.3f},{rows[-1][5]:+.3f}) |{rows[-1][6]:.3f}| m  "
          f"yaw 差 {rows[-1][7]:+.1f}°")
    print(f"平均误差：({mx:+.3f},{my:+.3f})  |{md:.3f}| m")
    print(f"最大误差：{max(r[6] for r in rows):.3f} m")
    on = [r for r in rows if r[8] == r[8]]
    if on:
        print(f"里程计误差：({sum(r[8] for r in on)/len(on):+.3f},"
              f"{sum(r[9] for r in on)/len(on):+.3f}) m（不含 AMCL 校正，越小说明底盘 odom 越准）")
    a = n.amcl
    if a:
        print(f"AMCL 位姿 ({a[0]:.3f},{a[1]:.3f}) yaw {math.degrees(a[2]):.1f}°，"
              f"协方差 xx {a[3][0]:.3f} yy {a[3][7]:.3f} yawyaw {a[3][35]:.3f}")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())