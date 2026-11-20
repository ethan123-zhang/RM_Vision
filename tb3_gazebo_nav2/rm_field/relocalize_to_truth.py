#!/usr/bin/env python3
"""把 AMCL 的初始位姿直接设成 Gazebo 真值，验证"地图是否和世界对齐"。

用途：出现"地图坐标和实际坐标差很远"时，先区分两种原因：
  a) 地图/世界本身没对齐（拓扑错、镜像、偏移）→ 用真值重定位后误差仍然大；
  b) AMCL 只是丢了 → 用真值重定位后误差立刻回到几厘米。

用法：python3 relocalize_to_truth.py [--settle 4]
输出：重定位前误差、重定位后误差（样本中位数）。
"""
import argparse
import math
import os
import statistics
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
ap.add_argument("--settle", type=float, default=4.0)
args = ap.parse_args()


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class N(Node):
    def __init__(self):
        super().__init__("relocalize_to_truth",
                         parameter_overrides=[Parameter("use_sim_time",
                                                       Parameter.Type.BOOL, True)])
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)


def main():
    rclpy.init()
    n = N()
    truth = {}

    def cb(msg):
        for p in msg.pose:
            if "waffle" in p.name and "wheel" not in p.name:
                truth["pose"] = (p.position.x, p.position.y, yaw_of(p.orientation))

    gz = GzNode()
    gz.subscribe(Pose_V, "/world/rmul2027_3v3/pose/info", cb)

    t0 = time.time()
    while "pose" not in truth and time.time() - t0 < 10:
        rclpy.spin_once(n, timeout_sec=0.1)
    if "pose" not in truth:
        print("!! 收不到 Gazebo 真值")
        return 1

    def estimate():
        try:
            tr = n.tf.lookup_transform("map", "base_footprint", Time())
        except Exception:                                        # noqa: BLE001
            return None
        t = tr.transform.translation
        return t.x, t.y, yaw_of(tr.transform.rotation)

    # TF 缓冲刚起来时要等一会才有 map→odom→base 链
    t0 = time.time()
    while estimate() is None and time.time() - t0 < 15:
        rclpy.spin_once(n, timeout_sec=0.2)

    def err(a):
        return math.hypot(a[0] - truth["pose"][0], a[1] - truth["pose"][1])

    e = estimate()
    print(f"真值 ({truth['pose'][0]:.3f},{truth['pose'][1]:.3f}) yaw {math.degrees(truth['pose'][2]):.1f}°")
    print(f"重定位前：估计 ({e[0]:.3f},{e[1]:.3f}) 误差 {err(e):.3f} m" if e else "重定位前：没有 TF")

    m = PoseWithCovarianceStamped()
    m.header.frame_id = "map"
    m.header.stamp = n.get_clock().now().to_msg()
    m.pose.pose.position.x = truth["pose"][0]
    m.pose.pose.position.y = truth["pose"][1]
    m.pose.pose.orientation.z = math.sin(truth["pose"][2] / 2)
    m.pose.pose.orientation.w = math.cos(truth["pose"][2] / 2)
    m.pose.covariance = [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0,
                         0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0.0685, 0, 0,
                         0, 0, 0, 0, 0.0685, 0, 0, 0, 0, 0, 0, 0.0685]
    n.pub.publish(m)
    rclpy.spin_once(n, timeout_sec=0.2)
    print(f"已发 /initialpose = 真值，等 {args.settle:.0f}s 收敛 ...")
    time.sleep(args.settle)

    errs = []
    t1 = time.time()
    while time.time() - t1 < 4.0:
        rclpy.spin_once(n, timeout_sec=0.1)
        a = estimate()
        if a:
            errs.append(err(a))
        time.sleep(0.2)
    if not errs:
        print("!! 重定位后仍取不到 TF")
        return 1
    print(f"重定位后误差：中位数 {statistics.median(errs):.3f} m，最小 {min(errs):.3f} m，"
          f"样本 {len(errs)}")
    med = statistics.median(errs)
    if med < 0.15:
        print("结论：地图与世界是对齐的；之前的偏差是 AMCL 跟丢（不是地图错）")
    else:
        print("结论：用真值重定位后仍然偏 → 地图/世界对齐有问题（或 AMCL 参数/扫描模型不匹配）")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())