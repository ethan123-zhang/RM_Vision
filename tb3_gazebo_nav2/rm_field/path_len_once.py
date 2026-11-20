#!/usr/bin/env python3
"""订阅一次 /plan，打印当前全局路径的长度与抽样点（用来核对"绕了多远"）。

用法：python3 path_len_once.py [--timeout 15]
"""
import argparse
import math
import os
import sys

os.environ.setdefault("ROS_DOMAIN_ID", "42")

import rclpy                                                    # noqa: E402
from rclpy.node import Node                                     # noqa: E402
from rclpy.parameter import Parameter                           # noqa: E402
from nav_msgs.msg import Path                                   # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--timeout", type=float, default=15.0)
args = ap.parse_args()


class Probe(Node):
    def __init__(self):
        super().__init__("path_len_once",
                         parameter_overrides=[Parameter("use_sim_time",
                                                       Parameter.Type.BOOL, True)])
        self.path = None
        self.create_subscription(Path, "/plan", self._cb, 10)

    def _cb(self, msg):
        self.path = msg


def main():
    rclpy.init()
    n = Probe()
    print("等 /plan ...", flush=True)
    import time
    t0 = time.time()
    while n.path is None and time.time() - t0 < args.timeout:
        rclpy.spin_once(n, timeout_sec=0.2)
    if n.path is None:
        print(f"!! {args.timeout:.0f}s 内没收到 /plan");
        rclpy.shutdown()
        return 1
    p = n.path
    pts = [(q.pose.position.x, q.pose.position.y) for q in p.poses]
    L = sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
            for i in range(len(pts) - 1))
    print(f"路径点数 {len(pts)}，长度 {L:.2f} m，"
          f"起点 ({pts[0][0]:.2f},{pts[0][1]:.2f}) → 终点 ({pts[-1][0]:.2f},{pts[-1][1]:.2f})")
    print("  frame:", p.header.frame_id)
    for fr in (0.0, 0.2, 0.4, 0.6, 0.8, 1.0):
        x, y = pts[int(fr * (len(pts) - 1))]
        print(f"  {int(fr*100):3d}% ({x:6.2f},{y:5.2f})")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())