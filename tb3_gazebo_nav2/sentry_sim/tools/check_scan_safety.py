#!/usr/bin/env python3
"""a 验收探针：低位 2D 安全雷达 /scan_safety 体检。

为什么单独做体检：这条雷达是"补盲"用的，它自己坏掉的两种方式都很像"导航坏了"——
  ① 装在车体内部 → 每一束都打在自家壳上（~0.25 m 的贴脸回波），代价地图满格、车永远停；
  ② 帧号不平移 / 时间戳不跟仿真钟 → 两张 costmap 一句 "Could not transform" 后静默忽略它，
     看起来和"雷达没起作用"一模一样。
所以先把这三件事测出来，再谈导航。

判据（逐条打印，全过返回 0）：
  1) 频率 >= 6 Hz（配置 10 Hz；RTF 0.8 时仿真钟 10 Hz 仍是 10 Hz，取宽松下限）
  2) 每帧 >= 300 束（360° 360 线）
  3) 有限量程都在 [0.05, 2.55] 内
  4) 最小有限量程 >= 0.30 m —— 空旷处不该有自身回波
  5) frame_id 能与 base_footprint 建立 TF（"帧号对得上"）

用法（栈已在跑、ROS_DOMAIN_ID=45）：
  python3 tools/check_scan_safety.py
  python3 tools/check_scan_safety.py --frames 20 --min-range-expect 0.30
"""
import argparse
import math
import os
import sys

os.environ.setdefault("ROS_DOMAIN_ID", "45")

import rclpy                                                        # noqa: E402
from rclpy.node import Node                                         # noqa: E402
from rclpy.parameter import Parameter                               # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy                 # noqa: E402
from sensor_msgs.msg import LaserScan                               # noqa: E402
from tf2_ros import Buffer, TransformListener                        # noqa: E402


def sectors(scan):
    """按 0.25 圈分 4 格（前/左/后/右）报最小有限距离，用来证明"它确实在看"。"""
    n = len(scan.ranges)
    if n == 0:
        return []
    names = ["前方", "左方", "后方", "右方"]
    out = []
    for k, label in enumerate(names):
        lo = int(n * (0.125 + 0.25 * k)) % n
        hi = int(n * (0.125 + 0.25 * (k + 1))) % n
        vals = []
        idx = lo
        while True:
            r = scan.ranges[idx]
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                vals.append(r)
            idx = (idx + 1) % n
            if idx == hi:
                break
        out.append((label, min(vals) if vals else float("inf"), len(vals)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/scan_safety")
    ap.add_argument("--frames", type=int, default=15, help="收多少帧再判")
    ap.add_argument("--min-range-expect", type=float, default=0.30,
                    help="最小有限距离下限（自身回波判据）")
    ap.add_argument("--expect-clear", action="store_true",
                    help="空旷区自检：要求全部量程为 inf/超量程（车在空地里跑时用）")
    args = ap.parse_args()

    rclpy.init()
    node = Node("check_scan_safety",
                parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    tf = Buffer()
    TransformListener(tf, node)
    frames = []
    stamps = []
    sub = node.create_subscription(
        LaserScan, args.topic,
        lambda m: (frames.append(m), stamps.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)),
        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
    if sub is None:
        print("FAIL: 订阅失败")
        rclpy.shutdown()
        return 2

    t_start = None
    while len(frames) < args.frames:
        rclpy.spin_once(node, timeout_sec=0.1)
        if t_start is None:
            t_start = node.get_clock().now().nanoseconds
        elif (node.get_clock().now().nanoseconds - t_start) / 1e9 > 30.0:
            print(f"FAIL: 30 s 内只收到 {len(frames)}/{args.frames} 帧 —— "
                  f"gz 传感器没起来或桥没连上")
            node.destroy_node()
            rclpy.shutdown()
            return 3

    first = frames[0]
    finite = [r for m in frames for r in m.ranges
              if math.isfinite(r) and m.range_min <= r <= m.range_max]
    all_finite_ok = True
    for m in frames:
        for r in m.ranges:
            if math.isfinite(r) and (r < m.range_min - 1e-6 or r > m.range_max + 0.05):
                all_finite_ok = False
                break
    sim_s = (stamps[-1] - stamps[0]) if len(stamps) > 1 else 0.0
    hz = (len(frames) - 1) / sim_s if sim_s > 0 else 0.0
    has_tf = tf.can_transform("base_footprint", first.header.frame_id, rclpy.time.Time(),
                              timeout=rclpy.duration.Duration(seconds=2.0))
    min_r = min(finite) if finite else float("inf")
    max_r = max(finite) if finite else float("inf")
    mean_r = sum(finite) / len(finite) if finite else float("inf")

    print(f"topic={args.topic}  frames={len(frames)}  n_per_frame={len(first.ranges)}  "
          f"frame_id={first.header.frame_id!r}")
    print(f"range_min={first.range_min:.3f} range_max={first.range_max:.3f} "
          f"angle_min={first.angle_min:+.4f} angle_max={first.angle_max:+.4f} "
          f"inc={first.angle_increment:.5f}")
    print(f"频率 {hz:.1f} Hz（仿真钟）  有限束 {len(finite)}/{len(frames) * len(first.ranges)}  "
          f"最小 {min_r:.3f} / 均值 {mean_r:.3f} / 最大 {max_r:.3f} m")
    for label, r, cnt in sectors(first):
        print(f"  {label}: 最小 {r:.3f} m（{cnt} 束有限）")

    checks = [
        ("频率>=6 Hz", hz >= 6.0, f"{hz:.1f} Hz"),
        ("每帧>=300束", len(first.ranges) >= 300, f"{len(first.ranges)} 束"),
        ("量程都在合法区间", all_finite_ok,
         f"channel range [{first.range_min}, {first.range_max}]"),
        (f"无自身回波(最小>={args.min_range_expect:.2f} m)", min_r >= args.min_range_expect,
         f"最小有限 {min_r:.3f} m"),
        ("frame_id 有 TF", bool(has_tf), f"{first.header.frame_id!r} -> base_footprint"),
    ]
    if args.expect_clear:
        checks.append(("空旷区全超量程", len(finite) == 0,
                       f"有限束 {len(finite)}（应 0）"))
    ok = all(c[1] for c in checks)
    print("\n===== 判据 =====")
    for label, passed, detail in checks:
        print(f"  [{'PASS' if passed else 'FAIL'}] {label}: {detail}")

    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 3


if __name__ == "__main__":
    sys.exit(main())