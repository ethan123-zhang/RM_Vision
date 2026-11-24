#!/usr/bin/env python3
"""验收 c：「缝窄过不去就快速停」——死路判定 + 取消目标，而不是原地磨 200 s。

为什么判定放在客户端：ROS 2 没有"按条件取消别人目标"的公开 API，取消要靠发起方手上的
goal handle；点地图窗口/本脚本自己发目标，所以自己判、自己取消（判定逻辑在 tools/path_guard.py）。

判据（逐条打印，全过才返回 0）：
  1) 死路在 <= --judge-timeout（默认 20 s 仿真）内被判定（路径最窄处代价 >= --threshold）
  2) 目标被取消，且取消后 3 s 内位移 < 0.15 m（车真的停住，不是继续磨）
  3) 与敌车**面距** >= 0.05 m（没撞）
  4) 没有跑满 --timeout（默认 60 s）

反向回归（--expect-no-dead-end）：开阔场地应当**不**误判，目标 SUCCEEDED。

用法（栈已在跑，ROS_DOMAIN_ID=45）：
  # 窄缝：敌车 (0,1.2) 挡在车头，目标 (0,2.6) 在它后面
  python3 tools/verify_dead_end.py --goal 0 2.6 --obstacle 0 1.2 --spawn --spawn-after 1.5 --remove
  # 开阔：不该误判
  python3 tools/verify_dead_end.py --goal 3 -2.5 --obstacle 5 5 --expect-no-dead-end
"""
import argparse
import json
import math
import os
import subprocess
import sys
import threading
import time

os.environ.setdefault("ROS_DOMAIN_ID", "45")

import rclpy                                                         # noqa: E402
from rclpy.action import ActionClient                                # noqa: E402
from rclpy.node import Node                                          # noqa: E402
from rclpy.parameter import Parameter                                # noqa: E402
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy  # noqa: E402
from nav2_msgs.action import NavigateToPose                          # noqa: E402
from nav_msgs.msg import OccupancyGrid, Path                         # noqa: E402
from tf2_ros import Buffer, TransformListener                         # noqa: E402

from path_guard import clearance_grid, peek_plan_clearance       # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROBOT_HALF = 0.25          # 0.5x0.5 车体：正面到中心 0.25 m


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--goal", nargs=2, type=float, required=True, metavar=("X", "Y"))
    ap.add_argument("--obstacle", nargs=2, type=float, metavar=("X", "Y"))
    ap.add_argument("--obstacle-size", type=float, default=0.40)
    ap.add_argument("--spawn", action="store_true")
    ap.add_argument("--spawn-after", type=float, default=0.0)
    ap.add_argument("--remove", action="store_true")
    ap.add_argument("--threshold", type=float, default=0.20,
                    help="路径最小净空低于它（m）判死路")
    ap.add_argument("--judge-timeout", type=float, default=20.0, help="多少仿真秒内必须判定")
    ap.add_argument("--timeout", type=float, default=60.0, help="总仿真时间上限")
    ap.add_argument("--expect-no-dead-end", action="store_true")
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    gx, gy = args.goal

    if args.spawn and args.obstacle:
        def do_spawn():
            subprocess.run([sys.executable, os.path.join(HERE, "spawn_obstacle.py"),
                            "spawn", str(args.obstacle[0]), str(args.obstacle[1]),
                            "--size", str(args.obstacle_size)], check=False)
        if args.spawn_after > 0:
            threading.Timer(args.spawn_after, do_spawn).start()
            print(f"[spawn] {args.spawn_after:.1f} s 后出现在 "
                  f"({args.obstacle[0]:.2f},{args.obstacle[1]:.2f})")
        else:
            do_spawn()

    rclpy.init()
    node = Node("verify_dead_end",
                parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    tf = Buffer()
    TransformListener(tf, node)
    client = ActionClient(node, NavigateToPose, "navigate_to_pose")

    plan = {}
    cmap = {}

    node.create_subscription(Path, "/plan",
                             lambda m: plan.update(msg=m, t=clock_s(node)), 10)
    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    node.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                             lambda m: cmap.update(msg=m, t=clock_s(node)), qos)

    def clock_s(n=None):
        n = n or node
        return n.get_clock().now().nanoseconds / 1e9

    def pose_now():
        try:
            tr = tf.lookup_transform("map", "base_footprint", rclpy.time.Time())
        except Exception:                                            # noqa: BLE001
            return None
        t = tr.transform.translation
        return t.x, t.y, yaw_of(tr.transform.rotation)

    if not client.wait_for_server(timeout_sec=30.0):
        print("FAIL: /navigate_to_pose 30 s 内没就绪（栈起了吗？）")
        node.destroy_node()
        rclpy.shutdown()
        return 3
    deadline = time.monotonic() + 20.0
    while pose_now() is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    start = pose_now()
    if start is None:
        print("FAIL: 拿不到 map->base_footprint")
        node.destroy_node()
        rclpy.shutdown()
        return 3

    def costmap_tuple():
        m = cmap.get("msg")
        if m is None:
            return None
        i = m.info
        return (list(m.data), i.origin.position.x, i.origin.position.y,
                i.resolution, i.width, i.height)

    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = gx
    goal.pose.pose.position.y = gy
    goal.pose.pose.orientation.w = 1.0
    print(f"START ({start[0]:.2f},{start[1]:.2f}) -> GOAL ({gx:.2f},{gy:.2f})"
          + (f"，敌车 {args.obstacle}" if args.obstacle else ""))

    fut = client.send_goal_async(goal)
    t0 = time.monotonic()
    while not fut.done() and time.monotonic() - t0 < 20.0:
        rclpy.spin_once(node, timeout_sec=0.05)
    gh = fut.result() if fut.done() else None
    if gh is None or not gh.accepted:
        print("FAIL: 目标被拒绝")
        node.destroy_node()
        rclpy.shutdown()
        return 3
    res_fut = gh.get_result_async()

    t_goal = clock_s()
    judge_t = None
    judge_info = None
    hits = 0
    min_obs = 999.0
    min_clear_seen = 9.9
    last_plan = None
    grid = None
    grid_key = None
    samples = []
    while True:
        rclpy.spin_once(node, timeout_sec=0.02)
        now = clock_s()
        p = pose_now()
        if p is not None:
            samples.append((now, p[0], p[1]))
            if args.obstacle:
                min_obs = min(min_obs, math.hypot(p[0] - args.obstacle[0],
                                                  p[1] - args.obstacle[1]))
        info = None
        msg = plan.get("msg")
        cm = cmap.get("msg")
        # 只在"新的 /plan"上算一次（5 Hz），别在每个 spin 里都做距离变换
        if msg is not None and cm is not None and msg is not last_plan:
            last_plan = msg
            if now - plan.get("t", -9) < 1.0 and now - cmap.get("t", -9) < 3.0:
                ct = costmap_tuple()
                if ct is not None:
                    if grid_key != (ct[3], ct[4], ct[5], ct[0][:3]):
                        grid = clearance_grid(ct)
                        grid_key = (ct[3], ct[4], ct[5], ct[0][:3])
                    info = peek_plan_clearance(msg, ct, grid=grid)
                    if info:
                        min_clear_seen = min(min_clear_seen, info["min_clear"])
                        hits = hits + 1 if info["min_clear"] < args.threshold else 0
        if res_fut.done():
            break
        if now - t_goal > args.timeout:
            break
        if not args.expect_no_dead_end and hits >= 2 and judge_t is None:
            judge_t = now - t_goal
            judge_info = info
            gh.cancel_goal_async()
            print(f"DEAD END: 起点后 {judge_t:.1f} s 判定（路径最小净空 "
                  f"{info['min_clear']:.2f} m @ ({info['worst_xy'][0]:+.2f},"
                  f"{info['worst_xy'][1]:+.2f})） -> 取消目标")
            # 取消后再观测 3 s，确认车停下
            stop_from = clock_s()
            p_stop = pose_now()
            while clock_s() - stop_from < 3.0:
                rclpy.spin_once(node, timeout_sec=0.02)
                if res_fut.done():
                    break
            p_end = pose_now()
            moved = (math.hypot(p_end[0] - p_stop[0], p_end[1] - p_stop[1])
                     if p_end and p_stop else 999.0)
            break

    # 让结果收尾（取消也可能需要一会儿）
    t_wait = time.monotonic()
    while not res_fut.done() and time.monotonic() - t_wait < 8.0:
        rclpy.spin_once(node, timeout_sec=0.05)
    status = int(res_fut.result().status) if res_fut.done() else None
    name = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}.get(status, f"NONE/{status}")
    duration = clock_s() - t_goal
    node.destroy_node()
    rclpy.shutdown()

    gap_face = (min_obs - args.obstacle_size / 2.0 - ROBOT_HALF
                if args.obstacle else 999.0)
    moved = locals().get("moved", None)
    stopped = moved is not None and moved < 0.15

    if args.expect_no_dead_end:
        checks = [
            ("没有误判死路", judge_t is None, f"judge={'none' if judge_t is None else f'{judge_t:.1f} s'}"),
            ("目标到达", name == "SUCCEEDED", f"{name}"),
            ("用时 <= timeout", duration <= args.timeout, f"{duration:.1f} s"),
        ]
    else:
        checks = [
            ("死路及时判定", judge_t is not None and judge_t <= args.judge_timeout,
             f"判定 {judge_t:.1f} s（上限 {args.judge_timeout:.0f} s），路径最小净空 "
             f"{judge_info['min_clear']:.2f} m（阈值 {args.threshold:.2f} m）"
             if judge_t is not None else "根本没判定"),
            ("取消后停住", stopped,
             f"取消后 3 s 内位移 {moved:.2f} m" if moved is not None else "未测到"),
            ("不撞敌车(面距>=0.05 m)", gap_face >= 0.05,
             f"最小中心距 {min_obs:.3f} m → 面距 {gap_face:+.3f} m"
             if args.obstacle else "无碍"),
            ("没有磨满超时", duration <= args.timeout,
             f"总用时 {duration:.1f} s（上限 {args.timeout:.0f} s）, 结果 {name}"),
        ]
    ok = all(c[1] for c in checks)
    print("\n===== 判据 =====")
    for label, passed, detail in checks:
        print(f"  [{'PASS' if passed else 'FAIL'}] {label}: {detail}")
    print(f"\n全程路径净空最小值 {min_clear_seen:.2f} m，结果 {name}，采样 {len(samples)} 点")

    out = args.out or os.path.join(HERE, "..", "results",
                                   f"report_dead_end_{time.strftime('%H%M%S')}.json")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    payload = {
        "pass": ok,
        "expect_no_dead_end": bool(args.expect_no_dead_end),
        "result": name,
        "judge_s": round(judge_t, 2) if judge_t is not None else None,
        "judge_min_clear_m": round(judge_info["min_clear"], 3) if judge_info else None,
        "threshold_clear_m": args.threshold,
        "min_clear_seen_m": round(min_clear_seen, 3) if min_clear_seen < 9 else None,
        "stopped_after_cancel_m": round(moved, 3) if moved is not None else None,
        "min_obstacle_center_m": round(min_obs, 3) if args.obstacle else None,
        "gap_face_m": round(gap_face, 3) if args.obstacle else None,
        "duration_sim_s": round(duration, 2),
        "start": [round(start[0], 3), round(start[1], 3)],
        "goal": [gx, gy],
        "obstacle": list(args.obstacle) if args.obstacle else None,
        "checks": {c[0]: {"pass": c[1], "detail": c[2]} for c in checks},
    }
    with open(out, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    print(f"报告: {os.path.abspath(out)}")
    if args.remove:
        subprocess.run([sys.executable, os.path.join(HERE, "spawn_obstacle.py"), "remove"],
                       check=False)
    return 0 if ok else 3


if __name__ == "__main__":
    sys.exit(main())