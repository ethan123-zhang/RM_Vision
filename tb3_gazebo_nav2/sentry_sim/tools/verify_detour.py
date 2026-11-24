#!/usr/bin/env python3
"""验收：路径上放一个"敌方机器人"障碍，看哨兵车能不能绕开（而不是硬挤/撞墙）。

判据（逐条打印，全过才返回 0）：
  1) NavigateToPose 结果 SUCCEEDED，到点误差 <= 0.30 m
  2) 与障碍**表面**的最小间距 >= 0.18 m（中心距 >= 障碍半宽 + 0.36 车体半径 + 0.18）
  3) 与静态地图（墙/平台）的最小净空 >= 0.05 m —— 没蹭墙
  4) 轨迹相对"起点→终点直线"的最大横向偏移 >= 0.40 m —— 真的绕了，不是硬挤过去
  5) 最长单段停滞 <= 5 s（仿真时间）—— 没有陷入"规划失败→恢复"循环
另外打印最大速度 / 用时 / 路径长度，供速度调参用。

用法（栈已在跑、domain 45）：
  python3 tools/verify_detour.py --goal 3 -2.5 --obstacle 1.5 -1.25
  python3 tools/verify_detour.py --goal 3 -2.5 --obstacle 1.5 -1.25 --spawn
  （--spawn 会先调 spawn_obstacle.py 生成，--remove 结束时删掉）
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

os.environ.setdefault("ROS_DOMAIN_ID", "45")

import numpy as np                                                   # noqa: E402
from PIL import Image                                                # noqa: E402
from scipy import ndimage                                            # noqa: E402

import rclpy                                                         # noqa: E402
from rclpy.action import ActionClient                                # noqa: E402
from rclpy.node import Node                                          # noqa: E402
from rclpy.parameter import Parameter                                # noqa: E402
from nav2_msgs.action import NavigateToPose                          # noqa: E402
from tf2_ros import Buffer, TransformListener                        # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MAP = "/mnt/e/RM/tb3_gazebo_nav2/rm_field/maps/rmul2027_3v3_up.yaml"
ROBOT_RADIUS = 0.354        # 0.5x0.5 车体外接半径


def load_map(yaml_path):
    cfg = {}
    with open(yaml_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if ":" in line:
                k, v = line.split(":", 1)
                cfg[k.strip()] = v.strip()
    res = float(cfg["resolution"])
    ox, oy = [float(x) for x in cfg["origin"].strip("[] ").split(",")[:2]]
    img = Image.open(os.path.join(os.path.dirname(yaml_path), cfg["image"]))
    occ = np.array(img) < 80
    clear = ndimage.distance_transform_edt(~occ) * res
    return res, ox, oy, occ, clear


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--goal", nargs=2, type=float, required=True, metavar=("X", "Y"))
    ap.add_argument("--obstacle", nargs=2, type=float, metavar=("X", "Y"),
                    help="静态敌车位置；用 --moving 时改成起点")
    ap.add_argument("--moving", nargs=4, type=float,
                    metavar=("X1", "Y1", "X2", "Y2"),
                    help="移动敌车：在 (X1,Y1)↔(X2,Y2) 之间往返")
    ap.add_argument("--moving-speed", type=float, default=0.4, help="移动敌车速度 m/s（墙钟）")
    ap.add_argument("--obstacle-size", type=float, default=0.40)
    ap.add_argument("--map", default=DEFAULT_MAP)
    ap.add_argument("--spawn", action="store_true", help="先 spawn 障碍")
    ap.add_argument("--spawn-after", type=float, default=0.0,
                    help="延迟这么多秒再 spawn（测'突然出现在路径上'）")
    ap.add_argument("--remove", action="store_true", help="结束后删障碍")
    ap.add_argument("--timeout", type=float, default=240.0, help="墙钟上限 s")
    ap.add_argument("--dump", default="",
                    help="把 TF 轨迹写成 CSV（sim_s,wall_s,x,y），供事后看绕行/抖动")
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    gx, gy = args.goal
    if args.moving:
        args.obstacle = [args.moving[0], args.moving[1]]
        args.spawn = True
    if not args.obstacle:
        ap.error("要给 --obstacle 或 --moving")
    ox_w, oy_w = args.obstacle
    moving_track = []        # (wall_t, x, y)，只在 --moving 时填
    mover_stop = None

    def do_spawn():
        cmd = [sys.executable, os.path.join(HERE, "spawn_obstacle.py"),
               "spawn", str(ox_w), str(oy_w), "--size", str(args.obstacle_size)]
        if args.moving:
            cmd.append("--dynamic")
        rc = subprocess.run(cmd).returncode
        print(f"[spawn] rc={rc}")
        return rc

    if args.spawn:
        if args.spawn_after > 0:
            # "突然出现在路径上"：车先跑一会儿，再 spawn（测试它识别/停车能力）
            import threading

            def _later():
                time.sleep(args.spawn_after)
                do_spawn()

            threading.Thread(target=_later, daemon=True).start()
            print(f"[spawn] {args.spawn_after:.1f} s 后出现在 ({ox_w:.2f},{oy_w:.2f})")
        else:
            if do_spawn() != 0:
                return 4

    if args.moving:
        import threading
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.pose_pb2 import Pose
        from gz.transport13 import Node as GzNode
        from moving_obstacle import pose_at
        gw = GzNode()
        x1, y1, x2, y2 = args.moving
        mover_stop = threading.Event()

        def _drive():
            t0 = time.monotonic()
            while not mover_stop.is_set():
                t = time.monotonic() - t0
                x, y = pose_at(t, x1, y1, x2, y2, args.moving_speed)
                req = Pose(name="enemy_robot")
                req.position.x = x
                req.position.y = y
                req.position.z = 0.125
                req.orientation.w = 1.0
                gw.request("/world/rmul2027_3v3/set_pose", req, Pose, Boolean, 800)
                moving_track.append((time.monotonic(), x, y))
                time.sleep(0.1)

        threading.Thread(target=_drive, daemon=True).start()
        print(f"[moving] 敌车 ({x1:.2f},{y1:.2f})↔({x2:.2f},{y2:.2f}) @ {args.moving_speed} m/s")

    res, mox, moy, occ, clear = load_map(args.map)
    H = occ.shape[0]

    def static_clear(x, y):
        r = H - 1 - int(round((y - moy) / res))
        c = int(round((x - mox) / res))
        if not (0 <= r < H and 0 <= c < occ.shape[1]):
            return -1.0
        return float(clear[r, c])

    rclpy.init()
    node = Node("verify_detour",
                parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    tf = Buffer()
    TransformListener(tf, node)
    client = ActionClient(node, NavigateToPose, "navigate_to_pose")
    # 30 s：WSL 上两次重启之间 DDS 发现偶发要十几秒（早先 15 s 会假失败）
    if not client.wait_for_server(timeout_sec=30.0):
        print("FAIL: /navigate_to_pose 30 s 内没就绪（栈起了吗？）")
        node.destroy_node()
        rclpy.shutdown()
        return 3

    def pose_now():
        try:
            tr = tf.lookup_transform("map", "base_footprint", rclpy.time.Time())
        except Exception:                                            # noqa: BLE001
            return None
        t = tr.transform.translation
        return t.x, t.y, yaw_of(tr.transform.rotation)

    # 等 TF：按墙钟计上限（早先是"200 次 spin"，WSL 上消息密集时 200 次不到 1 s 就转完，
    # 栈刚起来时偶发"拿不到 map->base_footprint"假失败）。
    deadline = time.monotonic() + 20.0
    while pose_now() is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    start = pose_now()
    if start is None:
        print("FAIL: 拿不到 map->base_footprint")
        node.destroy_node()
        rclpy.shutdown()
        return 3

    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = gx
    goal.pose.pose.position.y = gy
    goal.pose.pose.orientation.w = 1.0
    print(f"START ({start[0]:.3f},{start[1]:.3f}) -> GOAL ({gx:.2f},{gy:.2f}), "
          f"障碍 ({ox_w:.2f},{oy_w:.2f}) 边长 {args.obstacle_size:.2f}"
          + (f"，移动 {args.moving} @ {args.moving_speed} m/s" if args.moving else "，静态"))

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
    samples = []                                                     # (sim_s, wall_s, x, y)
    last_sample_t = 0.0
    while not res_fut.done() and time.monotonic() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.02)
        now = node.get_clock().now().nanoseconds / 1e9
        if now - last_sample_t >= 0.10:
            p = pose_now()
            if p is not None:
                samples.append((now, time.monotonic(), p[0], p[1]))
                last_sample_t = now
    result_status = None
    if res_fut.done():
        result_status = int(res_fut.result().status)
    else:
        gh.cancel_goal_async()
        print("TIMEOUT: 目标没在期限内结束，已取消")
    p_end = pose_now()
    if p_end is not None:
        samples.append((node.get_clock().now().nanoseconds / 1e9, time.monotonic(),
                        p_end[0], p_end[1]))
    if mover_stop is not None:
        mover_stop.set()
    node.destroy_node()
    rclpy.shutdown()

    name = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}.get(result_status,
                                                              f"NONE/{result_status}")
    arrival = math.hypot(p_end[0] - gx, p_end[1] - gy) if p_end else 999.0
    straight = math.hypot(gx - start[0], gy - start[1])
    path_len = 0.0
    max_speed = 0.0
    for (ta, _, xa, ya), (tb, _, xb, yb) in zip(samples, samples[1:]):
        d = math.hypot(xb - xa, yb - ya)
        path_len += d
        dt = tb - ta
        if dt > 0:
            max_speed = max(max_speed, d / dt)

    def enemy_at(wall_t):
        """移动模式：按墙钟时间在轨迹里线性插值敌车位置；静态模式返回固定点。"""
        if not moving_track:
            return ox_w, oy_w
        lo, hi = moving_track[0], moving_track[-1]
        if wall_t <= lo[0]:
            return lo[1], lo[2]
        if wall_t >= hi[0]:
            return hi[1], hi[2]
        for (t_a, xa, ya), (t_b, xb, yb) in zip(moving_track, moving_track[1:]):
            if t_a <= wall_t <= t_b:
                k = (wall_t - t_a) / max(1e-6, t_b - t_a)
                return xa + (xb - xa) * k, ya + (yb - ya) * k
        return hi[1], hi[2]

    # 横向偏移：相对起点→终点直线
    dx, dy = gx - start[0], gy - start[1]
    L = math.hypot(dx, dy) or 1.0
    ux, uy = dx / L, dy / L
    max_lat = 0.0
    min_obs = 999.0
    min_static = 999.0
    longest_stall = 0.0
    stall_start = None
    for _, wall_t, x, y in samples:
        lat = abs((x - start[0]) * uy - (y - start[1]) * ux)
        max_lat = max(max_lat, lat)
        ex, ey = enemy_at(wall_t)
        min_obs = min(min_obs, math.hypot(x - ex, y - ey))
        sc = static_clear(x, y)
        if sc >= 0:
            min_static = min(min_static, sc)
    for (ta, _, xa, ya), (tb, _, xb, yb) in zip(samples, samples[1:]):
        if math.hypot(xb - xa, yb - ya) < 0.01:
            if stall_start is None:
                stall_start = ta
        else:
            if stall_start is not None:
                longest_stall = max(longest_stall, ta - stall_start)
            stall_start = None
    if stall_start is not None and samples:
        longest_stall = max(longest_stall, samples[-1][0] - stall_start)

    obs_half = args.obstacle_size / 2.0
    # 车是 0.5x0.5 方体：正面到中心的距离 0.25；外接半径 0.354 只在斜着过时才算。
    # 走侧面平行通过时真正决定撞不撞的是"面到面"，所以判据用面距，另报保守的外接半径口径。
    gap_face = min_obs - obs_half - 0.25
    gap_circum = min_obs - obs_half - ROBOT_RADIUS
    duration = samples[-1][0] - samples[0][0] if len(samples) > 1 else 0.0

    checks = [
        ("到达(SUCCEEDED, <=0.30 m)", name == "SUCCEEDED" and arrival <= 0.30,
         f"{name} 到点误差 {arrival:.3f} m"),
        ("绕障面距(>=0.05 m)", gap_face >= 0.05,
         f"中心距 {min_obs:.3f} m → 面距 {gap_face:+.3f} m（外接半径口径 {gap_circum:+.3f} m）"),
        ("不蹭墙(静态净空>=0.05 m)", min_static >= 0.05,
         f"最小静态净空 {min_static:.3f} m"),
        ("确实绕行(横向偏移>=0.40 m)", max_lat >= 0.40,
         f"最大横向偏移 {max_lat:.3f} m"),
        ("无死循环(停滞<=5 s)", longest_stall <= 5.0,
         f"最长停滞 {longest_stall:.1f} s"),
    ]
    ok = all(c[1] for c in checks)
    print("\n===== 判据 =====")
    for label, passed, detail in checks:
        print(f"  [{'PASS' if passed else 'FAIL'}] {label}: {detail}")
    print(f"\n用时 {duration:.1f} s（仿真）  路径 {path_len:.2f} m（直线 {straight:.2f} m）"
          f"  最大速度 {max_speed:.2f} m/s  采样 {len(samples)} 点")
    out = args.out or os.path.join(HERE, "..", "results",
                                   f"report_detour_{time.strftime('%H%M%S')}.json")
    payload = {
        "pass": ok, "result": name, "arrival_m": round(arrival, 3),
        "straight_m": round(straight, 3), "path_m": round(path_len, 3),
        "max_speed_mps": round(max_speed, 3), "duration_sim_s": round(duration, 2),
        "min_obstacle_center_m": round(min_obs, 3),
        "gap_face_m": round(gap_face, 3),
        "gap_circumscribed_m": round(gap_circum, 3),
        "min_static_clearance_m": round(min_static, 3),
        "max_lateral_offset_m": round(max_lat, 3),
        "longest_stall_sim_s": round(longest_stall, 2),
        "start": [round(start[0], 3), round(start[1], 3)],
        "goal": [gx, gy], "obstacle": [ox_w, oy_w],
        "moving": bool(args.moving),
        "moving_line": args.moving or None,
        "moving_speed": args.moving_speed if args.moving else 0.0,
        "checks": {c[0]: {"pass": c[1], "detail": c[2]} for c in checks},
    }
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    print(f"报告: {os.path.abspath(out)}")
    if args.dump:
        with open(args.dump, "w", encoding="utf-8") as f:
            f.write("sim_s,wall_s,x,y\n")
            for sim_t, wall_t, x, y in samples:
                f.write(f"{sim_t:.3f},{wall_t:.3f},{x:.4f},{y:.4f}\n")
        print(f"轨迹: {os.path.abspath(args.dump)}")

    if args.remove:
        subprocess.run([sys.executable, os.path.join(HERE, "spawn_obstacle.py"), "remove"])
    return 0 if ok else 3


if __name__ == "__main__":
    sys.exit(main())