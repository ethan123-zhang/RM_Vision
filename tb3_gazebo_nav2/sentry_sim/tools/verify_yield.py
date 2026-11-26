#!/usr/bin/env python3
"""让行验收（S3e）：1.2 m 走廊被敌车堵住时，哨兵不是"判死路停住"，而是
退到安全位等对方让开，路通后自己继续走完。

为什么放在客户端（和 path_guard 同因）：ROS 2 没有"按条件取消别人目标"的公开 API，
取消和重发都必须用发起方手里的 goal handle。所以让行是**任务层策略**：
  ① 盯 /scan_safety 前扇区 + path_guard 的路径净空；
  ② 前方有障碍（<1.5 m）且路径净空 <0.20 m（过不去）→ 取消目标、退 1.0 m、等；
  ③ 目标清除后等待 2 s（防抖）→ 重发原目标。
这比"死路→取消→停住"多一步：退让 + 恢复，不把任务丢在地上。

场景（默认值）：场地走廊里用两块 0.8 m 墙夹出 **1.2 m 缝**（缝在 (0,1.2)），
敌车停在缝中挡住；哨兵从 (0,0) 去 (0,2.6)。退让中途把敌车移走（模拟对手离开），
哨兵应恢复并穿过缝到达目标。

判据（逐条打印，全过返回 0）：
  1) 触发让行：前方障碍 <1.5 m 且路径净空 <0.20 m
  2) 真的退让：退让位移 >= 0.3 m
  3) 不接触：与敌车最小面距 >= 0.05 m
  4) 恢复后到达：SUCCEEDED 且到点误差 <= 0.30 m
用法：
  python3 tools/verify_yield.py
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

os.environ.setdefault("ROS_DOMAIN_ID", "45")

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from path_guard import clearance_grid, peek_plan_clearance

HERE = os.path.dirname(os.path.abspath(__file__))
ROBOT_HALF = 0.25
ENEMY_HALF = 0.20
WALL_L, WALL_R, ENEMY = "yield_wall_l", "yield_wall_r", "yield_enemy"


def call_spawn(*cmd):
    return subprocess.run([sys.executable, os.path.join(HERE, "spawn_obstacle.py")] + list(cmd),
                          capture_output=True, text=True).returncode


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def clock_s(node):
    return node.get_clock().now().nanoseconds * 1e-9


class Yield(Node):

    def __init__(self):
        super().__init__('verify_yield', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.scan = None
        self.plan = None
        self.plan_t = None
        self.cmap = None
        self.gt = None
        qos_be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, '/scan_safety',
                                 lambda m: setattr(self, 'scan', m), qos_be)
        self.create_subscription(Odometry, '/gt_odom',
                                 lambda m: setattr(self, 'gt', m), qos_be)
        self.create_subscription(Path, '/plan', self.on_plan, 10)
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 lambda m: setattr(self, 'cmap', m), qos)

    def on_plan(self, msg):
        self.plan = msg
        self.plan_t = time.monotonic()

    def pose(self):
        try:
            tr = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except Exception:                                            # noqa: BLE001
            return None
        t = tr.transform.translation
        return t.x, t.y, yaw_of(tr.transform.rotation)

    def front_min(self):
        """前扇区 ±20° 的最小有限量程（m）。"""
        if self.scan is None:
            return None
        vals = []
        for i, r in enumerate(self.scan.ranges):
            if not (0.0 < r < self.scan.range_max):
                continue
            a = self.scan.angle_min + i * self.scan.angle_increment
            if abs(math.atan2(math.sin(a), math.cos(a))) <= math.radians(20):
                vals.append(r)
        return min(vals) if vals else None

    def path_clear(self):
        if self.plan is None or self.cmap is None:
            return None
        i = self.cmap.info
        ct = (list(self.cmap.data), i.origin.position.x, i.origin.position.y,
              i.resolution, i.width, i.height)
        info = peek_plan_clearance(self.plan, ct, grid=clearance_grid(ct))
        return None if info is None else info['min_clear']

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def send_goal(self, x, y, yaw=0.0):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        fut = self.client.send_goal_async(goal)
        while not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        return fut.result()

    def wait_result(self, handle, timeout):
        fut = handle.get_result_async()
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not fut.done():
            return None
        return fut.result()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--goal', nargs=2, type=float, default=(0.0, 2.6))
    ap.add_argument('--gap-at', nargs=2, type=float, default=(0.0, 1.2))
    ap.add_argument('--retreat', type=float, default=1.0)
    ap.add_argument('--wait', type=float, default=2.0)
    ap.add_argument('--timeout', type=float, default=150.0)
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    for name in (WALL_L, WALL_R, ENEMY):
        call_spawn('remove', '--name', name)
    gx_at, gy_at = args.gap_at
    # 缝宽 1.2 m：两块 0.8 m 墙放在 x=±1.0（内侧 -0.6/+0.6），两侧到走廊墙只剩 0.2/0.1 m
    call_spawn('spawn', str(gx_at - 1.0), str(gy_at), '--size', '0.8', '--name', WALL_L)
    call_spawn('spawn', str(gx_at + 1.0), str(gy_at), '--size', '0.8', '--name', WALL_R)
    call_spawn('spawn', str(gx_at), str(gy_at), '--dynamic', '--name', ENEMY)
    time.sleep(2.0)

    rclpy.init()
    node = Yield()
    if not node.client.wait_for_server(timeout_sec=30.0):
        print('FAIL: /navigate_to_pose 未就绪')
        return 3
    t0 = time.monotonic()
    while node.pose() is None and time.monotonic() - t0 < 20:
        rclpy.spin_once(node, timeout_sec=0.1)
    start = node.pose()
    if start is None:
        print('FAIL: 拿不到 map->base_footprint')
        return 3
    print(f'START ({start[0]:.2f},{start[1]:.2f}) -> GOAL {args.goal}；'
          f'1.2 m 缝在 {args.gap_at}，敌车堵在缝中')

    handle = node.send_goal(args.goal[0], args.goal[1])
    if handle is None or not handle.accepted:
        print('FAIL: 目标被拒绝')
        return 3
    # 结果 future 只取一次：rclpy 的 goal handle 重复 get_result_async() 时，
    # 若 action server 的结果响应已被消费，后续 future 可能永远不 resolve
    # （2026-11-25 踩：车早就到点了，done 一直 False，白等 95 s）。
    rfut = handle.get_result_async()

    triggered = False
    backed = 0.0
    retreat_pose = None
    min_center = 999.0
    enemy_present = True          # 敌车被移走后，"离 spawn 点的距离"不再有接触含义
    resume_count = 0
    cleared_at = None
    result = None
    stage = 'drive'
    sim0 = node.get_clock().now().nanoseconds * 1e-9
    while node.get_clock().now().nanoseconds * 1e-9 - sim0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.02)
        p = node.pose()
        if p is not None and node.gt is not None and enemy_present:
            min_center = min(min_center, math.hypot(p[0] - args.gap_at[0],
                                                    p[1] - args.gap_at[1]))
        if stage == 'drive':
            fm, cl = node.front_min(), node.path_clear()
            no_plan = (time.monotonic() - node.plan_t) > 1.5 if node.plan_t else True
            elapsed = node.get_clock().now().nanoseconds * 1e-9 - sim0
            goal_done = rfut is not None and rfut.done()
            # 触发让行：前方确实有障碍，且（路径判死路 或 规划器根本给不出路径）
            blocked = (fm is not None and fm < 1.5
                       and ((cl is not None and cl < 0.20) or (no_plan and elapsed > 3.0)))
            if blocked or (goal_done and fm is not None and fm < 1.5):
                triggered = True
                retreat_pose = p
                if not goal_done:
                    handle.cancel_goal_async()
                node.spin(0.3)
                yaw = p[2]
                rx = p[0] - args.retreat * math.cos(yaw)
                ry = p[1] - args.retreat * math.sin(yaw)
                print(f'YIELD trigger: 前方 {fm:.2f} m、路径净空 {cl} m、无路径={no_plan} '
                      f'-> 退到 ({rx:+.2f},{ry:+.2f})', flush=True)
                handle = node.send_goal(rx, ry, yaw)
                rfut = handle.get_result_async() if handle is not None else None
                stage = 'retreat'
                retreat_t0 = time.monotonic()
            elif goal_done:
                result = rfut.result()
                print('目标在未触发让行前就结束了，按失败处理', flush=True)
                break
        elif stage == 'retreat':
            p = node.pose()
            if p is not None and retreat_pose is not None:
                backed = max(backed, math.hypot(p[0] - retreat_pose[0],
                                                p[1] - retreat_pose[1]))
            done = rfut is not None and rfut.done()
            if done or time.monotonic() - retreat_t0 > 20:
                print(f'YIELD retreated {backed:.2f} m；开始等待路通', flush=True)
                stage = 'wait'
                wait_t0 = time.monotonic()
        elif stage == 'wait':
            if cleared_at is None and time.monotonic() - wait_t0 > 1.0:
                # 模拟对手离开：把敌车移走
                call_spawn('remove', '--name', ENEMY)
                enemy_present = False
                cleared_at = time.monotonic()
                print(f'CLEAR: 敌车移走（t={clock_s(node) - sim0:.1f} s sim），'
                      f'等 2 s 后恢复原目标', flush=True)
            fm = node.front_min()
            if cleared_at is not None and time.monotonic() - cleared_at >= args.wait:
                resume_count += 1
                print(f'RESUME #{resume_count}: 重发原目标 {args.goal}（前方 {fm} m）', flush=True)
                handle = node.send_goal(args.goal[0], args.goal[1])
                rfut = handle.get_result_async() if handle is not None else None
                stage = 'finish'
        elif stage == 'finish':
            done_now = rfut is not None and rfut.done()
            t_sim = node.get_clock().now().nanoseconds * 1e-9 - sim0
            if int(t_sim) % 5 == 0 and int(t_sim) != getattr(node, '_last_log', -1):
                node._last_log = int(t_sim)
                q = node.pose()
                print(f'  [finish t={t_sim:.0f}s] pose={None if q is None else [round(v, 2) for v in q]} '
                      f'done={done_now} front={node.front_min()}', flush=True)
            if done_now:
                result = rfut.result()
                if int(result.status) == 4 or resume_count >= 3:
                    break
                fm = node.front_min()
                if fm is not None and fm > 1.2:
                    resume_count += 1
                    print(f'RESUME retry #{resume_count}（前方 {fm:.2f} m 已通）', flush=True)
                    handle = node.send_goal(args.goal[0], args.goal[1])
                    rfut = handle.get_result_async() if handle is not None else None
                else:
                    break

    if result is None and rfut is not None:
        end = time.monotonic() + 10
        while not rfut.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        result = rfut.result() if rfut.done() else None

    end_pose = node.pose()
    status = int(result.status) if result is not None else -1
    name = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(status, f'NONE/{status}')
    arrival = (math.hypot(end_pose[0] - args.goal[0], end_pose[1] - args.goal[1])
               if end_pose else 999.0)
    face = min_center - (ROBOT_HALF + ENEMY_HALF)
    dur = node.get_clock().now().nanoseconds * 1e-9 - sim0

    for nm in (WALL_L, WALL_R, ENEMY):
        call_spawn('remove', '--name', nm)
    node.destroy_node()
    rclpy.shutdown()

    checks = [
        ('触发让行(前方<1.5m 且净空<0.20m)', triggered, f'triggered={triggered}'),
        ('退让位移>=0.3 m', backed >= 0.3, f'{backed:.2f} m'),
        ('不接触敌车(面距>=0.05 m)', face >= 0.05, f'最小中心距 {min_center:.3f} m -> 面距 {face:+.3f} m'),
        ('恢复后到达', name == 'SUCCEEDED' and arrival <= 0.30,
         f'{name}，到点 {arrival:.2f} m，恢复 {resume_count} 次'),
        ('没磨满超时', dur < args.timeout, f'{dur:.1f} s / {args.timeout:.0f} s'),
    ]
    print('\n===== 判据 =====')
    ok = True
    for label, passed, detail in checks:
        ok = ok and passed
        print(f"  [{'PASS' if passed else 'FAIL'}] {label}: {detail}")
    payload = {'pass': ok, 'status': name, 'triggered': triggered,
               'backed_off_m': round(backed, 3), 'min_center_m': round(min_center, 3),
               'gap_face_m': round(face, 3), 'arrival_err_m': round(arrival, 3),
               'resumes': resume_count, 'sim_s': round(dur, 1),
               'checks': {c[0]: {'pass': c[1], 'detail': c[2]} for c in checks}}
    out = args.out or os.path.join(HERE, '..', 'results',
                                   'report_yield_%s.json' % time.strftime('%H%M%S'))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, 'w', encoding='utf-8') as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    print('报告:', os.path.abspath(out))
    return 0 if ok else 3


if __name__ == '__main__':
    sys.exit(main())