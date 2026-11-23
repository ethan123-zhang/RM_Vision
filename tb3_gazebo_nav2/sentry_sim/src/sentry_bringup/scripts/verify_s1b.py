#!/usr/bin/env python3
"""S1b acceptance probe: /scan ring, TF chain, two NavigateToPose goals.

Run with the S1b stack up (ROS_DOMAIN_ID=45, use_sim_time).
Prints one JSON line prefixed S1B_JSON and a human-readable log on stderr-free
stdout lines prefixed '-- '.
"""

import json
import math
import sys
import time

import numpy as np

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

import tf2_ros
from nav2_msgs.action import NavigateToPose

ODOM_QOS = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST)
SCAN_QOS = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST)
MAP_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST)


STATUS_NAMES = {
    0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
    4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED',
}


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class Probe(Node):

    def __init__(self):
        super().__init__(
            's1b_verify',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.scans = []
        self.odom = []
        self.gt = []
        self.map = None
        self.tf_edges = {}
        self.recording = False
        self.trace = []
        self.cmd_vel = []

        self.create_subscription(LaserScan, '/scan', self._scan, SCAN_QOS)
        self.create_subscription(Odometry, '/odom', self._odom, ODOM_QOS)
        self.create_subscription(Odometry, '/gt_odom', self._gt, ODOM_QOS)
        self.create_subscription(OccupancyGrid, '/map', self._map, MAP_QOS)
        self.create_subscription(TFMessage, '/tf', self._tf, 50)
        self.create_subscription(TFMessage, '/tf_static', self._tf_static, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _scan(self, msg):
        self.scans.append(msg)

    def _odom(self, msg):
        self.odom.append(msg)

    def _gt(self, msg):
        self.gt.append(msg)
        if self.recording:
            p = msg.pose.pose
            self.trace.append((self.sim_now(), p.position.x, p.position.y,
                               yaw_from_quat(p.orientation.x, p.orientation.y,
                                             p.orientation.z, p.orientation.w)))

    def _map(self, msg):
        self.map = msg

    def _tf(self, msg):
        for tr in msg.transforms:
            self.tf_edges.setdefault(tr.child_frame_id, set()).add(tr.header.frame_id)

    def _tf_static(self, msg):
        self._tf(msg)

    def sim_now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def wait_sim_clock(self, timeout=180.0):
        w0 = time.monotonic()
        while self.sim_now() == 0.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() - w0 > timeout:
                raise RuntimeError('no /clock within %.0f s' % timeout)
        return time.monotonic() - w0

    def pump(self, sim_sec, wall_cap=None):
        t0 = self.sim_now()
        w0 = time.monotonic()
        if wall_cap is None:
            wall_cap = sim_sec * 25.0 + 60.0
        while self.sim_now() - t0 < sim_sec:
            rclpy.spin_once(self, timeout_sec=0.01)
            if time.monotonic() - w0 > wall_cap:
                raise RuntimeError('sim clock stalled')

    def lookup(self, parent, child, tries=100):
        for _ in range(tries):
            rclpy.spin_once(self, timeout_sec=0.02)
            try:
                return self.tf_buffer.lookup_transform(parent, child,
                                                       rclpy.time.Time())
            except Exception:  # noqa: BLE001 - tf2 raises many types
                continue
        return None


def finite_ranges(vals, rmax):
    out = []
    for v in vals:
        if v != v or v == 0.0 or math.isinf(v) or abs(v) > rmax + 1e-6:
            continue
        out.append(v)
    return out


def scan_stats(scans):
    allf = []
    for s in scans:
        allf.extend(finite_ranges(s.ranges, s.range_max))
    if not allf:
        return {'finite_rays': 0}
    s = scans[-1]
    ns = len(s.ranges)
    sectors = {}
    for k in range(8):
        lo = s.angle_min + (s.angle_max - s.angle_min) * k / 8.0
        hi = s.angle_min + (s.angle_max - s.angle_min) * (k + 1) / 8.0
        i0 = max(0, int((lo - s.angle_min) / s.angle_increment))
        i1 = min(ns, int((hi - s.angle_min) / s.angle_increment))
        vals = finite_ranges(s.ranges[i0:i1], s.range_max)
        sectors['%ddeg' % int(round(math.degrees((lo + hi) / 2.0)))] = (
            round(min(vals), 2) if vals else None)
    return {
        'msgs': len(scans),
        'frame': s.header.frame_id,
        'range_max': s.range_max,
        'finite_rays': len(allf),
        'r_min': round(min(allf), 2),
        'r_max': round(max(allf), 2),
        'r_median': round(float(np.median(allf)), 2),
        'frac_3.3_3.5': round(sum(1 for v in allf if 3.3 <= v <= 3.5) / len(allf), 3),
        'frac_ge_3.6': round(sum(1 for v in allf if v >= 3.6) / len(allf), 3),
        'frac_ge_4.3': round(sum(1 for v in allf if v >= 4.3) / len(allf), 3),
        'sector_min_range': sectors,
    }


def map_clearance(grid):
    """Return (occupied_xy Nx2, resolution, origin) from an OccupancyGrid."""
    data = np.asarray(grid.data, dtype=np.int8).reshape(grid.info.height,
                                                       grid.info.width)
    occ = np.argwhere(data >= 65)
    xs = grid.info.origin.position.x + (occ[:, 1] + 0.5) * grid.info.resolution
    ys = grid.info.origin.position.y + (occ[:, 0] + 0.5) * grid.info.resolution
    return np.column_stack([xs, ys]), grid.info.resolution


def min_clearance(occ, points):
    if len(points) == 0 or len(occ) == 0:
        return None, None
    pts = np.asarray(points, dtype=float)
    best = 1e9
    best_at = None
    for x, y in pts:
        d = np.min(np.hypot(occ[:, 0] - x, occ[:, 1] - y))
        if d < best:
            best = d
            best_at = (round(float(x), 3), round(float(y), 3))
    return round(float(best), 3), best_at


def pose_of(tf):
    t = tf.transform.translation
    r = tf.transform.rotation
    return (t.x, t.y, yaw_from_quat(r.x, r.y, r.z, r.w))


class Nav:

    def __init__(self, node):
        self.node = node
        self.client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    def wait_server(self, timeout=90.0):
        w0 = time.monotonic()
        while not self.client.server_is_ready():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.monotonic() - w0 > timeout:
                return False
        return True

    def goto(self, x, y, yaw, wall_timeout=180.0):
        node = self.node
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        qx, qy, qz, qw = quat_from_yaw(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        node.trace = []
        node.recording = True
        sim0 = node.sim_now()
        wall0 = time.monotonic()
        send = self.client.send_goal_async(goal, feedback_callback=self._feedback)
        while not send.done():
            rclpy.spin_once(node, timeout_sec=0.02)
        gh = send.result()
        if gh is None or not gh.accepted:
            node.recording = False
            return {'accepted': False}

        rf = gh.get_result_async()
        while not rf.done() and time.monotonic() - wall0 < wall_timeout:
            rclpy.spin_once(node, timeout_sec=0.02)
        node.recording = False
        if not rf.done():
            gh.cancel_goal_async()
            return {'accepted': True, 'timed_out': True,
                    'wall_s': round(time.monotonic() - wall0, 1)}

        resp = rf.result()
        sim_dt = node.sim_now() - sim0
        wall_dt = time.monotonic() - wall0
        return {
            'accepted': True,
            'status': int(resp.status),
            'status_name': STATUS_NAMES.get(int(resp.status), 'UNKNOWN'),
            'error_code': int(resp.result.error_code),
            'error_msg': resp.result.error_msg,
            'sim_s': round(sim_dt, 2),
            'wall_s': round(wall_dt, 2),
        }

    def _feedback(self, msg):
        fb = msg.feedback
        self.last_feedback = {
            'distance_remaining': round(float(fb.distance_remaining), 3),
            'number_of_recoveries': int(fb.number_of_recoveries),
        }


def main():
    rclpy.init()
    node = Probe()
    out = {}

    out['clock_wait_wall_s'] = round(node.wait_sim_clock(), 2)

    w0 = time.monotonic()
    while not (node.scans and node.odom and node.gt and node.map is not None):
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.monotonic() - w0 > 90.0:
            break
    out['topics_ready_wall_s'] = round(time.monotonic() - w0, 2)
    out['counts_at_ready'] = {'scan': len(node.scans), 'odom': len(node.odom),
                              'gt': len(node.gt), 'map': node.map is not None}

    if node.map is None:
        print('S1B_JSON ' + json.dumps({'error': 'no /map'}), flush=True)
        return 1

    out['map'] = {
        'frame_id': node.map.header.frame_id,
        'width': node.map.info.width,
        'height': node.map.info.height,
        'resolution': node.map.info.resolution,
        'origin': [node.map.info.origin.position.x, node.map.info.origin.position.y],
        'occupied_cells': int(sum(1 for v in node.map.data if v >= 65)),
        'data_len': len(node.map.data),
    }
    occ, _ = map_clearance(node.map)

    # --- rates over a 5 sim s rest window ---
    node.scans.clear()
    node.odom.clear()
    w0 = time.monotonic()
    sim0 = node.sim_now()
    node.pump(5.0)
    sim_dt = node.sim_now() - sim0
    wall_dt = time.monotonic() - w0
    out['rtf'] = round(sim_dt / wall_dt, 3)
    out['rates'] = {'scan_hz': round(len(node.scans) / sim_dt, 2),
                    'odom_hz': round(len(node.odom) / sim_dt, 2)}
    out['scan'] = scan_stats(node.scans)

    # --- TF chain ---
    tf = {}
    for parent, child in (('map', 'base_footprint'), ('odom', 'base_footprint'),
                          ('map', 'odom'), ('base_footprint', 'base_link')):
        tr = node.lookup(parent, child, tries=50)
        tf['%s->%s' % (parent, child)] = (
            {'ok': True, 'xyz': [round(v, 3) for v in
                                 (tr.transform.translation.x,
                                  tr.transform.translation.y,
                                  tr.transform.translation.z)],
             'yaw': round(yaw_from_quat(tr.transform.rotation.x,
                                        tr.transform.rotation.y,
                                        tr.transform.rotation.z,
                                        tr.transform.rotation.w), 3)}
            if tr is not None else {'ok': False})
    out['tf'] = tf
    multi = {c: sorted(p) for c, p in node.tf_edges.items() if len(p) > 1}
    out['tf_multi_parent'] = multi
    out['tf_frames'] = len(node.tf_edges)

    # --- /odom health ---
    bad = 0
    for m in node.odom:
        p = m.pose.pose
        vals = (p.position.x, p.position.y, p.position.z,
                p.orientation.x, p.orientation.y, p.orientation.z,
                p.orientation.w)
        if any(not math.isfinite(v) for v in vals):
            bad += 1
    out['odom_nan_msgs'] = bad

    # --- navigation ---
    nav = Nav(node)
    out['action_server_ready'] = nav.wait_server(120.0)

    legs = []
    for name, (gx, gy, gyaw) in (('A', (3.0, 0.0, 0.0)), ('B', (0.0, 3.0, 0.0))):
        before = node.lookup('map', 'base_footprint', tries=50)
        start = pose_of(before) if before is not None else None
        res = nav.goto(gx, gy, gyaw)
        after = node.lookup('map', 'base_footprint', tries=50)
        end = pose_of(after) if after is not None else None
        gt_end = None
        if node.trace:
            gt_end = node.trace[-1][1:4]
        dist = None
        if end is not None:
            dist = round(math.hypot(end[0] - gx, end[1] - gy), 3)
        gt_dist = None
        if gt_end is not None:
            gt_dist = round(math.hypot(gt_end[0] - gx, gt_end[1] - gy), 3)
        trace_xy = [(t[1], t[2]) for t in node.trace]
        c, c_at = min_clearance(occ, trace_xy)
        legs.append({
            'goal': name, 'target': [gx, gy, gyaw],
            'start_tf': [round(v, 3) for v in start] if start else None,
            'end_tf': [round(v, 3) for v in end] if end else None,
            'end_gt': [round(v, 3) for v in gt_end] if gt_end else None,
            'dist_to_goal_tf_m': dist,
            'dist_to_goal_gt_m': gt_dist,
            'min_clearance_m': c,
            'min_clearance_at': c_at,
            'trace_points': len(trace_xy),
            'feedback_last': getattr(nav, 'last_feedback', None),
            'result': res,
        })

    out['legs'] = legs
    print('S1B_JSON ' + json.dumps(out), flush=True)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())