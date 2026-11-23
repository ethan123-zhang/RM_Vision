#!/usr/bin/env python3
"""S1a acceptance probe: rates, RTF, cmd_vel tracking error, TF."""

import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan, PointCloud2

import tf2_ros

QOS = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST)


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class Probe(Node):

    def __init__(self):
        super().__init__(
            's1a_verify',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.count = {'cloud': 0, 'imu': 0, 'odin': 0, 'scan': 0, 'odom': 0}
        self.cloud_pts = []
        self.scan_n = []
        self.last_odom = None
        self.last_odin = None
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(PointCloud2, '/livox/lidar', self._cloud, QOS)
        self.create_subscription(Imu, '/livox/imu', self._imu, QOS)
        self.create_subscription(Odometry, '/odin1/odometry', self._odin, QOS)
        self.create_subscription(LaserScan, '/scan', self._scan, QOS)
        self.create_subscription(Odometry, '/odom', self._odom, QOS)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _cloud(self, msg):
        self.count['cloud'] += 1
        self.cloud_pts.append(msg.width * msg.height)

    def _imu(self, msg):
        self.count['imu'] += 1

    def _odin(self, msg):
        self.count['odin'] += 1
        self.last_odin = msg

    def _scan(self, msg):
        self.count['scan'] += 1
        self.scan_n.append(len(msg.ranges))

    def _odom(self, msg):
        self.count['odom'] += 1
        self.last_odom = msg

    def sim_now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def pump(self, sim_sec, pub=None, msg=None, rate=20.0, wall_cap=None, trace=None):
        t0 = self.sim_now()
        if wall_cap is None:
            wall_cap = sim_sec * 20.0 + 30.0
        w0 = time.monotonic()
        nxt = t0
        nxt_trace = t0
        while self.sim_now() - t0 < sim_sec:
            if pub is not None and self.sim_now() >= nxt:
                pub.publish(msg)
                nxt += 1.0 / rate
            if trace is not None and self.sim_now() >= nxt_trace and self.last_odom is not None:
                p = self.last_odom.pose.pose
                q = p.orientation
                trace.append([round(self.sim_now() - t0, 2),
                              round(p.position.x, 3), round(p.position.y, 3),
                              round(p.position.z, 3),
                              round(yaw_from_quat(q), 3)])
                nxt_trace += 0.25
            rclpy.spin_once(self, timeout_sec=0.002)
            if time.monotonic() - w0 > wall_cap:
                raise RuntimeError('sim clock stalled for %.1f s wall' % wall_cap)


def main():
    rclpy.init()
    node = Probe()
    out = {}

    w0 = time.monotonic()
    while node.sim_now() == 0.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.monotonic() - w0 > 180.0:
            print(json.dumps({'error': 'no /clock within 180 s'}), flush=True)
            return 1
    out['clock_wait_wall_s'] = round(time.monotonic() - w0, 2)

    ready_wall = time.monotonic()
    for _ in range(1200):
        rclpy.spin_once(node, timeout_sec=0.05)
        if (node.count['odom'] and node.count['cloud'] and node.count['imu']
                and node.count['scan'] and node.count['odin']):
            break
    out['topics_ready_wall_s'] = round(time.monotonic() - ready_wall, 2)
    out['ready_counts'] = dict(node.count)

    # --- rate window (5 sim s, robot at rest) ---
    for key in node.count:
        node.count[key] = 0
    node.cloud_pts.clear()
    node.scan_n.clear()
    wall0 = time.monotonic()
    sim0 = node.sim_now()
    node.pump(5.0)
    sim_dt = node.sim_now() - sim0
    wall_dt = time.monotonic() - wall0
    out['rtf'] = round(sim_dt / wall_dt, 3)
    out['window_sim_s'] = round(sim_dt, 2)
    out['rates'] = {
        'livox_lidar_hz': round(node.count['cloud'] / sim_dt, 2),
        'livox_imu_hz': round(node.count['imu'] / sim_dt, 2),
        'odin1_odometry_hz': round(node.count['odin'] / sim_dt, 2),
        'scan_hz': round(node.count['scan'] / sim_dt, 2),
        'odom_hz': round(node.count['odom'] / sim_dt, 2),
    }
    out['cloud_points'] = {
        'msgs': len(node.cloud_pts),
        'min': min(node.cloud_pts) if node.cloud_pts else 0,
        'max': max(node.cloud_pts) if node.cloud_pts else 0,
    }
    out['scan_ranges'] = {
        'msgs': len(node.scan_n),
        'min': min(node.scan_n) if node.scan_n else 0,
        'max': max(node.scan_n) if node.scan_n else 0,
    }

    # --- cmd_vel tracking ---
    cmds = [
        ('linear_x', Twist()),
        ('linear_y', Twist()),
        ('angular_z', Twist()),
    ]
    cmds[0][1].linear.x = 0.3
    cmds[1][1].linear.y = 0.3
    cmds[2][1].angular.z = 0.5
    zero = Twist()

    results = []
    for name, twist in cmds:
        node.pump(1.0)
        start = node.last_odom.pose.pose
        x0, y0, yaw0 = start.position.x, start.position.y, yaw_from_quat(start.orientation)
        trace = []
        node.pump(2.0, pub=node.cmd_pub, msg=twist, trace=trace)
        node.pump(0.7, pub=node.cmd_pub, msg=zero)
        end = node.last_odom.pose.pose
        dx = end.position.x - x0
        dy = end.position.y - y0
        dyaw = wrap(yaw_from_quat(end.orientation) - yaw0)
        results.append({'cmd': name, 'dx': round(dx, 4), 'dy': round(dy, 4),
                        'dyaw_rad': round(dyaw, 4), 'trace': trace})
    out['motion'] = results
    out['motion_errors_pct'] = {
        'linear_x': round(abs(results[0]['dx'] - 0.6) / 0.6 * 100.0, 2),
        'linear_y': round(abs(results[1]['dy'] - 0.6) / 0.6 * 100.0, 2),
        'angular_z': round(abs(results[2]['dyaw_rad'] - 1.0) / 1.0 * 100.0, 2),
    }

    # --- TF (S1b: standard chain, no map -> imu_link edge any more) ---
    tf_ok, tf_txt = False, ''
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.05)
        try:
            tr = node.tf_buffer.lookup_transform('map', 'base_footprint',
                                                 rclpy.time.Time())
            t = tr.transform.translation
            tf_ok = True
            tf_txt = 'xyz=(%.3f, %.3f, %.3f) frame=%s->%s' % (
                t.x, t.y, t.z, tr.header.frame_id, tr.child_frame_id)
            break
        except Exception as exc:  # noqa: BLE001 - report whatever tf2 says
            tf_txt = str(exc).splitlines()[0] if str(exc) else type(exc).__name__
    out['tf_map_to_base'] = {'ok': tf_ok, 'detail': tf_txt}

    if node.last_odin is not None:
        out['odin_frame'] = [node.last_odin.header.frame_id, node.last_odin.child_frame_id]

    print('S1A_JSON ' + json.dumps(out), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())