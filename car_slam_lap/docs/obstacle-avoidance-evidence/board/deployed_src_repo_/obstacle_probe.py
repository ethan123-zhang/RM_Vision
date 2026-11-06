"""静止点云诊断入口：不创建速度发布者，不启动底盘。"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
import tf2_ros
import numpy as np

from .localization_guard import OdomGuard
from .obstacle_avoidance import ObstacleAvoidance


class ObstacleProbe(Node):
    def __init__(self):
        super().__init__('obstacle_probe')
        self.frame, self.base_frame = 'map', 'imu'
        self.declare_parameter('base_offset_x', .2626)
        self.declare_parameter('base_offset_y', -.0471)
        self.declare_parameter('duration_s', 15.)
        self.declare_parameter('out_file', '')
        self.base_offset_x = float(self.get_parameter('base_offset_x').value)
        self.base_offset_y = float(self.get_parameter('base_offset_y').value)
        self._odom_guard = OdomGuard()
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self.create_subscription(Odometry, '/odin1/odometry', self._on_odom, qos_profile_sensor_data)
        self.avoidance = ObstacleAvoidance(self)
        self.max_odom_gap = 0.
        self._last_odom_rx = None

    def _rotation_center(self, x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        return x-c*self.base_offset_x+s*self.base_offset_y, y-s*self.base_offset_x-c*self.base_offset_y

    def _on_odom(self, msg):
        now = time.monotonic()
        if self._last_odom_rx is not None:
            self.max_odom_gap = max(self.max_odom_gap, now-self._last_odom_rx)
        self._last_odom_rx = now
        p = msg.pose.pose
        self._odom_guard.update(now, msg.header.stamp.sec, msg.header.stamp.nanosec,
                                msg.header.frame_id, msg.child_frame_id,
                                (p.position.x, p.position.y, p.position.z),
                                (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleProbe()
    started = time.monotonic()
    samples = []
    next_sample = started + 2.
    duration = float(node.get_parameter('duration_s').value)
    try:
        while rclpy.ok() and time.monotonic() - started < duration:
            rclpy.spin_once(node, timeout_sec=.02)
            now = time.monotonic()
            if now < next_sample:
                continue
            next_sample = now + 1.
            a = node.avoidance
            row = dict(elapsed_s=round(now-started, 2), reason=a.ready_reason(now),
                       odom_fresh=node._odom_guard.is_fresh(now), **a.summary())
            if a.view is not None:
                try:
                    a.set_pose()
                    row['forward_preview_clear'] = a.view.motion_free(a.control_pose, .08, 0.)
                    row['known_cells'] = int(a.view.known.sum())
                    row['occupied_inflated_cells'] = int(a.view.blocked.sum())
                except Exception as exc:
                    row['pose_error'] = str(exc)
            samples.append(row)
            print(json.dumps(row, ensure_ascii=False), flush=True)
        result = dict(samples=samples, summary=node.avoidance.summary(),
                      max_odom_gap_s=round(node.max_odom_gap, 4),
                      odom_accepted=node._odom_guard.accepted,
                      odom_rejected=node._odom_guard.rejected,
                      speed_publishers=0)
        if node.avoidance.control_pose is not None:
            a = node.avoidance
            result['rotation_center_odom'] = list(a.control_pose)
            occupied = [((k[0]+.5)*a.grid.resolution, (k[1]+.5)*a.grid.resolution)
                        for k, hit in a.grid.cells.items() if hit]
            result['nearest_obstacle_m'] = min((math.dist(a.control_pose[:2], p) for p in occupied), default=None)
            result['center_clear'] = a.view.free(a.control_pose, require_known=False)
            out_file = str(node.get_parameter('out_file').value)
            if out_file:
                np.savez_compressed(str(Path(out_file).with_suffix('.grid.npz')),
                                    occupied=np.asarray(occupied),
                                    known=a.view.known, blocked=a.view.blocked,
                                    origin=a.view.origin, resolution=a.grid.resolution,
                                    pose=a.control_pose, radius=a.radius)
        out_file = str(node.get_parameter('out_file').value)
        if out_file:
            Path(out_file).write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        healthy = len(samples) >= 3 and all(s['reason'] is None and s['odom_fresh'] for s in samples[-3:])
        return 0 if healthy else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
