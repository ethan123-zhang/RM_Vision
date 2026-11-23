"""rclpy node: PointCloud2 + TF -> incremental log-odds map -> pgm/yaml.

Only portable API is used on purpose (Humble 22.04 / Jazzy 24.04):
  * rclpy.spin / spin_once with keyword arguments;
  * tf2_ros.Buffer + TransformListener, exceptions caught generically
    (tf2_ros grew/renamed its Python exception aliases between distros);
  * sensor_msgs PointCloud2 parsed by hand (see cloud.py);
  * std_srvs/Trigger for save, no lifecycle, no Nav2 imports.
"""

import json
import math
import os

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rclpy.time import Time
from std_srvs.srv import Trigger
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

from .cloud import filter_points, pointcloud2_xyz, quat_to_matrix, transform_points
from .grid import FREE, OCC, OccGrid

CLOUD_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST)
MAP_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST)

DEFAULTS = {
    'cloud_topic': '/livox/lidar',
    'map_frame': 'map',
    'base_frame': 'base_footprint',
    'ground_z': 0.0,
    'z_min': 0.10,
    'z_max': 1.20,
    'min_range': 0.35,
    'max_range': 10.0,
    'max_points': 4000,
    'ray_origin': 'sensor',       # sensor | base
    'resolution': 0.025,
    'origin_x': -6.4,
    'origin_y': -4.4,
    'width': 512,
    'height': 352,
    'free_miss': -0.4,
    'l_min': -4.0,
    'l_max': 4.0,
    'free_exit': -0.6,
    'min_occ_hits': 2,
    'block_hits': 3,
    'behind_cells': 1,
    'end_margin': 2,
    'despeckle': 1,
    'dump_dir': '',
    'dump_every': 2,
    'publish_every': 10,
    'save_path': os.path.expanduser('~/sentry_ws/results/odin_map'),
}


def _yaw_of(quaternion):
    x, y, z, w = quaternion
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class PcdToPgmNode(Node):

    def __init__(self, overrides=None):
        super().__init__('pcd_to_pgm')
        for name, default in DEFAULTS.items():
            self.declare_parameter(name, default)
        if overrides:
            self.set_parameters([Parameter(name, value=value)
                                 for name, value in overrides.items()])

        get = lambda name: self.get_parameter(name).value  # noqa: E731
        self.cloud_topic = get('cloud_topic')
        self.map_frame = get('map_frame')
        self.base_frame = get('base_frame')
        self.ground_z = float(get('ground_z'))
        self.z_min = float(get('z_min'))
        self.z_max = float(get('z_max'))
        self.min_range = float(get('min_range'))
        self.max_range = float(get('max_range'))
        self.max_points = int(get('max_points'))
        self.ray_origin = str(get('ray_origin'))
        self.save_path = str(get('save_path'))
        self.publish_every = max(1, int(get('publish_every')))
        self.dump_dir = str(get('dump_dir'))
        self.dump_every = max(1, int(get('dump_every')))
        self._dump_index = []

        self.grid = OccGrid(
            resolution=float(get('resolution')),
            origin=(float(get('origin_x')), float(get('origin_y'))),
            width=int(get('width')), height=int(get('height')),
            free_miss=float(get('free_miss')),
            l_min=float(get('l_min')), l_max=float(get('l_max')),
            free_exit=float(get('free_exit')),
            min_occ_hits=int(get('min_occ_hits')),
            block_hits=int(get('block_hits')),
            behind_cells=int(get('behind_cells')),
            end_margin=int(get('end_margin')),
            despeckle=int(get('despeckle')))

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._warned = set()
        self._clouds = 0
        self._raw_points = 0
        self._used_points = 0
        self._tf_misses = 0
        self._traj = []
        self._last_stamp = None

        self.create_subscription(PointCloud2, self.cloud_topic, self._on_cloud,
                                 CLOUD_QOS)
        self._map_pub = self.create_publisher(OccupancyGrid, '~/map', MAP_QOS)
        self.create_service(Trigger, '~/save_map', self._on_save)

        self.get_logger().info(
            'pcd_to_pgm up: cloud=%s map=%s base=%s grid=%dx%d @ %.3f m, '
            'z=[%.2f, %.2f] m range<=%s m'
            % (self.cloud_topic, self.map_frame, self.base_frame,
               self.grid.width, self.grid.height, self.grid.resolution,
               self.z_min, self.z_max, self.max_range))

    # ------------------------------------------------------------------ tf
    def _lookup(self, target, source, stamp_msg=None):
        """map <- source at the message stamp, falling back to the latest."""
        attempts = []
        if stamp_msg is not None:
            attempts.append(Time.from_msg(stamp_msg))
        attempts.append(Time())
        for moment in attempts:
            try:
                return self._tf_buffer.lookup_transform(
                    target, source, moment, timeout=Duration(seconds=0.05))
            except Exception:
                continue
        return None

    def _warn_once(self, key, text):
        if key not in self._warned:
            self._warned.add(key)
            self.get_logger().warn(text)

    # --------------------------------------------------------------- cloud
    def _on_cloud(self, msg):
        self._clouds += 1
        frame = msg.header.frame_id
        if not frame:
            self._warn_once('noframe', 'PointCloud2 without header.frame_id')
            return
        tf_cloud = self._lookup(self.map_frame, frame, msg.header.stamp)
        if tf_cloud is None:
            self._tf_misses += 1
            self._warn_once('notf-%s' % frame,
                            'no TF %s <- %s yet' % (self.map_frame, frame))
            return
        translation = tf_cloud.transform.translation
        rotation = tf_cloud.transform.rotation
        t = np.array([translation.x, translation.y, translation.z])
        rot = quat_to_matrix((rotation.x, rotation.y, rotation.z, rotation.w))

        try:
            xyz = pointcloud2_xyz(msg)
        except Exception as exc:  # malformed cloud: skip, keep mapping
            self._warn_once('badcloud', 'bad PointCloud2: %s' % exc)
            return
        self._raw_points += int(xyz.shape[0])
        xyz_map = transform_points(xyz, t, rot)

        ray_xy = (float(t[0]), float(t[1]))
        base_xy = None
        tf_base = self._lookup(self.map_frame, self.base_frame, msg.header.stamp)
        if tf_base is not None:
            base_xy = (float(tf_base.transform.translation.x),
                       float(tf_base.transform.translation.y))
        if self.ray_origin == 'base' and base_xy is not None:
            ray_xy = base_xy

        points = filter_points(xyz_map, ray_xy, ground_z=self.ground_z,
                               z_min=self.z_min, z_max=self.z_max,
                               min_range=self.min_range, max_range=self.max_range,
                               max_points=self.max_points)
        if points.shape[0] == 0:
            return
        self.grid.add_scan(points[:, 0], points[:, 1], ray_xy[0], ray_xy[1])
        self._used_points += int(points.shape[0])
        self._record_pose(msg, tf_cloud, base_xy, ray_xy)
        self._maybe_dump(points, ray_xy)

        if self._clouds % self.publish_every == 0:
            self._publish_map(msg.header.stamp)
            stats = self.grid.stats()
            self.get_logger().info(
                'scans=%d pts=%d used=%d occ=%.2f m2 free=%.2f m2'
                % (stats['scans'], self._raw_points, self._used_points,
                   stats['occ_area_m2'], stats['free_area_m2']))

    def _maybe_dump(self, points, ray_xy):
        """Optional: keep the filtered map-frame points so a run can be replayed
        offline (tools/replay_map.py) while tuning parameters."""
        if not self.dump_dir or self._clouds % self.dump_every != 0:
            return
        try:
            os.makedirs(self.dump_dir, exist_ok=True)
            name = 'pts_%05d.npy' % self._clouds
            np.save(os.path.join(self.dump_dir, name),
                    np.ascontiguousarray(points[:, :2], dtype=np.float32))
            self._dump_index.append({'file': name, 'ox': float(ray_xy[0]),
                                     'oy': float(ray_xy[1])})
        except Exception as exc:
            self._warn_once('dump', 'cloud dump failed: %s' % exc)

    def _record_pose(self, msg, tf_cloud, base_xy, ray_xy):
        stamp = self._last_stamp
        try:
            stamp = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        except Exception:
            stamp = self._clouds
        self._last_stamp = stamp
        rotation = tf_cloud.transform.rotation
        entry = {
            't': round(stamp, 4),
            'sx': round(float(ray_xy[0]), 3),
            'sy': round(float(ray_xy[1]), 3),
            'yaw': round(_yaw_of((rotation.x, rotation.y, rotation.z,
                                  rotation.w)), 4),
        }
        if base_xy is not None:
            entry['bx'] = round(float(base_xy[0]), 3)
            entry['by'] = round(float(base_xy[1]), 3)
        self._traj.append(entry)

    # ---------------------------------------------------------------- publish
    def _publish_map(self, stamp_msg):
        cls = self.grid.classify()
        data = np.full(cls.shape, -1, dtype=np.int8)
        data[cls == FREE] = 0
        data[cls == OCC] = 100
        msg = OccupancyGrid()
        msg.header.stamp = stamp_msg
        msg.header.frame_id = self.map_frame
        msg.info.resolution = self.grid.resolution
        msg.info.width = self.grid.width
        msg.info.height = self.grid.height
        msg.info.origin.position.x = self.grid.origin[0]
        msg.info.origin.position.y = self.grid.origin[1]
        msg.info.origin.orientation.w = 1.0
        msg.data = data.ravel().tolist()
        self._map_pub.publish(msg)

    # ------------------------------------------------------------------ save
    def _on_save(self, request, response):
        try:
            stats = self.save()
            response.success = True
            response.message = json.dumps(stats)
        except Exception as exc:
            response.success = False
            response.message = 'save failed: %s' % exc
        return response

    def save(self, base=None):
        base = str(base or self.get_parameter('save_path').value)
        os.makedirs(os.path.dirname(os.path.abspath(base)), exist_ok=True)
        stats = self.grid.save(base, comment='PointCloud2 + TF, Odin1 map frame')
        stats.update({
            'clouds': self._clouds,
            'raw_points': self._raw_points,
            'used_points': self._used_points,
            'tf_misses': self._tf_misses,
            'cloud_topic': self.cloud_topic,
            'map_frame': self.map_frame,
            'z_min': self.z_min,
            'z_max': self.z_max,
            'max_range': self.max_range,
            'ray_origin': self.ray_origin,
        })
        traj_path = base + '_traj.json'
        with open(traj_path, 'w') as handle:
            json.dump({'map_frame': self.map_frame, 'poses': self._traj},
                      handle)
        with open(base + '_stats.json', 'w') as handle:
            json.dump(stats, handle, indent=2, sort_keys=True)
        if self.dump_dir and self._dump_index:
            with open(os.path.join(self.dump_dir, 'index.json'), 'w') as handle:
                json.dump({'map_frame': self.map_frame,
                           'resolution': self.grid.resolution,
                           'origin': list(self.grid.origin),
                           'scans': self._dump_index}, handle)
            stats['dump_dir'] = self.dump_dir
        stats['traj_path'] = traj_path
        stats['pgm'] = base + '.pgm'
        stats['yaml'] = base + '.yaml'
        self.get_logger().info(
            'saved %s.pgm/.yaml: %d scans, occ %.2f m2, free %.2f m2'
            % (base, stats['scans'], stats['occ_area_m2'], stats['free_area_m2']))
        return stats