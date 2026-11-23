"""map_align_check: estimate the Odin1 map frame vs field/CAD frame offset.

One shot (robot parked, e.g. at the spawn pose):

    source /opt/ros/jazzy/setup.bash
    source ~/sentry_ws/install/setup.bash
    ros2 run map_align_check map_align_check \
        --map /mnt/e/RM/tb3_gazebo_nav2/rm_field/maps/rmul2027_3v3_up.yaml \
        --sim-time --out ~/sentry_ws/results/map_align_check.json \
        --fix-yaml ~/sentry_ws/results/map_align_fix.yaml

Continuous (estimate once per second while driving/walking the field):

    ros2 run map_align_check map_align_check --continuous --rate 0.5 --sim-time

Odin1's map frame is assumed to satisfy p_map = R(dyaw) * p_field + (dx, dy);
the printed estimate is exactly what odin1_sim_bridge's map_offset_x/y/yaw
injects, and ``--fix-yaml`` also writes the inverse correction for a consumer
that wants to move the map frame back onto the field frame.
"""

import argparse
import json
import math
import os
import sys
import time
from collections import deque

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (HistoryPolicy, QoSProfile, ReliabilityPolicy)
from rclpy.time import Time
from rclpy.utilities import remove_ros_args

from sensor_msgs.msg import PointCloud2

import tf2_ros

from . import cloud as cloudmod
from . import estimator
from .grid import CadGrid

DEFAULT_MAP = '/mnt/e/RM/tb3_gazebo_nav2/rm_field/maps/rmul2027_3v3_up.yaml'

CLOUD_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST)


def build_parser():
    p = argparse.ArgumentParser(
        prog='map_align_check',
        description='Estimate the (dx, dy, dyaw) offset between the Odin1 map '
                    'frame and the CAD/field frame.')
    p.add_argument('--map', default=DEFAULT_MAP,
                   help='CAD map_server yaml (field frame), default: %(default)s')
    p.add_argument('--cloud', default='/livox/lidar',
                   help='PointCloud2 topic in the Odin1 map frame chain')
    p.add_argument('--map-frame', default='map', help='Odin1 map frame name')
    p.add_argument('--frames', type=int, default=5,
                   help='cloud messages to accumulate (robot must be still)')
    p.add_argument('--timeout', type=float, default=60.0,
                   help='s to wait for those clouds + TF before giving up')
    p.add_argument('--z-min', type=float, default=0.10,
                   help='height band lower bound in the map frame (m)')
    p.add_argument('--z-max', type=float, default=1.20,
                   help='height band upper bound in the map frame (m)')
    p.add_argument('--ground-z', type=float, default=0.0,
                   help='map-frame z of the ground (m), added to the band')
    p.add_argument('--range-max', type=float, default=8.0,
                   help='max horizontal sensor range used (m)')
    p.add_argument('--range-min', type=float, default=0.35,
                   help='min horizontal sensor range used (m)')
    p.add_argument('--max-points', type=int, default=3000,
                   help='points kept for the estimate (deterministic thinning)')
    p.add_argument('--xy-max', type=float, default=estimator.DEFAULTS['xy_max'])
    p.add_argument('--xy-step', type=float, default=estimator.DEFAULTS['xy_step'])
    p.add_argument('--yaw-max-deg', type=float,
                   default=estimator.DEFAULTS['yaw_max_deg'])
    p.add_argument('--yaw-step-deg', type=float,
                   default=estimator.DEFAULTS['yaw_step_deg'])
    p.add_argument('--coarse-points', type=int,
                   default=estimator.DEFAULTS['coarse_points'])
    p.add_argument('--fine-points', type=int,
                   default=estimator.DEFAULTS['fine_points'])
    p.add_argument('--backend', choices=['auto', 'scipy', 'numpy'],
                   default='auto', help='distance field backend (default auto)')
    p.add_argument('--out', default='map_align_check.json',
                   help='JSON report path ("" disables the file)')
    p.add_argument('--fix-yaml', default='',
                   help='write the estimated offset + correction to this yaml')
    p.add_argument('--dump', default='',
                   help='save the filtered map-frame points to this .npz '
                        '(offline replay / debugging)')
    p.add_argument('--continuous', action='store_true',
                   help='keep estimating at --rate Hz instead of exiting')
    p.add_argument('--rate', type=float, default=0.5,
                   help='Hz for --continuous')
    p.add_argument('--expect', default='', metavar='DX,DY,DYAW_DEG',
                   help='optional injected offset to report the error against; '
                        'use --expect=-0.15,0.25,-5.0 when the first value is '
                        'negative')
    p.add_argument('--sim-time', action='store_true',
                   help='set use_sim_time (Gazebo) before touching TF')
    p.add_argument('--quiet', action='store_true', help='only the JSON line')
    return p


def _write_text(path, text):
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, 'w') as handle:
        handle.write(text)


def _inverse_offset(dx, dy, dyaw):
    """Inverse of p_map = R(dyaw) p_field + t."""
    c, s = math.cos(dyaw), math.sin(dyaw)
    return (-(c * dx + s * dy), -(-s * dx + c * dy), -dyaw)


def _fix_yaml(result, cfg, stamp):
    est = result['estimate']
    inv = _inverse_offset(est['dx'], est['dy'], est['dyaw'])
    lines = [
        '# map_align_check: Odin1 map frame vs field/CAD frame, %s'
        % time.strftime('%Y-%m-%d %H:%M:%S'),
        '# Same sign convention as odin1_sim_bridge map_offset_x/y/yaw:',
        '#   p_map = R(dyaw) * p_field + (dx, dy)',
        '# align = the offset the Odin1 map frame carries (what was measured).',
        '# correction = inverse, i.e. what to apply to move map back onto field.',
        'map_yaml: %s' % cfg.map,
        'cloud_topic: %s' % cfg.cloud,
        'stamp: %.6f' % stamp,
        'align:',
        '  dx: %.5f' % est['dx'],
        '  dy: %.5f' % est['dy'],
        '  dyaw: %.6f' % est['dyaw'],
        '  dyaw_deg: %.4f' % est['dyaw_deg'],
        'correction:',
        '  dx: %.5f' % inv[0],
        '  dy: %.5f' % inv[1],
        '  dyaw: %.6f' % inv[2],
        '  dyaw_deg: %.4f' % math.degrees(inv[2]),
        'residual_mean_m: %.4f' % result['residual']['mean_m'],
        'residual_p90_m: %.4f' % result['residual']['p90_m'],
        'residual_occupied_mean_m: %.4f' % result['residual']['occupied_mean_m'],
        'inlier_ratio_005m: %.4f' % result['residual']['inlier_ratio_005m'],
        'confidence_gap_m: %.4f' % result['confidence']['gap_m'],
        'confidence_ratio: %s'
        % ('%.2f' % result['confidence']['ratio']
           if result['confidence']['ratio'] is not None else 'null'),
        'points_used: %d' % result['points']['used_fine'],
        'backend: %s' % result['backend'],
    ]
    return '\n'.join(lines) + '\n'


class AlignCheck(Node):

    def __init__(self, cfg, cad):
        super().__init__(
            'map_align_check',
            parameter_overrides=[Parameter('use_sim_time',
                                           Parameter.Type.BOOL,
                                           bool(cfg.sim_time))])
        self.cfg = cfg
        self.cad = cad
        self.clouds = deque(maxlen=max(2, int(cfg.frames)))
        self.create_subscription(PointCloud2, cfg.cloud, self._on_cloud, CLOUD_QOS)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self,
                                                     spin_thread=False)
        self._count = 0
        self._warned_tf_fallback = False

    def _on_cloud(self, msg):
        self._count += 1
        self.clouds.append(msg)

    def _spin(self, seconds=0.2):
        rclpy.spin_once(self, timeout_sec=seconds)

    def _map_from(self, frame_id, stamp):
        """T_map_source for one cloud, waiting up to 10 s for TF to catch up.

        The stamped lookup is preferred; while the buffer has no transform at
        that exact sim stamp yet, the newest available transform is accepted
        (the one-shot mode requires a parked robot, so a frame of TF lag is
        harmless). Both are retried because /tf discovery can lose the race
        with the first lidar messages.
        """
        t_end = time.monotonic() + 10.0
        last = None
        while time.monotonic() < t_end:
            try:
                return self.tf_buffer.lookup_transform(
                    self.cfg.map_frame, frame_id, stamp).transform
            except Exception as exc:
                last = exc
                try:
                    newest = self.tf_buffer.lookup_transform(
                        self.cfg.map_frame, frame_id, Time()).transform
                    if not self._warned_tf_fallback:
                        self._warned_tf_fallback = True
                        if not self.cfg.quiet:
                            print('-- note: no TF at the cloud stamp yet, using '
                                  'the newest map<-%s transform (robot must be '
                                  'still)' % frame_id)
                    return newest
                except Exception as exc2:
                    last = exc2
                self._spin(0.05)
                time.sleep(0.05)
        raise RuntimeError('no TF %s <- %s at %.3f: %s'
                           % (self.cfg.map_frame, frame_id,
                              stamp.sec + stamp.nanosec * 1e-9, last))

    def gather(self):
        """Wait for --frames clouds and return map-frame (N,3) points + timing."""
        t0 = time.monotonic()
        deadline = t0 + self.cfg.timeout
        while time.monotonic() < deadline and len(self.clouds) < self.cfg.frames:
            self._spin(0.2)
        if len(self.clouds) < self.cfg.frames:
            raise RuntimeError('only %d cloud messages on %s after %.1f s '
                               '(need %d); is the lidar up?'
                               % (len(self.clouds), self.cfg.cloud,
                                  self.cfg.timeout, self.cfg.frames))
        chunks, sensor_xy = [], []
        for msg in list(self.clouds)[-self.cfg.frames:]:
            transform = self._map_from(msg.header.frame_id, msg.header.stamp)
            trans = transform.translation
            rot = cloudmod.quat_to_matrix((transform.rotation.x,
                                           transform.rotation.y,
                                           transform.rotation.z,
                                           transform.rotation.w))
            xyz = cloudmod.pointcloud2_xyz(msg)
            chunks.append(cloudmod.transform_points(
                xyz, [trans.x, trans.y, trans.z], rot))
            sensor_xy.append([trans.x, trans.y])
        points = np.vstack(chunks) if chunks else np.zeros((0, 3))
        filtered = cloudmod.filter_points(
            points, np.mean(np.asarray(sensor_xy), axis=0),
            z_min=self.cfg.z_min, z_max=self.cfg.z_max,
            range_max=self.cfg.range_max, range_min=self.cfg.range_min,
            ground_z=self.cfg.ground_z, max_points=self.cfg.max_points)
        return filtered, time.monotonic() - t0, len(points)

    def run_estimate(self, label=''):
        points, capture_s, n_raw = self.gather()
        if points.shape[0] < 50:
            raise RuntimeError('only %d filtered points in the height band '
                               '[%.2f, %.2f] m up to %.1f m range'
                               % (points.shape[0], self.cfg.z_min,
                                  self.cfg.z_max, self.cfg.range_max))
        if self.cfg.dump:
            np.savez_compressed(self.cfg.dump, points=points,
                                map_yaml=str(self.cfg.map),
                                z_min=self.cfg.z_min, z_max=self.cfg.z_max,
                                range_max=self.cfg.range_max)
        result = estimator.estimate(
            points[:, :2], self.cad,
            xy_max=self.cfg.xy_max, xy_step=self.cfg.xy_step,
            yaw_max_deg=self.cfg.yaw_max_deg,
            yaw_step_deg=self.cfg.yaw_step_deg,
            coarse_points=self.cfg.coarse_points,
            fine_points=self.cfg.fine_points)
        newest = list(self.clouds)[-1]
        result['input'] = {
            'map_frame': self.cfg.map_frame,
            'cloud_topic': self.cfg.cloud,
            'cloud_frame': newest.header.frame_id,
            'cloud_stamp': newest.header.stamp.sec
            + newest.header.stamp.nanosec * 1e-9,
            'map_yaml': self.cfg.map,
            'map_resolution': self.cad.res,
            'map_origin': self.cad.origin,
            'map_occupied_cells': self.cad.n_occupied,
            'map_surface_cells': self.cad.n_surface_cells,
            'cad': self.cad.describe(),
            'frames': int(self.cfg.frames),
            'raw_points': int(n_raw),
            'points': int(points.shape[0]),
            'height_band_m': [self.cfg.ground_z + self.cfg.z_min,
                              self.cfg.ground_z + self.cfg.z_max],
            'range_max_m': self.cfg.range_max,
        }
        result['timing']['capture_s'] = capture_s
        result['timing']['total_wall_s'] = capture_s + result['timing']['total_s']
        if self.cfg.expect:
            exp = [float(v) for v in self.cfg.expect.replace(',', ' ').split()]
            result['expected'] = {'dx': exp[0], 'dy': exp[1],
                                  'dyaw': math.radians(exp[2]),
                                  'dyaw_deg': exp[2]}
            result['error'] = {
                'dx': result['estimate']['dx'] - exp[0],
                'dy': result['estimate']['dy'] - exp[1],
                'dyaw': result['estimate']['dyaw'] - math.radians(exp[2]),
                'dyaw_deg': result['estimate']['dyaw_deg'] - exp[2],
                'xy_m': math.hypot(result['estimate']['dx'] - exp[0],
                                   result['estimate']['dy'] - exp[1]),
            }
        return result

    def report(self, result, label=''):
        est, res, conf, timing = (result['estimate'], result['residual'],
                                  result['confidence'], result['timing'])
        if not self.cfg.quiet:
            print('-- map_align_check%s: estimate dx=%+.3f m dy=%+.3f m '
                  'dyaw=%+.3f deg' % (label, est['dx'], est['dy'],
                                      est['dyaw_deg']))
            print('--   residual to CAD surface: mean=%.4f m p90=%.4f m '
                  '<=5cm=%.2f; to nearest occupied cell: mean=%.4f m'
                  % (res['mean_m'], res['p90_m'], res['inlier_ratio_005m'],
                     res['occupied_mean_m']))
            print('--   confidence best=%.4f m second=%.4f m gap=%.4f m '
                  'ratio=%s'
                  % (conf['best_m'], conf['second_best_m'], conf['gap_m'],
                     ('%.1fx' % conf['ratio'])
                     if conf['ratio'] is not None else 'n/a'))
            if 'error' in result:
                err = result['error']
                print('--   error vs expected: dxy=%.4f m dyaw=%.4f deg '
                      '(dx=%+.4f dy=%+.4f)' % (err['xy_m'], err['dyaw_deg'],
                                               err['dx'], err['dy']))
            print('--   timing capture=%.2f s coarse=%.2f s refine=%.2f s '
                  'estimate=%.2f s total=%.2f s backend=%s'
                  % (timing['capture_s'], timing['coarse_s'],
                     timing['refine_s'], timing['total_s'],
                     timing['total_wall_s'], result['backend']))
            print('--   points %d/%d raw, grid %dx%dx%d @ %.3f m / %.2f deg'
                  % (result['points']['used_fine'], result['input']['raw_points'],
                     result['coarse']['grid']['n_dx'],
                     result['coarse']['grid']['n_dy'],
                     result['coarse']['grid']['n_yaw'],
                     result['coarse']['grid']['xy_step'],
                     result['coarse']['grid']['yaw_step_deg']))
        print('MAP_ALIGN_JSON ' + json.dumps(result, sort_keys=True))

    def persist(self, result):
        if self.cfg.out:
            _write_text(self.cfg.out, json.dumps(result, indent=2,
                                                 sort_keys=True) + '\n')
        if self.cfg.fix_yaml:
            _write_text(self.cfg.fix_yaml,
                        _fix_yaml(result, self.cfg,
                                  result['input']['cloud_stamp']))


def main(args=None):
    argv = rclpy.utilities.remove_ros_args(args=args)[1:]
    cfg = build_parser().parse_args(argv)
    cad = CadGrid(cfg.map, backend=cfg.backend)
    if not cfg.quiet:
        print('-- CadGrid %s: %dx%d cells @ %.3f m, origin (%.2f, %.2f), '
              '%d occupied, distance field %s'
              % (os.path.basename(cfg.map), cad.width, cad.height, cad.res,
                 cad.origin[0], cad.origin[1], cad.n_occupied, cad.backend))

    rclpy.init()
    node = AlignCheck(cfg, cad)
    exit_code = 0
    try:
        if cfg.continuous:
            period = 1.0 / max(0.05, cfg.rate)
            print('-- continuous mode: %.2f Hz, Ctrl-C to stop' % cfg.rate)
            while rclpy.ok():
                start = time.monotonic()
                result = node.run_estimate(' [continuous]')
                node.report(result, ' [continuous]')
                node.persist(result)
                while rclpy.ok() and time.monotonic() - start < period:
                    node._spin(0.05)
        else:
            result = node.run_estimate()
            node.report(result)
            node.persist(result)
            if not cfg.quiet and cfg.out:
                print('-- wrote %s%s' % (cfg.out,
                                         (' and ' + cfg.fix_yaml)
                                         if cfg.fix_yaml else ''))
    except RuntimeError as exc:
        print('-- map_align_check FAILED: %s' % exc, file=sys.stderr)
        exit_code = 2
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())