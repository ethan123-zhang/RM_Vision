#!/usr/bin/env python3
"""Drive the 5 patrol waypoints in a loop while pcd_to_pgm maps.

One NavigateToPose at a time, so the robot sweeps the field instead of cutting
corners.  Order is chosen for coverage (spawn -> SE -> SW -> NE -> NW).

    python3 ~/sentry_ws/tools/patrol_driver.py --duration 120
    kill -TERM <pid>     # stops after the current leg, cancels the goal

Signal handling follows the "rclpy with a background spin" lesson: install
SignalHandlerOptions.NO, let the handler only raise a flag, stop the executor
before rclpy.shutdown().
"""

import argparse
import math
import os
import signal
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions

POINTS_YAML = os.path.expanduser(
    '~/sentry_ws/src/sentry_bringup/config/patrol_points.yaml')
# 0-based indices into patrol_points.yaml, ordered for field coverage
ORDER = [0, 3, 2, 4, 1]


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def load_points(path):
    with open(path) as handle:
        return yaml.safe_load(handle)


class PatrolDriver(Node):

    def __init__(self):
        super().__init__(
            'patrol_driver',
            parameter_overrides=[Parameter('use_sim_time',
                                           Parameter.Type.BOOL, True)])
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.stop = False

    def _spin_until(self, future, timeout=None, cancel_handle=None):
        deadline = time.time() + timeout if timeout else None
        while not future.done():
            if self.stop:
                if cancel_handle is not None:
                    cancel_handle.cancel_goal_async()
                return False
            if deadline is not None and time.time() >= deadline:
                if cancel_handle is not None:
                    cancel_handle.cancel_goal_async()
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
            if not rclpy.ok():
                return False
        return True

    def wait_server(self, timeout=30.0):
        deadline = time.time() + timeout
        while time.time() < deadline and not self.stop:
            if self.client.server_is_ready():
                return True
            rclpy.spin_once(self, timeout_sec=0.2)
        return False

    def go(self, x, y, yaw, timeout=90.0):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        qx, qy, qz, qw = quat_from_yaw(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        started = time.time()
        send_future = self.client.send_goal_async(goal)
        if not self._spin_until(send_future, timeout=20.0):
            return 'SEND_TIMEOUT'
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return 'REJECTED'
        result_future = handle.get_result_async()
        ok = self._spin_until(result_future, timeout=timeout,
                              cancel_handle=handle)
        elapsed = time.time() - started
        if not ok:
            return 'STOPPED_OR_TIMEOUT(%.1fs)' % elapsed
        status = result_future.result().status
        label = {2: 'ACCEPTED', 3: 'EXECUTING', 4: 'SUCCEEDED',
                 5: 'CANCELED', 6: 'ABORTED'}.get(status, 'STATUS_%s' % status)
        return '%s(%.1fs)' % (label, elapsed)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=0.0,
                        help='stop after this many wall seconds (0 = forever)')
    parser.add_argument('--points', default=POINTS_YAML)
    parser.add_argument('--once', action='store_true', help='one pass only')
    args = parser.parse_args(argv)

    points = load_points(args.points)
    order = [points[i] for i in ORDER]

    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    node = PatrolDriver()

    def handler(signum, frame):
        node.get_logger().info('signal %d: stopping after this leg' % signum)
        node.stop = True

    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)

    started = time.time()
    try:
        if not node.wait_server(60.0):
            print('[patrol_driver] navigate_to_pose not ready', flush=True)
            return 1
        lap = 0
        while not node.stop:
            for index, point in enumerate(order):
                if node.stop:
                    break
                if args.duration and time.time() - started >= args.duration:
                    node.stop = True
                    break
                label = node.go(point['x'], point['y'], point.get('yaw', 0.0))
                print('[patrol_driver] lap %d goal P%d (%.2f, %.2f): %s'
                      % (lap, ORDER[index] + 1, point['x'], point['y'], label),
                      flush=True)
            lap += 1
            if args.once:
                break
            if args.duration and time.time() - started >= args.duration:
                break
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print('[patrol_driver] finished in %.1f s' % (time.time() - started),
          flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())