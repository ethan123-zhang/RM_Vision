#!/usr/bin/env python3
"""Send one NavigateToPose goal and wait for the result (sim time aware).

    python3 ~/sentry_ws/tools/send_nav_goal.py 0.0 3.0 --yaw-deg 0 --timeout 180

Prints one JSON line with the action status (SUCCEEDED / ABORTED / timeout) and
the final map-frame pose. use_sim_time is on by default because every sentry_*
launch in this workspace runs on the Gazebo clock; pass --no-sim-time for a
wall-clock stack.
"""

import argparse
import json
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.utilities import remove_ros_args

import tf2_ros

from nav2_msgs.action import NavigateToPose

STATUS = {0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
          4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GoalNode(Node):

    def __init__(self, sim_time):
        super().__init__('send_nav_goal',
                         parameter_overrides=[Parameter('use_sim_time',
                                                        Parameter.Type.BOOL,
                                                        sim_time)])
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

    def wait_server(self, timeout):
        start = time.monotonic()
        while not self.client.server_is_ready():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - start > timeout:
                return False
        return True

    def pose(self, wait_s=0.0):
        end = time.monotonic() + wait_s
        while True:
            try:
                tr = self.buffer.lookup_transform('map', 'base_footprint',
                                                  rclpy.time.Time())
                t = tr.transform.translation
                return [round(t.x, 3), round(t.y, 3),
                        round(math.degrees(yaw_of(tr.transform.rotation)), 2)]
            except Exception:
                if time.monotonic() >= end:
                    return None
                rclpy.spin_once(self, timeout_sec=0.05)
                time.sleep(0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('x', type=float)
    ap.add_argument('y', type=float)
    ap.add_argument('--yaw-deg', type=float, default=0.0)
    ap.add_argument('--timeout', type=float, default=180.0)
    ap.add_argument('--no-sim-time', action='store_true')
    cfg = ap.parse_args(remove_ros_args(args=sys.argv)[1:])

    rclpy.init()
    node = GoalNode(not cfg.no_sim_time)
    out = {'goal': [cfg.x, cfg.y, cfg.yaw_deg]}
    try:
        if not node.wait_server(60.0):
            out['error'] = 'no /navigate_to_pose action server'
            print('NAV_GOAL_JSON ' + json.dumps(out))
            return 2
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = cfg.x
        goal.pose.pose.position.y = cfg.y
        goal.pose.pose.orientation.z = math.sin(math.radians(cfg.yaw_deg) / 2.0)
        goal.pose.pose.orientation.w = math.cos(math.radians(cfg.yaw_deg) / 2.0)
        out['start_pose'] = node.pose(wait_s=10.0)

        start = time.monotonic()
        send = node.client.send_goal_async(goal)
        while not send.done():
            rclpy.spin_once(node, timeout_sec=0.02)
        handle = send.result()
        out['accepted'] = bool(handle and handle.accepted)
        if not out['accepted']:
            print('NAV_GOAL_JSON ' + json.dumps(out))
            return 1
        result = handle.get_result_async()
        while not result.done() and time.monotonic() - start < cfg.timeout:
            rclpy.spin_once(node, timeout_sec=0.02)
        if not result.done():
            handle.cancel_goal_async()
            out['timed_out'] = True
            out['wall_s'] = round(time.monotonic() - start, 1)
            out['status_name'] = 'TIMEOUT'
            print('NAV_GOAL_JSON ' + json.dumps(out))
            return 1
        response = result.result()
        out['status'] = int(response.status)
        out['status_name'] = STATUS.get(int(response.status), 'UNKNOWN')
        out['error_code'] = int(response.result.error_code)
        out['error_msg'] = response.result.error_msg
        out['wall_s'] = round(time.monotonic() - start, 1)
        out['end_pose'] = node.pose()
        print('NAV_GOAL_JSON ' + json.dumps(out))
        return 0 if out['status_name'] == 'SUCCEEDED' else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())