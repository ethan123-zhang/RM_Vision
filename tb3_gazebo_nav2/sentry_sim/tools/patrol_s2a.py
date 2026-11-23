#!/usr/bin/env python3
"""S2a patrol acceptance: 5 consecutive NavigateToPose goals + truth-based metrics.

Run inside ~/sentry_ws after ./run_sentry_nav.sh, with

  export ROS_DOMAIN_ID=45
  source /opt/ros/jazzy/setup.bash
  source ~/sentry_ws/install/setup.bash
  python3 tools/patrol_s2a.py

Records per goal: action status/error code, arrival error vs /gt_odom, sim time.
Whole run: min clearance to the 48 box collision rects (analytic), collision
flag, lethal-map/costmap hits, RTF, CPU of the whole ROS domain, /scan stats.
Writes ~/sentry_ws/results/report_sentry_patrol.json and prints a summary.
"""
import json
import math
import os
import time

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped

from nav2_msgs.action import NavigateToPose

WS = os.path.expanduser('~/sentry_ws')
BOXES = '/mnt/e/RM/tb3_gazebo_nav2/rm_field/debug/box_collisions.json'
POINTS = os.path.join(WS, 'src/sentry_bringup/config/patrol_points.yaml')
OUT = os.path.join(WS, 'results/report_sentry_patrol.json')
LOG = '/tmp/sentry_nav/patrol.log'

STATUS = {0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
          4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}

ODOM_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST)
SCAN_QOS = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST)
MAP_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST)

CPU_PAT = ('gz sim', 'gz-sim-server', 'ros_gz_bridge', 'parameter_bridge',
           'nav2_', 'controller_server', 'planner_server', 'bt_navigator',
           'behavior_server', 'smoother_server', 'velocity_smoother',
           'collision_monitor', 'map_server', 'lifecycle_manager',
           'odin1_sim_bridge', 'pointcloud_to_laserscan',
           'robot_state_publisher', 'ros2 launch', 'sentry_sim')


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def load_boxes():
    data = json.load(open(BOXES))
    out = []
    for b in data['boxes']:
        cx, cy, _ = b['center']
        sx, sy, _ = b['size']
        out.append((cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2))
    return np.asarray(out, dtype=float)


BOX_RECTS = load_boxes()


def clearance(x, y):
    """Distance from (x,y) to the nearest box collision rect (metres)."""
    dx = np.maximum(np.maximum(BOX_RECTS[:, 0] - x, 0.0), x - BOX_RECTS[:, 1])
    dy = np.maximum(np.maximum(BOX_RECTS[:, 2] - y, 0.0), y - BOX_RECTS[:, 3])
    return float(np.min(np.hypot(dx, dy)))


def domain_pids():
    pats = [p.lower() for p in CPU_PAT]
    out = []
    for name in os.listdir('/proc'):
        if not name.isdigit():
            continue
        pid = int(name)
        if pid == os.getpid():
            continue
        try:
            with open('/proc/%d/cmdline' % pid, 'rb') as f:
                cmd = f.read().replace(b'\x00', b' ').decode('utf-8', 'ignore')
        except OSError:
            continue
        low = cmd.lower()
        if not any(p in low for p in pats):
            continue
        try:
            if b'ROS_DOMAIN_ID=45\x00' not in open('/proc/%d/environ' % pid, 'rb').read():
                continue
        except OSError:
            continue
        out.append(pid)
    return out


def cpu_seconds(pids):
    hz = os.sysconf('SC_CLK_TCK')
    total = 0.0
    for pid in pids:
        try:
            fields = open('/proc/%d/stat' % pid).read().split()
        except OSError:
            continue
        total += (int(fields[13]) + int(fields[14])) / float(hz)
    return total


class Patrol(Node):

    def __init__(self):
        super().__init__('patrol_s2a', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.gt = None
        self.gt_hist = []
        self.map = None
        self.cost = None
        self.first_scan = None
        self.scan_hist = []
        self.create_subscription(Odometry, '/gt_odom', self.on_gt, ODOM_QOS)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, MAP_QOS)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self.on_cost, MAP_QOS)
        self.create_subscription(LaserScan, '/scan', self.on_scan, SCAN_QOS)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def on_gt(self, msg):
        p = msg.pose.pose
        t = self.get_clock().now().nanoseconds * 1e-9
        self.gt = (t, p.position.x, p.position.y, yaw_of(p.orientation))
        self.gt_hist.append(self.gt)

    def on_map(self, msg):
        if self.map is None:
            self.map = msg

    def on_cost(self, msg):
        self.cost = msg

    def on_scan(self, msg):
        if self.first_scan is None:
            self.first_scan = msg
        self.scan_hist.append(msg)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def scan_stats(msg):
    ranges = np.asarray(msg.ranges, dtype=float)
    angles = msg.angle_min + np.arange(ranges.size) * msg.angle_increment
    fin = np.isfinite(ranges) & (ranges > 0.0)
    r = ranges[fin]
    a = angles[fin]
    out = {'frame': msg.header.frame_id, 'rays': int(ranges.size),
           'finite': int(fin.sum()), 'r_min': round(float(r.min()), 3),
           'r_max': round(float(r.max()), 3),
           'r_median': round(float(np.median(r)), 3),
           'frac_3.3_3.5': round(float(((r >= 3.3) & (r <= 3.5)).mean()), 4),
           'hist_0p5m': {str(round(x, 1)): int(((r >= x) & (r < x + 0.5)).sum())
                         for x in np.arange(1.0, 7.5, 0.5)}}
    band = (r >= 3.3) & (r <= 3.5)
    out['band_3.3_3.5_rays'] = int(band.sum())
    if band.sum():
        out['band_azimuth_deg'] = [round(float(np.degrees(a[band].min())), 1),
                                   round(float(np.degrees(a[band].max())), 1)]
        out['band_azimuth_hist'] = np.histogram(np.degrees(a[band]),
                                                bins=np.arange(-180, 181, 30))[0].tolist()
        out['band_azimuth_hist_bins_deg'] = list(range(-180, 180, 30))
        # where do those returns land in the map frame (truth pose)?
        pose = patrol.gt
        if pose is not None:
            _, px, py, pyaw = pose
            wx = px + r[band] * np.cos(a[band] + pyaw)
            wy = py + r[band] * np.sin(a[band] + pyaw)
            dx = np.maximum(np.maximum(BOX_RECTS[None, :, 0] - wx[:, None], 0.0),
                            wx[:, None] - BOX_RECTS[None, :, 1])
            dy = np.maximum(np.maximum(BOX_RECTS[None, :, 2] - wy[:, None], 0.0),
                            wy[:, None] - BOX_RECTS[None, :, 3])
            d = np.min(np.hypot(dx, dy), axis=1)
            out['band_to_nearest_box_m'] = {
                'median': round(float(np.median(d)), 4),
                'p90': round(float(np.percentile(d, 90)), 4),
                'max': round(float(np.max(d)), 4),
                'frac_within_5cm': round(float((d <= 0.05).mean()), 4)}
            out['band_world_xy_sample'] = [[round(float(x), 3), round(float(y), 3)]
                                           for x, y in list(zip(wx, wy))[:8]]
    return out


def cost_at(costmap, x, y):
    if costmap is None:
        return None
    info = costmap.info
    c = int(math.floor((x - info.origin.position.x) / info.resolution))
    r = int(math.floor((y - info.origin.position.y) / info.resolution))
    if not (0 <= c < info.width and 0 <= r < info.height):
        return None
    return int(costmap.data[r * info.width + c])


def main():
    global patrol
    points = yaml.safe_load(open(POINTS))
    rclpy.init()
    patrol = Patrol()
    node = patrol
    out = {'points_file': POINTS, 'points': points}

    w0 = time.monotonic()
    while (node.map is None or node.gt is None or node.first_scan is None) and \
            time.monotonic() - w0 < 120:
        rclpy.spin_once(node, timeout_sec=0.05)
    out['ready_wall_s'] = round(time.monotonic() - w0, 2)
    out['map'] = {'width': node.map.info.width, 'height': node.map.info.height,
                  'resolution': node.map.info.resolution,
                  'origin': [node.map.info.origin.position.x,
                             node.map.info.origin.position.y],
                  'occupied_cells': int(sum(1 for v in node.map.data if v >= 65))}
    out['scan_at_spawn'] = scan_stats(node.first_scan)
    out['start_gt'] = [round(v, 3) for v in node.gt]
    out['server_ready'] = node.client.wait_for_server(timeout_sec=120.0)

    # ---- baseline: RTF / CPU over 5 sim s at rest
    cpu_pids = domain_pids()
    c0 = cpu_seconds(cpu_pids)
    wall0 = time.monotonic()
    sim0 = node.get_clock().now().nanoseconds * 1e-9
    node.spin_for(5.0)
    sim1 = node.get_clock().now().nanoseconds * 1e-9
    wall1 = time.monotonic()
    c1 = cpu_seconds(cpu_pids)
    out['rest'] = {'sim_s': round(sim1 - sim0, 2), 'wall_s': round(wall1 - wall0, 2),
                   'rtf': round((sim1 - sim0) / (wall1 - wall0), 3),
                   'cpu_pct_of_one_core': round((c1 - c0) / (wall1 - wall0) * 100, 1),
                   'procs': len(cpu_pids)}

    # ---- patrol
    legs = []
    all_clear = []
    for i, p in enumerate(points):
        gx, gy, gyaw = float(p['x']), float(p['y']), float(p['yaw'])
        before = node.gt
        sim_start = node.get_clock().now().nanoseconds * 1e-9
        wall_start = time.monotonic()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        q = quat(gyaw)
        (goal.pose.pose.orientation.x, goal.pose.pose.orientation.y,
         goal.pose.pose.orientation.z, goal.pose.pose.orientation.w) = q

        last_fb = {}

        def on_fb(msg):
            last_fb['msg'] = msg.feedback

        send = node.client.send_goal_async(goal, feedback_callback=on_fb)
        rclpy.spin_until_future_complete(node, send, timeout_sec=30.0)
        handle = send.result()
        accepted = handle is not None and handle.accepted
        result = None
        if accepted:
            ft = handle.get_result_async()
            end = time.monotonic() + 900.0
            while not ft.done() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
            result = ft.result()
        sim_end = node.get_clock().now().nanoseconds * 1e-9
        wall_end = time.monotonic()
        after = node.gt

        status = int(result.status) if result is not None else -1
        res = result.result if result is not None else None
        err_code = getattr(res, 'error_code', None) if res is not None else None
        err_msg = getattr(res, 'error_msg', None) if res is not None else None
        fb = last_fb.get('msg')
        recov = getattr(fb, 'number_of_recoveries', None) if fb is not None else None
        dist_rem = getattr(fb, 'distance_remaining', None) if fb is not None else None

        trace = [g for g in node.gt_hist if sim_start - 0.5 <= g[0] <= sim_end + 0.2]
        clr = [(clearance(g[1], g[2]), g) for g in trace]
        leg_min, leg_at = (min(clr)[0], min(clr)[1]) if clr else (None, None)
        all_clear += clr
        gt_err = (math.hypot(after[1] - gx, after[2] - gy)
                  if after is not None else None)
        yaw_err = None
        if after is not None:
            yaw_err = abs((after[3] - gyaw + math.pi) % (2 * math.pi) - math.pi)
        cost = cost_at(node.cost, after[1], after[2]) if after is not None else None
        legs.append({
            'index': i + 1, 'point': [gx, gy, gyaw],
            'accepted': bool(accepted), 'status': status,
            'status_name': STATUS.get(status, 'NO_RESULT'),
            'error_code': None if err_code is None else int(err_code),
            'error_msg': err_msg,
            'recoveries': None if recov is None else int(recov),
            'distance_remaining_fb': None if dist_rem is None else round(float(dist_rem), 3),
            'sim_s': round(sim_end - sim_start, 2),
            'wall_s': round(wall_end - wall_start, 2),
            'start_gt': [round(v, 3) for v in before] if before else None,
            'end_gt': [round(v, 3) for v in after] if after else None,
            'arrival_err_m_gt': None if gt_err is None else round(gt_err, 3),
            'yaw_err_rad_gt': None if yaw_err is None else round(yaw_err, 3),
            'min_clear_m_gt': None if leg_min is None else round(leg_min, 3),
            'min_clear_at_gt': [round(v, 3) for v in (leg_at[1:3] if leg_at else ())],
            'costmap_at_goal': cost,
            'truth_samples': len(trace),
        })
        print('leg %d %s: %s err=%.3f m clear=%.3f sim=%.1fs'
              % (i + 1, [gx, gy, gyaw], legs[-1]['status_name'],
                 gt_err if gt_err is not None else -1,
                 leg_min if leg_min is not None else -1, legs[-1]['sim_s']),
              flush=True)

    wall_end = time.monotonic()
    c2 = cpu_seconds(domain_pids())
    sim_total = sum(l['sim_s'] for l in legs)
    wall_total = sum(l['wall_s'] for l in legs)
    out['legs'] = legs
    out['patrol'] = {
        'n_points': len(points),
        'succeeded': sum(1 for l in legs if l['status_name'] == 'SUCCEEDED'),
        'sim_s_total': round(sim_total, 2),
        'wall_s_total': round(wall_total, 2),
        'rtf_run': round(sim_total / wall_total, 3),
        'cpu_pct_of_one_core': round((c2 - c1) / max(1e-9, wall_end - wall0) * 100, 1),
        'cpu_pct_of_24_cores': round((c2 - c1) / max(1e-9, wall_end - wall0) * 100
                                     / os.cpu_count(), 1),
        'procs': len(cpu_pids),
    }
    if all_clear:
        mn = min(all_clear, key=lambda t: t[0])
        out['safety'] = {
            'min_clear_m_gt': round(mn[0], 3),
            'min_clear_at_gt': [round(v, 3) for v in mn[1][1:3]],
            'min_clear_sim_t': round(mn[1][0], 2),
            'truth_samples': len(all_clear),
            'below_0p35_m_samples': sum(1 for c, _ in all_clear if c < 0.35),
            'below_0p25_m_samples': sum(1 for c, _ in all_clear if c < 0.25),
            'collision': any(c < 0.25 for c, _ in all_clear),
        }
    # lethal / map occupancy along the truth trace
    if node.map is not None and all_clear:
        info = node.map.info
        lethal = mapocc = 0
        for c, g in all_clear:
            col = int(math.floor((g[1] - info.origin.position.x) / info.resolution))
            row = int(math.floor((g[2] - info.origin.position.y) / info.resolution))
            if 0 <= col < info.width and 0 <= row < info.height:
                v = node.map.data[row * info.width + col]
                if v >= 65:
                    mapocc += 1
                if v >= 99:
                    lethal += 1
        out['safety']['map_occupied_samples'] = mapocc
        out['safety']['map_lethal_samples'] = lethal
    out['scan_late'] = scan_stats(node.scan_hist[-1]) if node.scan_hist else None
    out['elapsed_wall_s'] = round(wall_end - wall0, 1)

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(out, open(OUT, 'w', encoding='utf-8'), indent=1, ensure_ascii=False)
    print(json.dumps({k: out[k] for k in ('patrol', 'safety') if k in out},
                     ensure_ascii=False))
    print('REPORT', OUT)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()