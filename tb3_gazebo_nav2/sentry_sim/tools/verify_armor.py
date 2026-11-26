#!/usr/bin/env python3
"""装甲板检测验收（S3e）：把带装甲板的敌车摆到已知位置，量检测的
「颜色 / 距离 / 方位」误差。

前提：栈带相机（`./run_sentry_nav.sh gui:=false camera:=true`），
     并且机器人静止（位置从 /gt_odom 真值读，不用定位）。

几何口径（相机装在 base_link (0.10, 0, 0.20)，朝 +x）：
  距离（检测）= 到装甲板平面的深度 = fx*0.135/w_px；
  期望深度 = hypot(dx-0.10, dy) - (车半宽 0.20 + 板厚/2 0.003)  —— 只对正对相机（dy=0）成立；
  横移用例只验方位：期望世界方位 = atan2(dy, dx-0.10)，
  而检测的相机系方位符号相反（实测：目标在 +y 时落在图像左侧）⇒ world = -bearing_cam。

判据（逐条打印，全过返回 0）：
  1) 正对 1.0 / 2.0 / 3.0 m：颜色对、距离误差 <= max(0.20 m, 8%)、方位 |b| <= 0.06 rad
  2) 横移 ±0.5 m @2.0 m：方位误差 <= 0.06 rad、颜色对
  3) 蓝色装甲板 2.0 m：颜色对
  4) 每个用例的检出率 >= 50%（相机 ~5-6 Hz，采 2 s）

用法：
  python3 tools/verify_armor.py
  python3 tools/verify_armor.py --out results/report_armor.json
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
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

HERE = os.path.dirname(os.path.abspath(__file__))
CAM_X = 0.10
ROBOT_HALF = 0.20
PLATE_T = 0.006
ENEMY = "enemy_armor"


def spawn(dx, dy, color, rx, ry, yaw=0.0):
    cmd = [sys.executable, os.path.join(HERE, "spawn_obstacle.py"), "spawn",
           str(rx + dx), str(ry + dy), "--armor", color, "--name", ENEMY,
           "--yaw", str(yaw)]
    return subprocess.run(cmd, capture_output=True, text=True).returncode


def remove():
    subprocess.run([sys.executable, os.path.join(HERE, "spawn_obstacle.py"),
                    "remove", "--name", ENEMY], capture_output=True, text=True)


class Armor(Node):

    def __init__(self):
        super().__init__('verify_armor', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.robot = None
        self.dets = []
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/gt_odom', self.on_odom, qos)
        self.create_subscription(String, '/armor/detections', self.on_det, 10)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.robot = (p.x, p.y)

    def on_det(self, msg):
        try:
            self.dets.append(json.loads(msg.data))
        except json.JSONDecodeError:
            pass

    def collect(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--settle', type=float, default=1.5, help='spawn 后等多久再采')
    ap.add_argument('--window', type=float, default=2.0, help='采多久（墙钟）')
    ap.add_argument('--out', default='')
    ap.add_argument('--no-detector', action='store_true',
                    help='不自己起 armor_detect.py（默认会起，验完杀掉）')
    args = ap.parse_args()

    detector = None
    if not args.no_detector:
        detector = subprocess.Popen([sys.executable, os.path.join(HERE, 'armor_detect.py')],
                                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1.5)                     # 等它订阅上 /camera/image

    rclpy.init()
    node = Armor()
    t0 = time.monotonic()
    while node.robot is None and time.monotonic() - t0 < 20:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.robot is None:
        print('FAIL: 拿不到 /gt_odom')
        return 3
    rx, ry = node.robot
    print(f'robot at ({rx:+.2f},{ry:+.2f})')

    cases = [
        {'tag': 'red_1.0m', 'dx': 1.0, 'dy': 0.0, 'color': 'red', 'geometry': True},
        {'tag': 'red_2.0m', 'dx': 2.0, 'dy': 0.0, 'color': 'red', 'geometry': True},
        {'tag': 'red_3.0m', 'dx': 3.0, 'dy': 0.0, 'color': 'red', 'geometry': True},
        {'tag': 'red_2.0m_left', 'dx': 2.0, 'dy': +0.5, 'color': 'red', 'geometry': False},
        {'tag': 'red_2.0m_right', 'dx': 2.0, 'dy': -0.5, 'color': 'red', 'geometry': False},
        {'tag': 'blue_2.0m', 'dx': 2.0, 'dy': 0.0, 'color': 'blue', 'geometry': True},
    ]

    results = []
    for c in cases:
        remove()
        time.sleep(0.3)
        if spawn(c['dx'], c['dy'], c['color'], rx, ry) != 0:
            print(f"FAIL: 生成 {c['tag']} 失败")
            remove()
            continue
        c['time'] = time.strftime('%H%M%S')
        node.dets = []
        node.collect(args.settle)
        node.dets = []
        node.collect(args.window)
        remove()

        matched = [d for msg in node.dets for d in msg.get('detections', [])
                   if d.get('color') == c['color']]
        n_frames = len(node.dets)
        rate = (len({id(m) for m in node.dets if any(
            d.get('color') == c['color'] for d in m.get('detections', []))})
                / float(n_frames)) if n_frames else 0.0
        exp_depth = math.hypot(c['dx'] - CAM_X, c['dy']) - (ROBOT_HALF + PLATE_T / 2.0)
        exp_bearing = math.atan2(c['dy'], c['dx'] - CAM_X)
        row = {'tag': c['tag'], 'color': c['color'], 'dx': c['dx'], 'dy': c['dy'],
               'geometry': c['geometry'],
               'exp_depth_m': round(exp_depth, 3),
               'exp_bearing_rad': round(exp_bearing, 4),
               'frames': n_frames, 'det_rate': round(rate, 2)}
        if matched:
            depth = float(np.median([d['distance_m'] for d in matched]))
            # 相机系方位 → 世界系方位：符号相反（实测）
            bear = float(np.median([-d['bearing_rad'] for d in matched]))
            wpx = float(np.median([d['w_px'] for d in matched]))
            row.update({'depth_m': round(depth, 3), 'bearing_rad': round(bear, 4),
                        'w_px': wpx,
                        'depth_err_m': round(abs(depth - exp_depth), 3),
                        'bearing_err_rad': round(abs(bear - exp_bearing), 4)})
        results.append(row)
        print(f"  {c['tag']:>16s}: frames={n_frames} det={rate:.2f} "
              f"depth={row.get('depth_m')} (exp {exp_depth:.3f}) "
              f"bearing={row.get('bearing_rad')} (exp {exp_bearing:+.3f})", flush=True)

    node.destroy_node()
    rclpy.shutdown()
    if detector is not None:
        detector.terminate()
        try:
            detector.wait(timeout=5)
        except subprocess.TimeoutExpired:
            detector.kill()

    checks = []
    for r in results:
        got = r.get('depth_err_m')
        checks.append((f"{r['tag']} 检出率>=0.5", r['det_rate'] >= 0.5, f"{r['det_rate']}"))
        checks.append((f"{r['tag']} 颜色匹配", bool(got is not None), 'detected' if got is not None
                       else 'no detection'))
        if got is None:
            continue
        if r['geometry']:
            tol = max(0.20, 0.08 * r['exp_depth_m'])
            checks.append((f"{r['tag']} 距离误差<=max(0.2,8%)",
                           got <= tol, f"{got:.3f} m (tol {tol:.2f})"))
        checks.append((f"{r['tag']} 方位误差<=0.06 rad",
                       r['bearing_err_rad'] <= 0.06, f"{r['bearing_err_rad']:.4f} rad"))

    print('\n===== 判据 =====')
    ok = True
    for label, passed, detail in checks:
        ok = ok and passed
        print(f"  [{'PASS' if passed else 'FAIL'}] {label}: {detail}")
    payload = {'pass': ok, 'robot': [rx, ry], 'cases': results,
               'checks': {c[0]: {'pass': c[1], 'detail': c[2]} for c in checks}}
    out = args.out or os.path.join(HERE, '..', 'results',
                                   'report_armor_%s.json' % time.strftime('%H%M%S'))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, 'w', encoding='utf-8') as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    print('报告:', os.path.abspath(out))
    return 0 if ok else 3


if __name__ == '__main__':
    sys.exit(main())