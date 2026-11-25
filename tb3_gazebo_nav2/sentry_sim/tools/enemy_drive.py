#!/usr/bin/env python3
"""动力学敌车：用速度指令驱动（VelocityControl + 物理），而不是 set_pose 瞬移。

和 moving_obstacle.py 的分工：
  - moving_obstacle.py 用 set_pose 摆位置：落点精确、适合"按几何算间距"的验收；
    但它是 0.1 s 步进的瞬移，测不了加速度/惯性，撞上也不产生物理接触。
  - enemy_drive.py（本工具）给模型发 /model/<name>/cmd_vel（gz.msgs.Twist），
    由 gz 的 VelocityControl 系统积分、物理求解碰撞：能测"真实速度"、能推/被推。
    代价：落点由动力学决定，不能当精确几何夹具用。

模型必须用 spawn_obstacle.py --dynamic 生成（SDF 里带 gz-sim-velocity-control-system）。

用法：
  # 生成 + 驱动 3 s（0.5 m/s 沿自身 +x），结束后自动停住
  python3 tools/enemy_drive.py --spawn -1.0 1.2 --linear 0.5 --duration 3
  # 只驱动已存在的模型（例如敌车从别处开过来）
  python3 tools/enemy_drive.py --linear 0.4 --angular 0.5 --duration 2.5
  # 删掉
  python3 tools/enemy_drive.py --remove

判据（--verify，默认开）：1 s 窗口速度 = 指令 ±0.05 m/s；逐帧无瞬移尖峰（<=2.5x 指令）。
"""
import argparse
import math
import os
import subprocess
import sys
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist
from gz.msgs10.vector3d_pb2 import Vector3d
from gz.transport13 import Node

WORLD = "/world/rmul2027_3v3"
HERE = os.path.dirname(os.path.abspath(__file__))
POSE_INFO = WORLD + "/pose/info"


class PoseTracker:
    """从 /world/<w>/pose/info 里抽某个模型的 (t, x, y, yaw)。

    t 优先用消息头里的仿真时间（帧率稳定、不会因为传输批量到达把 dt 算成 ~0）；
    没有 header 时退回到达时刻的单调时钟。
    """

    def __init__(self, name):
        self.name = name
        self.samples = []
        self.node = Node()
        self.node.subscribe(Pose_V, POSE_INFO, self._cb)

    def _cb(self, msg):
        try:
            t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        except AttributeError:
            t = 0.0
        if t <= 0.0:
            t = time.monotonic()
        for pose in msg.pose:
            if pose.name != self.name:
                continue
            q = pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            self.samples.append((t, pose.position.x, pose.position.y, yaw))

    def _steady(self, settle=0.6):
        """丢掉起步加速段（指令阶跃后的 settle 秒）。"""
        if not self.samples:
            return []
        t0 = self.samples[0][0]
        out = [s for s in self.samples if s[0] - t0 >= settle]
        return out if len(out) >= 2 else self.samples

    def window_speeds(self, win=1.0, settle=0.6):
        """把样本按 win 秒窗口配对，返回每个窗口的平均速度（m/s）。"""
        samples = self._steady(settle)
        out = []
        n = len(samples)
        for k in range(n):
            t0, x0, y0, _ = samples[k]
            for j in range(k + 1, n):
                t1, x1, y1, _ = samples[j]
                if t1 - t0 >= win:
                    out.append(math.hypot(x1 - x0, y1 - y0) / (t1 - t0))
                    break
        return out

    def frame_speeds(self, settle=0.6):
        samples = self._steady(settle)
        out = []
        for k in range(1, len(samples)):
            t0, x0, y0, _ = samples[k - 1]
            t1, x1, y1, _ = samples[k]
            dt = t1 - t0
            if dt > 1e-3:
                out.append(math.hypot(x1 - x0, y1 - y0) / dt)
        return out

    def displacement(self):
        if len(self.samples) < 2:
            return 0.0, 0.0
        a, b = self.samples[0], self.samples[-1]
        return b[1] - a[1], b[2] - a[2]


def spawn(name, x, y, yaw, size, height):
    cmd = [sys.executable, os.path.join(HERE, "spawn_obstacle.py"), "spawn",
           str(x), str(y), "--name", name, "--size", str(size),
           "--height", str(height), "--yaw", str(yaw), "--dynamic"]
    return subprocess.run(cmd).returncode


def remove(name):
    return subprocess.run([sys.executable, os.path.join(HERE, "spawn_obstacle.py"),
                           "remove", "--name", name]).returncode


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--spawn", nargs=2, type=float, metavar=("X", "Y"),
                    help="先生成一个动力学敌车（必须 --dynamic）")
    ap.add_argument("--name", default="enemy_robot")
    ap.add_argument("--size", type=float, default=0.40)
    ap.add_argument("--height", type=float, default=0.25)
    ap.add_argument("--yaw", type=float, default=0.0, help="spawn 朝向 deg")
    ap.add_argument("--linear", type=float, default=0.5, help="前进速度 m/s（车体 +x）")
    ap.add_argument("--angular", type=float, default=0.0, help="偏航角速度 rad/s")
    ap.add_argument("--duration", type=float, default=3.0, help="发指令的时长（墙钟秒）")
    ap.add_argument("--verify", dest="verify", action="store_true", default=True)
    ap.add_argument("--no-verify", dest="verify", action="store_false")
    ap.add_argument("--remove", action="store_true", help="结束后删除模型")
    ap.add_argument("--remove-only", action="store_true", help="只删模型")
    args = ap.parse_args()

    if args.remove_only:
        return remove(args.name)

    if args.spawn:
        rc = spawn(args.name, args.spawn[0], args.spawn[1], args.yaw,
                   args.size, args.height)
        if rc != 0:
            print(f"spawn 失败 rc={rc}", flush=True)
            return 4
        time.sleep(1.0)                     # 等模型进物理世界/话题挂上

    tracker = PoseTracker(args.name)
    pub = Node().advertise("/model/%s/cmd_vel" % args.name, Twist)

    def send(vx, wz):
        msg = Twist()
        msg.linear.CopyFrom(Vector3d(x=vx))
        msg.angular.CopyFrom(Vector3d(z=wz))
        pub.publish(msg)

    t0 = time.monotonic()
    n_pub = 0
    try:
        while time.monotonic() - t0 < args.duration:
            send(args.linear, args.angular)
            n_pub += 1
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    for _ in range(6):                      # 停住
        send(0.0, 0.0)
        time.sleep(0.05)
    time.sleep(0.6)                         # 收尾样本

    dx, dy = tracker.displacement()
    dist = math.hypot(dx, dy)
    wins = tracker.window_speeds(1.0)
    frames = tracker.frame_speeds()
    cmd = abs(args.linear)
    print(f"drive {args.name}: cmd={args.linear:+.2f} m/s  wz={args.angular:+.2f} rad/s  "
          f"pub={n_pub}  samples={len(tracker.samples)}", flush=True)
    print(f"  displacement=({dx:+.2f},{dy:+.2f}) m  dist={dist:.2f} m", flush=True)
    if wins:
        wins_sorted = sorted(wins)
        med = wins_sorted[len(wins_sorted) // 2]
        print(f"  window_speed(1s): n={len(wins)} min={min(wins):.3f} "
              f"median={med:.3f} max={max(wins):.3f} m/s", flush=True)
    if frames:
        print(f"  frame_speed: n={len(frames)} max={max(frames):.3f} m/s "
              f"(瞬移会出 >3 m/s 尖峰)", flush=True)

    rc = 0
    if args.verify:
        checks = []
        if cmd > 0.05:
            if wins:
                med = sorted(wins)[len(wins) // 2]
                checks.append(("窗口速度≈指令(±0.05)",
                               abs(med - cmd) <= 0.05, f"median={med:.3f} cmd={cmd:.3f}"))
            else:
                checks.append(("窗口速度≈指令(±0.05)", False, "没有 1 s 窗口样本"))
        if frames:
            checks.append(("无瞬移尖峰(<=2.5x指令+0.2)",
                           max(frames) <= 2.5 * cmd + 0.2,
                           f"max_frame={max(frames):.2f}"))
        checks.append(("真的动了(dist>=0.3 m)", dist >= 0.3, f"dist={dist:.2f} m"))
        print("===== 判据 =====", flush=True)
        for name, ok, detail in checks:
            print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}", flush=True)
            if not ok:
                rc = 3
    if args.remove:
        remove(args.name)
    return rc


if __name__ == "__main__":
    sys.exit(main())