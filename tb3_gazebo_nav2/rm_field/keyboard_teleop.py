#!/usr/bin/env python3
"""键盘遥控仿真小车：WASD / 方向键 → gz `/cmd_vel`。

用法：
    bash drive_teleop.sh              # 一键：起 GUI 世界 + 小车 + 键盘遥控
    python3 keyboard_teleop.py        # 世界已在跑时，只起键盘遥控
    python3 keyboard_teleop.py --selftest   # 自检：world+waffle 在跑时验证"真能开/真能停"

按键（按住即走，松开自动停）：
    w / ↑  前进          s / ↓  后退
    a / ←  左转          d / →  右转
    q 前进左转   e 前进右转   z 后退左转   c 后退右转
    空格   立即停车（并把速度档复位）
    + / -  前进速度加/减档（0.10 … 0.50 m/s）
    [ / ]  转向速度加/减档（0.3 … 1.5 rad/s）
    Ctrl+C 退出（退出前发零）

为什么"松开自动停"能成立：终端收不到 keyup，只有按键重复；所以每个按键给命令
0.5 s 的 TTL，按住时系统重复发键会不断续期，松手 0.5 s 后自动发零。
"""
import argparse
import math
import os
import select
import sys
import threading
import time

import gz.msgs10.pose_v_pb2 as pose_v_pb2
import gz.msgs10.twist_pb2 as twist_pb2
from gz.transport13 import Node

VMAX_STEPS = [0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50]
WMAX_STEPS = [0.3, 0.5, 0.8, 1.0, 1.5]

# 键 → (lx, az)，都是 -1..1 的归一化值
KEYS = {
    "w": (1, 0), "up": (1, 0),
    "s": (-1, 0), "down": (-1, 0),
    "a": (0, 1), "left": (0, 1),
    "d": (0, -1), "right": (0, -1),
    "q": (1, 1), "e": (1, -1),
    "z": (-1, 1), "c": (-1, -1),
}


class Teleop:
    def __init__(self, topic="/cmd_vel", vmax=0.25, wmax=0.8, ttl=0.5,
                 rate=20.0, world="drive", quiet=False):
        self.topic = topic
        self.vmax = float(vmax)
        self.wmax = float(wmax)
        self.ttl = float(ttl)
        self.rate = float(rate)
        self.world = world
        self.quiet = quiet
        self.lock = threading.Lock()
        self.lx = 0.0
        self.az = 0.0
        self.last_key = 0.0
        self.sx = 0.0
        self.sz = 0.0
        self.stop = False
        self.node = Node()
        self.pub = self.node.advertise(topic, twist_pb2.Twist)
        self._poses = {}
        self.node.subscribe(pose_v_pb2.Pose_V, f"/world/{world}/pose/info", self._on_pose)

    def _on_pose(self, msg):
        for p in msg.pose:
            self._poses[p.name] = {
                "x": p.position.x, "y": p.position.y, "z": p.position.z,
                "qx": p.orientation.x, "qy": p.orientation.y,
                "qz": p.orientation.z, "qw": p.orientation.w,
            }

    # ---------------- 指令 ----------------
    def key(self, k):
        k = k.lower()
        with self.lock:
            if k in (" ", "stop"):
                self.lx = self.az = 0.0
                self.last_key = 0.0
                return "stop"
            if k in ("+", "="):
                self.vmax = self._next(VMAX_STEPS, self.vmax, +1)
                return "vmax=%.2f" % self.vmax
            if k in ("-", "_"):
                self.vmax = self._next(VMAX_STEPS, self.vmax, -1)
                return "vmax=%.2f" % self.vmax
            if k == "]":
                self.wmax = self._next(WMAX_STEPS, self.wmax, +1)
                return "wmax=%.2f" % self.wmax
            if k == "[":
                self.wmax = self._next(WMAX_STEPS, self.wmax, -1)
                return "wmax=%.2f" % self.wmax
            if k in KEYS:
                self.lx, self.az = KEYS[k]
                self.last_key = time.time()
                return "cmd lx=%+d az=%+d" % (self.lx, self.az)
            return None

    @staticmethod
    def _next(steps, cur, direction):
        if direction > 0:
            for s in steps:
                if s > cur + 1e-9:
                    return s
            return steps[-1]
        for s in reversed(steps):
            if s < cur - 1e-9:
                return s
        return steps[0]

    def command(self, lx, az, ttl=None):
        """给自检用：直接设指令并续期。"""
        with self.lock:
            self.lx = float(lx)
            self.az = float(az)
            self.last_key = time.time() if (lx or az) else 0.0

    def zero(self):
        with self.lock:
            self.lx = self.az = 0.0
            self.last_key = 0.0

    # ---------------- 发布 ----------------
    def run(self):
        dt = 1.0 / self.rate
        while not self.stop:
            with self.lock:
                age = time.time() - self.last_key if self.last_key else 1e9
                tgt = (self.lx, self.az) if age <= self.ttl else (0.0, 0.0)
            alpha = 0.45
            self.sx += (tgt[0] - self.sx) * alpha
            self.sz += (tgt[1] - self.sz) * alpha
            if abs(self.sx) < 1e-3:
                self.sx = 0.0
            if abs(self.sz) < 1e-3:
                self.sz = 0.0
            tw = twist_pb2.Twist()
            tw.linear.x = self.sx * self.vmax
            tw.angular.z = self.sz * self.wmax
            self.pub.publish(tw)
            time.sleep(dt)

    def shutdown(self):
        self.stop = True
        time.sleep(0.1)
        self.pub.publish(twist_pb2.Twist())

    # ---------------- 状态显示 ----------------
    def hud(self):
        return ("  %+.2f m/s  %+.2f rad/s   vmax=%.2f  wmax=%.2f   "
                % (self.sx * self.vmax, self.sz * self.wmax, self.vmax, self.wmax))

    def pose(self, name="waffle", timeout=2.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            p = self._poses.get(name)
            if p is not None:
                yaw = math.atan2(2.0 * (p["qw"] * p["qz"] + p["qx"] * p["qy"]),
                                 1.0 - 2.0 * (p["qy"] ** 2 + p["qz"] ** 2))
                return {**p, "yaw": yaw}
            time.sleep(0.02)
        return None


def read_keys():
    """逐键产出；tty 下用 cbreak 读单字节并解析方向键转义序列。"""
    if not sys.stdin.isatty():
        # 非交互（管道/测试）：一行一个键
        for line in sys.stdin:
            for ch in line.strip():
                yield ch
        return
    import termios
    import tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while True:
            r, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not r:
                continue
            ch = sys.stdin.read(1)
            if not ch:                     # stdin 被关掉（管道/终端断开）
                break
            if ch == "\x1b":
                seq = ""
                r2, _, _ = select.select([sys.stdin], [], [], 0.05)
                if r2:
                    seq = sys.stdin.read(2)
                yield {"[A": "up", "[B": "down", "[C": "right", "[D": "left"}.get(seq, "esc")
                continue
            if ch in ("\r", "\n"):
                yield " "
                continue
            yield ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def run_interactive(t):
    print("键盘遥控就绪：w/s 前进后退，a/d 转向，q/e/z/c 斜向，空格停车，+/- 调速，[ ] 转向调速")
    print("注意：键盘焦点要在这个终端窗口；Gazebo 窗口只用来看，不用点它。Ctrl+C 退出。")
    print()
    sys.stdout.write("\r" + t.hud())
    sys.stdout.flush()
    last_hud = 0.0
    try:
        for k in read_keys():
            if k == "\x03":            # Ctrl+C
                break
            msg = t.key(k)
            if msg in ("stop",) or (msg and msg.startswith(("vmax", "wmax"))):
                print("\r" + (" " * 70) + "\r" + (msg or ""))
            if time.time() - last_hud > 0.1:
                sys.stdout.write("\r" + t.hud())
                sys.stdout.flush()
                last_hud = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        t.shutdown()
        print("\r" + (" " * 70) + "\r已发零停车，退出")


# ---------------- 自检 ----------------
def hold(t, keys, duration, tick=0.08):
    t0 = time.time()
    while time.time() - t0 < duration:
        for k in keys:
            t.key(k)
        time.sleep(tick)


def selftest(t):
    p0 = t.pose()
    if p0 is None:
        print("!! 读不到 waffle 位姿：world/waffle 在跑吗？（bash gui_colored.sh）")
        return 1
    print("初始位姿: x=%.3f y=%.3f yaw=%.3f" % (p0["x"], p0["y"], p0["yaw"]))
    ok = True

    # T1 前进
    hold(t, ["w"], 1.5)
    t.zero()
    time.sleep(1.0)
    p1 = t.pose()
    dx = p1["x"] - p0["x"]
    print("[T1] 按住 w 1.5 s：位移 = %+.3f m" % dx)
    ok &= dx > 0.12
    print("     ", "PASS" if dx > 0.12 else "FAIL（没走 / 太慢）")

    # T2 原地左转
    hold(t, ["a"], 1.5)
    t.zero()
    time.sleep(1.0)
    p2 = t.pose()
    dyaw = math.atan2(math.sin(p2["yaw"] - p1["yaw"]), math.cos(p2["yaw"] - p1["yaw"]))
    dist = math.hypot(p2["x"] - p1["x"], p2["y"] - p1["y"])
    print("[T2] 按住 a 1.5 s：航向变化 = %+.3f rad，位置移动 %.3f m" % (dyaw, dist))
    ok &= dyaw > 0.25
    print("     ", "PASS" if dyaw > 0.25 else "FAIL（没转）")

    # T3 松手 0.5 s 后自动停车
    t.zero()
    time.sleep(0.8)
    p3 = t.pose()
    time.sleep(1.0)
    p4 = t.pose()
    drift = math.hypot(p4["x"] - p3["x"], p4["y"] - p3["y"])
    print("[T3] 无指令 1 s：漂移 = %.4f m" % drift)
    ok &= drift < 0.02
    print("     ", "PASS" if drift < 0.02 else "FAIL（没有自动停）")

    # T4 方向键也要能开（w 与 ↑ 等价，这里改用 ↑ 的映射）
    p5 = t.pose()
    hold(t, ["up"], 1.2)
    t.zero()
    time.sleep(0.8)
    p6 = t.pose()
    dx2 = math.hypot(p6["x"] - p5["x"], p6["y"] - p5["y"])
    print("[T4] 按住 ↑ 1.2 s：位移 = %.3f m" % dx2)
    ok &= dx2 > 0.08
    print("     ", "PASS" if dx2 > 0.08 else "FAIL（方向键没生效）")

    print("\n自检:", "全部通过" if ok else "有失败项")
    return 0 if ok else 3


def main():
    ap = argparse.ArgumentParser(description="键盘遥控 Gazebo 小车")
    ap.add_argument("--topic", default="/cmd_vel")
    ap.add_argument("--world", default=os.environ.get("GZ_WORLD", "drive"))
    ap.add_argument("--vmax", type=float, default=0.25)
    ap.add_argument("--wmax", type=float, default=0.8)
    ap.add_argument("--ttl", type=float, default=0.5, help="松开后多久自动停（s）")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    t = Teleop(args.topic, args.vmax, args.wmax, args.ttl, world=args.world)
    threading.Thread(target=t.run, daemon=True).start()
    if args.selftest:
        time.sleep(1.0)
        return selftest(t)
    run_interactive(t)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())