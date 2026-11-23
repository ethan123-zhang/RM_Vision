#!/usr/bin/env python3
"""移动的"敌方机器人"：把一个敌方车模型沿线段来回推，用来测"横穿的敌车能不能绕开"。

为什么用 set_pose 推、而不是给模型加轮子/速度：位置完全可控（验收时能精确算间距），
而且 static 模型不参与碰撞求解、不会持续重绘 —— WSL2 的 llvmpipe 下 GUI 不会被拖死
（GUI 里插动态模型会把 RTF 打到 0.1，见 README §2.6）。

用法：
  python3 tools/moving_obstacle.py --from 0.5 1.5 --to 0.5 -2.5 --speed 0.5
  python3 tools/moving_obstacle.py --remove          # 删掉
  # --dynamic 让模型参与物理（会与车相撞），默认 static 只做"感知障碍"
"""
import argparse
import math
import subprocess
import sys
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node

WORLD = "/world/rmul2027_3v3"
HERE = __file__.rsplit("/", 1)[0]


def pose_at(t, x1, y1, x2, y2, speed):
    """线段上往返（三角波），返回 t 秒后的位置。"""
    length = math.hypot(x2 - x1, y2 - y1)
    if length < 1e-6:
        return x1, y1
    s = (t * speed) % (2.0 * length)
    if s > length:
        s = 2.0 * length - s
    frac = s / length
    return x1 + (x2 - x1) * frac, y1 + (y2 - y1) * frac


def spawn(name, x, y, size, dynamic):
    cmd = [sys.executable, HERE + "/spawn_obstacle.py", "spawn", str(x), str(y),
           "--name", name, "--size", str(size)]
    if dynamic:
        cmd.append("--dynamic")
    return subprocess.run(cmd).returncode


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--from", dest="p1", nargs=2, type=float, default=(0.5, 1.5),
                    metavar=("X1", "Y1"))
    ap.add_argument("--to", dest="p2", nargs=2, type=float, default=(0.5, -2.5),
                    metavar=("X2", "Y2"))
    ap.add_argument("--speed", type=float, default=0.5, help="m/s（墙钟）")
    ap.add_argument("--duration", type=float, default=120.0, help="跑多久墙钟秒")
    ap.add_argument("--name", default="enemy_robot")
    ap.add_argument("--size", type=float, default=0.40)
    ap.add_argument("--dynamic", action="store_true", help="参与物理（会被撞/推）")
    ap.add_argument("--remove", action="store_true", help="只删模型")
    args = ap.parse_args()

    if args.remove:
        rc = subprocess.run([sys.executable, HERE + "/spawn_obstacle.py", "remove",
                             "--name", args.name]).returncode
        return rc

    x1, y1 = args.p1
    x2, y2 = args.p2
    rc = spawn(args.name, x1, y1, args.size, args.dynamic)
    if rc != 0:
        print(f"spawn 失败 rc={rc}", flush=True)
        return 4
    node = Node()
    t0 = time.monotonic()
    last_print = 0.0
    try:
        while time.monotonic() - t0 < args.duration:
            t = time.monotonic() - t0
            x, y = pose_at(t, x1, y1, x2, y2, args.speed)
            req = Pose(name=args.name)
            req.position.x = x
            req.position.y = y
            req.position.z = 0.125
            req.orientation.w = 1.0
            node.request(WORLD + "/set_pose", req, Pose, Boolean, 1000)
            if t - last_print >= 1.0:
                print(f"MOVE t={t:6.1f}s ({x:+.2f}, {y:+.2f})", flush=True)
                last_print = t
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    print("MOVE done", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())