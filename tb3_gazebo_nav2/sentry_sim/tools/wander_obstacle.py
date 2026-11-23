#!/usr/bin/env python3
"""随机漫游的"敌方机器人"：在地图自由区里随机选点来回走，用来做"我开车、它乱走"的碰撞测试。

实现要点：
- 走位用 `gz set_pose` 推（static 模型），位置完全可控、不参与物理求解，WSL2 的 llvmpipe 下
  不会像 GUI 里插动态模型那样把 RTF 打到 0.1；
- 目标点只在"车能站"的自由格里挑（离静态障碍 >= --clearance），并且**离我方车 >= --car-clear**
  ——否则敌车会主动撞过来，测的就不是"我撞它"了；
- 发布 `/enemy_pose`（PoseStamped, map 系），配合 tools/teleop_gui.py 显示"离敌车多远/有没有碰撞"。

用法：
  python3 tools/wander_obstacle.py                       # 默认 0.45 m/s、净空 >=0.9 m、离车 >=1.2 m
  python3 tools/wander_obstacle.py --speed 0.6 --clearance 1.0
  python3 tools/wander_obstacle.py --remove              # 结束并删模型
"""
import argparse
import math
import os
import random
import subprocess
import sys
import time

import numpy as np
from PIL import Image
from scipy import ndimage

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node as GzNode

WORLD = "/world/rmul2027_3v3"
DEFAULT_MAP = "/mnt/e/RM/tb3_gazebo_nav2/rm_field/maps/rmul2027_3v3_up.yaml"
HERE = os.path.dirname(os.path.abspath(__file__))


def load_map(yaml_path):
    cfg = {}
    with open(yaml_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if ":" in line:
                k, v = line.split(":", 1)
                cfg[k.strip()] = v.strip()
    res = float(cfg["resolution"])
    ox, oy = [float(x) for x in cfg["origin"].strip("[] ").split(",")[:2]]
    occ = np.array(Image.open(os.path.join(os.path.dirname(yaml_path), cfg["image"]))) < 80
    clear = ndimage.distance_transform_edt(~occ) * res
    return res, ox, oy, occ, clear


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default=DEFAULT_MAP)
    ap.add_argument("--speed", type=float, default=0.45, help="m/s")
    ap.add_argument("--clearance", type=float, default=0.9, help="目标点离静态障碍的最小距离 m")
    ap.add_argument("--car-clear", type=float, default=0.0,
                    help="敌车离我方车至少这么远 m（0=不管车，纯随机漫游；要它别撞你就给 1.2）")
    ap.add_argument("--contact", type=float, default=0.47, help="判定碰撞的中心距 m")
    ap.add_argument("--name", default="enemy_robot")
    ap.add_argument("--size", type=float, default=0.40)
    ap.add_argument("--dynamic", action="store_true", help="让模型参与物理（默认 static）")
    ap.add_argument("--duration", type=float, default=0.0, help="跑多久墙钟秒，0=一直跑")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--remove", action="store_true", help="只删模型")
    args = ap.parse_args()

    if args.remove:
        return subprocess.run([sys.executable, os.path.join(HERE, "spawn_obstacle.py"),
                               "remove", "--name", args.name]).returncode

    random.seed(args.seed)
    res, ox, oy, occ, clear = load_map(args.map)
    H, W = occ.shape

    def clr(x, y):
        r = H - 1 - int(round((y - oy) / res))
        c = int(round((x - ox) / res))
        if not (0 <= r < H and 0 <= c < W):
            return -1.0
        return float(clear[r, c])

    # 候选自由格：净空够、且不贴地图边界
    free_r, free_c = np.nonzero((clear >= max(0.55, args.clearance * 0.6)) & (clear > 0))
    candidates = [(ox + c * res, oy + (H - 1 - r) * res) for r, c in zip(free_r, free_c)]

    def pick_target(car_xy):
        for _ in range(400):
            x, y = random.choice(candidates)
            if clr(x, y) < args.clearance:
                continue
            if car_xy is not None and math.hypot(x - car_xy[0], y - car_xy[1]) < args.car_clear:
                continue
            return x, y
        return None

    # 先 spawn 到第一个随机点
    rclpy.init()
    node = Node("wander_obstacle",
                parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    pub = node.create_publisher(PoseStamped, "/enemy_pose", 10)
    state = {"car": None}

    def on_odom(m):
        p = m.pose.pose.position
        state["car"] = (p.x, p.y)

    node.create_subscription(Odometry, "/odin1/odometry", on_odom, 10)
    gz = GzNode()
    spawn_xy = pick_target(None) or (1.5, -1.25)
    rc = subprocess.run([sys.executable, os.path.join(HERE, "spawn_obstacle.py"), "spawn",
                         f"{spawn_xy[0]:.3f}", f"{spawn_xy[1]:.3f}", "--name", args.name,
                         "--size", str(args.size)] + (["--dynamic"] if args.dynamic else [])).returncode
    if rc != 0:
        print(f"spawn 失败 rc={rc}", flush=True)
        return 4
    print(f"[wander] {args.name} 起点 ({spawn_xy[0]:.2f},{spawn_xy[1]:.2f}) 速度 {args.speed} m/s "
          f"净空>={args.clearance} 离车>={args.car_clear}", flush=True)

    pos = list(spawn_xy)
    target = pick_target(state["car"])
    min_car = 999.0
    contact_reported = False
    t0 = time.monotonic()
    last_print = 0.0
    try:
        while args.duration <= 0 or time.monotonic() - t0 < args.duration:
            rclpy.spin_once(node, timeout_sec=0.0)
            car = state["car"]
            if target is None:
                target = pick_target(car)
            if target is not None:
                dx, dy = target[0] - pos[0], target[1] - pos[1]
                d = math.hypot(dx, dy)
                if d < 0.12:
                    target = pick_target(car)
                else:
                    step = min(d, args.speed * 0.1)
                    pos[0] += dx / d * step
                    pos[1] += dy / d * step
                if car is not None:
                    dc = math.hypot(pos[0] - car[0], pos[1] - car[1])
                    min_car = min(min_car, dc)
                    if dc < args.car_clear * 0.75:      # 车冲过来了：换一个更远的目标
                        target = pick_target(car)
                    if dc < args.contact and not contact_reported:
                        contact_reported = True
                        print(f"CONTACT 敌车({pos[0]:.2f},{pos[1]:.2f}) 车({car[0]:.2f},{car[1]:.2f}) "
                              f"中心距 {dc:.2f} m  ← 撞上了", flush=True)
                    if dc > args.contact + 0.15:
                        contact_reported = False
            req = Pose(name=args.name)
            req.position.x = pos[0]
            req.position.y = pos[1]
            req.position.z = 0.125
            req.orientation.w = 1.0
            gz.request(WORLD + "/set_pose", req, Pose, Boolean, 800)
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.pose.position.x = pos[0]
            msg.pose.position.y = pos[1]
            msg.pose.orientation.w = 1.0
            pub.publish(msg)
            now = time.monotonic()
            if now - last_print >= 2.0:
                last_print = now
                print(f"WANDER ({pos[0]:+.2f},{pos[1]:+.2f}) → 目标 "
                      f"{'None' if target is None else f'({target[0]:+.2f},{target[1]:+.2f})'}"
                      f" | 离车 {min_car:.2f} m", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    print(f"WANDER done | 全程离车最近 {min_car:.2f} m", flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())