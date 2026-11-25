#!/usr/bin/env python3
"""在场地里生成/删除一个"敌方机器人"障碍（静态模型，走 gz 服务，不依赖 GUI）。

为什么不用 GUI 里的 Insert：WSL2 的 Gazebo GUI 是 llvmpipe 软件渲染，GUI 插动态模型
（球/箱子落下滚动）会让 GUI 持续重绘 —— 2026-11-23 实测 `gz sim gui` 飙到 349% CPU、
RTF 掉到 0.10，表现就是"死机"。这个脚本在 server 侧生成一个 static 模型，渲染循环
只在它出现/消失时重绘一次。

用法：
  python3 tools/spawn_obstacle.py spawn 1.5 -1.25          # 生成（默认边长 0.40 m）
  python3 tools/spawn_obstacle.py spawn 2.0 1.0 --yaw 90 --name enemy2
  python3 tools/spawn_obstacle.py remove                   # 删掉默认名字
  python3 tools/spawn_obstacle.py remove --name enemy2

remove 会显式带 `type=MODEL` 并回读 `gz model --list` 校验（2026-11-25：只给 name 时
服务返回 true 但不删模型 → 世界/雷达/代价地图里留"幽灵障碍"，后一个用例被环境污染）。
"""
import argparse
import subprocess
import sys
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node

WORLD = "/world/rmul2027_3v3"


def _model_list():
    """当前世界里的模型名集合（走 gz CLI；读不到就当空集合，不阻塞主流程）。"""
    try:
        out = subprocess.run(["gz", "model", "--list"], capture_output=True,
                             text=True, timeout=15).stdout
    except (OSError, subprocess.SubprocessError):
        return set()
    names = set()
    for line in out.splitlines():
        line = line.strip()
        if line.startswith("- "):
            names.add(line[2:].strip())
    return names

SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>{static}</static>
    <pose>0 0 0 0 0 0</pose>
    <link name="body">
      <collision name="collision">
        <geometry><box><size>{size} {size} {height}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{size} {size} {height}</size></box></geometry>
        <material>
          <ambient>0.45 0.03 0.03 1</ambient>
          <diffuse>0.82 0.08 0.08 1</diffuse>
          <specular>0.15 0.15 0.15 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("action", choices=["spawn", "remove"])
    ap.add_argument("x", nargs="?", type=float, default=1.5, help="世界坐标 x（spawn 用）")
    ap.add_argument("y", nargs="?", type=float, default=-1.25, help="世界坐标 y（spawn 用）")
    ap.add_argument("--name", default="enemy_robot")
    ap.add_argument("--size", type=float, default=0.40, help="车体边长 m（RM 规则上限 0.6）")
    ap.add_argument("--height", type=float, default=0.25)
    ap.add_argument("--yaw", type=float, default=0.0, help="朝向 deg")
    ap.add_argument("--dynamic", action="store_true",
                    help="参与物理（可被 set_pose 推动、会与车相撞）；默认 static 只做感知障碍")
    args = ap.parse_args()

    node = Node()

    if args.action == "remove":
        if args.name not in _model_list():
            print(f"remove {args.name}: 世界里没有这个模型（already gone）")
            return 0
        req = Entity()
        req.name = args.name
        # 必须显式给 type=MODEL：只给 name 时 gz 的 /world/<w>/remove 返回 true 却不删模型
        # （2026-11-25 实测：模型还在，/scan_safety 仍量到 1.000 m，代价地图留"幽灵障碍"，
        #  后续用例被环境污染）。type 缺失 → 服务当成空实体，静默成功。
        req.type = getattr(Entity, "MODEL", 2)
        ok, resp = node.request(WORLD + "/remove", req, Entity, Boolean, 3000)
        result = getattr(resp, "data", None)
        time.sleep(0.5)
        gone = args.name not in _model_list()
        print(f"remove {args.name}: rc={ok} result={result} verified_gone={gone}")
        return 0 if (ok and result and gone) else 1

    import math
    req = EntityFactory()
    req.name = args.name
    req.allow_renaming = False
    req.sdf = SDF.format(name=args.name, size=args.size, height=args.height,
                         static="false" if args.dynamic else "true")
    req.pose.CopyFrom(Pose(name=args.name))
    req.pose.position.x = args.x
    req.pose.position.y = args.y
    req.pose.position.z = args.height / 2.0
    half = math.radians(args.yaw) / 2.0
    req.pose.orientation.z = math.sin(half)
    req.pose.orientation.w = math.cos(half)
    ok, resp = node.request(WORLD + "/create", req, EntityFactory, Boolean, 5000)
    result = getattr(resp, "data", None)
    print(f"spawn {args.name} at ({args.x:.2f}, {args.y:.2f}) size={args.size:.2f}: "
          f"rc={ok} result={result}")
    return 0 if ok and result else 1


if __name__ == "__main__":
    sys.exit(main())