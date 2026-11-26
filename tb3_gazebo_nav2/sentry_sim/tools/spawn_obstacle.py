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
      <inertial>
        <mass>15.0</mass>
        <inertia>
          <ixx>0.35</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
          <iyy>0.35</iyy><iyz>0.0</iyz>
          <izz>0.45</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>{size} {size} {height}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{size} {size} {height}</size></box></geometry>
        <material>
          <ambient>{body_ambient}</ambient>
          <diffuse>{body_diffuse}</diffuse>
          <specular>0.15 0.15 0.15 1</specular>
        </material>
      </visual>{armor}
    </link>{extra}
  </model>
</sdf>
"""

# 动力学的敌车：VelocityControl 系统让模型接受 /model/<name>/cmd_vel（gz.msgs.Twist）
# 的速度指令，由物理积分、与机器人真实碰撞 —— 取代"set_pose 瞬移"的测法。
# 注意它只认速度、不认位置：要精确落点仍用 moving_obstacle.py 的 set_pose。
VEL_CTRL = """
    <plugin filename="gz-sim-velocity-control-system"
            name="gz::sim::systems::VelocityControl">
      <topic>/model/{name}/cmd_vel</topic>
    </plugin>"""

# S3e 装甲板：RM 小装甲板 135x55 mm（这里 3 mm 厚，纯视觉、不做碰撞）。四面各一块、
# 离地 0.15 m；检测靠前视相机（`./run_sentry_nav.sh camera:=true`）+ tools/armor_detect.py。
PLATE_W = 0.135
PLATE_H = 0.055
PLATE_T = 0.006

ARMOR_VISUAL = """
      <visual name="armor_{face}">
        <pose>{x} {y} {z} 0 0 0</pose>
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
          <emissive>{er} {eg} {eb} 1</emissive>
        </material>
      </visual>"""


def _armor_visuals(size, armor_color, height):
    """四面装甲板的视觉块（相对 link 原点；模型落地后 link 中心在 height/2）。"""
    if armor_color == "red":
        rgb = (0.90, 0.05, 0.05)
    elif armor_color == "blue":
        rgb = (0.05, 0.10, 0.90)
    else:
        return ""
    r, g, b = rgb
    er, eg, eb = (0.25 * r, 0.25 * g, 0.25 * b)      # 自发光一点，暗光下也能检出
    z = 0.15 - height / 2.0
    half = size / 2.0 + PLATE_T / 2.0
    faces = [
        ("front", half, 0.0, PLATE_T, PLATE_W, PLATE_H),
        ("back", -half, 0.0, PLATE_T, PLATE_W, PLATE_H),
        ("left", 0.0, half, PLATE_W, PLATE_T, PLATE_H),
        ("right", 0.0, -half, PLATE_W, PLATE_T, PLATE_H),
    ]
    return "".join(ARMOR_VISUAL.format(face=face, x=x, y=y, z=z, sx=sx, sy=sy, sz=sz,
                                       r=r, g=g, b=b, er=er, eg=eg, eb=eb)
                   for face, x, y, sx, sy, sz in faces)


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
    ap.add_argument("--armor", choices=["none", "red", "blue"], default="none",
                    help="给敌车四面加装甲板（颜色 side）；配 camera:=true 做检测")
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
    req.sdf = SDF.format(
        name=args.name, size=args.size, height=args.height,
        static="false" if args.dynamic else "true",
        extra=VEL_CTRL.format(name=args.name) if args.dynamic else "",
        armor=_armor_visuals(args.size, args.armor, args.height),
        # 有装甲板时车体压成深灰，避免红底板把红装甲板的 HSV 检出搅浑
        body_ambient=("0.25 0.25 0.27 1" if args.armor != "none" else "0.45 0.03 0.03 1"),
        body_diffuse=("0.35 0.35 0.38 1" if args.armor != "none" else "0.82 0.08 0.08 1"))
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