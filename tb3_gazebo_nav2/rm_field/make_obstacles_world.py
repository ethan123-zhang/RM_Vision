#!/usr/bin/env python3
"""从 config/avoid_obstacles.json 生成两个验收用世界（xacro）：

  A. worlds/rmul2027_3v3_up_nav2_obstacles.sdf.xacro —— 绕障验收用
     静态箱（static）+ 动态箱（kinematic，triggered_retreat：停在走道正中，机器人靠近后让开）
  B. worlds/rmul2027_3v3_up_nav2_braking.sdf.xacro —— 安全层（collision_monitor）刹车验收用
     只有一个"突然出现"的宽挡板（kinematic，先藏在角落，机器人进到 0.7 m 内瞬移到正前方）

为什么要生成而不是手写：障碍物几何在**两处**被使用——
  1. 世界里的碰撞/视觉模型（Gazebo 物理 + 雷达）；
  2. 验收脚本里的投影（判"离障碍物多远算安全"）。
两处必须同源，否则又是"视觉网格 vs 碰撞几何"那类事故。这里以 JSON 为唯一源，
生成世界；验收脚本直接读同一份 JSON。

生成规则：
  - 基世界 = worlds/rmul2027_3v3_up_nav2.sdf.xacro（翻转后 3V3 场地）
  - 静态箱：<static>true</static>
  - 动态/弹出箱：<kinematic>true</kinematic> —— 由 dynamic_obstacle.py 用
    /world/<world>/set_pose 平移；探针实测 static 与 kinematic 模型的 set_pose 都会真实改
    pose/info，用 kinematic 是因为物理里会逐步跟随 pose（机器人推不飞它，撞上等于撞墙）。
  - 插入位置：gz-sim-physics-system 插件之前（世界 <world> 内容末尾），两端有 BEGIN/END 标记，
    可重复生成（旧块会被替换，不会叠加）。

用法：
  python3 make_obstacles_world.py                 # 生成 A+B 并回读自校核
  python3 make_obstacles_world.py --check         # 只校核
  python3 make_obstacles_world.py --only braking  # 只生成/校核 B

判据（生成后自动跑）：
  1. 每个箱体的 <model name>/<pose>/<size> 与 JSON 一致；static/kinematic 标记正确；
  2. 模型数与配置一致，BEGIN/END 成对；
  3. 箱体/线段都在场内（内侧面 ±6.00/±4.00 以内 0.3 m）；
  4. 动态箱移动线段 ≥0.40 m（0.3 m/s 走 1.3 s 以上，1 s 窗口才测得出速度）；
     挡板尺寸要真的挡得住走廊（宽 ≥2.6 m），否则"必须刹车"这个前提就不成立。
"""
import argparse
import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(HERE, "config", "avoid_obstacles.json")
BASE = os.path.join(HERE, "worlds", "rmul2027_3v3_up_nav2.sdf.xacro")
OUT = {
    "obstacles": os.path.join(HERE, "worlds", "rmul2027_3v3_up_nav2_obstacles.sdf.xacro"),
    "braking": os.path.join(HERE, "worlds", "rmul2027_3v3_up_nav2_braking.sdf.xacro"),
    "gap": os.path.join(HERE, "worlds", "rmul2027_3v3_up_nav2_gap.sdf.xacro"),
}
MARK_BEGIN = "    <!-- AVOID_OBSTACLES_BEGIN（由 make_obstacles_world.py 从 config/avoid_obstacles.json 生成，勿手改）-->"
MARK_END = "    <!-- AVOID_OBSTACLES_END -->"
ANCHOR = '    <plugin filename="gz-sim-physics-system"'


def load_cfg(path=CFG):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def box_model(name, center, size, z_center, color, mode="static"):
    r, g, b = color
    cx, cy = center
    sx, sy, sz = size
    mode_tag = f"      <{mode}>true</{mode}>\n"
    return f"""    <model name="{name}">
{mode_tag}      <pose>{cx:.4f} {cy:.4f} {z_center:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          <material>
            <ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient>
            <diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse>
            <specular>0.10 0.10 0.10 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""


def build_text(cfg, variant):
    blocks = [MARK_BEGIN]
    if variant == "obstacles":
        for ob in cfg["static"]:
            size = ob["size"]
            blocks.append(box_model(ob["name"], ob["center"], size, size[2] / 2.0,
                                    ob.get("color", [0.90, 0.45, 0.05]), "static"))
        dyn = cfg["dynamic"]
        size = dyn["size"]
        start = dyn.get("start", dyn.get("p0"))
        blocks.append(box_model(dyn["name"], start, size,
                                dyn.get("z_center", size[2] / 2.0),
                                dyn.get("color", [0.10, 0.55, 0.90]), "kinematic"))
    elif variant == "braking":
        pop = cfg["popup"]
        size = pop["size"]
        blocks.append(box_model(pop["name"], pop["hidden"], size,
                                pop.get("z_center", size[2] / 2.0),
                                pop.get("color", [0.85, 0.25, 0.20]), "kinematic"))
    else:      # gap：窄通道（两道墙之间留一个刚好能过的门）
        for ob in cfg["gate"]["boxes"]:
            size = ob["size"]
            blocks.append(box_model(ob["name"], ob["center"], size, size[2] / 2.0,
                                    ob.get("color", [0.30, 0.75, 0.30]), "static"))
    blocks.append(MARK_END)
    return "\n".join(blocks)


def generate(variant):
    with open(BASE, "r", encoding="utf-8") as f:
        base = f.read()
    cfg = load_cfg()
    base = re.sub(re.escape(MARK_BEGIN) + r".*?" + re.escape(MARK_END) + r"\n?", "",
                  base, flags=re.S)
    if ANCHOR not in base:
        raise SystemExit(f"基世界缺少锚点: {ANCHOR}")
    out = base.replace(ANCHOR, build_text(cfg, variant) + "\n" + ANCHOR, 1)
    with open(OUT[variant], "w", encoding="utf-8") as f:
        f.write(out)
    print(f"生成: {os.path.relpath(OUT[variant], HERE)}")


def _check_box(text, name, center, size, mode, errs):
    m = re.search(rf'<model name="{re.escape(name)}">.*?</model>', text, re.S)
    if not m:
        errs.append(f"缺模型 {name}")
        return
    block = m.group(0)
    if f"<{mode}>true</{mode}>" not in block:
        errs.append(f"{name} 应为 {mode}")
    if mode == "kinematic" and "<static>true</static>" in block:
        errs.append(f"{name} 不能是 static")
    cx, cy = center
    if not re.search(rf"<pose>\s*{cx:.4f}\s+{cy:.4f}\s+[\d.]+\s", block):
        errs.append(f"{name} 位姿与 JSON 不一致")
    sx, sy, sz = size
    if not re.search(rf"<box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box>", block):
        errs.append(f"{name} 尺寸与 JSON 不一致")


def check(variant):
    cfg = load_cfg()
    with open(OUT[variant], "r", encoding="utf-8") as f:
        text = f.read()
    errs = []
    if text.count(MARK_BEGIN) != 1 or text.count(MARK_END) != 1:
        errs.append(f"BEGIN/END 标记次数不对: {text.count(MARK_BEGIN)}/{text.count(MARK_END)}")
    names = []
    if variant == "obstacles":
        for ob in cfg["static"]:
            names.append(ob["name"])
            _check_box(text, ob["name"], ob["center"], ob["size"], "static", errs)
        dyn = cfg["dynamic"]
        names.append(dyn["name"])
        d_start = dyn.get("start", dyn.get("p0"))
        d_end = dyn.get("end", dyn.get("p1"))
        _check_box(text, dyn["name"], d_start, dyn["size"], "kinematic", errs)
        L = math.hypot(d_end[0] - d_start[0], d_end[1] - d_start[1])
        for p, tag in ((d_start, "start"), (d_end, "end")):
            if abs(p[0]) > 6.0 - 0.3 or abs(p[1]) > 4.0 - 0.3:
                errs.append(f"动态箱 {tag}={p} 太贴外墙")
        # 线段下限 0.40 m：按 0.3 m/s 走完 ≥1.3 s，1 s 窗口才测得出来；
        # 再短就落到 set_pose 0.1 s 步进的量化误差里。
        if L < 0.40:
            errs.append(f"动态箱移动线段 {L:.2f} m < 0.40 m（1 s 窗口测不出速度）")
        summary = f"静态箱 {len(cfg['static'])} 块 + 动态箱 1 块；动态 {dyn.get('mode')} {d_start}->{d_end}（{L:.2f} m）@ {dyn['speed']} m/s"
    elif variant == "gap":
        gate = cfg["gate"]
        for ob in gate["boxes"]:
            names.append(ob["name"])
            _check_box(text, ob["name"], ob["center"], ob["size"], "static", errs)
            for p, tag in ((ob["center"], "center"),):
                if abs(p[0]) > 6.0 - 0.3 or abs(p[1]) > 4.0 - 0.3:
                    errs.append(f"{ob['name']} {tag}={p} 太贴外墙")
        # 门洞 = 两块箱子内侧面的间距；外侧余量必须小于机器人最小可通行宽度（车宽 0.44 m），
        # 否则机器人会从门旁边绕过去，"窄通道"就没测到。
        ws, es = gate["boxes"]
        west_inner = ws["center"][0] + ws["size"][0] / 2.0
        east_inner = es["center"][0] - es["size"][0] / 2.0
        gap = east_inner - west_inner
        # 走廊内侧面（A 块东面 -1.6 / B 块西面 1.6 → 可走 x∈[-1.4,1.4]）
        outer_w = west_inner - ws["size"][0] - (-1.4)
        outer_e = 1.4 - (es["center"][0] + es["size"][0] / 2.0)
        for tag, v in (("west", outer_w), ("east", outer_e)):
            if v >= 0.42:
                errs.append(f"门{tag}侧余量 {v:.2f} m ≥ 车宽 0.44 m：机器人会绕过去，"
                            f"窄通道失效")
        if not (0.60 <= gap <= 1.20):
            errs.append(f"门洞 {gap:.2f} m 不在 0.60~1.20 m 的窄通道范围")
        if gate["goal"][1] <= es["center"][1]:
            errs.append("目标点必须在门北侧")
        summary = (f"窄通道：门洞 {gap:.2f} m（车宽 0.44，两侧余量 "
                   f"{outer_w:.2f}/{outer_e:.2f} m），目标 {gate['goal']}；"
                   f"5 cm 栅格量化 ±0.025 m")
    n = len(re.findall(r'<model name="(?:avoid_|dynamic_|popup_|gate_)', text))
    if n != len(names):
        errs.append(f"生成的障碍物模型数 {n} != 配置 {len(names)}")
    if errs:
        print(f"!! {variant} 校核失败:")
        for e in errs:
            print("   -", e)
        return 1
    print(f"校核通过[{variant}]: {summary}")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--only", choices=sorted(OUT), default=None)
    args = ap.parse_args()
    variants = [args.only] if args.only else ["obstacles", "braking", "gap"]
    rc = 0
    for v in variants:
        if not args.check:
            generate(v)
        rc |= check(v)
    return rc


if __name__ == "__main__":
    sys.exit(main())