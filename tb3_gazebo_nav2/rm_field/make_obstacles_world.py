#!/usr/bin/env python3
"""从 config/avoid_obstacles.json 生成两个验收用世界（xacro）：

  A. worlds/rmul2027_3v3_up_nav2_obstacles.sdf.xacro —— 绕障验收用
     静态箱（static）+ 动态箱（kinematic，triggered_retreat：停在走道正中，机器人靠近后让开）
  B. worlds/rmul2027_3v3_up_nav2_braking.sdf.xacro —— 安全层（collision_monitor）刹车验收用
     只有一个"突然出现"的宽挡板（kinematic，先藏在角落，机器人进到 0.7 m 内瞬移到正前方）
  C. worlds/rmul2027_3v3_up_nav2_gap.sdf.xacro —— 窄通道（贴边）验收用
     两块 0.6×0.5 m 小箱，门洞 0.80 m（5 cm 栅格下能过）
  D. worlds/rmul2027_3v3_up_nav2_gap_deep.sdf.xacro —— 窄通道 + 定位压力回归用
     门洞同样 0.80 m，但箱子放大到 1.0×1.2 m（≈GUI 默认箱量级）：AMCL 在未建图大障碍物之间
     会漂（修复前实测 2.9 m），用来回归 likelihood_field_prob + do_beamskip

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
# 2.5 cm 碰撞栅格变体的基世界（由 make_25mm_variant.sh 生成；模型名/地图都带 _25mm 后缀，
# 不覆盖 5 cm 基线）
BASE_25MM = os.path.join(HERE, "worlds", "rmul2027_3v3_up_nav2_25mm.sdf.xacro")
OUT = {
    "obstacles": os.path.join(HERE, "worlds", "rmul2027_3v3_up_nav2_obstacles.sdf.xacro"),
    "braking": os.path.join(HERE, "worlds", "rmul2027_3v3_up_nav2_braking.sdf.xacro"),
    "gap": os.path.join(HERE, "worlds", "rmul2027_3v3_up_nav2_gap.sdf.xacro"),
    "gate_deep": os.path.join(HERE, "worlds", "rmul2027_3v3_up_nav2_gap_deep.sdf.xacro"),
    "gate_tight": os.path.join(HERE, "worlds", "rmul2027_3v3_up_nav2_gate_tight.sdf.xacro"),
    "gate_tight_25mm": os.path.join(HERE, "worlds",
                                    "rmul2027_3v3_up_nav2_gate_tight_25mm.sdf.xacro"),
}
BASE_OF = {"gate_tight_25mm": BASE_25MM}
MARK_BEGIN = "    <!-- AVOID_OBSTACLES_BEGIN（由 make_obstacles_world.py 从 config/avoid_obstacles.json 生成，勿手改）-->"
MARK_END = "    <!-- AVOID_OBSTACLES_END -->"
ANCHOR = '    <plugin filename="gz-sim-physics-system"'


def load_cfg(path=CFG):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def resolve_tier(dyn, name=None):
    """档位 → 实际几何（{name, speed, start, end}）；与 dynamic_obstacle.py 同一规则。"""
    tiers = dyn.get("tiers") or {}
    if not tiers:
        return {"name": "legacy", "speed": dyn["speed"],
                "start": list(dyn["start"]), "end": list(dyn["end"])}
    name = name or dyn.get("default_tier") or sorted(tiers)[0]
    if name not in tiers:
        raise SystemExit(f"未知档位 {name!r}；可选: {', '.join(sorted(tiers))}")
    t = tiers[name]
    return {"name": name, "speed": float(t["speed"]),
            "start": list(t["start"]), "end": list(t["end"])}


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


def build_text(cfg, variant, tier_name=None):
    blocks = [MARK_BEGIN]
    if variant == "obstacles":
        for ob in cfg["static"]:
            size = ob["size"]
            blocks.append(box_model(ob["name"], ob["center"], size, size[2] / 2.0,
                                    ob.get("color", [0.90, 0.45, 0.05]), "static"))
        dyn = cfg["dynamic"]
        tier = resolve_tier(dyn, tier_name)
        size = dyn["size"]
        blocks.append(box_model(dyn["name"], tier["start"], size,
                                dyn.get("z_center", size[2] / 2.0),
                                dyn.get("color", [0.10, 0.55, 0.90]), "kinematic"))
    elif variant == "braking":
        pop = cfg["popup"]
        size = pop["size"]
        blocks.append(box_model(pop["name"], pop["hidden"], size,
                                pop.get("z_center", size[2] / 2.0),
                                pop.get("color", [0.85, 0.25, 0.20]), "kinematic"))
    else:      # gap / gate_deep / gate_tight(_25mm)：两道墙之间留一个刚好能过的门
        key = {"gap": "gate", "gate_deep": "gate_deep",
               "gate_tight": "gate_tight", "gate_tight_25mm": "gate_tight"}[variant]
        for ob in cfg[key]["boxes"]:
            size = ob["size"]
            blocks.append(box_model(ob["name"], ob["center"], size, size[2] / 2.0,
                                    ob.get("color", [0.30, 0.75, 0.30]), "static"))
    blocks.append(MARK_END)
    return "\n".join(blocks)


def generate(variant, tier_name=None):
    base_path = BASE_OF.get(variant, BASE)
    if not os.path.exists(base_path):
        raise SystemExit(f"缺基世界: {os.path.relpath(base_path, HERE)}"
                         f"（2.5 cm 变体先跑 bash make_25mm_variant.sh）")
    with open(base_path, "r", encoding="utf-8") as f:
        base = f.read()
    cfg = load_cfg()
    base = re.sub(re.escape(MARK_BEGIN) + r".*?" + re.escape(MARK_END) + r"\n?", "",
                  base, flags=re.S)
    if ANCHOR not in base:
        raise SystemExit(f"基世界缺少锚点: {ANCHOR}")
    out = base.replace(ANCHOR, build_text(cfg, variant, tier_name) + "\n" + ANCHOR, 1)
    with open(OUT[variant], "w", encoding="utf-8") as f:
        f.write(out)
    extra = ""
    if variant == "obstacles":
        extra = f"（动态箱档位 {resolve_tier(cfg['dynamic'], tier_name)['name']}）"
    print(f"生成: {os.path.relpath(OUT[variant], HERE)}{extra}")


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


def check(variant, tier_name=None):
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
        tier = resolve_tier(dyn, tier_name)
        names.append(dyn["name"])
        d_start = tier["start"]
        d_end = tier["end"]
        _check_box(text, dyn["name"], d_start, dyn["size"], "kinematic", errs)
        L = math.hypot(d_end[0] - d_start[0], d_end[1] - d_start[1])
        for p, tag in ((d_start, "start"), (d_end, "end")):
            if abs(p[0]) > 6.0 - 0.3 or abs(p[1]) > 4.0 - 0.3:
                errs.append(f"动态箱 {tag}={p} 太贴外墙")
        # 线段下限：验收用 1 s 窗口独立测速，运动段必须 ≥1.2 s，否则窗口会跨进停车段。
        # 0.40 m 是底线（0.3 m/s 档位 0.59 m 的 1.97 s 也满足）。
        if L < max(0.40, tier["speed"] * 1.2):
            errs.append(f"档位 {tier['name']}: 线段 {L:.2f} m 太短"
                        f"（{tier['speed']} m/s 需 ≥{max(0.40, tier['speed'] * 1.2):.2f} m 才有 1.2 s 运动段）")
        # 其它档位也一起校核：生成的是其中一个档位，但配置里每个档位都会被验收跑到
        for tname, t in (dyn.get("tiers") or {}).items():
            tl = math.hypot(t["end"][0] - t["start"][0], t["end"][1] - t["start"][1])
            if t["speed"] <= 0:
                errs.append(f"档位 {tname}: speed={t['speed']} 必须 >0")
            if tl < max(0.40, t["speed"] * 1.2):
                errs.append(f"档位 {tname}: 线段 {tl:.2f} m 太短"
                            f"（{t['speed']} m/s 需 ≥{max(0.40, t['speed'] * 1.2):.2f} m）")
            for p, tag in ((t["start"], "start"), (t["end"], "end")):
                if abs(p[0]) > 6.0 - 0.3 or abs(p[1]) > 4.0 - 0.3:
                    errs.append(f"档位 {tname} {tag}={p} 太贴外墙")
        summary = (f"静态箱 {len(cfg['static'])} 块 + 动态箱 1 块（档位 {tier['name']}，"
                   f"共 {len(dyn.get('tiers') or {}) or 1} 档）；{tier['speed']} m/s "
                   f"{d_start}->{d_end}（{L:.2f} m）")
    elif variant == "braking":
        # 刹车场景只有一个"先藏后弹"的挡板：这里校核它的存在性/尺寸/kinematic 标记，
        # 以及"真能挡死走廊"（宽度 ≥2.6 m），否则 collision_monitor 的前提不成立。
        pop = cfg["popup"]
        names.append(pop["name"])
        _check_box(text, pop["name"], pop["hidden"], pop["size"], "kinematic", errs)
        for p, tag in ((pop["hidden"], "hidden"), (pop["appear"], "appear")):
            if abs(p[0]) > 6.0 - 0.3 or abs(p[1]) > 4.0 - 0.3:
                errs.append(f"挡板 {tag}={p} 太贴外墙")
        width = pop["size"][0]
        if width < 2.6:
            errs.append(f"挡板宽 {width:.2f} m < 2.6 m：挡不住 3.2 m 宽的走廊")
        if abs(pop["appear"][0]) > 0.1:
            errs.append(f"挡板弹出点 x={pop['appear'][0]:.2f} 不在中线 x=0 附近")
        summary = (f"刹车挡板 {width:.2f}×{pop['size'][1]:.2f}×{pop['size'][2]:.2f} m，"
                   f"藏在 {pop['hidden']} → 弹到 {pop['appear']}（kinematic）")
    elif variant in ("gap", "gate_deep", "gate_tight", "gate_tight_25mm"):
        key = {"gap": "gate", "gate_deep": "gate_deep",
               "gate_tight": "gate_tight", "gate_tight_25mm": "gate_tight"}[variant]
        gate = cfg[key]
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
        if key == "gate_tight":
            # <0.70 m 档：0.62 m（车宽 0.44 + 两侧各 0.09 m）。太宽就不叫"超窄门"，
            # 太窄（<0.55）连 nav2 的 inflation 都不给过，测出来的只是规划器拒绝。
            if not (0.55 <= gap <= 0.70):
                errs.append(f"超窄门洞 {gap:.2f} m 不在 0.55~0.70 m 范围")
        elif not (0.60 <= gap <= 1.20):
            errs.append(f"门洞 {gap:.2f} m 不在 0.60~1.20 m 的窄通道范围")
        if gate["goal"][1] <= es["center"][1]:
            errs.append("目标点必须在门北侧")
        if variant == "gate_deep":
            # 定位压力场景要的是"长窄段"：箱子沿行进方向要比 0.5 m 小门深得多，
            # 否则门里待不到一个 AMCL 更新周期，复现不出漂移。
            depth = min(ws["size"][1], es["size"][1])
            if depth < 1.0:
                errs.append(f"gate_deep 箱子深度 {depth:.2f} m < 1.0 m（复现不出 AMCL 漂移）")
        grid = "2.5 cm" if variant == "gate_tight_25mm" else "5 cm"
        quant = "±0.0125 m" if variant == "gate_tight_25mm" else "±0.025 m"
        summary = (f"窄通道：门洞 {gap:.2f} m（车宽 0.44，外侧余量 "
                   f"{outer_w:.2f}/{outer_e:.2f} m），箱子深度 "
                   f"{min(ws['size'][1], es['size'][1]):.2f} m，目标 {gate['goal']}；"
                   f"{grid} 栅格量化 {quant}")
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
    ap.add_argument("--tier", default=None,
                    help="绕障世界用哪个动态箱档位（config.dynamic.tiers 的键；"
                         "默认 default_tier）。其它变体忽略此项")
    args = ap.parse_args()
    variants = [args.only] if args.only else ["obstacles", "braking", "gap", "gate_deep"]
    rc = 0
    for v in variants:
        if not args.check:
            generate(v, args.tier)
        rc |= check(v, args.tier)
    return rc


if __name__ == "__main__":
    sys.exit(main())