#!/usr/bin/env python3
"""生成本项目的 Nav2 参数（config/nav2_params_rm.yaml = 官方参数 + AMCL 针对性调整）。

为什么要改 AMCL：官方 `nav2_params.yaml` 的 amcl 段有三处在这个仿真里明显不对：
  1. `laser_max_range: 100.0`，而 Gazebo 里 TB3 雷达量程只有 3.5 m →
     打到最大量程的光束被当成"3.5 m 处真有障碍"参与打分，开阔区里和地图预测冲突 → 长期偏置；
  2. `recovery_alpha_slow/fast: 0.0` → **恢复机制关闭**，一旦跟丢/被设错初始位姿就永远回不来；
  3. `laser_model_type: likelihood_field` + `do_beamskip: false` → **对"地图里没有的障碍物"零容错**：
     未建图障碍物（临时摆放的箱子、对手车）的回波会被当成"地图自由区上的测量"，每束只给底分；
     窄通道里这类回波能占到 in-range 束的 70~90%，真值位姿的似然被压到贴墙/墙角位姿之下，
     估计被拉到错误分支（实测钻 0.8 m 门洞后漂 2.9 m，AMCL 不自愈）。
     改成 `likelihood_field_prob` + `do_beamskip: true`：把"所有粒子都解释不了"的束丢掉，
     实测同场景不再跳变（最大误差 0.39 m）。
本脚本改这几处（以及更新频率、粒子数），其余参数与官方保持一致。
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = "/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml"
OUT = os.path.join(HERE, "config", "nav2_params_rm.yaml")

AMCL_NEW = """    laser_max_range: 3.4
    laser_min_range: 0.12
    laser_likelihood_max_dist: 1.5
    laser_model_type: "likelihood_field_prob"
    do_beamskip: true
    max_beams: 180
    min_particles: 800
    max_particles: 3000
    update_min_d: 0.10
    update_min_a: 0.10
    recovery_alpha_slow: 0.001
    recovery_alpha_fast: 0.10
    resample_interval: 1
    alpha1: 0.10
    alpha2: 0.10
    alpha3: 0.10
    alpha4: 0.05
    alpha5: 0.05
"""
# 只替换 amcl 段里这几行的值（其余保持官方）
OVERRIDES = {
    "laser_max_range": "3.4",
    "laser_min_range": "0.12",
    "laser_likelihood_max_dist": "1.5",
    "laser_model_type": '"likelihood_field_prob"',
    "do_beamskip": "true",
    "max_beams": "180",
    "min_particles": "800",
    "max_particles": "3000",
    "update_min_d": "0.10",
    "update_min_a": "0.10",
    "recovery_alpha_slow": "0.001",
    "recovery_alpha_fast": "0.10",
    "resample_interval": "1",
    "alpha1": "0.10",
    "alpha2": "0.10",
    "alpha3": "0.10",
    "alpha4": "0.05",
    "alpha5": "0.05",
}


def main():
    with open(SRC, "r", encoding="utf-8") as f:
        lines = f.readlines()
    out = []
    in_amcl = False
    changed = set()
    for line in lines:
        if re.match(r"^amcl:\s*$", line):
            in_amcl = True
            out.append(line)
            continue
        if in_amcl and re.match(r"^[A-Za-z_]", line):
            in_amcl = False
        m = re.match(r"^(\s+)([a-z0-9_]+):\s*(.*)$", line)
        if in_amcl and m and m.group(2) in OVERRIDES:
            key = m.group(2)
            out.append(f"{m.group(1)}{key}: {OVERRIDES[key]}\n")
            changed.add(key)
            continue
        out.append(line)
    missing = set(OVERRIDES) - changed
    if missing:
        print(f"!! 没找到这些 amcl 参数，未改: {sorted(missing)}")
        return 1
    with open(OUT, "w", encoding="utf-8") as f:
        f.writelines(out)
    print(f"生成: {OUT}（改了 {len(changed)} 个 amcl 参数）")
    for k in sorted(changed):
        print(f"  amcl.{k} = {OVERRIDES[k]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())