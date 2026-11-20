#!/usr/bin/env python3
"""生成本项目的 Nav2 参数（config/nav2_params_rm.yaml = 官方参数 + AMCL 针对性调整）。

为什么要改 AMCL：官方 `nav2_params.yaml` 的 amcl 段有两处在这个仿真里明显不对：
  1. `laser_max_range: 100.0`，而 Gazebo 里 TB3 雷达量程只有 3.5 m →
     打到最大量程的光束被当成"3.5 m 处真有障碍"参与打分，开阔区里和地图预测冲突 → 长期偏置；
  2. `recovery_alpha_slow/fast: 0.0` → **恢复机制关闭**，一旦跟丢/被设错初始位姿就永远回不来。
本脚本把它们（以及更新频率、粒子数）改成适合本仿真的值，其余参数与官方保持一致。
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