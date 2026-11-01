#!/usr/bin/env bash
# 汇总当前仿真状态：世界/模型/机器人位姿/RTF + 两份验收报告的关键数字
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
echo "=== 进程 ==="
pgrep -af 'gz sim' | head -4
echo "=== 世界里的模型 ==="
timeout 3 gz topic -e -t /world/rmul2027_3v3/pose/info 2>/dev/null | grep 'name:' | sort -u | head -10
echo "=== 机器人位姿 ==="
timeout 3 gz topic -e -t /world/rmul2027_3v3/pose/info 2>/dev/null | grep -A4 'name: "waffle"' | head -6
echo "=== 报告文件 ==="
ls -la "$ROOT/results"/*.json
python3 - <<'PY'
import json, os
root = "/mnt/e/RM/tb3_gazebo_nav2/rm_field/results"
for name in ("report_field_test.json", "report_field_test_mesh.json"):
    p = os.path.join(root, name)
    if not os.path.exists(p):
        print(name, "缺失"); continue
    r = json.load(open(p))
    ev = r["evaluation"]
    f = ev["flat"]
    ex, ey = ev["edge_x"], ev["edge_y"]
    rb = r["robot"]
    print(f"\n--- {name} (variant={r.get('variant')}) ---")
    print(f"  台面平整度: n={f['n']} max_dev={f['max_dev_from_0']} m  surface_z={f['mean']}")
    print(f"  X 边界: 实测 {ex['measured_boundary']} (期望 {ex['limit_expected']})  落差 {ex['drop_height_measured']}")
    print(f"  Y 边界: 实测 {ey['measured_boundary']} (期望 {ey['limit_expected']})  落差 {ey['drop_height_measured']}")
    print(f"  机器人静置: z={rb['B1_rest']['z']} (期望 {rb['B1_rest']['expected_z']}) 漂移={rb['B1_rest']['drift_1s_m']}")
    print(f"  直行 3s: {rb['B2_drive']['distance_m']} m (期望 {rb['B2_drive']['expected_m']}), z范围 [{rb['B2_drive']['z_min']},{rb['B2_drive']['z_max']}]")
    b3 = rb["B3_edge"]
    print(f"  冲出边缘: 过边t={b3['cross_boundary_t']} 下坠t={b3['tip_t']} final_z={b3['final']['z']} (期望 {b3['expected_final_z']})")
PY