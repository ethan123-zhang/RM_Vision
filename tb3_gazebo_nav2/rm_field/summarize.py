#!/usr/bin/env python3
"""从 report_field_test.json 抽取摘要，方便肉眼核对。"""
import json
import os

ROOT = os.environ.get("FIELD_ROOT", "/mnt/e/RM/tb3_gazebo_nav2/rm_field")
p = os.path.join(ROOT, "results", "report_field_test.json")
r = json.load(open(p))
ev = r["evaluation"]
print("field_expected_m:", r["field_expected_m"])
print("\n== 台面平整度 ==")
print(json.dumps(ev.get("flat"), indent=2))
for k in ("edge_x", "edge_y"):
    e = ev.get(k)
    if not e:
        continue
    print(f"\n== {k} ==")
    for kk in ("limit_expected", "on_field_last", "off_field_first",
               "measured_boundary", "on_field_surface_z", "off_field_surface_z",
               "drop_height_measured"):
        print(f"  {kk:22s} {e.get(kk)}")
    print("  samples:")
    for s in e["samples"]:
        print(f"    at={s['at']:<6.2f} surface_z={s['surface_z']:<8} final={s['final']}")
print("\n== robot ==")
print(json.dumps(r["robot"], indent=2))