#!/usr/bin/env python3
"""生成"点地图发目标"用的 RViz 配置（config/rm_field_click.rviz）。

为什么要生成：Nav2 官方 `nav2_default_view.rviz` 里
  - 只有 **Nav2 Goal** 工具（nav2_rviz_plugins/GoalTool，直接发 action），
  - 没有 **2D Goal Pose**（rviz_default_plugins/SetGoal，只往 `/goal_pose` 发 PoseStamped），
  - 而且工具列表第一项是 **MoveCamera**——**RViz 启动后默认激活第一个工具**，
    于是"在地图上点一下"只是平移/旋转视角，机器人毫无反应（2026-11-20 用户实测踩到）。

做法：
  1. 读官方配置，补上 SetGoal 工具；
  2. **把 Nav2 Goal 排到 Tools 第一个**（启动即激活，点地图就是发目标），
     后面依次 2D Goal Pose、2D Pose Estimate、MoveCamera、Select、FocusCamera、Measure、PublishPoint；
  3. 写 config/rm_field_click.rviz 并回读断言（工具齐全、第一个是 GoalTool、Fixed Frame=map）。
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = "/opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz"
OUT = os.path.join(HERE, "config", "rm_field_click.rviz")
SET_GOAL = """    - Class: rviz_default_plugins/SetGoal
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /goal_pose
"""
# 工具顺序：第一个是 RViz 启动后默认激活的工具，所以把"发目标"放最前
PRIORITY = [
    "nav2_rviz_plugins/GoalTool",
    "rviz_default_plugins/SetGoal",
    "rviz_default_plugins/SetInitialPose",
    "rviz_default_plugins/MoveCamera",
    "rviz_default_plugins/Select",
    "rviz_default_plugins/FocusCamera",
    "rviz_default_plugins/Measure",
    "rviz_default_plugins/PublishPoint",
]


def split_tools(text):
    """把 Tools: 段切成 entries（每项含续行），返回 (prefix, entries, suffix)。"""
    start = text.index("  Tools:\n") + len("  Tools:\n")
    end = text.index("  Transformation:\n", start)
    block = text[start:end]
    entries, cur = [], []
    for line in block.splitlines(keepends=True):
        if line.startswith("    - "):
            if cur:
                entries.append("".join(cur))
            cur = [line]
        else:
            cur.append(line)
    if cur:
        entries.append("".join(cur))
    return text[:start], entries, text[end:]


def reorder(entries):
    def rank(entry):
        for i, cls in enumerate(PRIORITY):
            if f"Class: {cls}" in entry:
                return i
        return len(PRIORITY)

    known = [e for e in entries if rank(e) < len(PRIORITY)]
    unknown = [e for e in entries if rank(e) == len(PRIORITY)]
    known.sort(key=rank)
    return known + unknown


def main():
    with open(SRC, "r", encoding="utf-8") as f:
        text = f.read()
    if "rviz_default_plugins/SetGoal" not in text:
        anchor = "    - Class: nav2_rviz_plugins/GoalTool"
        if anchor not in text:
            print(f"!! 官方配置里找不到锚点: {anchor}")
            return 1
        text = text.replace(anchor, SET_GOAL + anchor, 1)
    prefix, entries, suffix = split_tools(text)
    text = prefix + "".join(reorder(entries)) + suffix
    with open(OUT, "w", encoding="utf-8") as f:
        f.write(text)

    # 回读断言
    with open(OUT, "r", encoding="utf-8") as f:
        out = f.read()
    errs = []
    for needle in ("rviz_default_plugins/SetGoal", "nav2_rviz_plugins/GoalTool",
                   "rviz_default_plugins/SetInitialPose", "Value: /goal_pose",
                   "Value: /initialpose"):
        if needle not in out:
            errs.append(f"生成配置里缺 {needle}")
    if "Fixed Frame: map" not in out:
        errs.append("Fixed Frame 不是 map")
    _pre, entries2, _suf = split_tools(out)
    first = entries2[0] if entries2 else ""
    if "nav2_rviz_plugins/GoalTool" not in first:
        errs.append("Tools 第一个不是 Nav2 Goal（RViz 启动后默认激活的工具就不是发目标）")
    if errs:
        print("!! RViz 配置生成失败:")
        for e in errs:
            print("   -", e)
        return 1
    order = [e.split("Class: ")[1].split("\n")[0] for e in entries2 if "Class: " in e]
    print(f"生成: {OUT}")
    print(" 工具顺序（第一个=启动即激活）:", " → ".join(order))
    return 0


if __name__ == "__main__":
    sys.exit(main())