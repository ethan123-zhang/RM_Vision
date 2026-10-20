"""轨迹文件的读写 + 几何小工具。

**不依赖 ROS**，所以能在 PC 上直接跑测试（见 test/test_path_io.py）。
record_path 写、follow_path 读，两边共用这里的实现，避免"写的格式和读的格式
悄悄不一致"这种最难查的 bug。
"""

from __future__ import annotations

import math
import os
from typing import Iterable, List, Optional, Tuple

import yaml

#: (x, y, yaw, t)
Point = Tuple[float, float, float, float]


# ---------------------------------------------------------------- 几何
def wrap_pi(angle: float) -> float:
    """把角度折到 (-pi, pi]。"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """四元数 -> 偏航角（弧度）。"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def path_length(points: Iterable[Point]) -> float:
    """折线总长（米）。"""
    pts = list(points)
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))


# ---------------------------------------------------------------- 写
def write_path(path: str, frame_id: str, points: Iterable[Point],
               note: str = "") -> None:
    """原子写入轨迹文件（先写 .tmp 再 rename，Ctrl-C 杀进程不会留半个文件）。"""
    pts = list(points)
    if not pts:
        raise ValueError("空轨迹不写文件")

    lines = [
        "# car_slam_lap 轨迹（record_path.py 生成，follow_path.py 读取）",
    ]
    if note:
        lines.append(f"# {note}")
    lines += [
        f"# 点数 {len(pts)}  长度 {path_length(pts):.2f} m  时长 {pts[-1][3]:.1f} s",
        f"frame_id: {frame_id}",
        "points:",
    ]
    for x, y, yaw, t in pts:
        lines.append(f"  - {{x: {x:.4f}, y: {y:.4f}, yaw: {yaw:.4f}, t: {t:.3f}}}")

    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")
    os.replace(tmp, path)


# ---------------------------------------------------------------- 读
def load_path(path: str) -> Tuple[str, List[Point]]:
    """读轨迹文件，返回 (frame_id, points)。

    格式不对时抛 ValueError 并说清哪儿不对——这比后面跑出诡异轨迹再回来查便宜。
    """
    with open(path, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)

    if not isinstance(data, dict):
        raise ValueError(f"{path} 不是合法的轨迹文件（顶层应为映射）")
    frame = str(data.get("frame_id", "map"))
    raw = data.get("points")
    if not isinstance(raw, list) or not raw:
        raise ValueError(f"{path} 缺少 points 列表")

    pts: List[Point] = []
    for i, item in enumerate(raw):
        if not isinstance(item, dict) or "x" not in item or "y" not in item:
            raise ValueError(f"{path} 第 {i} 个点缺少 x/y：{item!r}")
        pts.append((float(item["x"]), float(item["y"]),
                    float(item.get("yaw", 0.0)), float(item.get("t", 0.0))))
    return frame, pts
