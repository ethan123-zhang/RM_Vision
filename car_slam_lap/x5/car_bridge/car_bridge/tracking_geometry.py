"""路线几何：弧长、局部最近点、沿弧长目标与进度上限（纯 Python，不依赖 ROS）。

follow_path 的安全门和状态机留在节点里；这里只放能脱离 ROS 单测的几何计算。

为什么目标按弧长挑，而不是按欧式距离扫：
    掉头/回程段和去程段在空间上常常只隔十几厘米，但弧长差出整段掉头。
    从去程某点开始沿索引扫"第一个欧式距离 >= 前视距离"的点，可能整段掉头
    都在前视圆内，最后一头扎到回程段的点上（实测路线 162 号点会选到 192
    号点，夹角 167°，车却还在往前开）。按弧长取 s[idx] + lookahead 就不会
    被空间上近、弧长上远的分支吸走。
"""

from __future__ import annotations

import bisect
import math
from typing import List, Sequence, Tuple


def cumulative_arc_length(px: Sequence[float], py: Sequence[float]) -> List[float]:
    """逐点累积弦长；重复点/零长度段让 s 保持不变，绝不倒退。"""
    s = [0.0] * len(px)
    for i in range(1, len(px)):
        s[i] = s[i - 1] + math.hypot(px[i] - px[i - 1], py[i] - py[i - 1])
    return s


def nearest_index(
    s: Sequence[float],
    px: Sequence[float],
    py: Sequence[float],
    x: float,
    y: float,
    current_idx: int,
    *,
    back_m: float,
    forward_m: float,
) -> Tuple[int, float]:
    """只在当前进度附近的弧长窗口里找最近点。

    窗口至少包含当前点和下一个点：稀疏路线（点距 > forward_m）也要能推进，
    否则索引会永远卡在原地。返回值允许小于 current_idx——横向偏差时最近点
    确实可能在身后，由 limit_progress_index 负责让有效进度不回退。
    """
    lo = bisect.bisect_left(s, s[current_idx] - back_m)
    hi = bisect.bisect_right(s, s[current_idx] + forward_m)
    lo = min(lo, current_idx)
    hi = max(hi, current_idx + 2)
    hi = min(hi, len(s))

    best_i, best_d2 = current_idx, float("inf")
    for i in range(lo, hi):
        d2 = (px[i] - x) ** 2 + (py[i] - y) ** 2
        if d2 < best_d2:
            best_i, best_d2 = i, d2
    return best_i, math.sqrt(best_d2)


def target_index_by_arc(
    s: Sequence[float], current_idx: int, lookahead: float
) -> int:
    """沿路线弧长取前视目标：第一个 s >= s[idx] + lookahead 的点；不够就末点。"""
    target_s = s[current_idx] + lookahead
    index = bisect.bisect_left(s, target_s)
    if index >= len(s):
        return len(s) - 1
    return max(current_idx, index)


def limit_progress_index(
    s: Sequence[float],
    current_idx: int,
    candidate_idx: int,
    traveled_m: float,
    slack_m: float,
) -> int:
    """路线进度只增不减，且弧长不超过实际里程 + 余量。

    闭合路线起点与终点重合、掉头两侧空间也贴近，最近点可能一步跳到远处；
    用实际行驶里程给进度封顶，保留 slack_m 吸收横向偏差和拐角。
    """
    if candidate_idx <= current_idx:
        return current_idx
    cap_s = traveled_m + slack_m
    if s[candidate_idx] <= cap_s:
        return candidate_idx
    capped = bisect.bisect_right(s, cap_s) - 1
    return max(current_idx, capped)
