"""死路判定：把 Nav2 规划出的路径投到全局代价地图上，看它离"内切区"有多近。

为什么用"路径净空"而不是"缝宽"：缝宽要靠两侧障碍都量到，实际影响能不能过的是
footprint 的 inscribed 半径。把代价地图里 `>=99`（99=内切 / 100=致命）的格子当障碍，
做距离变换就得到"路径离内切区多远"：

- 开阔场地实测（goal (3,−2.5)）：去掉起点 0.6 m 与终点附近 0.4 m 后，路径净空 **≥0.65 m**
  （目标点本身贴墙时会有 0.2 m 的读数，所以终点附近必须排除）。
- 窄缝实测（敌车 0.4 m 摆在 1.3 m 走廊中间、goal (0,2.6)）：路径中段净空 **0.05–0.08 m**
  —— 路径等于贴着内切边界擦过去，footprint 半径 0.6 的车在这一段就是"过不去"，
  实测 MPPI 会原地磨 200 s（10–11 次 Failed to make progress）而不是走过去。

所以判据是"路径（去掉首尾）最小净空 < 0.20 m"。阈值 0.2 m 对上面两组读数有 3 倍以上裕度。

调用方（点地图窗口 / 验收脚本）自己持有 goal handle，判定后用它去 cancel —— ROS 2 没有
"按条件取消别人目标"的公开 API，所以死路判定只能挂在客户端这一侧。
"""

import numpy as np
from scipy import ndimage

LETHAL_COST = 99          # 发布尺度：100 = 致命, 99 = footprint 内切
DEAD_END_CLEAR_M = 0.20   # 路径最小净空低于它就判"过不去"
SKIP_START_M = 0.6        # 起点附近本来就踩在自身 footprint 的 inflation 里
SKIP_GOAL_M = 0.40        # 目标点可以合法地贴障碍，终点附近不算"过不去"


def clearance_grid(costmap):
    """costmap = (data, origin_x, origin_y, resolution, width, height) -> 净空矩阵（m）。"""
    data, _ox, _oy, res, w, h = costmap
    arr = np.asarray(data, dtype=np.int16).reshape(h, w)
    lethal = arr >= LETHAL_COST
    return ndimage.distance_transform_edt(~lethal) * res


def peek_plan_clearance(path_msg, costmap, skip_start_m=SKIP_START_M,
                        skip_goal_m=SKIP_GOAL_M, grid=None):
    """沿 /plan 量净空。返回 dict：min_clear / worst_xy / n / path_m，或 None。"""
    if costmap is None or path_msg is None or not path_msg.poses:
        return None
    _data, ox, oy, res, w, h = costmap
    grid = clearance_grid(costmap) if grid is None else grid
    pts = [(p.pose.position.x, p.pose.position.y) for p in path_msg.poses]
    gx, gy = pts[-1]
    travelled = 0.0
    min_clear = 1e9
    worst = None
    n = 0
    for i, (x, y) in enumerate(pts):
        if i:
            travelled += ((x - pts[i - 1][0]) ** 2 + (y - pts[i - 1][1]) ** 2) ** 0.5
        if travelled < skip_start_m:
            continue
        if ((x - gx) ** 2 + (y - gy) ** 2) ** 0.5 < skip_goal_m:
            continue
        c = int((x - ox) / res)
        r = int((y - oy) / res)
        if not (0 <= c < w and 0 <= r < h):
            continue
        n += 1
        cl = float(grid[r, c])
        if cl < min_clear:
            min_clear = cl
            worst = (x, y)
    if n == 0 or worst is None:
        return None
    return {"min_clear": min_clear, "worst_xy": worst, "n": n, "path_m": travelled}


def is_dead_end(path_msg, costmap, threshold=DEAD_END_CLEAR_M, **kwargs):
    """返回 (dead_end: bool, info: dict|None)。只判当前一帧，连续命中由调用方计数。"""
    info = peek_plan_clearance(path_msg, costmap, **kwargs)
    if info is None:
        return False, None
    return info["min_clear"] < threshold, info