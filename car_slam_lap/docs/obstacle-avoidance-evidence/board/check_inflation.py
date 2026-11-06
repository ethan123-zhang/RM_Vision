"""用 planning_probe 的真实格子验证：不同膨胀半径下是否存在左侧绕行路径。

自包含实现（不 import car_bridge，避免与正在修改的代码竞争）。
只做只读计算，不连板端、不发布速度。
"""
import heapq
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).parent
RES = 0.1
SIZE = 81


def inflate(known, occupied, origin, radius):
    """与 GridView 同样的方向：blocked[i,j] |= occupied[i-dx, j-dy]，圆盘半径 radius。"""
    blocked = occupied.copy()
    n = int(math.ceil(radius / RES))
    for dx in range(-n, n + 1):
        for dy in range(-n, n + 1):
            if math.hypot(dx, dy) * RES > radius:
                continue
            sx = slice(max(0, -dx), min(SIZE, SIZE - dx))
            sy = slice(max(0, -dy), min(SIZE, SIZE - dy))
            tx = slice(max(0, dx), min(SIZE, SIZE + dx))
            ty = slice(max(0, dy), min(SIZE, SIZE + dy))
            blocked[tx, ty] |= occupied[sx, sy]
    return blocked & known | (blocked & ~known)  # 允许 blocked 落在未知格（与生产代码一致）


def astar(known, blocked, start, goal, budget_s=2.0):
    a, b = start, goal
    if not (0 <= a[0] < SIZE and 0 <= a[1] < SIZE) or not (0 <= b[0] < SIZE and 0 <= b[1] < SIZE):
        return None, "out of view"
    if blocked[a]:
        return None, "start blocked"
    if blocked[b]:
        return None, "goal blocked"
    if not known[b]:
        return None, "goal unknown"
    kn = known.copy()
    kn[a] = True
    queue = [(math.dist(a, b), 0.0, a)]
    costs = {a: 0.0}
    parents = {}
    deadline = __import__("time").perf_counter() + budget_s
    expanded = 0
    while queue:
        _, cost, cur = heapq.heappop(queue)
        if cost > costs[cur]:
            continue
        if cur == b:
            cells = [b]
            while cells[-1] != a:
                cells.append(parents[cells[-1]])
            return list(reversed(cells)), "ok"
        expanded += 1
        if expanded % 64 == 0 and __import__("time").perf_counter() > deadline:
            return None, "budget"
        x, y = cur
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nx, ny = x + dx, y + dy
            if not (0 <= nx < SIZE and 0 <= ny < SIZE):
                continue
            if blocked[nx, ny] or not kn[nx, ny]:
                continue
            if dx and dy and (blocked[nx, y] or blocked[x, ny] or not kn[nx, y] or not kn[x, ny]):
                continue
            nc = cost + math.hypot(dx, dy)
            if nc >= costs.get((nx, ny), float("inf")):
                continue
            costs[nx, ny] = nc
            parents[nx, ny] = cur
            heapq.heappush(queue, (nc + math.hypot(nx - b[0], ny - b[1]), nc, (nx, ny)))
    return None, "unreachable"


def main():
    z = np.load(HERE / "planning_probe.npz", allow_pickle=False)
    known_ref, occupied_ref, origin = z["known"], z["blocked"], z["origin"]
    pose, goals = z["pose"], z["goals"]

    # 用 occupied 重建（occupied 未在 npz 中单独给出时，用 blocked 的原始格反推不可行，
    # 因此这里直接用 npz 的 occupied 米制格中心重画）
    occupied = np.zeros((SIZE, SIZE), dtype=bool)
    for p in z["occupied"]:
        ix = int(math.floor(p[0] / RES)) - origin[0]
        iy = int(math.floor(p[1] / RES)) - origin[1]
        if 0 <= ix < SIZE and 0 <= iy < SIZE:
            occupied[ix, iy] = True
    known = known_ref.copy()

    def cell(p):
        return (int(math.floor(p[0] / RES)) - origin[0], int(math.floor(p[1] / RES)) - origin[1])

    start = cell(pose[:2])
    print("pose=%s start cell=%s  occupied=%d known=%d"
          % (np.round(pose, 3), start, occupied.sum(), known.sum()))
    print()
    print("膨胀半径  goal(0.47)  goal(0.91)  goal(1.36)  goal(1.78)   路径最大|y-0.03|")
    for radius in (0.375, 0.526, 0.667, 0.767):
        blocked = inflate(known, occupied, origin, radius)
        row = []
        maxlat = None
        for g in goals:
            path, why = astar(known, blocked, start, cell(g[:2]))
            row.append("OK" if path else why[:10])
            if path and maxlat is None:
                maxlat = max(abs((c[1] + origin[1] + .5) * RES - pose[1]) for c in path)
        print("  %.3f    " % radius + "  ".join(f"{r:>10s}" for r in row)
              + ("      %.2f m" % maxlat if maxlat else "      -"))
    print()
    print("参考：箱体左缘 y=0.15，左墙 y=1.38 → 左侧空隙 1.23 m；右侧空隙 0.67 m")


if __name__ == "__main__":
    main()