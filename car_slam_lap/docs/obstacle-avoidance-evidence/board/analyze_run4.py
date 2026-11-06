"""离线复算 run4 局部网格：为什么 plan() 返回 None。

只读本地 npz，不连板端、不发布任何运动指令。
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(r"E:\RM\car_slam_lap\x5\car_bridge")))
from car_bridge.obstacle_grid import LocalGrid  # noqa: E402

HERE = Path(__file__).parent
RES = 0.1


def ascii_map(known, blocked, planning, origin, center, marks=(), span=25):
    """以 center 为中心画 span x span 格；#=碰撞阻挡 + 未知, .=观测空闲, ?=未知且不阻挡"""
    cx = int(math.floor(center[0] / RES)) - origin[0]
    cy = int(math.floor(center[1] / RES)) - origin[1]
    rows = []
    for dy in range(span, -span - 1, -1):
        line = []
        for dx in range(-span, span + 1):
            x, y = cx + dx, cy + dy
            if not (0 <= x < known.shape[0] and 0 <= y < known.shape[1]):
                line.append(" ")
                continue
            ch = "?"
            if planning[x, y]:
                ch = "#"
            elif known[x, y]:
                ch = "."
            for (mk, idx) in marks:
                if (int(math.floor(mk[0] / RES)) - origin[0], int(math.floor(mk[1] / RES)) - origin[1]) == (x, y):
                    ch = idx
            if (x, y) == (cx, cy):
                ch = "R"
            line.append(ch)
        rows.append("".join(line))
    return "\n".join(rows)


def plan_like_view(known, planning_blocked, start, goal, budget_s=0.035, allow_partial=True):
    """复刻 GridView.plan 的 8 邻接 A*，返回 (reached, best_m_from_start, best_xy, expanded)。"""
    import heapq
    import time

    size = known.shape[0]
    origin = None
    return None  # 不使用，见 plan_cells


def plan_cells(cells_known, cells_planning, start_cell, goal_cell, budget_s=0.035, allow_partial=True):
    import heapq
    import time

    size = cells_known.shape[0]
    a, b = start_cell, goal_cell
    known = cells_known.copy()
    if not (0 <= a[0] < size and 0 <= a[1] < size):
        return None
    known[a] = True
    queue = [(math.dist(a, b), 0.0, a)]
    costs = {a: 0.0}
    parents = {}
    deadline = time.perf_counter() + budget_s
    expanded = 0
    best, best_distance = a, math.dist(a, b)
    reached = False
    while queue:
        _, cost, current = heapq.heappop(queue)
        if cost > costs[current]:
            continue
        distance = math.dist(current, b)
        if distance < best_distance:
            best, best_distance = current, distance
        if current == b:
            best, reached = b, True
            break
        expanded += 1
        if expanded % 32 == 0 and time.perf_counter() > deadline:
            break
        x, y = current
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nx, ny = x + dx, y + dy
            if not (0 <= nx < size and 0 <= ny < size):
                continue
            if cells_planning[nx, ny] or not known[nx, ny]:
                continue
            if dx and dy and (cells_planning[nx, y] or cells_planning[x, ny]
                              or not known[nx, y] or not known[x, ny]):
                continue
            nc = cost + math.hypot(dx, dy)
            if nc >= costs.get((nx, ny), float("inf")):
                continue
            costs[nx, ny] = nc
            parents[nx, ny] = current
            heapq.heappush(queue, (nc + math.hypot(nx - b[0], ny - b[1]), nc, (nx, ny)))
    return dict(reached=reached, best=best, best_distance_m=best_distance,
                expanded=expanded, reached_budget=math.dist(a, best) * RES)


def cell_of(point, origin):
    return (int(math.floor(point[0] / RES)) - origin[0], int(math.floor(point[1] / RES)) - origin[1])


def report_run4():
    z = np.load(HERE / "live_run4_actual.yaml.obstacles.npz", allow_pickle=False)
    known, blocked, planning = z["known"], z["blocked"], z["planning_blocked"]
    origin = z["origin"]
    pose = z["pose"]
    cand = z["candidate_xy"]
    print("=" * 72)
    print("run4 快照：pose=(%.3f, %.3f, %.3f) origin=%s" % (pose[0], pose[1], pose[2], origin))
    print("known=%d blocked=%d planning=%d (violations blocked&~known=%d)"
          % (known.sum(), blocked.sum(), planning.sum(), (blocked & ~known).sum()))
    print("候选接回点 n=%d: %s" % (len(cand), np.round(cand, 3).tolist()))
    sc = cell_of(pose[:2], origin)
    print("start cell=%s in_bounds=%s known=%s blocked=%s planning=%s"
          % (sc, all(0 <= s < 81 for s in sc), known[sc[0], sc[1]],
             blocked[sc[0], sc[1]], planning[sc[0], sc[1]]))
    marks = [((c[0], c[1]), "T") for c in cand]
    print(ascii_map(known, blocked, planning, origin, pose[:2], marks=marks))
    for c in cand:
        gc = cell_of(c[:2], origin)
        inb = all(0 <= s < 81 for s in gc)
        print("goal %s cell=%s in_bounds=%s known=%s blocked=%s planning=%s  dist=%.2f m"
              % (np.round(c[:2], 3), gc, inb,
                 known[gc[0], gc[1]] if inb else "-", blocked[gc[0], gc[1]] if inb else "-",
                 planning[gc[0], gc[1]] if inb else "-", math.dist(pose[:2], c[:2])))
        if inb:
            r = plan_cells(known, planning, sc, gc)
            print("   A*(budget .035): reached=%s expanded=%d best=%s dist_start_to_best=%.2f m -> %s"
                  % (r["reached"], r["expanded"], r["best"], r["reached_budget"],
                     "path" if r["reached"] else ("None" if r["reached_budget"] < .35 else "partial")))


def report_replay():
    z = np.load(HERE / "sources_before_clear.npz", allow_pickle=False)
    params = __import__("json").loads(str(z["params_json"]))
    center = z["center_pose"]
    yaw = float(center[2])
    radius = math.hypot(max(0.325, 0.65 - 0.325), 0.55 / 2) + params["obstacle_margin_m"]
    print("=" * 72)
    print("单帧重放：center=(%.3f, %.3f) yaw=%.3f radius=%.3f" % (center[0], center[1], yaw, radius))
    grid = LocalGrid(RES, radius, 4.0)
    origin_xy = z["odom_lidar"][:2, 3]
    grid.observe(z["obstacles_xy"], origin_xy, center[:2], 0.0,
                 clear_occupied=False, min_points=params["min_points_per_cell"])
    grid.mark_footprint(center[:2], yaw, params["robot_length_m"], params["robot_width_m"],
                        params["robot_front_m"])
    view = grid.view(center[:2], 0.0)
    print("单帧已知格=%d 阻挡格=%d planning=%d (violations=%d)"
          % (view.known.sum(), view.blocked.sum(), view.planning_blocked.sum(),
             (view.blocked & ~view.known).sum()))
    occ = np.asarray([((k[0] + .5) * RES, (k[1] + .5) * RES) for k, hit in grid.cells.items() if hit])
    near = np.linalg.norm(occ - center[:2], axis=1) if len(occ) else np.array([])
    if len(near):
        print("最近障碍格距离=%.2f m；0.60 m 内障碍格=%d；0.30 m 内=%d"
              % (near.min(), (near < .6).sum(), (near < .3).sum()))
    print(ascii_map(view.known, view.blocked, view.planning_blocked, view.origin, center[:2]))


if __name__ == "__main__":
    report_run4()
    report_replay()