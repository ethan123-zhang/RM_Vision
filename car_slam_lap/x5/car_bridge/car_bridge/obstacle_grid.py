"""实时二维观测网格与局部绕行几何（无 ROS 依赖）。

所有坐标属于同一次运行的 odom。只清除被新射线观测到的障碍，离开视野
不会清除；规划中心不进入未知格。
用车身外接圆保证转向时也有相同净空。

膨胀只用外接圆半径本身：早先额外叠的 `√2·res` 与"规划再加一格"把
1.23 m 的可行通道算成了 1.53 m 的死路（见 diagnosis-20261105）。
"""
from __future__ import annotations

import heapq
import math
import time

import numpy as np


def _unique_keys(keys, min_count=1):
    """整数线性键去重，避免 np.unique(axis=0) 的结构化字段排序开销。"""
    if not len(keys):
        return keys
    low = keys.min(axis=0)
    span = int(keys[:, 1].max() - low[1]) + 1
    code = (keys[:, 0] - low[0]) * span + keys[:, 1] - low[1]
    _, first, counts = np.unique(code, return_index=True, return_counts=True)
    return keys[first[counts >= min_count]]


class LocalGrid:
    def __init__(self, resolution=.10, radius=.53, half_extent=4.):
        for value in (resolution, radius, half_extent):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("网格分辨率、车身半径和窗口必须是有限正数")
        if half_extent < radius * 2 or resolution > radius:
            raise ValueError("局部窗口太小或网格分辨率过粗")
        self.resolution = resolution
        self.radius = radius
        self.half_extent = half_extent
        self.cells = {}  # (ix, iy) -> occupied；有界窗口，未知格不存在。

    def _cell(self, point):
        return tuple(np.floor(np.asarray(point[:2]) / self.resolution).astype(int))

    def observe(self, points, origin, robot_center, now, clear_occupied=True, min_points=1):
        """端点为障碍；射线经过的格为空闲。先清除再统一标记，近障碍优先。

        输入是已去地面/车身的平面端点；同一方位使用最近端点清除，避免
        高处远墙的射线穿过前方低矮箱体，把刚检测到的箱体清掉。
        """
        pts = np.asarray(points, dtype=float).reshape(-1, 2)
        pts = pts[np.isfinite(pts).all(axis=1)]
        origin = np.asarray(origin[:2], dtype=float)
        center = np.asarray(robot_center[:2], dtype=float)
        keep = self.half_extent + self.radius + self.resolution * 2
        low = np.floor((center - keep) / self.resolution).astype(int)
        high = np.ceil((center + keep) / self.resolution).astype(int)
        lx, ly, hx, hy = int(low[0]), int(low[1]), int(high[0]), int(high[1])
        self.cells = {k: v for k, v in self.cells.items() if lx <= k[0] <= hx and ly <= k[1] <= hy}
        if len(pts) == 0:
            return
        delta = pts - origin
        ranges = np.linalg.norm(delta, axis=1)
        valid = ranges > .05
        pts, delta, ranges = pts[valid], delta[valid], ranges[valid]
        if not len(pts):
            return
        # 720 个方位桶，射线数量与点云点数脱钩。
        angles = np.floor((np.arctan2(delta[:, 1], delta[:, 0]) + math.pi) * 720 / (2 * math.pi)).astype(int)
        order = np.argsort(ranges)
        _, first = np.unique(angles[order], return_index=True)
        chosen = order[first]
        lengths = np.minimum(ranges[chosen], self.half_extent * 1.5)
        d = delta[chosen] * (lengths / ranges[chosen])[:, None]
        steps = np.ceil(lengths / (self.resolution * .45)).astype(int)
        n = int(max(steps)) + 1
        fractions = np.arange(n)[None, :] / steps[:, None]
        mask = fractions < 1.0
        samples = origin + d[:, None, :] * fractions[:, :, None]
        keys = _unique_keys(np.floor(samples[mask] / self.resolution).astype(int))
        for k in keys:
            key = (int(k[0]), int(k[1]))
            if lx <= key[0] <= hx and ly <= key[1] <= hy:
                if clear_occupied:
                    self.cells[key] = False
                else:
                    self.cells.setdefault(key, False)
        hits = _unique_keys(np.floor(pts / self.resolution).astype(int), min_count=min_points)
        for k in hits:
            key = (int(k[0]), int(k[1]))
            if lx <= key[0] <= hx and ly <= key[1] <= hy:
                self.cells[key] = True

    def view(self, center, now):
        return GridView(self, center)

    def mark_footprint(self, center, yaw, length, width, front_extent=None):
        """车体内部已由实体车占据：这里不可能真有障碍，陈旧占据格允许清掉。

        只清车身矩形本身，不外扩到车外盲区；车外占据仍按"离开视野不清除"保留。
        """
        c, s = math.cos(yaw), math.sin(yaw)
        front = length/2 if front_extent is None else front_extent
        rear = length - front
        r = math.hypot(max(front, rear), width/2)
        base = self._cell(center)
        steps = int(math.ceil(r / self.resolution)) + 1
        for ix in range(base[0] - steps, base[0] + steps + 1):
            for iy in range(base[1] - steps, base[1] + steps + 1):
                dx = (ix + .5) * self.resolution - center[0]
                dy = (iy + .5) * self.resolution - center[1]
                if -rear <= c*dx+s*dy <= front and abs(-s*dx+c*dy) <= width/2:
                    self.cells[(ix, iy)] = False


class GridView:
    def __init__(self, grid, center):
        self.resolution = grid.resolution
        self.radius = grid.radius
        self.origin = np.floor((np.asarray(center[:2]) - grid.half_extent) / self.resolution).astype(int)
        self.size = int(math.ceil(2 * grid.half_extent / self.resolution)) + 1
        self.known = np.zeros((self.size, self.size), dtype=bool)
        occupied = np.zeros_like(self.known)
        self.occupied = occupied
        if grid.cells:
            keys = np.asarray(list(grid.cells), dtype=int) - self.origin
            values = np.fromiter(grid.cells.values(), dtype=bool)
            valid = (keys >= 0).all(axis=1) & (keys < self.size).all(axis=1)
            keys = keys[valid]
            self.known[keys[:, 0], keys[:, 1]] = True
            occupied[keys[:, 0], keys[:, 1]] = values[valid]
        # 中心离障碍要留外接圆半径，并多留一格对角量覆盖栅格量化误差。
        self.blocked = self._dilate(occupied, grid.radius + math.sqrt(2) * self.resolution)
        # 路径中心再多留一格：折线格心与跟踪圆弧的连续位置之间还差这一格，
        # 不留会把"能过但贴边"的通道判成不可跟踪。
        self.planning_blocked = self._dilate(
            occupied, grid.radius + (1 + math.sqrt(2)) * self.resolution)

    def _dilate(self, occupied, clearance):
        out = occupied.copy()
        n = int(math.ceil(clearance / self.resolution))
        for dx in range(-n, n + 1):
            for dy in range(-n, n + 1):
                if math.hypot(dx, dy) * self.resolution > clearance:
                    continue
                sx = slice(max(0, -dx), min(self.size, self.size - dx))
                sy = slice(max(0, -dy), min(self.size, self.size - dy))
                tx = slice(max(0, dx), min(self.size, self.size + dx))
                ty = slice(max(0, dy), min(self.size, self.size + dy))
                out[tx, ty] |= occupied[sx, sy]
        return out

    def _cell(self, point):
        return tuple(np.floor(np.asarray(point[:2]) / self.resolution).astype(int) - self.origin)

    def _point(self, cell):
        return tuple((np.asarray(cell) + self.origin + .5) * self.resolution)

    def free(self, point, require_known=True, planning=False):
        x, y = self._cell(point)
        return (0 <= x < self.size and 0 <= y < self.size
                and not (self.planning_blocked if planning else self.blocked)[x, y]
                and (not require_known or self.known[x, y]))

    def segment_free(self, a, b, planning=False):
        d = math.dist(a[:2], b[:2])
        count = max(1, int(math.ceil(d / (self.resolution * .4))))
        return all(self.free((a[0] + (b[0] - a[0]) * i / count,
                              a[1] + (b[1] - a[1]) * i / count), planning=planning) for i in range(count + 1))

    def motion_free(self, pose, v, w, horizon_s=2.):
        """检验实际 Twist 的圆弧扫掠；原地转同样检查外接圆净空。

        当前转心已被机器人占据，允许它尚未被前向雷达射线标空；一旦向前
        行驶，预测位置必须已观测。此处不宣称能检测雷达盲区的新障碍。
        """
        if not all(math.isfinite(t) for t in (*pose, v, w)) or v < 0:
            return False
        if not self.free(pose, require_known=False):
            return False
        x, y, yaw = pose
        steps = max(1, int(math.ceil(abs(v) * horizon_s / (self.resolution * .4))))
        for t in np.linspace(0, horizon_s, steps + 1)[1:]:
            if abs(w) < 1e-6:
                px, py = x + v * t * math.cos(yaw), y + v * t * math.sin(yaw)
            else:
                px = x + v / w * (math.sin(yaw + w * t) - math.sin(yaw))
                py = y - v / w * (math.cos(yaw + w * t) - math.cos(yaw))
            if not self.free((px, py), require_known=abs(v) > 1e-6):
                return False
        return True

    def _direct_path(self, start, goal, max_len=.5):
        """贴身目标（例如回起点最后 0.2 m）不值得走 A*：直接两点路径。

        只要求目标不在碰撞区、整段格都空闲；实际发送的运动仍由 motion_free 逐拍审查。
        """
        if math.dist(start[:2], goal[:2]) > max_len:
            return None
        if not self.segment_free(start, goal, planning=True):
            return None
        return [tuple(start[:2]), tuple(goal[:2])]

    def plan(self, start, goal, budget_s=.06, allow_partial=False):
        """8 邻接 A*；禁止穿角、未知格；有时间预算，失败返回 None。"""
        direct = self._direct_path(start, goal)
        if direct is not None:
            return direct
        a, b = self._cell(start), self._cell(goal)
        # 起点可以落在规划缓冲内，但绝不能在碰撞区；允许从该格向外脱离。
        if not self.free(start, require_known=False) or (not allow_partial and not self.free(goal, require_known=False)):
            return None
        known = self.known.copy()
        known[a] = True
        queue = [(math.dist(a, b), 0., a)]
        costs = {a: 0.}
        parents = {}
        deadline = time.perf_counter() + budget_s
        expanded = 0
        best = a
        best_distance = math.dist(a, b)
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
                if not (0 <= nx < self.size and 0 <= ny < self.size):
                    continue
                if self.planning_blocked[nx, ny] or not known[nx, ny]:
                    continue
                if dx and dy and (self.planning_blocked[nx, y] or self.planning_blocked[x, ny]
                                  or not known[nx, y] or not known[x, ny]):
                    continue
                nc = cost + math.hypot(dx, dy)
                if nc >= costs.get((nx, ny), float("inf")):
                    continue
                costs[nx, ny] = nc
                parents[nx, ny] = current
                heapq.heappush(queue, (nc + math.hypot(nx - b[0], ny - b[1]), nc, (nx, ny)))
        if not reached and (not allow_partial or math.dist(a, best) * self.resolution < .35):
            return None
        cells = [best]
        while cells[-1] != a:
            cells.append(parents[cells[-1]])
        path = [tuple(start[:2])] + [self._point(c) for c in reversed(cells[:-1])]
        if reached:
            path.append(tuple(goal[:2]))
        return path


def follow_detour(path, index, pose, v_max, w_max, view):
    """局部路径前进跟踪；索引局部递增，前视捷径必须无碰撞。"""
    x, y, yaw = pose
    if math.dist((x, y), path[-1]) <= .10:
        return 0., 0., len(path) - 1, True
    while index < len(path) - 1 and math.dist((x, y), path[index]) < .16:
        index += 1
    target = path[index]
    for j in range(index + 1, len(path)):
        if math.dist((x, y), path[j]) > .45:
            break
        if view.segment_free((x, y), path[j], planning=True):
            target = path[j]
            index = j
    heading = (math.atan2(target[1] - y, target[0] - x) - yaw + math.pi) % (2 * math.pi) - math.pi
    w = max(-w_max, min(w_max, 1.5 * heading))
    if abs(heading) > .35:
        return 0., w, index, False
    v = min(v_max, .12, max(.025, math.dist((x, y), path[-1]) * .4),
            math.dist((x, y), target) / 2.5)
    w = max(-v / .43, min(v / .43, w))
    # 预测圆弧碰到折线内侧时先原地修正，不能把 A* 折线当作 Twist 的安全证明。
    for scale in (1., .5, .25):
        if view.motion_free(pose, v * scale, w * scale):
            return v * scale, w * scale, index, False
    return 0., w, index, False