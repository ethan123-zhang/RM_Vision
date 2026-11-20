#!/usr/bin/env python3
"""离线分析：两点之间"只看障碍、忽略 inflation"的最短路，及其沿途最小净空。

用途：分辨"导航绕远"到底是哪种：
  a) 地图里根本没有能过的洞（最短路最小净空 < robot_radius）→ nav2 绕路是对的；
  b) 洞能过（净空 ≥ robot_radius）但 nav2 却绕远 → inflation/cost 参数让规划器嫌弃窄道。

坐标约定 = 地图/世界坐标（x 沿东西、y 沿南北），不是图像行列；像素→世界按 map_server 约定
（图像第 0 行是 y 最大那行，本仓库 PGM 落盘时已 img[::-1]）。

用法：
  python3 analyze_gap_route.py --start 0.0 0.0 --goal 2.99 3.67
  python3 analyze_gap_route.py --start 0.0 0.0 --goal 2.99 3.67 --robot-r 0.22
"""
import argparse
import heapq
import math
import os

import numpy as np
from PIL import Image
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
RES, OX, OY = 0.025, -6.4, -4.4


def load_free(path):
    a = np.array(Image.open(path))
    return a, a < 80                       # 只把"占据"当障碍（unknown/free 都能走）


def world_to_rc(x, y, H):
    """世界坐标 → 图像行列（map_server 约定：第 0 行是 y 最大）。"""
    r = H - 1 - int(round((y - OY) / RES))
    c = int(round((x - OX) / RES))
    return r, c


def rc_to_world(r, c, H):
    return OX + c * RES, OY + (H - 1 - r) * RES


def dijkstra(occ, start_rc, goal_rc):
    """8 邻接 Dijkstra；返回 (路径[(r,c)...], 长度 m)。不可达返回 (None, inf)。"""
    H, W = occ.shape
    INF = float("inf")
    dist = np.full((H, W), INF)
    prev = {}
    sr, sc = start_rc
    gr, gc = goal_rc
    dist[sr, sc] = 0.0
    pq = [(0.0, sr, sc)]
    dirs = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    while pq:
        d, r, c = heapq.heappop(pq)
        if d > dist[r, c] + 1e-12:
            continue
        if (r, c) == (gr, gc):
            path = [(r, c)]
            while path[-1] != (sr, sc):
                path.append(prev[path[-1]])
            return list(reversed(path)), d
        for dr, dc in dirs:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < H and 0 <= nc < W) or occ[nr, nc]:
                continue
            nd = d + math.hypot(dr, dc) * RES
            if nd < dist[nr, nc] - 1e-12:
                dist[nr, nc] = nd
                prev[(nr, nc)] = (r, c)
                heapq.heappush(pq, (nd, nr, nc))
    return None, INF


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default=os.path.join(HERE, "maps", "rmul2027_3v3_up.pgm"))
    ap.add_argument("--start", nargs=2, type=float, required=True)
    ap.add_argument("--goal", nargs=2, type=float, required=True)
    ap.add_argument("--robot-r", type=float, default=0.22)
    ap.add_argument("--clear-min", type=float, default=0.0,
                    help="只允许净空 ≥ 该值的格子（默认 0 = 只看障碍；设 --robot-r 即模拟真实车体）")
    args = ap.parse_args()

    a, occ = load_free(args.map)
    H, W = a.shape
    clear = ndimage.distance_transform_edt(~occ) * RES
    sr, sc = world_to_rc(*args.start, H)
    gr, gc = world_to_rc(*args.goal, H)
    print(f"start ({args.start[0]},{args.start[1]}) 格 ({sr},{sc}) 占={bool(occ[sr,sc])} 净空 {clear[sr,sc]:.2f} m")
    print(f"goal  ({args.goal[0]},{args.goal[1]}) 格 ({gr},{gc}) 占={bool(occ[gr,gc])} 净空 {clear[gr,gc]:.2f} m")
    if occ[sr, sc] or occ[gr, gc]:
        print("!! 起点或终点落在障碍里，无法分析")
        return 1
    walk = occ.copy()
    if args.clear_min > 0:
        walk = walk | (clear < args.clear_min)
    if walk[sr, sc] or walk[gr, gc]:
        print(f"!! 起点或终点净空 < {args.clear_min} m（车体放不下），规划器会失败或绕到最近可达点")
        # 打印附近净空最大处，便于给"点得太靠墙"的用户看
        k = 2
        ys, xs = np.mgrid[max(0, gr - 40):min(H, gr + 41), max(0, gc - 40):min(W, gc + 41)]
        cs = clear[ys, xs]
        ii = np.unravel_index(np.argmax(cs), cs.shape)
        r0, c0 = ys[ii], xs[ii]
        x0, y0 = rc_to_world(r0, c0, H)
        print(f"   目标附近最空的地方: ({x0:.2f},{y0:.2f}) 净空 {clear[r0,c0]:.2f} m")
        return 1
    path, L = dijkstra(walk, (sr, sc), (gr, gc))
    if path is None:
        print("!! 该净空要求下到不了（被障碍完全隔开）")
        return 1
    cls = [clear[r, c] for r, c in path]
    k = int(np.argmin(cls))
    wx, wy = rc_to_world(*path[k], H)
    print(f"最短路：长 {L:.2f} m，{len(path)} 格，沿途最小净空 {min(cls):.2f} m @ ({wx:.2f},{wy:.2f})")
    for fr in (0.0, 0.2, 0.4, 0.6, 0.8, 1.0):
        r, c = path[int(fr * (len(path) - 1))]
        x, y = rc_to_world(r, c, H)
        print(f"  {int(fr*100):3d}% ({x:6.2f},{y:5.2f}) 净空 {clear[r,c]:.2f} m")
    print(f"robot_radius={args.robot_r}："
          f"{'洞能过 → 绕远是规划器参数问题' if min(cls) >= args.robot_r else '洞太窄 → nav2 绕路是对的（物理上过不去）'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())