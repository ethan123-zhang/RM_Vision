#!/usr/bin/env python3
"""S2a step 2: pick 5 patrol points on the Nav2 map with >= 0.45 m clearance.

- clearance field = distance transform of the occupied mask (cell = 0.025 m)
- reachable set   = free cells with clearance >= 0.45 m, 4-connected, containing
                    the spawn cell (0,0)
- points: 1 near spawn + 1 per quadrant, all pairwise Dijkstra-reachable
Writes: ~/sentry_ws/results/patrol_point_analysis.json
        ~/sentry_ws/src/sentry_bringup/config/patrol_points.yaml
"""
import json
import math
import os

import numpy as np
from scipy.ndimage import distance_transform_edt, label
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

ROOT = '/mnt/e/RM/tb3_gazebo_nav2/rm_field'
PGM = os.path.join(ROOT, 'maps', 'rmul2027_3v3_up.pgm')
YAML = os.path.join(ROOT, 'maps', 'rmul2027_3v3_up.yaml')
BOXES = os.path.join(ROOT, 'debug', 'box_collisions.json')
OUT_JSON = os.path.expanduser('~/sentry_ws/results/patrol_point_analysis.json')
OUT_YAML = os.path.expanduser(
    '~/sentry_ws/src/sentry_bringup/config/patrol_points.yaml')

CLEAR_MIN = 0.45
SPAWN = (0.0, 0.0)


def read_pgm(path):
    data = open(path, 'rb').read()
    idx, tok = 0, []
    while len(tok) < 4:
        while data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b'#':
            while data[idx:idx + 1] not in (b'\n', b''):
                idx += 1
            continue
        s = idx
        while not data[idx:idx + 1].isspace():
            idx += 1
        tok.append(data[s:idx])
    w, h = int(tok[1]), int(tok[2])
    idx += 1
    return w, h, np.frombuffer(data[idx:idx + w * h], dtype=np.uint8).reshape(h, w)


def read_yaml(path):
    out = {}
    for line in open(path):
        line = line.split('#')[0].strip()
        if ':' in line:
            k, v = line.split(':', 1)
            out[k.strip()] = v.strip()
    return float(out['resolution']), json.loads(out['origin'])


def main():
    res, origin = read_yaml(YAML)
    ox, oy = origin[0], origin[1]
    w, h, raw = read_pgm(PGM)
    img = np.flipud(raw)           # bottom-up grid, same as map_server
    occ = img < 90
    free = img > 205

    data = json.load(open(BOXES))
    boxes = []
    for b in data['boxes']:
        cx, cy, _ = b['center']
        sx, sy, _ = b['size']
        boxes.append((b['name'], cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2))

    # distance (m) from each free cell centre to the nearest occupied cell centre
    edt = distance_transform_edt(~occ) * res
    allowed = free & (edt >= CLEAR_MIN)

    def rc(x, y):
        return (int(math.floor((y - oy) / res)), int(math.floor((x - ox) / res)))

    def xy(r, c):
        return (ox + (c + 0.5) * res, oy + (r + 0.5) * res)

    # ---- connected component containing the spawn cell
    r0, c0 = rc(*SPAWN)
    assert free[r0, c0], 'spawn cell is not free'
    lab, nlab = label(allowed, structure=np.ones((3, 3), dtype=int))
    # spawn may be slightly below CLEAR_MIN; use the label of the nearest allowed cell
    comp = lab[r0, c0]
    if comp == 0:
        rr, cc = np.where(allowed)
        k = np.argmin((rr - r0) ** 2 + (cc - c0) ** 2)
        comp = lab[rr[k], cc[k]]
    reach = lab == comp

    # exact clearance to the box geometry (analytic), plus wall inner faces
    def exact_clear(x, y):
        best = min(6.0 - abs(x), 4.0 - abs(y))
        for _, x0, x1, y0, y1 in boxes:
            dx = max(x0 - x, 0.0, x - x1)
            dy = max(y0 - y, 0.0, y - y1)
            best = min(best, math.hypot(dx, dy))
        return best

    # ---- candidate points: max clearance inside each region, min separation
    picks = []
    cand = np.argwhere(reach)
    xs = ox + (cand[:, 1] + 0.5) * res
    ys = oy + (cand[:, 0] + 0.5) * res
    vals = edt[cand[:, 0], cand[:, 1]]
    dist_spawn = np.hypot(xs - SPAWN[0], ys - SPAWN[1])

    def add_pick(name, mask, why, min_sep=2.0):
        idx = np.where(mask)[0]
        assert idx.size, 'no reachable cell for %s' % name
        for i in idx[np.argsort(-vals[idx])]:
            if all(math.hypot(xs[i] - p['x'], ys[i] - p['y']) >= min_sep
                   for p in picks):
                picks.append({'name': name, 'x': round(float(xs[i]), 4),
                              'y': round(float(ys[i]), 4),
                              'edt_clear_m': round(float(vals[i]), 4),
                              'exact_clear_m': round(exact_clear(xs[i], ys[i]), 4),
                              'why': why})
                return
        raise AssertionError('no separated cell for %s' % name)

    add_pick('P1_spawn_near', (dist_spawn >= 0.8) & (dist_spawn <= 1.5),
             'best clearance in the 0.8-1.5 m annulus around spawn')
    add_pick('P2_NW', (xs <= -3.5) & (ys >= 1.5), 'NW quadrant, max clearance')
    add_pick('P3_SW', (xs <= -3.5) & (ys <= -1.5), 'SW quadrant, max clearance')
    add_pick('P4_SE', (xs >= 3.5) & (ys <= -1.5), 'SE quadrant, max clearance')
    add_pick('P5_NE', (xs >= 3.5) & (ys >= 1.5), 'NE quadrant, max clearance')

    # ---- pairwise Dijkstra on the reachable set (8-connected)
    idx = -np.ones((h, w), dtype=np.int64)
    pts_rc = np.argwhere(reach)
    idx[pts_rc[:, 0], pts_rc[:, 1]] = np.arange(len(pts_rc))
    rows_i, cols_i, wts = [], [], []
    for dr, dc, wgt in ((0, 1, res), (1, 0, res), (0, -1, res), (-1, 0, res),
                        (1, 1, res * math.sqrt(2)), (1, -1, res * math.sqrt(2)),
                        (-1, 1, res * math.sqrt(2)), (-1, -1, res * math.sqrt(2))):
        rr = pts_rc[:, 0] + dr
        cc = pts_rc[:, 1] + dc
        ok = (rr >= 0) & (rr < h) & (cc >= 0) & (cc < w)
        nbr = np.where(ok, idx[np.clip(rr, 0, h - 1), np.clip(cc, 0, w - 1)], -1)
        src = np.nonzero(ok & (nbr >= 0))[0]
        rows_i.append(src)
        cols_i.append(nbr[src])
        wts.append(np.full(src.size, wgt))
    graph = coo_matrix((np.concatenate(wts),
                        (np.concatenate(rows_i), np.concatenate(cols_i))),
                       shape=(len(pts_rc), len(pts_rc))).tocsr()

    pidx = []
    for p in picks:
        r, c = rc(p['x'], p['y'])
        pidx.append(int(idx[r, c]))
    assert min(pidx) >= 0, 'a patrol point is not in the reachable set'

    D, pred = dijkstra(graph, directed=False, indices=pidx,
                       return_predecessors=True)
    pairwise = {}
    for i in range(len(picks)):
        for j in range(len(picks)):
            pairwise['%s->%s' % (picks[i]['name'], picks[j]['name'])] = \
                round(float(D[i, pidx[j]]), 3)

    # shortest path from P1 through the quadrants; min clearance along it
    tour = list(range(5))
    route_len = 0.0
    route_min_clear = 1e9
    for a, b in zip(tour[:-1], tour[1:]):
        route_len += float(D[a, pidx[b]])
        node = pidx[b]
        while node != -1 and node != pidx[a]:
            r, c = pts_rc[node]
            route_min_clear = min(route_min_clear, float(edt[r, c]))
            node = pred[a, node]

    for p in picks:
        p['yaw'] = round(math.atan2(-p['y'], -p['x']), 4) if (p['x'] or p['y']) else 0.0
        if p['name'] == 'P1_spawn_near':
            p['yaw'] = 0.0

    out = {
        'map': {'resolution': res, 'origin': origin},
        'criterion': {'clearance_min_m': CLEAR_MIN, 'spawn': SPAWN,
                      'connectivity': '8-connected, free & clear>=0.45, spawn component'},
        'reachable_cells': int(reach.sum()),
        'reachable_area_m2': round(int(reach.sum()) * res * res, 2),
        'points': picks,
        'pairwise_dijkstra_m': pairwise,
        'tour_order': [p['name'] for p in picks],
        'tour_length_m': round(route_len, 2),
        'tour_min_clearance_edt_m': round(route_min_clear, 3),
    }
    os.makedirs(os.path.dirname(OUT_JSON), exist_ok=True)
    json.dump(out, open(OUT_JSON, 'w', encoding='utf-8'), indent=1, ensure_ascii=False)

    os.makedirs(os.path.dirname(OUT_YAML), exist_ok=True)
    with open(OUT_YAML, 'w', encoding='utf-8') as f:
        f.write('# S2a patrol waypoints (map frame, metres, yaw=rad)\n')
        f.write('# chosen on rmul2027_3v3_up.pgm: free & clearance >= 0.45 m,\n')
        f.write('# all 5 in one connected component; see results/patrol_point_analysis.json\n')
        for p in picks:
            f.write('- {x: %.3f, y: %.3f, yaw: %.3f}  # %s clear=%.2f m\n'
                    % (p['x'], p['y'], p['yaw'], p['name'], p['exact_clear_m']))

    print('reachable: %d cells (%.1f m2)' % (reach.sum(), reach.sum() * res * res))
    print('%-14s %8s %8s %9s %9s' % ('point', 'x', 'y', 'edt_clr', 'exact_clr'))
    for p in picks:
        print('%-14s %8.3f %8.3f %9.3f %9.3f'
              % (p['name'], p['x'], p['y'], p['edt_clear_m'], p['exact_clear_m']))
    off = [D[i, pidx[j]] for i in range(5) for j in range(5) if i != j]
    print('pairwise min/max: %.2f / %.2f m' % (min(off), max(off)))
    print('tour %.2f m via %s, min clearance along path %.3f m'
          % (route_len, out['tour_order'], route_min_clear))
    print('YAML', OUT_YAML)
    print('JSON', OUT_JSON)


if __name__ == '__main__':
    main()