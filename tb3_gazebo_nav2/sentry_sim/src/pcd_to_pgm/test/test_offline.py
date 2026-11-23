#!/usr/bin/env python3
"""Offline smoke test for pcd_to_pgm (no ROS, no simulator).

Builds a synthetic 6x4 m room from lidar-like scans at a few poses, maps it
with OccGrid, saves pgm/yaml, then runs the real compare code against a truth
map generated from the same geometry. Checks:

  * PGM round-trip (write -> read)
  * occupancy recall on the synthetic truth
  * free-area error / connectivity
  * yaml keys map_server needs

Run from anywhere:  python3 test/test_offline.py   (numpy + pyyaml only)
"""

import json
import os
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
# The package dir ~/sentry_ws/src/pcd_to_pgm is itself called pcd_to_pgm and has
# no __init__.py, so putting .../src on sys.path would import a *namespace*
# package.  Put the package dir itself first: pcd_to_pgm/pcd_to_pgm/__init__.py.
sys.path.insert(0, os.path.join(HERE, '..'))

from pcd_to_pgm.grid import OccGrid, read_pgm, write_pgm  # noqa: E402
from pcd_to_pgm.compare import (compare, format_report, load_map,  # noqa: E402
                                classify_map)

RES = 0.025
ORIGIN = (-4.0, -3.0)
WIDTH = int(round(8.0 / RES))
HEIGHT = int(round(6.0 / RES))


def room_points(x0, y0, x1, y1, step=0.01, z=0.5):
    """Points on the four inner walls of a rectangle, at height z."""
    xs = np.arange(x0, x1 + 1e-9, step)
    ys = np.arange(y0, y1 + 1e-9, step)
    pts = [(x, y0, z) for x in xs] + [(x, y1, z) for x in xs]
    pts += [(x0, y, z) for y in ys] + [(x1, y, z) for y in ys]
    return np.array(pts, dtype=np.float64)


def add_scan(grid, pose, pts, max_range=10.0, ground_z=0.0):
    """Mimic the node's filter + integration (map frame == identity here)."""
    keep = (pts[:, 2] >= ground_z + 0.10) & (pts[:, 2] <= ground_z + 1.20)
    dist = np.hypot(pts[:, 0] - pose[0], pts[:, 1] - pose[1])
    keep &= (dist <= max_range) & (dist >= 0.35)
    sel = pts[keep]
    return grid.add_scan(sel[:, 0], sel[:, 1], pose[0], pose[1])


def build_truth(path, walls):
    """Truth map, same convention as rm_field: the room interior is free, every
    cell whose centre is outside it is occupied (solid walls, no unknown)."""
    del walls   # the solid-wall truth follows from the rectangle, not the samples
    img = np.full((HEIGHT, WIDTH), 0, dtype=np.uint8)        # occupied walls
    cols = np.arange(WIDTH)
    rows = np.arange(HEIGHT)
    xs = ORIGIN[0] + (cols + 0.5) * RES
    ys = ORIGIN[1] + (rows + 0.5) * RES
    inside_x = (xs > -3.0) & (xs < 3.0)
    inside_y = (ys > -2.0) & (ys < 2.0)
    img[np.ix_(inside_y, inside_x)] = 254                   # free interior
    write_pgm(path + '.pgm', img)
    with open(path + '.yaml', 'w') as handle:
        handle.write('image: truth.pgm\nresolution: %.6g\n'
                     'origin: [%.6g, %.6g, 0.0]\nnegate: 0\n'
                     'occupied_thresh: 0.65\nfree_thresh: 0.196\n'
                     % (RES, ORIGIN[0], ORIGIN[1]))
    return path + '.yaml'


def main():
    tmp = tempfile.mkdtemp(prefix='pcd_to_pgm_test_')
    walls = room_points(-3.0, -2.0, 3.0, 2.0)
    truth_yaml = build_truth(os.path.join(tmp, 'truth'), walls)

    poses = [(x, y) for x in (-2.2, -1.0, 0.2, 1.4, 2.4)
             for y in (-1.4, -0.4, 0.6, 1.4)]
    grid = OccGrid(resolution=RES, origin=ORIGIN, width=WIDTH, height=HEIGHT)
    for pose in poses:
        for _ in range(3):
            add_scan(grid, pose, walls)
    out_base = os.path.join(tmp, 'gen')
    stats = grid.save(out_base)
    print('map stats:', json.dumps(stats))

    img = read_pgm(out_base + '.pgm')
    assert img.shape == (HEIGHT, WIDTH), img.shape
    assert int((img == 0).sum()) > 100, 'no occupied pixels'
    assert os.path.exists(out_base + '.yaml')

    loaded = load_map(out_base + '.yaml')
    occ, free = classify_map(loaded)
    assert occ.sum() > 100 and free.sum() > 1000, (occ.sum(), free.sum())

    traj = os.path.join(tmp, 'gen_traj.json')
    with open(traj, 'w') as handle:
        json.dump({'poses': [{'sx': p[0], 'sy': p[1]} for p in poses]}, handle)

    report = compare(truth_yaml, out_base + '.yaml', traj_path=traj,
                     diff_path=os.path.join(tmp, 'diff.png'))
    print(format_report(report))
    occupancy = report['occupancy']  # noqa: F841  (reported, no threshold)
    visible = report['observable']
    free_area = report['free_area']
    connectivity = report['connectivity']['ours']

    failures = []
    if visible['recall_visible'] < 0.95:
        failures.append('recall_visible %.3f < 0.95' % visible['recall_visible'])
    if free_area['err_tol_pct'] > 5.0:
        failures.append('free err(1 cell) %.2f%% > 5%%' % free_area['err_tol_pct'])
    if not connectivity['connected']:
        failures.append('free region not connected (%d components)'
                        % connectivity['components_ge_min'])
    if failures:
        print('FAIL: %s' % '; '.join(failures))
        return 1
    print('PASS: synthetic room, recall/area/connectivity in spec')
    print('artifacts in %s' % tmp)
    return 0


if __name__ == '__main__':
    sys.exit(main())