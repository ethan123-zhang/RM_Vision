#!/usr/bin/env python3
"""Offline self test for the map_align_check estimator (no ROS, no Gazebo).

Two synthetic observations of the CAD prior from a known offset:
  surface  every occupied cell that touches free space (worst case: the whole
           map boundary is visible, no occlusion)
  cast     720 rays from the sensor position, first hit only (what a 360 deg
           lidar at (0, 0) actually sees)
Runs the real estimator on both and checks that it recovers the offset, then
cross-checks the numpy distance-field fallback against scipy.

    python3 test/test_estimator.py [--yaml ...] [--no-numpy]
"""

import argparse
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from map_align_check.grid import CadGrid            # noqa: E402
from map_align_check import estimator               # noqa: E402

DEFAULT_YAML = '/mnt/e/RM/tb3_gazebo_nav2/rm_field/maps/rmul2027_3v3_up.yaml'


def surface_points(cad, sensor_xy=(0.0, 0.0), range_max=8.0, count=3000,
                   noise_m=0.0, seed=0):
    """Occupied cells that touch free space (a lidar only sees surfaces)."""
    occ = cad.occ
    p = np.pad(occ, 1, mode='constant', constant_values=False)
    core = p[1:-1, 1:-1]
    neighbours = (p[1:-1, :-2] & p[1:-1, 2:] & p[:-2, 1:-1] & p[2:, 1:-1])
    rows, cols = np.nonzero(core & ~neighbours)
    x = cad.origin[0] + (cols + 0.5) * cad.res
    y = cad.origin[1] + (rows + 0.5) * cad.res
    keep = np.hypot(x - sensor_xy[0], y - sensor_xy[1]) <= range_max
    x, y = x[keep], y[keep]
    rng = np.random.default_rng(seed)
    if x.size > count:
        idx = rng.choice(x.size, count, replace=False)
        x, y = x[idx], y[idx]
    pts = np.stack([x, y], axis=1) + rng.uniform(-0.5, 0.5, (x.size, 2)) * cad.res
    if noise_m > 0.0:
        pts = pts + rng.normal(0.0, noise_m, pts.shape)
    return pts


def cast_points(cad, sensor_xy=(0.0, 0.0), range_max=8.0, n_rays=720,
                noise_m=0.0, seed=0):
    """2-D ray cast: the first occupied cell along each ray (lidar-like)."""
    angles = np.linspace(0.0, 2.0 * math.pi, n_rays, endpoint=False)
    steps = np.arange(0.0, range_max, cad.res * 0.5)
    xs = sensor_xy[0] + np.cos(angles)[:, None] * steps[None, :]
    ys = sensor_xy[1] + np.sin(angles)[:, None] * steps[None, :]
    cols = np.floor((xs - cad.origin[0]) / cad.res).astype(np.int64)
    rows = np.floor((ys - cad.origin[1]) / cad.res).astype(np.int64)
    inside = ((rows >= 0) & (rows < cad.height) &
              (cols >= 0) & (cols < cad.width))
    hit = np.zeros_like(inside)
    hit[inside] = cad.occ[rows[inside], cols[inside]]
    first = np.argmax(hit, axis=1)
    found = hit[np.arange(n_rays), first]
    x = xs[np.arange(n_rays), first][found].copy()
    y = ys[np.arange(n_rays), first][found].copy()
    rng = np.random.default_rng(seed)
    pts = np.stack([x, y], axis=1)
    if noise_m > 0.0:
        pts = pts + rng.normal(0.0, noise_m, pts.shape)
    return pts


def inject(points_field, dx, dy, dyaw):
    """p_map = R(dyaw) p_field + t -- the same T_off odin1_sim_bridge injects."""
    c, s = math.cos(dyaw), math.sin(dyaw)
    return np.stack([c * points_field[:, 0] - s * points_field[:, 1] + dx,
                     s * points_field[:, 0] + c * points_field[:, 1] + dy],
                    axis=1)


def run_case(cad, name, expected, points_field):
    obs = inject(points_field, expected[0], expected[1],
                 math.radians(expected[2]))
    t0 = time.perf_counter()
    result = estimator.estimate(obs, cad)
    wall = time.perf_counter() - t0
    est = result['estimate']
    err_xy = math.hypot(est['dx'] - expected[0], est['dy'] - expected[1])
    err_yaw = abs(est['dyaw_deg'] - expected[2])
    ok = err_xy <= 0.05 and err_yaw <= 0.5
    print('%-24s inject (%+.3f, %+.3f, %+5.2f) -> est (%+.3f, %+.3f, '
          '%+5.2f)  err %.4f m / %.3f deg  res %.2f cm  gap %.2f m  '
          '%.2f s %s'
          % (name, expected[0], expected[1], expected[2], est['dx'], est['dy'],
             est['dyaw_deg'], err_xy, err_yaw,
             100.0 * result['residual']['mean_m'],
             result['confidence']['gap_m'], wall, 'OK' if ok else 'FAIL'))
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--yaml', default=DEFAULT_YAML)
    ap.add_argument('--no-numpy', action='store_true',
                    help='skip the numpy fallback cross-check')
    args = ap.parse_args()

    print('-- self test on %s' % args.yaml)
    t0 = time.perf_counter()
    cad = CadGrid(args.yaml)
    print('-- CadGrid: %dx%d @ %.3f m, %d occupied cells, field %s in %.2f s'
          % (cad.width, cad.height, cad.res, cad.n_occupied, cad.backend,
             time.perf_counter() - t0))

    cases = [
        ('zero', (0.0, 0.0, 0.0)),
        ('case2', (0.30, -0.20, 3.0)),
        ('case3', (-0.15, 0.25, -5.0)),
        ('corner', (0.55, 0.55, 9.0)),
    ]
    observations = {
        'surface': surface_points(cad, noise_m=0.01),
        'cast': cast_points(cad, noise_m=0.01),
    }
    failures = 0
    for obs_name, pts in observations.items():
        print('-- observation %-8s %4d points' % (obs_name, pts.shape[0]))
        for name, expected in cases:
            failures += 0 if run_case(cad, name, expected, pts) else 1

    if not args.no_numpy:
        cad_np = CadGrid(args.yaml, backend='numpy')
        diff = np.abs(cad_np.field.field - cad.field.field)
        print('-- numpy fallback %s: max |field - scipy| = %.3f m, '
              'mean = %.4f m'
              % (cad_np.backend, float(diff.max()), float(diff.mean())))
        for name, expected in cases[1:3]:
            failures += 0 if run_case(cad_np, name + ' [numpy]', expected,
                                      observations['cast']) else 1

    print('SELF_TEST %s (%d failure(s))'
          % ('PASS' if failures == 0 else 'FAIL', failures))
    return 0 if failures == 0 else 1


if __name__ == '__main__':
    sys.exit(main())