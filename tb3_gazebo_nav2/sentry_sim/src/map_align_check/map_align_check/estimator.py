"""Two-stage rigid 2-D registration of a lidar observation onto the CAD prior.

Search space is T = (dx, dy, dyaw) mapping **field/CAD coordinates into Odin1's
map frame**::

    p_map = R(dyaw) * p_field + (dx, dy)

so for a candidate pose the observation is pushed back into the CAD frame with
``p_field = R(-dyaw) * (p_map - (dx, dy))`` and scored by the mean distance from
those points to the nearest CAD **surface** cell, i.e. the free/occupied edge of
the grid (lower is better). Scoring against the CAD surface instead of the
occupied area matters: a lidar return sits *exactly* on the wall face, one half
cell outside the occupied cell centre, so a nearest-occupied-cell field pays for
the true pose and lets the optimum trade a sideways slide for a rotation --
measured on this field that was a systematic +0.57 deg yaw bias, gone with the
surface field. The estimate therefore has exactly the sign convention of
odin1_sim_bridge's ``map_offset_x/y/yaw``: inject (0.30, -0.20, 3 deg) and this
module estimates (+0.30, -0.20, +3 deg). The report also carries the classic
"distance to nearest occupied cell" so both numbers are available.

Pure numpy, no ROS, no scipy: the coarse grid search is vectorised over the
translation grid (one rotated copy of the cloud per yaw), the refinement is a
bounded pattern search on the full cloud using the bilinear distance field.
"""

import math
import time

import numpy as np

DEFAULTS = {
    'xy_max': 0.60,          # m, coarse search half width
    'xy_step': 0.025,        # m, coarse translation step
    'yaw_max_deg': 10.0,     # deg, coarse yaw half range
    'yaw_step_deg': 0.5,     # deg, coarse yaw step
    'coarse_points': 400,    # points used by the coarse pass
    'fine_points': 3000,     # points used by the refinement
    'second_gap_xy': 0.12,   # m, a rival > this far away counts as second-best
    'second_gap_yaw_deg': 1.5,
}


def _rot(xy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    x = xy[..., 0]
    y = xy[..., 1]
    return np.stack([c * x - s * y, s * x + c * y], axis=-1)


def field_coords(points_map, pose):
    """Map-frame points -> CAD-frame points for pose = (dx, dy, dyaw)."""
    p = np.asarray(points_map, dtype=np.float64)[:, :2] - np.asarray(pose[:2])
    return _rot(p, -pose[2])


def pose_distances(points_map, dist_field, pose, bilinear=True):
    pts = field_coords(points_map, pose)
    if bilinear:
        return dist_field.sample_bilinear(pts)
    return dist_field.sample_nearest(pts)


def pose_score(points_map, dist_field, pose, bilinear=True):
    return float(np.mean(pose_distances(points_map, dist_field, pose, bilinear)))


def subsample(points_xy, count, seed=0):
    """Deterministic spatially-agnostic subsample (rng seed 0)."""
    n = points_xy.shape[0]
    if count is None or n <= count:
        return points_xy
    idx = np.random.default_rng(seed).choice(n, int(count), replace=False)
    idx.sort()
    return points_xy[idx]


def coarse_search(points_map, dist_field, xy_max=None, xy_step=None,
                  yaw_max_deg=None, yaw_step_deg=None, coarse_points=None,
                  chunk=256):
    """Vectorised (dx, dy, dyaw) grid search. Returns scores + grids."""
    cfg = DEFAULTS
    xy_max = cfg['xy_max'] if xy_max is None else xy_max
    xy_step = cfg['xy_step'] if xy_step is None else xy_step
    yaw_max_deg = cfg['yaw_max_deg'] if yaw_max_deg is None else yaw_max_deg
    yaw_step_deg = cfg['yaw_step_deg'] if yaw_step_deg is None else yaw_step_deg
    coarse_points = cfg['coarse_points'] if coarse_points is None else coarse_points

    xs = np.arange(-xy_max, xy_max + 1e-9, xy_step)
    yaws = np.radians(np.arange(-yaw_max_deg, yaw_max_deg + 1e-9, yaw_step_deg))
    pts = subsample(np.asarray(points_map, dtype=np.float64), coarse_points)
    gx, gy = np.meshgrid(xs, xs, indexing='ij')
    trans = np.stack([gx.ravel(), gy.ravel()], axis=1)

    scores = np.empty((yaws.size, trans.shape[0]), dtype=np.float32)
    for iy, yaw in enumerate(yaws):
        # observed points rotated into the CAD frame (translation applied below)
        q = _rot(pts, -yaw)
        # -R(-yaw) * t for every candidate translation, computed in one go
        b = -_rot(trans, -yaw)
        for start in range(0, trans.shape[0], chunk):
            blk = b[start:start + chunk]
            coords = q[None, :, :] + blk[:, None, :]
            d = dist_field.sample_nearest(coords.reshape(-1, 2))
            scores[iy, start:start + chunk] = d.reshape(blk.shape[0], -1).mean(axis=1)
    return {
        'scores': scores.reshape(yaws.size, xs.size, xs.size),
        'xs': xs, 'yaws': yaws,
        'n_points': int(pts.shape[0]),
    }


def _refine(points_map, dist_field, pose0, steps, lo, hi, min_step,
            max_rounds=80):
    p = np.array(pose0, dtype=float)
    best = pose_score(points_map, dist_field, p, bilinear=True)
    step = np.array(steps, dtype=float)
    rounds = 0
    while rounds < max_rounds:
        rounds += 1
        improved = False
        for axis in range(3):
            for sign in (1.0, -1.0):
                q = p.copy()
                q[axis] = min(max(q[axis] + sign * step[axis], lo[axis]),
                              hi[axis])
                if np.allclose(q, p):
                    continue
                value = pose_score(points_map, dist_field, q, bilinear=True)
                if value < best - 1e-7:
                    best = value
                    p = q
                    improved = True
        if not improved:
            step *= 0.5
            if np.all(step < np.asarray(min_step, dtype=float)):
                break
    return p, best, rounds


def estimate(points_map, cad, **kwargs):
    """Full estimate. ``cad`` is a grid.CadGrid; ``points_map`` is (N, 2) in map."""
    cfg = dict(DEFAULTS)
    cfg.update({k: v for k, v in kwargs.items() if v is not None})
    t_start = time.perf_counter()

    points_map = np.asarray(points_map, dtype=np.float64).reshape(-1, 2)
    if points_map.shape[0] < 20:
        raise ValueError('only %d observation points, need >= 20'
                         % points_map.shape[0])

    t0 = time.perf_counter()
    coarse = coarse_search(points_map, cad.field,
                           xy_max=cfg['xy_max'], xy_step=cfg['xy_step'],
                           yaw_max_deg=cfg['yaw_max_deg'],
                           yaw_step_deg=cfg['yaw_step_deg'],
                           coarse_points=cfg['coarse_points'])
    coarse_s = time.perf_counter() - t0

    scores = coarse['scores']
    xs, yaws = coarse['xs'], coarse['yaws']
    iy, ix, iz = np.unravel_index(int(np.argmin(scores)), scores.shape)
    best_score = float(scores[iy, ix, iz])
    pose0 = (float(xs[ix]), float(xs[iz]), float(yaws[iy]))

    # confidence: best rival that is materially different in translation or yaw
    gx, gy = np.meshgrid(xs, xs, indexing='ij')
    dist2 = (gx - xs[ix]) ** 2 + (gy - xs[iz]) ** 2
    dyaw = np.abs(yaws - yaws[iy])[:, None, None]
    rival = (dist2[None, :, :] > cfg['second_gap_xy'] ** 2) | \
            (dyaw > math.radians(cfg['second_gap_yaw_deg']))
    if rival.any():
        masked = np.where(rival, scores, np.inf)
        second = float(np.min(masked))
        index = np.unravel_index(int(np.argmin(masked)), masked.shape)
        second_pose = (float(xs[index[1]]), float(xs[index[2]]),
                       float(yaws[index[0]]))
    else:  # pragma: no cover - a 49 x 49 x 41 grid always has rivals
        second, second_pose = float('inf'), (0.0, 0.0, 0.0)

    t0 = time.perf_counter()
    pts_fine = subsample(points_map, cfg['fine_points'])
    lo = np.array([-cfg['xy_max'] - 0.05, -cfg['xy_max'] - 0.05,
                   -math.radians(cfg['yaw_max_deg']) - math.radians(0.5)])
    hi = -lo
    pose, fine_score, rounds = _refine(
        pts_fine, cad.field, pose0,
        steps=(cfg['xy_step'], cfg['xy_step'], math.radians(cfg['yaw_step_deg'])),
        lo=lo, hi=hi,
        min_step=(0.0005, 0.0005, math.radians(0.005)))
    refine_s = time.perf_counter() - t0

    residual = cad.residual_stats(field_coords(points_map, pose))
    residual['score_mean_capped_m'] = fine_score

    return {
        'estimate': {
            'dx': float(pose[0]),
            'dy': float(pose[1]),
            'dyaw': float(pose[2]),
            'dyaw_deg': math.degrees(float(pose[2])),
        },
        'residual': residual,
        'confidence': {
            'best_m': best_score,
            'second_best_m': second,
            'gap_m': second - best_score,
            'ratio': (second / best_score) if best_score > 1e-9 else None,
            'second_best_pose': {'dx': second_pose[0], 'dy': second_pose[1],
                                 'dyaw_deg': math.degrees(second_pose[2])},
            'gap_xy_m': cfg['second_gap_xy'],
            'gap_yaw_deg': cfg['second_gap_yaw_deg'],
        },
        'coarse': {
            'pose': {'dx': pose0[0], 'dy': pose0[1],
                     'dyaw_deg': math.degrees(pose0[2])},
            'n_points': coarse['n_points'],
            'grid': {'n_dx': int(xs.size), 'n_dy': int(xs.size),
                     'n_yaw': int(yaws.size), 'xy_step': cfg['xy_step'],
                     'yaw_step_deg': cfg['yaw_step_deg']},
        },
        'points': {'used_coarse': coarse['n_points'],
                   'used_fine': int(pts_fine.shape[0]),
                   'all': int(points_map.shape[0])},
        'timing': {'coarse_s': coarse_s, 'refine_s': refine_s,
                   'rounds': rounds,
                   'total_s': time.perf_counter() - t_start},
        'backend': cad.backend,
    }