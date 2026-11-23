"""Offline comparison of a generated map against a truth map.

Three numbers, as asked:

* **occupancy recall** -- truth-occupied cells that our map also calls
  occupied.  Reported twice: over *all* truth-occupied cells, and over the
  cells a 2-D lidar can actually see from the recorded trajectory.  The second
  one is the honest ceiling: a truth map that projects solid boxes contains
  unobservable interiors (a 0.4 m box hides everything behind its own walls),
  so no sensor-driven map can reach 100 % on the first definition.
* **free-area error** -- missed / extra free area in m2 and as % of truth free.
* **connectivity** -- 4-connected components of our free region (run-length
  union-find, no scipy dependency).

Pure numpy + stdlib; only the .yaml read needs PyYAML.
"""

import json
import math
import os

import numpy as np

from .grid import (FREE_THRESH, OCCUPIED_THRESH, read_pgm, write_pgm)


# --------------------------------------------------------------------- I/O
def load_map(yaml_path):
    import yaml
    with open(yaml_path) as handle:
        meta = yaml.safe_load(handle)
    image_ref = meta['image']
    if not os.path.isabs(image_ref):
        image_ref = os.path.join(os.path.dirname(os.path.abspath(yaml_path)),
                                 image_ref)
    return {
        'yaml': yaml_path,
        'image_path': image_ref,
        'image': read_pgm(image_ref),
        'resolution': float(meta['resolution']),
        'origin': (float(meta['origin'][0]), float(meta['origin'][1])),
        'negate': int(meta.get('negate', 0)),
        'occupied_thresh': float(meta.get('occupied_thresh', OCCUPIED_THRESH)),
        'free_thresh': float(meta.get('free_thresh', FREE_THRESH)),
    }


def classify_map(loaded):
    """Return (occupied, free) boolean masks, array row 0 == minimum world y."""
    img = loaded['image'].astype(np.float32)
    if loaded['negate']:
        occ_prob = img / 255.0
    else:
        occ_prob = (255.0 - img) / 255.0
    occupied = np.flipud(occ_prob > loaded['occupied_thresh'])
    free = np.flipud(occ_prob < loaded['free_thresh'])
    return occupied, free


# ------------------------------------------------------------------ raster
def _grid_shift(src, ref):
    res = ref['resolution']
    if abs(src['resolution'] - res) > 1e-9:
        raise ValueError('resolution differs: %r vs %r'
                         % (src['resolution'], res))
    dcol = (src['origin'][0] - ref['origin'][0]) / res
    drow = (src['origin'][1] - ref['origin'][1]) / res
    icol = int(round(dcol))
    irow = int(round(drow))
    if abs(dcol - icol) > 1e-3 or abs(drow - irow) > 1e-3:
        raise ValueError('map origins are not grid-aligned')
    return icol, irow


def align_mask(mask, shift, ref_shape):
    """Re-index a bottom-up mask onto the reference grid; returns (mask, dropped)."""
    dcol, drow = shift
    height, width = mask.shape
    ref_h, ref_w = ref_shape
    out = np.zeros((ref_h, ref_w), dtype=bool)
    r0, r1 = max(0, -drow), min(height, ref_h - drow)
    c0, c1 = max(0, -dcol), min(width, ref_w - dcol)
    if r1 > r0 and c1 > c0:
        out[r0 + drow:r1 + drow, c0 + dcol:c1 + dcol] = mask[r0:r1, c0:c1]
    dropped = int(np.count_nonzero(mask)) - int(np.count_nonzero(out))
    return out, dropped


def dilate(mask, radius=1):
    out = mask.copy()
    for _ in range(int(radius)):
        padded = np.pad(out, 1, mode='constant', constant_values=False)
        out = (out
               | padded[2:, 1:-1] | padded[:-2, 1:-1]
               | padded[1:-1, 2:] | padded[1:-1, :-2])
    return out


# ------------------------------------------------------------- connectivity
def label_components(mask):
    """4-connected runs -> (labels, sizes). Labels are 0 for background."""
    height, width = mask.shape
    parent = []

    def find(node):
        while parent[node] != node:
            parent[node] = parent[parent[node]]
            node = parent[node]
        return node

    def union(a, b):
        root_a, root_b = find(a), find(b)
        if root_a != root_b:
            parent[root_b] = root_a

    rows = []
    ids_rows = []
    for row in range(height):
        line = mask[row].astype(np.int8)
        edges = np.diff(np.concatenate(([0], line, [0])))
        starts = np.flatnonzero(edges == 1)
        ends = np.flatnonzero(edges == -1) - 1
        runs = list(zip(starts.tolist(), ends.tolist()))
        ids = []
        for _ in runs:
            parent.append(len(parent))
            ids.append(len(parent) - 1)
        if row > 0:
            prev_runs, prev_ids = rows[row - 1], ids_rows[row - 1]
            i = j = 0
            while i < len(runs) and j < len(prev_runs):
                start, end = runs[i]
                pstart, pend = prev_runs[j]
                if end < pstart:
                    i += 1
                elif pend < start:
                    j += 1
                else:
                    union(ids[i], prev_ids[j])
                    if end <= pend:
                        i += 1
                    if pend <= end:
                        j += 1
        rows.append(runs)
        ids_rows.append(ids)

    labels = np.zeros((height, width), dtype=np.int32)
    sizes = {}
    for row, (runs, ids) in enumerate(zip(rows, ids_rows)):
        for (start, end), ident in zip(runs, ids):
            root = find(ident)
            labels[row, start:end + 1] = root + 1
            sizes[root + 1] = sizes.get(root + 1, 0) + (end - start + 1)
    return labels, sizes


def connectivity(mask, min_cells=25):
    _, sizes = label_components(mask)
    total = int(np.count_nonzero(mask))
    ordered = sorted(sizes.values(), reverse=True)
    big = [s for s in ordered if s >= min_cells]
    return {
        'free_cells': total,
        'components_all': len(ordered),
        'components_ge_min': len(big),
        'min_component_cells': int(min_cells),
        'largest_cells': int(ordered[0]) if ordered else 0,
        'largest_frac': round(ordered[0] / total, 4) if total else 0.0,
        'connected': bool(len(big) <= 1),
    }


# ---------------------------------------------------------------- visibility
def _walk_until_blocked(occupied, col0, row0, max_range, resolution,
                        n_rays):
    """Cells a 360 deg sweep sees from (col0, row0), stopping on occupied cells."""
    height, width = occupied.shape
    theta = np.arange(n_rays, dtype=np.float64) * (2.0 * math.pi / n_rays)
    reach = max_range / resolution
    xe = np.clip(np.round(col0 + reach * np.cos(theta)), 0, width - 1).astype(np.int32)
    ye = np.clip(np.round(row0 + reach * np.sin(theta)), 0, height - 1).astype(np.int32)
    xs = np.full(n_rays, col0, dtype=np.int32)
    ys = np.full(n_rays, row0, dtype=np.int32)
    dxn = np.abs(xe - xs).astype(np.int64)
    dyn = -np.abs(ye - ys).astype(np.int64)
    sx = np.where(xe >= xs, 1, -1).astype(np.int32)
    sy = np.where(ye >= ys, 1, -1).astype(np.int32)
    err = dxn + dyn
    alive = np.ones(n_rays, dtype=bool)
    visited = np.zeros((height, width), dtype=bool)
    steps = int(max(dxn.max(), -dyn.max())) + 1
    for _ in range(steps):
        idx = np.flatnonzero(alive)
        if idx.size == 0:
            break
        cx = xs[idx]
        cy = ys[idx]
        visited[cy, cx] = True
        hit = occupied[cy, cx]
        if hit.any():
            alive[idx[hit]] = False
        idx = np.flatnonzero(alive)
        if idx.size == 0:
            break
        e2 = 2 * err[idx]
        step_x = e2 >= dyn[idx]
        step_y = e2 <= dxn[idx]
        xs[idx] += np.where(step_x, sx[idx], 0)
        ys[idx] += np.where(step_y, sy[idx], 0)
        err[idx] += np.where(step_x, dyn[idx], 0) + np.where(step_y, dxn[idx], 0)
    return visited


def visible_occupied_mask(occupied, poses, origin, resolution,
                          max_range=10.0, n_rays=2048, min_spacing=0.25):
    """Truth-occupied cells with line of sight from the recorded poses."""
    if not poses:
        return np.zeros_like(occupied), 0
    height, width = occupied.shape
    seen = np.zeros((height, width), dtype=bool)
    last = None
    used = 0
    for pose in poses:
        x = float(pose[0])
        y = float(pose[1])
        if last is not None and math.hypot(x - last[0], y - last[1]) < min_spacing:
            continue
        last = (x, y)
        col = int(math.floor((x - origin[0]) / resolution))
        row = int(math.floor((y - origin[1]) / resolution))
        if not (0 <= col < width and 0 <= row < height):
            continue
        seen |= _walk_until_blocked(occupied, col, row, max_range, resolution,
                                    n_rays)
        used += 1
    return seen & occupied, used


# ------------------------------------------------------------------- compare
def compare(truth_yaml, ours_yaml, traj_path=None, tol=1,
            min_component_cells=25, max_range=10.0, n_rays=2048,
            diff_path=None):
    truth = load_map(truth_yaml)
    ours = load_map(ours_yaml)
    truth_occ, truth_free = classify_map(truth)
    ours_occ, ours_free = classify_map(ours)
    shift = _grid_shift(ours, truth)
    ours_occ, occ_dropped = align_mask(ours_occ, shift, truth_occ.shape)
    ours_free, free_dropped = align_mask(ours_free, shift, truth_occ.shape)

    cell = truth['resolution'] ** 2
    n_truth_occ = int(np.count_nonzero(truth_occ))
    n_truth_free = int(np.count_nonzero(truth_free))
    n_ours_occ = int(np.count_nonzero(ours_occ))
    n_ours_free = int(np.count_nonzero(ours_free))

    hit = truth_occ & ours_occ
    recall_strict = hit.sum() / n_truth_occ if n_truth_occ else 0.0
    ours_occ_tol = dilate(ours_occ, tol)
    hit_tol = truth_occ & ours_occ_tol
    recall_tol = hit_tol.sum() / n_truth_occ if n_truth_occ else 0.0

    truth_occ_tol = dilate(truth_occ, tol)
    precision_tol = ((ours_occ & truth_occ_tol).sum() / n_ours_occ
                     if n_ours_occ else 0.0)

    # A truth map rasterised with the cell-centre rule puts a wall face that
    # lands exactly on a cell edge half a cell into the wall; any sensor map is
    # then one cell off along that edge.  Report the raw numbers *and* the
    # same numbers with a 1-cell (2.5 cm = one map pixel) tolerance, which is
    # the honest resolution of this comparison.
    missed_free = truth_free & ~ours_free
    extra_free = ours_free & ~truth_free
    missed_free_tol = truth_free & ~dilate(ours_free, 1)
    extra_free_tol = ours_free & ~dilate(truth_free, 1)
    tol_err_cells = int(missed_free_tol.sum()) + int(extra_free_tol.sum())

    result = {
        'truth': {
            'yaml': truth_yaml,
            'width': int(truth_occ.shape[1]),
            'height': int(truth_occ.shape[0]),
            'resolution': truth['resolution'],
            'origin': list(truth['origin']),
            'occupied_cells': n_truth_occ,
            'free_cells': n_truth_free,
            'occupied_area_m2': round(n_truth_occ * cell, 3),
            'free_area_m2': round(n_truth_free * cell, 3),
        },
        'ours': {
            'yaml': ours_yaml,
            'width': int(ours_occ.shape[1]),
            'height': int(ours_occ.shape[0]),
            'occupied_cells': n_ours_occ,
            'free_cells': n_ours_free,
            'occupied_area_m2': round(n_ours_occ * cell, 3),
            'free_area_m2': round(n_ours_free * cell, 3),
            'cells_cropped_occ': occ_dropped,
            'cells_cropped_free': free_dropped,
        },
        'occupancy': {
            'recall_all': round(recall_strict, 4),
            'recall_all_1cell': round(recall_tol, 4),
            'precision_1cell': round(precision_tol, 4),
            'detected_cells': int(hit.sum()),
            'false_occupied_cells': int(n_ours_occ - (ours_occ & truth_occ_tol).sum()),
        },
        'free_area': {
            'truth_m2': round(n_truth_free * cell, 3),
            'ours_m2': round(n_ours_free * cell, 3),
            'err_m2': round(abs(n_ours_free - n_truth_free) * cell, 3),
            'err_pct': round(100.0 * abs(n_ours_free - n_truth_free) / n_truth_free, 2)
                       if n_truth_free else 0.0,
            'missed_m2': round(int(missed_free.sum()) * cell, 3),
            'missed_pct': round(100.0 * missed_free.sum() / n_truth_free, 2)
                          if n_truth_free else 0.0,
            'extra_m2': round(int(extra_free.sum()) * cell, 3),
            'extra_pct': round(100.0 * extra_free.sum() / n_truth_free, 2)
                         if n_truth_free else 0.0,
            # 1-cell tolerance variant (see the comment above result)
            'err_tol_m2': round(tol_err_cells * cell, 3),
            'err_tol_pct': round(100.0 * tol_err_cells / n_truth_free, 2)
                           if n_truth_free else 0.0,
            'missed_tol_m2': round(int(missed_free_tol.sum()) * cell, 3),
            'missed_tol_pct': round(100.0 * missed_free_tol.sum() / n_truth_free, 2)
                              if n_truth_free else 0.0,
            'extra_tol_m2': round(int(extra_free_tol.sum()) * cell, 3),
            'extra_tol_pct': round(100.0 * extra_free_tol.sum() / n_truth_free, 2)
                             if n_truth_free else 0.0,
        },
        'connectivity': {
            'ours': connectivity(ours_free, min_component_cells),
            'truth': connectivity(truth_free, min_component_cells),
        },
    }

    if traj_path and os.path.exists(traj_path):
        with open(traj_path) as handle:
            traj = json.load(handle)
        poses = [(p['sx'], p['sy']) for p in traj.get('poses', [])]
        visible, poses_used = visible_occupied_mask(
            truth_occ, poses, truth['origin'], truth['resolution'],
            max_range=max_range, n_rays=n_rays)
        n_visible = int(np.count_nonzero(visible))
        detected = int((visible & (ours_occ | ours_occ_tol)).sum())
        detected_strict = int((visible & ours_occ).sum())
        result['observable'] = {
            'traj': traj_path,
            'poses_total': len(poses),
            'poses_used': poses_used,
            'visible_truth_occ_cells': n_visible,
            'visible_truth_occ_area_m2': round(n_visible * cell, 3),
            'unobservable_truth_occ_cells': int(n_truth_occ - n_visible),
            'unobservable_pct': round(100.0 * (n_truth_occ - n_visible) / n_truth_occ, 2)
                                if n_truth_occ else 0.0,
            'recall_visible': round(detected_strict / n_visible, 4) if n_visible else 0.0,
            'recall_visible_1cell': round(detected / n_visible, 4) if n_visible else 0.0,
            'detected_visible_cells': detected_strict,
        }

    if diff_path:
        result['diff_image'] = write_diff_image(
            diff_path, truth_occ, truth_free, ours_occ, ours_free)
    return result


def write_diff_image(path, truth_occ, truth_free, ours_occ, ours_free):
    """PPM (P6) difference picture; PNG if Pillow happens to be installed."""
    height, width = truth_occ.shape
    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[...] = (40, 40, 40)                                  # both unknown
    img[truth_free] = (235, 235, 235)                        # truth free
    img[truth_free & ours_free] = (190, 190, 190)            # agree free
    img[truth_occ & ours_occ] = (0, 0, 0)                    # agree occupied
    img[truth_occ & ~ours_occ] = (220, 30, 30)               # missed obstacle
    img[ours_occ & ~truth_occ] = (255, 150, 0)               # false obstacle
    img[truth_free & ~ours_free] = (60, 120, 220)            # missed free
    img = np.flipud(img)                                     # PGM/PPM top-down
    path = os.path.splitext(path)[0]
    try:
        from PIL import Image
        png = path + '.png'
        Image.fromarray(img).save(png)
        return png
    except Exception:
        ppm = path + '.ppm'
        with open(ppm, 'wb') as handle:
            handle.write(b'P6\n%d %d\n255\n' % (width, height))
            handle.write(np.ascontiguousarray(img).tobytes())
        return ppm


def write_json(report, path):
    with open(path, 'w') as handle:
        json.dump(report, handle, indent=2, sort_keys=True)
    return path


def format_report(report):
    """Human-readable table for the console."""
    t, o = report['truth'], report['ours']
    occ = report['occupancy']
    free = report['free_area']
    conn = report['connectivity']
    lines = [
        'truth : %s  %dx%d @ %.3f m' % (t['yaml'], t['width'], t['height'],
                                         t['resolution']),
        'ours  : %s  %dx%d' % (o['yaml'], o['width'], o['height']),
        'truth occupied %d cells (%.2f m2), free %d cells (%.2f m2)'
        % (t['occupied_cells'], t['occupied_area_m2'],
           t['free_cells'], t['free_area_m2']),
        'ours  occupied %d cells (%.2f m2), free %d cells (%.2f m2)'
        % (o['occupied_cells'], o['occupied_area_m2'],
           o['free_cells'], o['free_area_m2']),
        '',
        'occupancy recall  (all truth occ) : %.1f %% strict, %.1f %% +-1 cell'
        % (100 * occ['recall_all'], 100 * occ['recall_all_1cell']),
        'occupancy precision(+-1 cell)     : %.1f %%' % (100 * occ['precision_1cell']),
    ]
    if 'observable' in report:
        vis = report['observable']
        lines += [
            'truth occ visible from trajectory: %d cells (%.2f m2), '
            'unobservable %d (%.1f %%)'
            % (vis['visible_truth_occ_cells'], vis['visible_truth_occ_area_m2'],
               vis['unobservable_truth_occ_cells'], vis['unobservable_pct']),
            'occupancy recall  (visible truth) : %.1f %% strict, %.1f %% +-1 cell'
            % (100 * vis['recall_visible'], 100 * vis['recall_visible_1cell']),
        ]
    lines += [
        'free area  truth %.2f m2  ours %.2f m2  |err| %.2f m2 (%.2f %%)'
        % (free['truth_m2'], free['ours_m2'], free['err_m2'], free['err_pct']),
        '           missed %.2f m2 (%.2f %%)   extra %.2f m2 (%.2f %%)'
        % (free['missed_m2'], free['missed_pct'], free['extra_m2'],
           free['extra_pct']),
        '           same with 1-cell tolerance: err %.2f m2 (%.2f %%), '
        'missed %.2f, extra %.2f'
        % (free['err_tol_m2'], free['err_tol_pct'],
           free['missed_tol_m2'], free['extra_tol_m2']),
        'connectivity ours: %d components >=%d cells, largest %d cells (%.1f %%)'
        % (conn['ours']['components_ge_min'], conn['ours']['min_component_cells'],
           conn['ours']['largest_cells'], 100 * conn['ours']['largest_frac']),
        'connectivity truth: %d components >=%d cells, largest %d cells'
        % (conn['truth']['components_ge_min'], conn['truth']['min_component_cells'],
           conn['truth']['largest_cells']),
    ]
    return '\n'.join(lines)