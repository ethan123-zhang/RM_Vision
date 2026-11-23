"""CAD occupancy grid (map_server yaml + pgm) -> distance field in metres.

Only numpy + the standard library are imported at module level; scipy is
optional. When ``scipy.ndimage`` is importable the field is the exact Euclidean
distance transform, otherwise a numpy 8-neighbour wavefront is used (radial bias
below ~3 %, reported in ``backend``). No ROS imports, so the same code runs on
Humble (22.04 / py3.10) and Jazzy (24.04 / py3.12) and in the offline self test.

Coordinates are the map_server convention: cell ``(col, row)`` of the pgm
(top-down) spans ``[origin + i*res, origin + (i+1)*res)`` with its centre at
``origin + (i + 0.5) * res``, so the world-to-array row index flips.
"""

import json
import os

import numpy as np

try:  # scipy gives the exact EDT; numpy fallback is good enough if absent
    from scipy import ndimage as _ndimage
except Exception:  # pragma: no cover - exercised only without scipy
    _ndimage = None


def read_pgm(path):
    """Read a binary/ASCII PGM. Returns (width, height, array[row=top])."""
    with open(path, 'rb') as handle:
        data = handle.read()
    tokens = []
    pos, size = 0, len(data)
    while len(tokens) < 4:
        while pos < size and data[pos:pos + 1].isspace():
            pos += 1
        if pos < size and data[pos:pos + 1] == b'#':
            while pos < size and data[pos:pos + 1] not in (b'\n', b''):
                pos += 1
            continue
        if pos >= size:
            raise ValueError('%s: truncated PGM header' % path)
        start = pos
        while pos < size and not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    magic = tokens[0].decode('ascii', 'replace')
    width, height, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic not in ('P5', 'P2'):
        raise ValueError('%s: only P5/P2 PGM supported, got %r' % (path, magic))
    if maxval > 255:
        raise ValueError('%s: 16-bit PGM not supported' % path)
    pos += 1
    if magic == 'P5':
        pixels = np.frombuffer(data[pos:pos + width * height], dtype=np.uint8)
    else:
        pixels = np.array(data[pos:].split()[:width * height], dtype=np.uint8)
    if pixels.size < width * height:
        raise ValueError('%s: pixel data truncated' % path)
    return width, height, pixels.reshape(height, width)


def read_map_yaml(path):
    """Flat ``key: value`` map_server yaml -> dict (no PyYAML dependency)."""
    out = {}
    with open(path, 'r') as handle:
        for raw in handle:
            line = raw.split('#')[0].strip()
            if ':' not in line:
                continue
            key, value = line.split(':', 1)
            out[key.strip()] = value.strip().strip('"\'')
    if 'resolution' not in out or 'origin' not in out:
        raise ValueError('%s: not a map_server yaml (resolution/origin missing)'
                         % path)
    out['resolution'] = float(out['resolution'])
    out['origin'] = [float(v) for v in json.loads(out['origin'])]
    out['occupied_thresh'] = float(out.get('occupied_thresh', 0.65))
    out['free_thresh'] = float(out.get('free_thresh', 0.196))
    out['negate'] = int(out.get('negate', 0))
    return out


class DistanceField(object):
    """Distance (m) from any planar point to the nearest occupied CAD cell."""

    def __init__(self, occ_world, res, origin, cap_m=2.0, pad_m=0.25,
                 backend='auto'):
        self.res = float(res)
        self.x0 = float(origin[0]) - float(pad_m)
        self.y0 = float(origin[1]) - float(pad_m)
        self.cap = float(cap_m)
        pad_cells = int(np.ceil(float(pad_m) / self.res))
        padded = np.pad(np.asarray(occ_world, dtype=bool), pad_cells,
                        mode='constant', constant_values=False)
        self.shape = padded.shape
        if backend == 'numpy':
            self.backend = 'numpy-wavefront'
            field = _wavefront_distance(padded, self.res, self.cap)
        elif _ndimage is not None or backend == 'auto':
            if _ndimage is None:  # pragma: no cover
                raise RuntimeError('scipy backend requested but not importable')
            self.backend = 'scipy-edt'
            field = _ndimage.distance_transform_edt(~padded,
                                                    sampling=self.res)
            field = np.minimum(field, self.cap)
        else:  # pragma: no cover
            raise RuntimeError('unreachable backend selection')
        self.field = np.ascontiguousarray(field, dtype=np.float32)
        self.height, self.width = self.field.shape

    # -- sampling ---------------------------------------------------------
    def _coords(self, xy):
        xy = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
        col = (xy[:, 0] - self.x0) / self.res - 0.5
        row = (xy[:, 1] - self.y0) / self.res - 0.5
        return row, col

    def sample_nearest(self, xy):
        """Nearest-cell lookup (fast, used by the coarse grid search)."""
        row, col = self._coords(xy)
        ri = np.clip(np.rint(row), 0, self.height - 1).astype(np.int32)
        ci = np.clip(np.rint(col), 0, self.width - 1).astype(np.int32)
        return self.field[ri, ci]

    def sample_bilinear(self, xy):
        """Bilinear lookup; what the fine search and the residual use."""
        row, col = self._coords(xy)
        inside = ((row >= 0.0) & (row <= self.height - 1) &
                  (col >= 0.0) & (col <= self.width - 1))
        r = np.clip(row, 0.0, self.height - 1.000001)
        c = np.clip(col, 0.0, self.width - 1.000001)
        r0 = np.floor(r).astype(np.int32)
        c0 = np.floor(c).astype(np.int32)
        fr = (r - r0).astype(np.float32)
        fc = (c - c0).astype(np.float32)
        r1 = np.minimum(r0 + 1, self.height - 1)
        c1 = np.minimum(c0 + 1, self.width - 1)
        f = self.field
        top = f[r0, c0] * (1.0 - fc) + f[r0, c1] * fc
        bottom = f[r1, c0] * (1.0 - fc) + f[r1, c1] * fc
        out = top * (1.0 - fr) + bottom * fr
        return np.where(inside, out, self.cap).astype(np.float32)


def _wavefront_distance(occ, res, cap_m):
    """8-neighbour wavefront distance, clipped at cap_m (no scipy needed)."""
    steps = int(np.ceil(cap_m / res)) + 1
    cap_cells = float(cap_m) / res
    dist = np.where(occ, 0.0, cap_cells).astype(np.float32)
    # 1, sqrt(2) and 2 * sqrt(2) offsets keep the radial bias small
    offsets = [(0, 1, 1.0), (1, 0, 1.0), (1, 1, 1.41421356),
               (1, -1, 1.41421356), (0, 2, 2.0), (2, 0, 2.0),
               (2, 2, 2.82842712), (2, -2, 2.82842712)]
    for _ in range(steps):
        nxt = dist.copy()
        for dr, dc, weight in offsets:
            for sign in (1, -1):
                shifted = np.roll(dist, (sign * dr, sign * dc), axis=(0, 1))
                np.minimum(nxt, shifted + weight, out=nxt)
        nxt = np.minimum(nxt, cap_cells)
        if np.array_equal(nxt, dist):
            break
        dist = nxt
    return np.minimum(dist * res, cap_m)


def surface_band(occ):
    """Cells on either side of a free/occupied edge (the CAD surfaces).

    A distance field over this band has its zero set exactly on the walls, so a
    lidar return on a wall face scores ~0 and the optimiser is not rewarded for
    sliding the cloud *into* the occupied area -- that is what a plain
    nearest-occupied-cell field does (half a cell out, so the estimate trades a
    sideways slide for a rotation and comes out ~0.5 deg yaw biased).
    """
    occ = np.asarray(occ, dtype=bool)
    p = np.pad(occ, 1, mode='constant', constant_values=False)
    any_occ = (p[1:-1, :-2] | p[1:-1, 2:] | p[:-2, 1:-1] | p[2:, 1:-1])
    all_occ = (p[1:-1, :-2] & p[1:-1, 2:] & p[:-2, 1:-1] & p[2:, 1:-1])
    return (occ & ~all_occ) | (~occ & any_occ)


class CadGrid(object):
    """The CAD prior as occupied cells + a distance field, in field coordinates."""

    def __init__(self, yaml_path, backend='auto', cap_m=2.0, pad_m=0.5):
        meta = read_map_yaml(yaml_path)
        image = meta.get('image')
        if not image:
            raise ValueError('%s: no image: entry' % yaml_path)
        if not os.path.isabs(image):
            image = os.path.join(os.path.dirname(os.path.abspath(yaml_path)),
                                 image)
        width, height, pixels = read_pgm(image)
        self.yaml_path = yaml_path
        self.image_path = image
        self.res = meta['resolution']
        self.origin = meta['origin'][:2]
        prob = pixels.astype(np.float32) / 255.0
        if meta['negate']:
            prob = 1.0 - prob
        # map_server: occupied when occ_prob < 1-occupied_thresh
        self.occ_top = prob <= (1.0 - meta['occupied_thresh'])
        self.free_top = prob >= (1.0 - meta['free_thresh'])
        # world-oriented copy: row 0 == minimum y
        self.occ = self.occ_top[::-1].copy()
        self.width, self.height = width, height
        rows, cols = np.nonzero(self.occ)
        self.cell_x = self.origin[0] + (cols + 0.5) * self.res
        self.cell_y = self.origin[1] + (rows + 0.5) * self.res
        # objective field: distance to the CAD surface (free/occupied edge)
        self.band = surface_band(self.occ)
        self.field = DistanceField(self.band, self.res, self.origin,
                                   cap_m=cap_m, pad_m=pad_m, backend=backend)
        # diagnostic field: distance to the nearest occupied cell (the classic
        # "how far is this return from an obstacle" number)
        self.occ_field = DistanceField(self.occ, self.res, self.origin,
                                       cap_m=cap_m, pad_m=pad_m, backend=backend)

    @property
    def n_occupied(self):
        return int(self.cell_x.size)

    @property
    def n_surface_cells(self):
        return int(np.count_nonzero(self.band))

    @property
    def backend(self):
        return self.field.backend

    def distance(self, xy):
        """Distance (m) from map-frame points to the nearest CAD surface."""
        return self.field.sample_bilinear(xy)

    def occupied_distance(self, xy):
        return self.occ_field.sample_bilinear(xy)

    def residual_stats(self, points_xy):
        d = self.distance(points_xy)
        occ = self.occupied_distance(points_xy)
        stats = {
            'mean_m': float(np.mean(d)) if d.size else None,
            'median_m': float(np.median(d)) if d.size else None,
            'p90_m': float(np.percentile(d, 90)) if d.size else None,
            'inlier_ratio_010m': (float(np.mean(d <= 0.10)) if d.size else None),
            'inlier_ratio_005m': (float(np.mean(d <= 0.05)) if d.size else None),
            'occupied_mean_m': (float(np.mean(occ)) if occ.size else None),
            'occupied_p90_m': (float(np.percentile(occ, 90)) if occ.size else None),
        }
        return stats

    def describe(self):
        return {
            'resolution': self.res,
            'origin': [float(self.origin[0]), float(self.origin[1])],
            'width': int(self.width), 'height': int(self.height),
            'occupied_cells': self.n_occupied,
            'surface_cells': self.n_surface_cells,
            'objective_field': self.field.backend,
        }