"""Log-odds 2-D occupancy grid with vectorised Bresenham ray tracing.

Pure numpy + stdlib, no ROS imports: the same module runs on Humble (22.04)
and Jazzy (24.04) and is unit-testable offline.

Grid convention
---------------
``log_odds[row, col]`` with ``row 0 == origin_y`` (minimum world y), i.e. the
array is bottom-up.  PGM files are top-down (row 0 == maximum world y), so
save() and read_pgm()+flip convert once, in exactly one place.

Cell ``(col, row)`` spans ``[origin + i*res, origin + (i+1)*res)`` and its
centre is ``origin + (i + 0.5) * res`` -- the map_server convention and the
rule the rm_field truth map was rasterised with.
"""

import math

import numpy as np

FREE = 0
OCC = 1
UNKNOWN = -1

# map_server defaults, also what the rm_field truth yaml uses
OCCUPIED_THRESH = 0.65
FREE_THRESH = 0.196
PGM_OCC = 0
PGM_FREE = 254
PGM_UNKNOWN = 205


def write_pgm(path, img):
    """Write a 2-D uint8 array as binary PGM (P5, maxval 255)."""
    img = np.ascontiguousarray(img, dtype=np.uint8)
    height, width = img.shape
    with open(path, 'wb') as handle:
        handle.write(b'P5\n%d %d\n255\n' % (width, height))
        handle.write(img.tobytes())


def read_pgm(path):
    """Read a binary or ASCII PGM into a 2-D uint8 array (row 0 == top)."""
    with open(path, 'rb') as handle:
        data = handle.read()
    fields = []
    pos = 0
    size = len(data)
    while len(fields) < 4:
        while pos < size and data[pos:pos + 1].isspace():
            pos += 1
        if pos < size and data[pos:pos + 1] == b'#':
            while pos < size and data[pos:pos + 1] != b'\n':
                pos += 1
            continue
        if pos >= size:
            raise ValueError('%s: truncated PGM header' % path)
        start = pos
        while pos < size and not data[pos:pos + 1].isspace():
            pos += 1
        fields.append(data[start:pos])
    pos += 1
    magic = fields[0]
    width = int(fields[1])
    height = int(fields[2])
    maxval = int(fields[3])
    if maxval > 255:
        raise ValueError('%s: 16-bit PGM not supported' % path)
    if magic == b'P5':
        img = np.frombuffer(data[pos:pos + width * height], dtype=np.uint8)
    elif magic == b'P2':
        img = np.array(data[pos:].split(), dtype=np.uint16)
        if maxval != 255:
            img = (img.astype(np.float32) * 255.0 / maxval).astype(np.uint8)
        img = img.astype(np.uint8)
    else:
        raise ValueError('%s: unsupported PGM magic %r' % (path, magic))
    if img.size != width * height:
        raise ValueError('%s: expected %d px, got %d'
                         % (path, width * height, img.size))
    return img.reshape(height, width)


def _neighbour_count(mask):
    """8-neighbour count per cell (uint8)."""
    padded = np.pad(mask, 1, mode='constant', constant_values=False).astype(np.uint8)
    height, width = mask.shape
    total = np.zeros(mask.shape, dtype=np.uint8)
    for drow in (-1, 0, 1):
        for dcol in (-1, 0, 1):
            if drow or dcol:
                total += padded[1 + drow:1 + drow + height,
                                1 + dcol:1 + dcol + width]
    return total


class OccGrid:
    """Incremental log-odds grid, mapped in the world frame given by origin.

    The grid is fixed (origin/width/height) so the output is directly
    comparable with the truth map; anything outside is dropped.
    """

    def __init__(self, resolution=0.025, origin=(-6.4, -4.4), width=512,
                 height=352, free_miss=-0.4, l_min=-4.0, l_max=4.0,
                 free_exit=-0.6, min_occ_hits=2, block_hits=3,
                 behind_cells=1, end_margin=2, despeckle=1):
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self.width = int(width)
        self.height = int(height)
        self.free_miss = float(free_miss)
        self.l_min = float(l_min)
        self.l_max = float(l_max)
        self.free_exit = float(free_exit)
        # Occupancy is a *hit count*, free space is log-odds: occupied wins.
        # A return inside the height band means something is physically in that
        # cell; a free ray crossing it may simply have passed over a low
        # obstacle (this field's walls are 0.4 m, the lidar at 0.44 m), so free
        # evidence must never be allowed to erase a cell the sensor hit.
        self.min_occ_hits = max(1, int(min_occ_hits))
        self.block_hits = max(self.min_occ_hits, int(block_hits))
        # A lidar only returns the visible surface; the solid behind it is what
        # the truth map rasterised (cell-centre rule).  Marking N cells further
        # along the ray as occupied lines the two conventions up and is the
        # safe side for navigation (walls end up 2.5 cm thicker, not thinner).
        self.behind_cells = int(behind_cells)
        # A ray whose last cells are the ones it is about to hit travels nearly
        # parallel to the surface there (this field's walls are 0.4 m high and
        # the lidar sits at 0.44 m, so almost every wall return is a grazing
        # rim hit).  Excluding the last cells from the free update keeps those
        # rays from painting the very wall they hit as free.
        self.end_margin = max(0, int(end_margin))
        # Drops occupied cells with fewer than N occupied neighbours: a real
        # wall is never a lone 2.5 cm dot, a stray ray return often is.
        self.despeckle = max(0, int(despeckle))
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        self.occ_hits = np.zeros((self.height, self.width), dtype=np.int32)
        self.n_scans = 0
        self.n_points = 0
        self.n_rays = 0

    # ---------------------------------------------------------------- geometry
    def cell_of(self, x, y):
        return (int(math.floor((x - self.origin[0]) / self.resolution)),
                int(math.floor((y - self.origin[1]) / self.resolution)))

    def center_of(self, col, row):
        return (self.origin[0] + (col + 0.5) * self.resolution,
                self.origin[1] + (row + 0.5) * self.resolution)

    def contains(self, col, row):
        return 0 <= col < self.width and 0 <= row < self.height

    # ------------------------------------------------------------- integration
    def add_scan(self, ex, ey, ox, oy):
        """Integrate one scan: rays from (ox, oy) to every endpoint cell.

        Endpoint cells get an occupancy hit (plus ``behind_cells`` cells further
        along the ray: the solid behind the surface), cells strictly before them
        get free evidence.  A ray stops at a cell with ``block_hits`` hits so
        free space is not painted through a wall.  Returns the number of unique
        endpoint cells used.
        """
        ex = np.asarray(ex, dtype=np.float64).ravel()
        ey = np.asarray(ey, dtype=np.float64).ravel()
        if ex.size == 0:
            return 0
        w = self.width
        h = self.height
        res = self.resolution
        c0 = min(max(int(math.floor((ox - self.origin[0]) / res)), 0), w - 1)
        r0 = min(max(int(math.floor((oy - self.origin[1]) / res)), 0), h - 1)

        c1 = np.floor((ex - self.origin[0]) / res).astype(np.int64)
        r1 = np.floor((ey - self.origin[1]) / res).astype(np.int64)
        inside = (c1 >= 0) & (c1 < w) & (r1 >= 0) & (r1 < h)
        c1 = c1[inside]
        r1 = r1[inside]
        if c1.size == 0:
            return 0

        # one ray per occupied cell: 4k points collapse to a few hundred cells
        flat = r1 * w + c1
        uniq = np.unique(flat)
        xe = (uniq % w).astype(np.int32)
        ye = (uniq // w).astype(np.int32)
        n = uniq.size

        xs = np.full(n, c0, dtype=np.int32)
        ys = np.full(n, r0, dtype=np.int32)
        dxn = np.abs(xe - xs).astype(np.int64)
        dyn = -np.abs(ye - ys).astype(np.int64)
        sx = np.where(xe >= xs, 1, -1).astype(np.int32)
        sy = np.where(ye >= ys, 1, -1).astype(np.int32)
        err = dxn + dyn
        alive = np.ones(n, dtype=bool)

        # cells this very scan calls occupied: also block, or the 2-D
        # projection would cut straight through a box that is only 0.4 m high
        frame_occ = np.zeros((h, w), dtype=bool)
        frame_occ[ye, xe] = True
        blocks = (self.occ_hits >= self.block_hits) | frame_occ
        # never block the sensor's own cell, or a robot parked against a wall
        # would stop integrating altogether
        blocks[r0, c0] = False

        occ_flat = []
        free_flat = []
        max_cells = int(max(dxn.max(), np.abs(ye - ys).max())) + 2
        for _ in range(max_cells):
            idx = np.flatnonzero(alive)
            if idx.size == 0:
                break
            cx = xs[idx]
            cy = ys[idx]
            reached = (cx == xe[idx]) & (cy == ye[idx])
            if reached.any():
                hit = idx[reached]
                occ_flat.append(cy[reached].astype(np.int64) * w + cx[reached])
                if self.behind_cells > 0 and hit.size:
                    hx = xs[hit].astype(np.int64)
                    hy = ys[hit].astype(np.int64)
                    h_err = err[hit].astype(np.int64)
                    h_dxn = dxn[hit]
                    h_dyn = dyn[hit]
                    h_sx = sx[hit]
                    h_sy = sy[hit]
                    for _ in range(self.behind_cells):
                        e2b = 2 * h_err
                        step_xb = e2b >= h_dyn
                        step_yb = e2b <= h_dxn
                        hx += np.where(step_xb, h_sx, 0)
                        hy += np.where(step_yb, h_sy, 0)
                        h_err += np.where(step_xb, h_dyn, 0) + np.where(step_yb, h_dxn, 0)
                        okb = (hx >= 0) & (hx < w) & (hy >= 0) & (hy < h)
                        occ_flat.append(hy[okb] * w + hx[okb])
                alive[hit] = False
            cont = idx[~reached]
            if cont.size:
                cxl = xs[cont]
                cyl = ys[cont]
                blocked = blocks[cyl, cxl]
                if self.end_margin > 0:
                    # steps left to this ray's own target cell
                    left = np.maximum(np.abs(xe[cont] - cxl),
                                      np.abs(ye[cont] - cyl))
                    blocked = blocked | (left <= self.end_margin)
                if (~blocked).any():
                    free_flat.append((cyl[~blocked].astype(np.int64) * w
                                      + cxl[~blocked]))
                alive[cont[blocked & (blocks[cyl, cxl])]] = False
            idx = np.flatnonzero(alive)
            if idx.size == 0:
                break
            e2 = 2 * err[idx]
            step_x = e2 >= dyn[idx]
            step_y = e2 <= dxn[idx]
            xs[idx] += np.where(step_x, sx[idx], 0)
            ys[idx] += np.where(step_y, sy[idx], 0)
            err[idx] += np.where(step_x, dyn[idx], 0) + np.where(step_y, dxn[idx], 0)

        size = w * h
        occ_cnt = np.zeros((h, w), dtype=np.int32)
        if occ_flat:
            counts = np.bincount(np.concatenate(occ_flat), minlength=size)
            occ_cnt = np.minimum(counts.reshape(h, w), 3).astype(np.int32)
        if free_flat:
            free_cnt = np.bincount(np.concatenate(free_flat), minlength=size)
            free_cnt = np.minimum(free_cnt.reshape(h, w), 3)
        else:
            free_cnt = np.zeros((h, w), dtype=np.int64)

        # occupied evidence is a counter (never eroded); free evidence is
        # saturating log-odds so a cell observed from many poses becomes free
        self.occ_hits += occ_cnt
        self.log_odds += self.free_miss * free_cnt.astype(np.float32)
        np.clip(self.log_odds, self.l_min, self.l_max, out=self.log_odds)

        self.n_scans += 1
        self.n_points += int(ex.size)
        self.n_rays += int(n)
        return int(n)

    # ------------------------------------------------------------- inspection
    def classify(self):
        """UNKNOWN/FREE/OCC per cell (bottom-up array).  Occupied wins."""
        cls = np.full((self.height, self.width), UNKNOWN, dtype=np.int8)
        cls[self.log_odds <= self.free_exit] = FREE
        occupied = self.occ_hits >= self.min_occ_hits
        if self.despeckle > 0:
            occupied = occupied & (_neighbour_count(occupied) >= self.despeckle)
        cls[occupied] = OCC
        return cls

    def stats(self):
        cls = self.classify()
        cell = self.resolution ** 2
        occ = int(np.count_nonzero(cls == OCC))
        free = int(np.count_nonzero(cls == FREE))
        unk = int(cls.size - occ - free)
        return {
            'scans': self.n_scans,
            'points': self.n_points,
            'rays': self.n_rays,
            'occ_cells': occ,
            'free_cells': free,
            'unknown_cells': unk,
            'occ_area_m2': round(occ * cell, 3),
            'free_area_m2': round(free * cell, 3),
            'unknown_area_m2': round(unk * cell, 3),
        }

    # -------------------------------------------------------------------- save
    def pgm_bytes(self):
        """Top-down uint8 image: 0 occupied, 254 free, 205 unknown."""
        cls = self.classify()
        img = np.full((self.height, self.width), PGM_UNKNOWN, dtype=np.uint8)
        img[cls == FREE] = PGM_FREE
        img[cls == OCC] = PGM_OCC
        return np.flipud(img)

    def save(self, out_base, occupied_thresh=OCCUPIED_THRESH,
             free_thresh=FREE_THRESH, negate=0, comment=''):
        """Write <out_base>.pgm + <out_base>.yaml; returns the stats dict."""
        out_base = out_base[:-4] if out_base.endswith(('.pgm', '.yaml')) else out_base
        base_name = out_base.replace('\\', '/').rsplit('/', 1)[-1]
        pgm_path = out_base + '.pgm'
        yaml_path = out_base + '.yaml'
        write_pgm(pgm_path, self.pgm_bytes())
        lines = [
            '# pcd_to_pgm: %s' % (comment or 'PointCloud2 + TF log-odds map'),
            'image: %s.pgm' % base_name,
            'resolution: %.6g' % self.resolution,
            'origin: [%.6g, %.6g, 0.0]' % (self.origin[0], self.origin[1]),
            'negate: %d' % int(negate),
            'occupied_thresh: %.4g' % occupied_thresh,
            'free_thresh: %.4g' % free_thresh,
        ]
        with open(yaml_path, 'w') as handle:
            handle.write('\n'.join(lines) + '\n')
        return self.stats()