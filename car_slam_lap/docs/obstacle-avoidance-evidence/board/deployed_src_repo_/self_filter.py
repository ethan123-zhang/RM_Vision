"""只屏蔽经车体转动验证的雷达自身回波体素，不屏蔽整个近距离扇区。"""
from pathlib import Path
import math
import numpy as np


class SelfMask:
    def __init__(self, path):
        with np.load(Path(path).expanduser(), allow_pickle=False) as data:
            resolution = float(data['resolution'])
            raw_keys = np.asarray(data['keys'])
        if not math.isfinite(resolution) or not .005 <= resolution <= .03:
            raise ValueError('自回波掩码分辨率必须在 5–30 mm 内')
        if raw_keys.ndim != 2 or raw_keys.shape[1] != 3 or not 0 < len(raw_keys) <= 100000:
            raise ValueError('自回波掩码必须是非空 Nx3 体素坐标')
        if not np.isfinite(raw_keys).all() or not np.equal(raw_keys, np.floor(raw_keys)).all():
            raise ValueError('自回波体素坐标必须是有限整数')
        keys = raw_keys.astype(np.int64)
        self.resolution = resolution
        self.low = keys.min(axis=0) - 1
        high = keys.max(axis=0) + 1
        shape = high - self.low + 1
        if np.any(shape > 200) or np.prod(shape) > 2000000:
            raise ValueError('自回波掩码必须局限在车体附近')
        self.volume = np.zeros(tuple(shape), dtype=bool)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    shifted = keys - self.low + (dx, dy, dz)
                    self.volume[shifted[:, 0], shifted[:, 1], shifted[:, 2]] = True

    def filter(self, points):
        pts = np.asarray(points)
        finite = np.isfinite(pts).all(axis=1)
        pts = pts[finite]
        keys = np.floor(pts / self.resolution).astype(np.int64) - self.low
        inside = (keys >= 0).all(axis=1) & (keys < self.volume.shape).all(axis=1)
        indices = np.flatnonzero(inside)
        k = keys[inside]
        remove = np.zeros(len(pts), dtype=bool)
        remove[indices] = self.volume[k[:, 0], k[:, 1], k[:, 2]]
        return pts[~remove]
