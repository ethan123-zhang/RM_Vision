"""PointCloud2 -> numpy, TF -> 4x4, and the height / range filters.

Deliberately no sensor_msgs_py: the field layout is read straight from
PointCloud2.fields, which behaves the same on Humble and Jazzy (sensor_msgs_py
changed its read_points() return type between distros).
"""

import math

import numpy as np

# sensor_msgs/PointField datatype -> numpy dtype
_DTYPES = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
           5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}

_XYZ = ('x', 'y', 'z')


def pointcloud2_xyz(msg):
    """Return an (N, 3) float32 array of XYZ from a PointCloud2 message."""
    n = int(msg.width) * int(msg.height)
    step = int(msg.point_step)
    if n == 0 or step <= 0:
        return np.zeros((0, 3), dtype=np.float32)
    offsets = {}
    for field in msg.fields:
        offsets[field.name] = (int(field.offset), int(field.datatype))
    for name in _XYZ:
        if name not in offsets:
            raise ValueError('PointCloud2 has no %r field (fields: %s)'
                             % (name, sorted(offsets)))
    raw = np.frombuffer(msg.data, dtype=np.uint8)[:n * step].reshape(n, step)
    cols = []
    for name in _XYZ:
        offset, datatype = offsets[name]
        np_dtype = _DTYPES.get(datatype)
        if np_dtype is None:
            raise ValueError('unsupported PointField datatype %d for %r'
                             % (datatype, name))
        itemsize = np.dtype(np_dtype).itemsize
        if offset + itemsize > step:
            raise ValueError('field %r overruns point_step %d' % (name, step))
        col = raw[:, offset:offset + itemsize].copy().view(np_dtype)
        cols.append(col.reshape(-1).astype(np.float32))
    return np.stack(cols, axis=1)


def quat_to_matrix(q):
    """Quaternion (x, y, z, w) -> 3x3 rotation matrix."""
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


def transform_points(xyz, translation, rotation_matrix):
    """Apply p_map = R * p_frame + t to an (N, 3) array."""
    if xyz.shape[0] == 0:
        return xyz
    return (rotation_matrix @ xyz.astype(np.float64).T).T + translation


def filter_points(xyz_map, ray_xy, ground_z=0.0, z_min=0.10, z_max=1.20,
                  min_range=0.35, max_range=10.0, max_points=4000):
    """Height band + horizontal range + stride downsample.

    ``xyz_map`` is already in the map frame, so the height band is
    ``ground_z + z_min .. ground_z + z_max``: the map frame's z axis is
    gravity-aligned (Odin1 gives map_T_sensor, ground is z = 0).
    """
    if xyz_map.shape[0] == 0:
        return xyz_map
    finite = np.isfinite(xyz_map).all(axis=1)
    pts = xyz_map[finite]
    z = pts[:, 2]
    keep = (z >= ground_z + z_min) & (z <= ground_z + z_max)
    dx = pts[:, 0] - ray_xy[0]
    dy = pts[:, 1] - ray_xy[1]
    rng = np.hypot(dx, dy)
    keep &= (rng >= min_range) & (rng <= max_range)
    pts = pts[keep]
    if pts.shape[0] > max_points and max_points > 0:
        stride = int(math.ceil(pts.shape[0] / float(max_points)))
        pts = pts[::stride]
    return pts