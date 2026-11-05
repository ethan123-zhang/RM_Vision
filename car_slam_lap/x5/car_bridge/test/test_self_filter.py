import sys
import tempfile
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
try:
    from car_bridge.self_filter import SelfMask
except ImportError:
    SelfMask = None


class SelfFilterTest(unittest.TestCase):
    def test_only_calibrated_voxels_removed_new_obstacle_kept(self):
        self.assertIsNotNone(SelfMask, '需要确认自身回波的三维掩码')
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'self.npz'
            # 标定自回波在雷达前15cm、下10cm；附近以外的箱体不能被整段距离滤掉。
            np.savez(path, resolution=.015, keys=np.array([[10, 0, -7]]))
            mask = SelfMask(str(path))
            points = np.array([[.15, 0., -.10], [.30, 0., -.10], [.15, .15, -.10], [1., 0., 0.]])
            np.testing.assert_equal(mask.filter(points), points[1:])

    def test_missing_or_unbounded_mask_is_rejected(self):
        self.assertIsNotNone(SelfMask)
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'self.npz'
            with self.assertRaises(FileNotFoundError):
                SelfMask(str(path))
            np.savez(path, resolution=.015, keys=np.array([[0, 0, 0], [10000, 0, 0]]))
            with self.assertRaises(ValueError):
                SelfMask(str(path))


if __name__ == '__main__':
    unittest.main()
