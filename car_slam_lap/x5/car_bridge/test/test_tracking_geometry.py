"""跟踪几何（纯 Python，不依赖 ROS）的单元测试。

这些用例只验证"沿路线弧长"的进度/目标/窗口规则本身；follow_path 的安全门、
状态机和 cmd_vel 行为在 test_follow_safety.py 里用完整 _tick 验证。

跑法：
    python -m unittest discover -s x5/car_bridge/test -p 'test_tracking_geometry.py' -v
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from car_bridge.tracking_geometry import (  # noqa: E402
    cumulative_arc_length,
    limit_progress_index,
    nearest_index,
    target_index_by_arc,
)

#: "U 形"折返：回程与去程只隔 0.1 m，但同一空间位置的弧长差很远。
U_SHAPE_X = [0.0, 0.2, 0.4, 0.4, 0.2, 0.0]
U_SHAPE_Y = [0.0, 0.0, 0.0, 0.1, 0.1, 0.1]
#: s = [0.0, 0.2, 0.4, 0.5, 0.7, 0.9]


class ArcLengthTest(unittest.TestCase):
    def test_duplicate_points_keep_arc_length_flat(self) -> None:
        s = cumulative_arc_length([0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 0.0, 0.0])
        self.assertEqual(s, [0.0, 0.0, 0.0, 1.0])

    def test_arc_length_matches_polyline_length(self) -> None:
        s = cumulative_arc_length(U_SHAPE_X, U_SHAPE_Y)
        self.assertAlmostEqual(s[2], 0.4, places=9)
        self.assertAlmostEqual(s[3], 0.5, places=9)
        self.assertAlmostEqual(s[-1], 0.9, places=9)


class TargetIndexTest(unittest.TestCase):
    def test_target_follows_arc_length_not_euclidean_distance(self) -> None:
        # 车在 index 1（0.2, 0）朝 +x：回程点 index 4（0.2, 0.1）在空间上很近，
        # 但按弧长前视 0.35 m 应落在 index 4，而不是再往后的 index 5。
        s = cumulative_arc_length(U_SHAPE_X, U_SHAPE_Y)
        self.assertEqual(target_index_by_arc(s, 1, 0.35), 4)

    def test_remaining_shorter_than_lookahead_uses_last_point(self) -> None:
        s = cumulative_arc_length(U_SHAPE_X, U_SHAPE_Y)
        self.assertEqual(target_index_by_arc(s, 2, 0.35), len(s) - 1)

    def test_duplicate_arc_positions_still_pick_forward_index(self) -> None:
        s = [0.0, 0.0, 0.1, 0.1, 0.1, 0.4]
        target = target_index_by_arc(s, 1, 0.25)
        self.assertGreater(target, 1)
        self.assertGreaterEqual(s[target], 0.25)

    def test_zero_lookahead_never_goes_backwards(self) -> None:
        s = [0.0, 0.1, 0.1, 0.2]
        self.assertEqual(target_index_by_arc(s, 2, 0.0), 2)


class NearestIndexWindowTest(unittest.TestCase):
    def test_window_excludes_spatially_close_far_branch(self) -> None:
        s = cumulative_arc_length(U_SHAPE_X, U_SHAPE_Y)
        # 位姿正好在回程点 (0.2, 0.1)（index 4）上，但当前进度在 index 1；
        # 回程点比局部窗口（0.3 m）远得多，不能被吸过去。
        index, distance = nearest_index(
            s, U_SHAPE_X, U_SHAPE_Y, 0.2, 0.1, 1,
            back_m=0.3, forward_m=0.3)
        self.assertLess(index, 4)
        self.assertAlmostEqual(distance, 0.1, places=9)
        self.assertLessEqual(s[index] - s[1], 0.3 + 1e-9)

    def test_sparse_route_can_advance_to_next_point(self) -> None:
        # 点距 1 m > 窗口 0.3 m：窗口至少纳入下一个点，否则进度永远推不动。
        s = [0.0, 1.0, 2.0]
        px = [0.0, 1.0, 2.0]
        py = [0.0, 0.0, 0.0]
        index, distance = nearest_index(
            s, px, py, 0.6, 0.0, 0, back_m=0.3, forward_m=0.3)
        self.assertEqual(index, 1)
        self.assertAlmostEqual(distance, 0.4, places=9)

    def test_nearest_may_return_lower_index_for_lateral_offset(self) -> None:
        s = cumulative_arc_length(U_SHAPE_X, U_SHAPE_Y)
        index, _distance = nearest_index(
            s, U_SHAPE_X, U_SHAPE_Y, 0.05, 0.0, 2,
            back_m=0.3, forward_m=0.3)
        self.assertLess(index, 2)


class LimitProgressTest(unittest.TestCase):
    def test_progress_never_regresses(self) -> None:
        s = cumulative_arc_length(U_SHAPE_X, U_SHAPE_Y)
        self.assertEqual(limit_progress_index(s, 3, 1, 0.5, 0.1), 3)
        self.assertEqual(limit_progress_index(s, 3, 3, 0.5, 0.1), 3)

    def test_progress_capped_by_mileage_plus_slack(self) -> None:
        s = cumulative_arc_length(U_SHAPE_X, U_SHAPE_Y)
        # 实际里程 0.2 m + 余量 0.1 m = 0.3 m；index 2 的 s=0.4 已经超了，
        # 封顶只能停在当前进度 index 1，不能跳到远处的回程段。
        capped = limit_progress_index(s, 1, 5, 0.2, 0.1)
        self.assertEqual(capped, 1)
        self.assertLessEqual(s[capped], 0.2 + 0.1 + 1e-9)

    def test_forward_candidate_within_mileage_is_accepted(self) -> None:
        s = cumulative_arc_length(U_SHAPE_X, U_SHAPE_Y)
        self.assertEqual(limit_progress_index(s, 1, 3, 0.5, 0.1), 3)

    def test_duplicate_arc_positions_do_not_break_cap(self) -> None:
        s = [0.0, 0.0, 0.0, 0.5]
        self.assertEqual(limit_progress_index(s, 0, 2, 0.0, 0.1), 2)
        self.assertEqual(limit_progress_index(s, 2, 3, 0.1, 0.1), 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
