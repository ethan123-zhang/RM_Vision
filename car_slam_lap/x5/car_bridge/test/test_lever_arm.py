"""力臂标定测试：纯几何反解 + 采集状态机，不需要 ROS。

跑法：
    python test_lever_arm.py
    python -m unittest test_lever_arm.py -v
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from car_bridge import lever_arm as la  # noqa: E402
from car_bridge.path_io import wrap_pi  # noqa: E402


def _rotate(offset, yaw, dyaw):
    """传感器点绕转心的位移：Δs = R(yaw)·(R(dyaw)-I)·offset。"""
    ox, oy = offset
    c, s = math.cos(yaw), math.sin(yaw)
    cd, sd = math.cos(dyaw), math.sin(dyaw)
    # R(dyaw)-I
    ax, ay = (cd - 1.0) * ox - sd * oy, sd * ox + (cd - 1.0) * oy
    return c * ax - s * ay, s * ax + c * ay


def _poses(offset, yaw1, dyaw, start=(1.0, -2.0)):
    x1, y1 = start
    dsx, dsy = _rotate(offset, yaw1, dyaw)
    pose1 = (x1, y1, yaw1)
    pose2 = (x1 + dsx, y1 + dsy, wrap_pi(yaw1 + dyaw))
    return pose1, pose2


class SolveOffsetTest(unittest.TestCase):
    def test_recovers_offset_from_180_degree_rotation(self) -> None:
        # 180° 是条件最好的标定动作：Δs = -2R(yaw1)·offset。
        for yaw1 in (0.0, 0.7, -2.5, math.pi / 2):
            with self.subTest(yaw1=yaw1):
                pose1, pose2 = _poses((0.25, -0.10), yaw1, math.pi)
                ox, oy = la.solve_offset(pose1, pose2)
                self.assertAlmostEqual(ox, 0.25, places=9)
                self.assertAlmostEqual(oy, -0.10, places=9)

    def test_recovers_offset_from_partial_rotation(self) -> None:
        for dyaw in (math.radians(75.0), 1.2, -2.0, math.radians(140.0)):
            with self.subTest(dyaw=dyaw):
                pose1, pose2 = _poses((0.18, 0.05), 0.3, dyaw)
                ox, oy = la.solve_offset(pose1, pose2)
                self.assertAlmostEqual(ox, 0.18, places=9)
                self.assertAlmostEqual(oy, 0.05, places=9)

    def test_zero_offset_stays_zero(self) -> None:
        pose1, pose2 = _poses((0.0, 0.0), 1.0, math.pi)
        self.assertEqual(la.solve_offset(pose1, pose2), (0.0, 0.0))

    def test_rejects_rotation_below_minimum(self) -> None:
        pose1, pose2 = _poses((0.25, 0.0), 0.0, math.radians(30.0))
        with self.assertRaises(ValueError) as ctx:
            la.solve_offset(pose1, pose2)
        self.assertIn("至少要 60°", str(ctx.exception))

    def test_rejects_non_finite_pose(self) -> None:
        with self.assertRaises(ValueError):
            la.solve_offset((float("nan"), 0.0, 0.0), (0.0, 0.0, math.pi))
        with self.assertRaises(ValueError):
            la.solve_offset((0.0, 0.0, 0.0), (0.0, float("inf"), math.pi))

    def test_wrap_pi_accepts_yaw_beyond_pi(self) -> None:
        # 转过去又转回来，yaw2 写成 +2π 之外的等价角也必须得到同样的解。
        pose1, pose2 = _poses((0.2, 0.1), 0.4, math.pi)
        ox, oy = la.solve_offset(pose1, (pose2[0], pose2[1], pose2[2] + 2 * math.pi))
        self.assertAlmostEqual(ox, 0.2, places=9)
        self.assertAlmostEqual(oy, 0.1, places=9)


class FormatTest(unittest.TestCase):
    def test_param_args_format(self) -> None:
        text = la.offset_param_args(0.25, -0.1)
        self.assertEqual(
            text, "-p base_offset_x:=0.2500 -p base_offset_y:=-0.1000")

    def test_warns_on_implausibly_large_offset(self) -> None:
        warnings = la.sanity_warnings(0.9, 0.0)
        self.assertTrue(any("过大" in w or "平移" in w for w in warnings), warnings)

    def test_warns_on_near_zero_offset(self) -> None:
        warnings = la.sanity_warnings(0.001, 0.0)
        self.assertTrue(any("接近 0" in w for w in warnings), warnings)

    def test_plausible_offset_has_no_warnings(self) -> None:
        self.assertEqual(la.sanity_warnings(0.22, -0.05), [])

    def test_result_lines_contain_params_and_radius(self) -> None:
        # 节点只负责搬这几行字，格式在纯模块里测，别让节点逻辑无法单测。
        lines = la.result_lines(0.24, -0.07)
        self.assertTrue(any("base_offset_x:=0.2400" in line for line in lines), lines)
        self.assertTrue(any("base_offset_y:=-0.0700" in line for line in lines), lines)
        self.assertTrue(any("0.250" in line for line in lines), lines)
        self.assertTrue(any("follow_path" in line for line in lines), lines)


class LeverArmCaptureTest(unittest.TestCase):
    """三段式采集：静止起点 -> 原地转 ≥120° -> 停稳后再采第二个位姿。"""

    def _feed(self, capture, pose, now):
        return capture.feed(pose, now)

    def test_completes_after_rotation_and_settle(self) -> None:
        capture = la.LeverArmCapture(settle_s=1.0)
        pose1, pose2 = _poses((0.25, 0.0), 0.0, math.pi)
        t = 0.0
        # 起点静置 1 s 才采第一个位姿
        while t < 1.0:
            self.assertIsNone(self._feed(capture, pose1, t))
            t += 0.1
        result = self._feed(capture, pose1, t)
        self.assertIsNone(result)          # 采到起点后开始等旋转
        self.assertEqual(capture.state, "rotating")
        # 半路转动（未到 120°）不给结果
        mid = (pose1[0], pose1[1], pose1[2] + math.radians(60.0))
        t += 0.1
        self.assertIsNone(self._feed(capture, mid, t))
        # 转到 180° 后停稳 1 s 出结果
        t += 0.1
        self.assertIsNone(self._feed(capture, pose2, t))
        self.assertEqual(capture.state, "settle")
        t += 0.1                       # 旋转后第一个静止帧，开始计时
        self.assertIsNone(self._feed(capture, pose2, t))
        t += 0.9                       # 还没停够 1 s
        self.assertIsNone(self._feed(capture, pose2, t))
        t += 0.1                       # 正好 1 s
        ox, oy = self._feed(capture, pose2, t)
        self.assertAlmostEqual(ox, 0.25, places=6)
        self.assertAlmostEqual(oy, 0.0, places=6)
        self.assertEqual(capture.state, "done")

    def test_motion_during_settle_restarts_timer(self) -> None:
        capture = la.LeverArmCapture(settle_s=1.0)
        pose1, pose2 = _poses((0.2, 0.0), 0.0, math.pi)
        t = 0.0
        while capture.state == "wait_first":
            self.assertIsNone(self._feed(capture, pose1, t))
            t += 0.1
        self.assertIsNone(self._feed(capture, pose2, t))   # 转到阈値 -> settle
        t += 0.1
        self.assertIsNone(self._feed(capture, pose2, t))   # 静止计时开始
        # 停稳一段时间后又挪开再挪回来：必须重新计时，不能提前给结果
        t += 0.5
        moved = (pose2[0] + 0.05, pose2[1], pose2[2])
        self.assertIsNone(self._feed(capture, moved, t))
        t += 0.1
        self.assertIsNone(self._feed(capture, pose2, t))   # 挪回原位
        t += 0.1
        self.assertIsNone(self._feed(capture, pose2, t))   # 重新静止计时
        t += 0.9
        self.assertIsNone(self._feed(capture, pose2, t))   # 0.9 s，还不够
        t += 0.1
        ox, oy = self._feed(capture, pose2, t)             # 1.0 s，出结果
        self.assertAlmostEqual(ox, 0.2, places=6)
        self.assertAlmostEqual(oy, 0.0, places=6)

    def test_insufficient_rotation_never_completes(self) -> None:
        capture = la.LeverArmCapture(settle_s=0.5)
        pose1 = (0.0, 0.0, 0.0)
        t = 0.0
        while capture.state == "wait_first":
            self.assertIsNone(self._feed(capture, pose1, t))
            t += 0.1
        # 只转 30° 就停住：一直等
        turned = (0.0, 0.0, math.radians(30.0))
        for _ in range(50):
            t += 0.1
            self.assertIsNone(self._feed(capture, turned, t))
        self.assertEqual(capture.state, "rotating")

    def test_progress_text_reports_rotation(self) -> None:
        capture = la.LeverArmCapture(settle_s=0.2)
        pose1 = (0.0, 0.0, 0.0)
        t = 0.0
        while capture.state == "wait_first":
            self.assertIsNone(self._feed(capture, pose1, t))
            t += 0.1
        capture.feed(pose1, t)
        capture.feed((0.0, 0.0, math.radians(70.0)), t + 0.1)
        self.assertIn("70", capture.progress_text())


if __name__ == "__main__":
    unittest.main()