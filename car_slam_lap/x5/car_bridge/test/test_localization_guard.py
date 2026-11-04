"""原始里程计门槛与路线参考系的单元测试（纯 Python，不依赖 ROS）。

跑法：
    python test_localization_guard.py
    python -m unittest discover -s x5/car_bridge/test -p 'test_localization_guard.py' -v

这一层刻意不 import rclpy：它只认 header.stamp / frame_id / child_frame_id
和单调钟接收时间，因此能在 PC 上把"TF 新鲜但原始位姿停更"的坑逐条钉死。
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from car_bridge.localization_guard import (  # noqa: E402
    LEGACY_FRAME,
    MAP_FRAME,
    ODOM_CHILD,
    ODOM_FRAME,
    OdomGuard,
    RouteFrameError,
    check_route_frame,
)


def _stamp_parts(stamp: float) -> tuple[int, int]:
    """把方便书写的秒数拆成 ROS 的 sec/nanosec。"""
    sec = int(math.floor(stamp))
    nsec = int(round((stamp - sec) * 1e9))
    return sec, nsec


class RouteFrameTest(unittest.TestCase):
    """路线参考系：默认只认 map，odom 必须显式授权。"""

    def test_map_route_accepted(self) -> None:
        frame, legacy = check_route_frame(MAP_FRAME, allow_legacy_odom=False)
        self.assertEqual(frame, MAP_FRAME)
        self.assertFalse(legacy)

    def test_legacy_odom_rejected_by_default(self) -> None:
        with self.assertRaises(RouteFrameError) as ctx:
            check_route_frame(LEGACY_FRAME, allow_legacy_odom=False)
        message = str(ctx.exception)
        self.assertIn("allow_legacy_odom", message)
        self.assertIn("跨会话", message)

    def test_legacy_odom_accepted_only_when_explicit(self) -> None:
        frame, legacy = check_route_frame("  odom  ", allow_legacy_odom=True)
        self.assertEqual(frame, LEGACY_FRAME)
        self.assertTrue(legacy)

    def test_unknown_or_empty_frame_rejected(self) -> None:
        for frame in ("world", "base_link", "", "   ", None):
            with self.subTest(frame=frame):
                with self.assertRaises(RouteFrameError):
                    check_route_frame(frame, allow_legacy_odom=True)

    def test_error_does_not_contain_silent_default(self) -> None:
        with self.assertRaises(RouteFrameError) as ctx:
            check_route_frame("", allow_legacy_odom=False)
        self.assertIn("参考系", str(ctx.exception))


class OdomGuardTest(unittest.TestCase):
    """原始 Odometry 的帧名、数值与时间推进检查。"""

    def setUp(self) -> None:
        self.guard = OdomGuard(timeout_s=0.5)
        self.now = 10.0

    def _update(
        self,
        stamp: float = 1.0,
        *,
        now: float | None = None,
        frame: str = ODOM_FRAME,
        child: str = ODOM_CHILD,
        position: tuple = (0.0, 0.0, 0.0),
        orientation: tuple = (0.0, 0.0, 0.0, 1.0),
    ) -> bool:
        sec, nsec = _stamp_parts(stamp)
        return self.guard.update(
            self.now if now is None else now,
            sec,
            nsec,
            frame,
            child,
            position,
            orientation,
        )

    # ---------------------------------------------------------- 时间推进
    def test_first_nonzero_stamp_establishes_baseline(self) -> None:
        self.assertFalse(self.guard.ready)
        self.assertTrue(self._update(1.0))
        self.assertTrue(self.guard.ready)
        # 只用接收 monotonic 时间算新鲜度，不拿设备 stamp 和主机墙钟相减。
        self.assertTrue(self.guard.is_fresh(10.5))
        self.assertFalse(self.guard.is_fresh(10.5001))

    def test_zero_stamp_never_becomes_baseline(self) -> None:
        self.assertFalse(self._update(0.0))
        self.assertFalse(self.guard.ready)
        self.assertIn("时间戳", self.guard.last_reject)

    def test_zero_stamp_does_not_refresh_after_baseline(self) -> None:
        self.assertTrue(self._update(1.0, now=10.0))
        self.assertFalse(self._update(0.0, now=10.4))
        self.assertFalse(self.guard.faulted)   # 零 stamp 不刷新，但锁存交给超时
        self.assertFalse(self.guard.is_fresh(10.6))

    def test_repeated_stamp_does_not_refresh(self) -> None:
        self.assertTrue(self._update(1.0, now=10.0))
        self.assertFalse(self._update(1.0, now=10.4))
        self.assertIn("重复", self.guard.last_reject)
        # 重复 stamp 只按停更阈值处理，不锁存成故障
        self.assertFalse(self.guard.faulted)
        self.assertFalse(self.guard.is_fresh(10.6))

    def test_backward_stamp_faults_without_auto_recovery(self) -> None:
        self.assertTrue(self._update(2.0, now=10.0))
        self.assertFalse(self._update(1.9, now=10.1))
        self.assertTrue(self.guard.faulted)
        self.assertIn("倒退", self.guard.fault_reason)
        # 倒退之后即使来了更新鲜的时间戳，也不允许恢复运动。
        self.assertFalse(self._update(2.5, now=10.2))
        self.assertTrue(self.guard.faulted)
        self.assertFalse(self.guard.is_fresh(10.2))

    def test_non_finite_stamp_faults(self) -> None:
        self.assertTrue(self._update(1.0, now=10.0))
        self.assertFalse(self.guard.update(
            10.1, float("nan"), 0, ODOM_FRAME, ODOM_CHILD,
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
        ))
        self.assertTrue(self.guard.faulted)
        self.assertIn("时间戳", self.guard.fault_reason)

    def test_stationary_pose_with_advancing_stamp_is_valid(self) -> None:
        pose = (0.0, 0.0, 0.0)
        self.assertTrue(self._update(1.0, now=10.0, position=pose))
        self.assertTrue(self._update(1.1, now=10.2, position=pose))
        self.assertTrue(self._update(1.2, now=10.4, position=pose))
        self.assertTrue(self.guard.is_fresh(10.4))
        self.assertEqual(self.guard.accepted, 3)

    def test_monotonic_now_is_used(self) -> None:
        self.assertTrue(self._update(1.0, now=10.0))
        self.assertAlmostEqual(self.guard.age(10.2), 0.2, places=9)

    # ---------------------------------------------------------- 数值/帧
    def test_non_finite_values_rejected(self) -> None:
        for bad in (float("nan"), float("inf"), float("-inf")):
            with self.subTest(bad=bad):
                guard = OdomGuard(timeout_s=0.5)
                sec, nsec = _stamp_parts(1.0)
                accepted = guard.update(
                    10.0, sec, nsec, ODOM_FRAME, ODOM_CHILD,
                    (bad, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
                )
                self.assertFalse(accepted)
                self.assertFalse(guard.ready)
                self.assertTrue(guard.faulted)
                self.assertIn("有限", guard.last_reject)

    def test_non_finite_quaternion_rejected(self) -> None:
        self.assertFalse(self._update(1.0, orientation=(0.0, 0.0, float("nan"), 1.0)))
        self.assertFalse(self.guard.ready)
        self.assertTrue(self.guard.faulted)

    def test_zero_norm_quaternion_rejected(self) -> None:
        self.assertFalse(self._update(1.0, orientation=(0.0, 0.0, 0.0, 0.0)))
        self.assertFalse(self.guard.ready)
        self.assertTrue(self.guard.faulted)
        self.assertIn("四元数", self.guard.last_reject)

    def test_wrong_frame_or_child_rejected(self) -> None:
        cases = [
            ("map", ODOM_CHILD),
            (ODOM_FRAME, "base_link"),
            ("", ODOM_CHILD),
            (ODOM_FRAME, ""),
        ]
        for frame, child in cases:
            with self.subTest(frame=frame, child=child):
                guard = OdomGuard(timeout_s=0.5)
                sec, nsec = _stamp_parts(1.0)
                accepted = guard.update(
                    10.0, sec, nsec, frame, child,
                    (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
                )
                self.assertFalse(accepted)
                self.assertFalse(guard.ready)
                self.assertTrue(guard.faulted)
                self.assertIn("不符", guard.last_reject)

    def test_rejected_message_latches_fault(self) -> None:
        # 先有一帧有效数据，再收到 NaN：不能继续拿旧的有效状态续命。
        self.assertTrue(self._update(1.0, now=10.0))
        self.assertFalse(self._update(1.1, now=10.1, position=(float("nan"), 0.0, 0.0)))
        self.assertTrue(self.guard.faulted)
        self.assertFalse(self.guard.is_fresh(10.1))
        # 之后再来了格式正确的递增帧也不恢复（故障锁存）。
        self.assertFalse(self._update(9.2, now=10.2))

    def test_rejected_message_does_not_refresh_baseline(self) -> None:
        self.assertTrue(self._update(1.0, now=10.0))
        self.assertFalse(self._update(1.1, now=10.4, position=(float("nan"), 0.0, 0.0)))
        self.assertTrue(self.guard.faulted)
        self.assertFalse(self.guard.is_fresh(10.6))


if __name__ == "__main__":
    unittest.main(verbosity=2)
