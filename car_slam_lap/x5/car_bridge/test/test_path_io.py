"""轨迹文件读写与几何工具的测试（纯 Python，不依赖 ROS）。

跑法：
    python3 test_path_io.py
"""

from __future__ import annotations

import math
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from car_bridge import path_io as pio  # noqa: E402


def test_round_trip() -> None:
    """record_path 写出去的文件，follow_path 必须能一字不差读回来。"""
    pts = [
        (0.0, 0.0, 0.0, 0.0),
        (1.25, -0.5, math.pi / 2, 1.5),
        (-2.0, 3.0, -math.pi + 0.1, 4.25),
    ]
    with tempfile.TemporaryDirectory() as d:
        f = str(Path(d) / "path.yaml")
        pio.write_path(f, "map", pts, note="测试")
        frame, got = pio.load_path(f)
        assert frame == "map"
        assert len(got) == 3
        for a, b in zip(pts, got):
            assert abs(a[0] - b[0]) < 1e-3
            assert abs(a[1] - b[1]) < 1e-3
            assert abs(a[2] - b[2]) < 1e-3
            assert abs(a[3] - b[3]) < 1e-3


def test_no_tmp_left_behind() -> None:
    """原子写：成功后不该留下 .tmp。"""
    with tempfile.TemporaryDirectory() as d:
        f = str(Path(d) / "p.yaml")
        pio.write_path(f, "odom", [(0, 0, 0, 0), (1, 1, 0, 1)])
        assert not Path(f + ".tmp").exists()


def test_empty_rejected() -> None:
    with tempfile.TemporaryDirectory() as d:
        try:
            pio.write_path(str(Path(d) / "p.yaml"), "map", [])
        except ValueError:
            return
        raise AssertionError("空轨迹应该抛 ValueError")


def test_load_rejects_broken_files() -> None:
    bad_cases = [
        "frame_id: map\n",                       # 没有 points
        "points: []\n",                          # 空列表
        "points:\n  - {y: 1.0}\n",               # 缺点 x
        "- 1\n- 2\n",                            # 顶层不是映射
    ]
    with tempfile.TemporaryDirectory() as d:
        for i, content in enumerate(bad_cases):
            f = str(Path(d) / f"bad{i}.yaml")
            Path(f).write_text(content, encoding="utf-8")
            try:
                pio.load_path(f)
            except ValueError:
                continue
            raise AssertionError(f"坏文件没被拒: {content!r}")


def test_load_accepts_hand_written() -> None:
    """以后想手写路点，格式也得认。"""
    with tempfile.TemporaryDirectory() as d:
        f = str(Path(d) / "hand.yaml")
        Path(f).write_text(
            "# 手写的\nframe_id: map\npoints:\n"
            "  - {x: 0.0, y: 0.0}\n"
            "  - {x: 1.0, y: 0.0, yaw: 0.0}\n",
            encoding="utf-8",
        )
        frame, pts = pio.load_path(f)
        assert frame == "map"
        assert len(pts) == 2
        assert pts[1][0] == 1.0 and pts[1][3] == 0.0


def test_wrap_pi() -> None:
    assert abs(pio.wrap_pi(0.0)) < 1e-12
    # 约定区间是 (-pi, pi]：pi 和 -pi 都留在 +pi
    assert abs(pio.wrap_pi(math.pi) - math.pi) < 1e-12
    assert abs(abs(pio.wrap_pi(-math.pi)) - math.pi) < 1e-12
    # 3pi 折回 pi（同样允许符号）
    assert abs(abs(pio.wrap_pi(3 * math.pi)) - math.pi) < 1e-9
    assert abs(pio.wrap_pi(2 * math.pi + 0.1) - 0.1) < 1e-9
    assert abs(pio.wrap_pi(-2 * math.pi - 0.1) + 0.1) < 1e-9


def test_yaw_from_quaternion() -> None:
    assert abs(pio.yaw_from_quaternion(0, 0, 0, 1)) < 1e-9              # 无旋转
    s = math.sin(math.pi / 4)
    c = math.cos(math.pi / 4)
    assert abs(pio.yaw_from_quaternion(0, 0, s, c) - math.pi / 2) < 1e-9  # 绕 z 转 90°


def test_path_length() -> None:
    assert abs(pio.path_length([(0, 0, 0, 0), (3, 4, 0, 1)]) - 5.0) < 1e-9
    assert pio.path_length([]) == 0.0


def _run_all() -> int:
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f"  ok    {name}")
        except AssertionError as exc:
            failed += 1
            print(f"  FAIL  {name}: {exc}")
    print(f"\n{len(tests) - failed}/{len(tests)} 通过")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(_run_all())
