"""协议层测试（Python 侧）。

和 ``tests/protocol_test.c`` 断言**同一组黄金向量**。两侧任何一边改错
（CRC 算法、字节序、LEN 含义、帧头）都会在对应测试里炸掉。

运行方式（二选一）：
    python3 test_protocol.py            # 直接跑，无需 pytest
    python3 -m pytest test/ -v          # 有 pytest 时
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from car_bridge import protocol as p  # noqa: E402

# ---------------------------------------------------------------- 黄金向量
# 与 protocol_test.c 中 test_golden_vectors() 完全一致
GOLDEN_VEL = [
    ((0, 0), "A55A050100000000EF"),
    ((400, -200), "A55A0501900138FF70"),
    ((-1000, 1000), "A55A050118FCE8034C"),
    ((123, -456), "A55A05017B0038FEF2"),
]
GOLDEN_STATUS = [
    ((p.ST_CAN_OK | p.ST_LINK_OK, p.SRC_SERIAL), "5AA50381030162"),
    ((0x00, p.SRC_NONE), "5AA5038100005A"),
]


def test_crc_external_vector() -> None:
    """CRC-8/ATM 的公开 check 值，是本文件唯一的"外部真值"。"""
    assert p.crc8(b"123456789") == 0xF4
    assert p.crc8(b"") == 0x00


def test_golden_set_vel() -> None:
    for (speed, angular), expected in GOLDEN_VEL:
        got = p.build_set_vel(speed, angular).hex().upper()
        assert got == expected, f"set_vel({speed},{angular}) {got} != {expected}"


def test_golden_status() -> None:
    for (status, src), expected in GOLDEN_STATUS:
        got = p.build_status(status, src).hex().upper()
        assert got == expected, f"status(0x{status:02X},{src}) {got} != {expected}"


def test_clamp() -> None:
    assert p.build_set_vel(2000, -2000) == p.build_set_vel(1000, -1000)
    assert p.build_set_vel(-1234, 567) == p.build_set_vel(-1000, 567)


def test_roundtrip() -> None:
    dec = p.FrameDecoder(p.SOF_DN)
    for speed, angular in [(0, 0), (400, -200), (-1000, 1000), (1000, 1000), (-1, 1)]:
        frames = dec.feed_bytes(p.build_set_vel(speed, angular))
        assert len(frames) == 1, f"({speed},{angular}) 收到 {len(frames)} 帧"
        assert frames[0].cmd == p.CMD_SET_VEL
        assert p.parse_set_vel(frames[0]) == (speed, angular)


def test_resync_after_junk() -> None:
    dec = p.FrameDecoder(p.SOF_DN)
    junk = bytes((0x00, 0x5A, 0x5A, 0xA5, 0xFF, 0x13))
    frames = dec.feed_bytes(junk + p.build_set_vel(123, -456))
    assert len(frames) == 1
    assert p.parse_set_vel(frames[0]) == (123, -456)


def test_bad_crc_rejected() -> None:
    dec = p.FrameDecoder(p.SOF_DN)
    bad = bytearray(p.build_set_vel(500, 500))
    bad[-1] ^= 0xFF
    assert dec.feed_bytes(bytes(bad)) == []
    assert dec.crc_errors == 1

    frames = dec.feed_bytes(p.build_set_vel(321, -654))
    assert len(frames) == 1
    assert p.parse_set_vel(frames[0]) == (321, -654)


def test_half_frame() -> None:
    dec = p.FrameDecoder(p.SOF_DN)
    frame = p.build_set_vel(111, 222)
    assert dec.feed_bytes(frame[:4]) == []
    dec.reset()
    frames = dec.feed_bytes(p.build_set_vel(-777, 0))
    assert len(frames) == 1
    assert p.parse_set_vel(frames[0]) == (-777, 0)


def test_status_frame() -> None:
    dec = p.FrameDecoder(p.SOF_UP)
    frames = dec.feed_bytes(p.build_status(p.ST_CAN_OK | p.ST_RC_OK, p.SRC_RC))
    assert len(frames) == 1
    status, src = p.parse_status(frames[0])
    assert status == (p.ST_CAN_OK | p.ST_RC_OK)
    assert src == p.SRC_RC
    assert p.status_text(status) == ["can_ok", "rc_ok"]


def test_wrong_sof_direction() -> None:
    """下行帧头初始化的解码器不该吃上行帧。"""
    dec = p.FrameDecoder(p.SOF_DN)
    assert dec.feed_bytes(p.build_status(0x03, p.SRC_SERIAL)) == []


def test_units_from_rate() -> None:
    assert p.units_from_rate(0.0, 0.8) == 0
    assert p.units_from_rate(0.8, 0.8) == 1000
    assert p.units_from_rate(0.4, 0.8) == 500
    assert p.units_from_rate(-0.8, 0.8) == -1000
    assert p.units_from_rate(99.0, 0.8) == 1000  # 钳位


def _run_all() -> int:
    tests = [(name, fn) for name, fn in sorted(globals().items())
             if name.startswith("test_") and callable(fn)]
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
