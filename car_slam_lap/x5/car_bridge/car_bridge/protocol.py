"""X5 <-> C8T6 串口链路协议 —— Python 侧实现。

与固件 ``firmware/C8T6_DT7_car/Hardware/LINK/protocol.c`` 是**同一套字节语义**，
两侧测试断言同一组"黄金向量"（见 ``test/test_protocol.py``）。

帧格式（LEN 含 CMD 字节）：

    下行 X5 -> C8T6  50 Hz  9 字节
    A5 5A | LEN=05 | CMD=01 | speed i16LE | angular i16LE | CRC8

    上行 C8T6 -> X5  10 Hz  7 字节
    5A A5 | LEN=03 | CMD=81 | status | src | CRC8

CRC-8/ATM：poly 0x07 / init 0x00 / 不反转 / xorout 0，覆盖 LEN 及其后 LEN 个字节。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

# `from __future__ import annotations` 让上面的 tuple[int, int] 写法在 3.8 也能解析；
# X5 上是 Python 3.10，这句是给 PC 上的旧解释器兜底。

# ---------------------------------------------------------------- 常量
SOF_DN: Tuple[int, int] = (0xA5, 0x5A)
SOF_UP: Tuple[int, int] = (0x5A, 0xA5)

MAX_PAYLOAD = 8
MIN_LEN = 2

CMD_SET_VEL = 0x01
CMD_STATUS = 0x81

SPEED_MAX = 1000
FRAME_DN_LEN = 9
FRAME_UP_LEN = 7

#: status 位
ST_CAN_OK = 0x01
ST_LINK_OK = 0x02
ST_RC_OK = 0x04
ST_ESTOP = 0x08

#: 控制源
SRC_NONE = 0
SRC_SERIAL = 1
SRC_RC = 2
SRC_NAMES = {SRC_NONE: "none", SRC_SERIAL: "serial", SRC_RC: "rc"}


# ---------------------------------------------------------------- 基础
def crc8(data: bytes) -> int:
    """CRC-8/ATM。外部校验向量：crc8(b"123456789") == 0xF4。"""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def clamp_units(value: float) -> int:
    """把任意浮点指令钳到 [-1000, 1000] 的整数。"""
    return int(max(-SPEED_MAX, min(SPEED_MAX, round(value))))


# ---------------------------------------------------------------- 组帧
def build_set_vel(speed: int, angular: int) -> bytes:
    """组下行速度帧（9 字节）。超量程自动钳位。"""
    speed = clamp_units(speed)
    angular = clamp_units(angular)
    body = bytes((5, CMD_SET_VEL)) + speed.to_bytes(2, "little", signed=True) + \
        angular.to_bytes(2, "little", signed=True)
    return bytes(SOF_DN) + body + bytes((crc8(body),))


def build_status(status: int, src: int) -> bytes:
    """组上行状态帧（7 字节）。X5 侧只在测试/仿真里用。"""
    body = bytes((3, CMD_STATUS, status & 0xFF, src & 0xFF))
    return bytes(SOF_UP) + body + bytes((crc8(body),))


# ---------------------------------------------------------------- 解帧
@dataclass(frozen=True)
class Frame:
    """一帧完整报文。payload 不含 CMD。"""

    cmd: int
    payload: bytes

    def __len__(self) -> int:  # type: ignore[override]
        return len(self.payload)


def parse_set_vel(frame: Frame) -> Tuple[int, int]:
    """返回 (speed, angular)；帧类型不对时抛 ValueError。"""
    if frame.cmd != CMD_SET_VEL or len(frame.payload) != 4:
        raise ValueError(f"不是 SET_VEL 帧: cmd=0x{frame.cmd:02X} len={len(frame.payload)}")
    speed = int.from_bytes(frame.payload[0:2], "little", signed=True)
    angular = int.from_bytes(frame.payload[2:4], "little", signed=True)
    return speed, angular


def parse_status(frame: Frame) -> Tuple[int, int]:
    """返回 (status, src)。"""
    if frame.cmd != CMD_STATUS or len(frame.payload) != 2:
        raise ValueError(f"不是 STATUS 帧: cmd=0x{frame.cmd:02X} len={len(frame.payload)}")
    return frame.payload[0], frame.payload[1]


def status_text(status: int) -> List[str]:
    """把人看的标志位列表（日志/JSON 用）。"""
    out = []
    if status & ST_CAN_OK:
        out.append("can_ok")
    if status & ST_LINK_OK:
        out.append("link_ok")
    if status & ST_RC_OK:
        out.append("rc_ok")
    if status & ST_ESTOP:
        out.append("estop")
    return out


# ---------------------------------------------------------------- 收帧
class FrameDecoder:
    """逐字节收帧状态机，语义与 protocol.c 的 proto_rx_feed() 一一对应。

    串口读线程里反复调用 ``feed(byte)``，收到完整且 CRC 正确的帧时返回 Frame。
    """

    def __init__(self, sof: Tuple[int, int] = SOF_UP) -> None:
        self.sof0, self.sof1 = sof
        self._buf = bytearray(3 + MAX_PAYLOAD + 1)
        self._idx = 0
        self._len = 0
        self.crc_errors = 0
        self.frames_ok = 0

    def reset(self) -> None:
        self._idx = 0
        self._len = 0

    def feed(self, byte: int) -> Optional[Frame]:
        idx = self._idx

        if idx == 0:
            if byte == self.sof0:
                self._buf[0] = byte
                self._idx = 1
            return None

        if idx == 1:
            if byte == self.sof1:
                self._buf[1] = byte
                self._idx = 2
            elif byte == self.sof0:
                # 0xA5 0xA5 → 第二个仍可能是帧头
                self._buf[0] = byte
            else:
                self.reset()
            return None

        if idx == 2:
            if byte < MIN_LEN or byte > MAX_PAYLOAD:
                self.reset()
                return None
            self._buf[2] = byte
            self._len = byte
            self._idx = 3
            return None

        need = 3 + self._len
        if idx < need:
            self._buf[idx] = byte
            self._idx = idx + 1
            return None

        # 当前字节是 CRC
        if crc8(bytes(self._buf[2:need])) == byte:
            cmd = self._buf[3]
            payload = bytes(self._buf[4:need])
            self.reset()
            self.frames_ok += 1
            return Frame(cmd=cmd, payload=payload)

        self.crc_errors += 1
        self.reset()
        return None

    def feed_bytes(self, data: bytes) -> List[Frame]:
        return [f for f in (self.feed(b) for b in data) if f is not None]


# ---------------------------------------------------------------- 单位换算
def units_from_rate(rate: float, at_max: float, limit: int = SPEED_MAX) -> int:
    """把 m/s 或 rad/s 换算成 [-limit, limit] 的指令单位。

    ``at_max`` 是"指令打满 = 多少物理量"，由 calibrate_speed 实测，不拍脑袋。
    """
    if at_max <= 0:
        raise ValueError("at_max 必须 > 0")
    return int(max(-limit, min(limit, round(rate / at_max * limit))))
