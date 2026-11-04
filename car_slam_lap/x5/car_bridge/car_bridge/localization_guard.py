"""路线参考系与原始里程计的安全门槛。

**不依赖 ROS**，所以能在 PC 上直接单测（见 test/test_localization_guard.py）。

为什么单独一层：
Odin1 驱动会缓存 odom->imu 并持续重发 TF，重发时还会刷新 TF 时间戳。
因此 tf2 查得到、而且看起来很"新鲜"，并不代表原始 /odin1/odometry 还在更新。
follow_path 必须直接盯住原始 Odometry 的 header.stamp 和**主机 monotonic 接收时间**，
不能拿设备 stamp 去减主机墙钟（两者没有共同原点）。
"""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

#: follow_path 默认只接受 map 系路线。
MAP_FRAME = "map"
#: 历史示教数据（demo/venue2/path.yaml）用的 odom 系，只有显式授权才能跑。
LEGACY_FRAME = "odom"

#: Odin1 原始里程计的约定帧名（已核实：frame=odom，child=imu）。
ODOM_FRAME = "odom"
ODOM_CHILD = "imu"

#: 四元数范数低于这个值就当作非法姿态。
_QUAT_NORM_EPS = 1e-9


class RouteFrameError(ValueError):
    """路线文件的参考系不被允许。"""


def check_route_frame(frame_id: str, allow_legacy_odom: bool) -> Tuple[str, bool]:
    """校验路线参考系，返回 ``(frame, is_legacy)``。

    - ``map``：默认接受；
    - ``odom``：只有 ``allow_legacy_odom=True`` 才接受，且调用方必须警告
      "仅供已人工对齐的历史诊断，不代表跨会话安全"；
    - 空 / 未知参考系：一律拒绝。
    """
    frame = "" if frame_id is None else str(frame_id).strip()
    if frame == MAP_FRAME:
        return frame, False
    if frame == LEGACY_FRAME:
        if allow_legacy_odom:
            return frame, True
        raise RouteFrameError(
            f"路线参考系是历史 {LEGACY_FRAME}，默认拒绝：跨会话重定位后 "
            f"{LEGACY_FRAME} 与建图时的坐标不再一致。确需复现历史数据，显式设置 "
            "allow_legacy_odom:=true（仅供已人工对齐的历史诊断，不代表跨会话安全）"
        )
    shown = frame if frame else "空"
    raise RouteFrameError(
        f"路线参考系为 {shown}，只接受 {MAP_FRAME}；"
        f"历史 {LEGACY_FRAME} 路线需显式 allow_legacy_odom:=true"
    )


class OdomGuard:
    """原始 Odometry 的帧名、数值与时间推进检查。

    "有效接收时间"只在 **stamp 严格递增** 时刷新：
    重复、零 stamp 都不续命；倒退直接故障且不自动恢复。
    帧名/数值/四元数非法会**锁存故障**——不能让一条坏消息之后
    继续拿旧的有效状态发非零指令。位置不变但 stamp 正常推进是有效的。
    """

    def __init__(self, timeout_s: float = 0.5,
                 expected_frame: str = ODOM_FRAME,
                 expected_child: str = ODOM_CHILD) -> None:
        self.timeout_s = float(timeout_s)
        self.expected_frame = str(expected_frame)
        self.expected_child = str(expected_child)

        self.last_stamp: Optional[float] = None
        self.last_rx: Optional[float] = None
        self.faulted = False
        self.fault_reason = ""
        self.last_reject = ""
        self.accepted = 0
        self.rejected = 0

    # ---------------------------------------------------------- 状态
    @property
    def ready(self) -> bool:
        """是否已经收到过至少一帧合格数据并建立了时间基线。"""
        return self.last_rx is not None

    def age(self, now: float) -> Optional[float]:
        """距最后一次有效接收的 monotonic 秒数（从未收到返回 None）。"""
        if self.last_rx is None:
            return None
        return max(0.0, float(now) - self.last_rx)

    def is_fresh(self, now: float) -> bool:
        if self.faulted or self.last_rx is None:
            return False
        return (float(now) - self.last_rx) <= self.timeout_s

    # ---------------------------------------------------------- 输入
    def update(self, now: float,
               stamp_sec: int, stamp_nanosec: int,
               frame_id: str, child_frame_id: str,
               position: Sequence[float],
               orientation: Sequence[float]) -> bool:
        """校验一帧原始 Odometry；合格且时间推进时刷新有效接收时间。

        返回 True 表示这一帧可以续命；False 表示被拒（原因见 ``last_reject``，
        时间倒退则见 ``faulted`` / ``fault_reason``）。
        """
        if self.faulted:
            return False

        frame = "" if frame_id is None else str(frame_id)
        child = "" if child_frame_id is None else str(child_frame_id)
        if frame != self.expected_frame:
            return self._fault(
                f"frame_id={frame!r} 与约定 {self.expected_frame!r} 不符"
            )
        if child != self.expected_child:
            return self._fault(
                f"child_frame_id={child!r} 与约定 {self.expected_child!r} 不符"
            )

        try:
            pos = [float(value) for value in position]
            ori = [float(value) for value in orientation]
            sec = float(stamp_sec)
            nsec = float(stamp_nanosec)
        except (TypeError, ValueError):
            return self._fault("字段不是数字")
        if len(pos) != 3:
            return self._fault(f"位置应有 3 个分量，收到 {len(pos)} 个")
        if len(ori) != 4:
            return self._fault(f"四元数应有 4 个分量，收到 {len(ori)} 个")
        if not all(math.isfinite(value) for value in pos):
            return self._fault("位置不是有限数（NaN/Inf）")
        if not all(math.isfinite(value) for value in ori):
            return self._fault("四元数不是有限数（NaN/Inf）")

        norm = math.sqrt(sum(value * value for value in ori))
        if norm <= _QUAT_NORM_EPS:
            return self._fault("四元数范数为 0，姿态非法")

        stamp = sec + nsec * 1e-9
        if not math.isfinite(stamp):
            return self._fault("时间戳不是有限数（NaN/Inf）")
        if stamp <= 0.0:
            return self._reject("时间戳为 0/非法，不刷新有效接收时间")

        if self.last_stamp is None:
            # 首次非零 stamp 建立基线。
            self.last_stamp = stamp
            self.last_rx = float(now)
            self.accepted += 1
            return True

        if stamp == self.last_stamp:
            return self._reject("时间戳重复，不刷新有效接收时间")
        if stamp < self.last_stamp:
            return self._fault(
                f"原始里程计时间戳倒退：{stamp:.6f} < 上次 {self.last_stamp:.6f}；"
                "为安全起见不自动恢复运动"
            )

        self.last_stamp = stamp
        self.last_rx = float(now)
        self.accepted += 1
        return True

    # ---------------------------------------------------------- 内部
    def _fault(self, reason: str) -> bool:
        """非法数据锁存为故障：之后不再拿旧的有效状态输出非零指令。"""
        self.faulted = True
        self.fault_reason = reason
        return self._reject(reason)

    def _reject(self, reason: str) -> bool:
        self.last_reject = reason
        self.rejected += 1
        return False
