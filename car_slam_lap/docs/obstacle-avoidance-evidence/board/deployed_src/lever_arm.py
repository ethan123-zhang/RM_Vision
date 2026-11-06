"""力臂标定：由一次原地旋转前后的位姿反解 `base_offset_x/y`。

为什么需要它：位姿来自装在车上的传感器（Odin1 / `base_frame=imu`），它与车体
转心不重合。实测转 77° 位姿点扫过 0.32 m，等效力臂 0.19–0.25 m。控制器发的
是转心的 `cmd_vel`，判据却用传感器点，于是"位置在起点"与"朝向对齐"互相排斥。

约定（与 `follow_path` 一致）：传感器位姿 = 转心 + R(yaw)·offset。
原地转 Δyaw 时传感器点位移
    Δs = R(yaw1)·(R(Δyaw) − I)·offset
解这个 2×2 线性方程即得 offset。转 180° 条件最好（Δs = −2R(yaw1)·offset），
最少也要转 60°，否则矩阵接近奇异、结果不可信。

标定步骤（车放地面、周围 1 m 空旷、已重定位）：
    ros2 run car_bridge calibrate_lever_arm
    # 等它提示"可以开始转"后，原地把车转 180°，停稳 1 s
    # 输出：-p base_offset_x:=... -p base_offset_y:=...
再把这两个参数加到 `follow_path` 的启动参数里（默认 0 = 不补偿）。

本模块不依赖 ROS，几何和采集状态机都能在 PC 上单测（test/test_lever_arm.py）。
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

from .path_io import wrap_pi

#: 低于这个旋转角的解不可信（矩阵接近奇异）。
MIN_ROTATION_RAD = math.radians(60.0)
#: 反解结果的可信范围（m）：超出通常说明车不是原地转，而是被拖着走了。
MAX_PLAUSIBLE_OFFSET_M = 0.60
MIN_PLAUSIBLE_OFFSET_M = 0.02
#: 判定"这一帧是静止的"的阈值（相邻两帧位移 / 转角）。
STATIC_POS_EPS = 0.01
STATIC_YAW_EPS = 0.02


def _checked_pose(pose, name: str) -> Tuple[float, float, float]:
    try:
        x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
    except (TypeError, IndexError) as exc:
        raise ValueError(f"{name} 必须是 (x, y, yaw)，收到 {pose!r}") from exc
    if not all(math.isfinite(value) for value in (x, y, yaw)):
        raise ValueError(f"{name} 必须都是有限数，收到 {pose!r}")
    return x, y, yaw


def solve_offset(pose1, pose2) -> Tuple[float, float]:
    """由原地旋转前后的两个 (x, y, yaw) 反解传感器相对转心的偏移。"""
    x1, y1, yaw1 = _checked_pose(pose1, "pose1")
    x2, y2, yaw2 = _checked_pose(pose2, "pose2")
    dyaw = wrap_pi(yaw2 - yaw1)
    if abs(dyaw) < MIN_ROTATION_RAD:
        raise ValueError(
            f"两次位姿只转了 {math.degrees(abs(dyaw)):.1f}°，"
            f"至少要 {math.degrees(MIN_ROTATION_RAD):.0f}°（转 180° 最准）；"
            "请原地多转一些再采第二个位姿"
        )
    cos1, sin1 = math.cos(yaw1), math.sin(yaw1)
    cosd, sind = math.cos(dyaw), math.sin(dyaw)
    # M = R(yaw1)·(R(dyaw) − I)，解 M·offset = Δs
    a11 = cos1 * (cosd - 1.0) - sin1 * sind
    a12 = -cos1 * sind - sin1 * (cosd - 1.0)
    a21 = sin1 * (cosd - 1.0) + cos1 * sind
    a22 = -sin1 * sind + cos1 * (cosd - 1.0)
    det = a11 * a22 - a12 * a21
    if abs(det) < 1e-12:
        raise ValueError(
            f"旋转 {math.degrees(abs(dyaw)):.1f}° 的方程退化，解不出来；"
            "请原地转 180° 再试"
        )
    dsx, dsy = x2 - x1, y2 - y1
    offset_x = (a22 * dsx - a12 * dsy) / det
    offset_y = (-a21 * dsx + a11 * dsy) / det
    return offset_x, offset_y


def offset_param_args(offset_x: float, offset_y: float) -> str:
    """输出可直接粘到 ros2 run 后面的参数。"""
    return f"-p base_offset_x:={offset_x:.4f} -p base_offset_y:={offset_y:.4f}"


def sanity_warnings(offset_x: float, offset_y: float) -> List[str]:
    """给出结果的可信度提示；空列表表示看起来合理。"""
    radius = math.hypot(offset_x, offset_y)
    warnings: List[str] = []
    if radius > MAX_PLAUSIBLE_OFFSET_M:
        warnings.append(
            f"反解出的力臂 {radius:.2f} m 过大（> {MAX_PLAUSIBLE_OFFSET_M:.2f} m）："
            "车可能不是原地转（被拖着走了或轮子打滑），或定位发生了跳变；"
            "请在空旷处原地重新标定"
        )
    elif radius < MIN_PLAUSIBLE_OFFSET_M:
        warnings.append(
            f"反解出的力臂 {radius:.3f} m 接近 0（< {MIN_PLAUSIBLE_OFFSET_M:.2f} m）："
            "可能传感器本来就装在转心附近，也可能两次位姿采在了相近角度；"
            "若实车确实偏离转心，请在空旷处重新标一次"
        )
    return warnings


def result_lines(offset_x: float, offset_y: float) -> List[str]:
    """标定结果的输出文本（节点只负责打印，格式在这里单测）。"""
    radius = math.hypot(offset_x, offset_y)
    params = offset_param_args(offset_x, offset_y)
    return [
        f"# 力臂 offset = ({offset_x:+.4f}, {offset_y:+.4f}) m，模长 {radius:.3f} m",
        f"# follow_path 启动参数：{params}",
        "# 例：ros2 run car_bridge follow_path --ros-args \\",
        f"#     -p path_file:=... {params}",
    ]


class LeverArmCapture:
    """三段式采集状态机：静止起点 → 原地转够角度 → 停稳后采第二点。

    `feed(pose, now)` 在拿到结果的那一帧返回 `(offset_x, offset_y)`，其余返回 None。
    只要轨迹不满足条件（没转够、转完又没停稳），它就一直等，不会给半成品。
    """

    def __init__(self, settle_s: float = 1.0,
                 min_rotation_rad: float = math.radians(120.0)) -> None:
        if not math.isfinite(settle_s) or settle_s <= 0.0:
            raise ValueError(f"settle_s 必须大于 0，收到 {settle_s!r}")
        if not math.isfinite(min_rotation_rad) or min_rotation_rad <= 0.0:
            raise ValueError(
                f"min_rotation_rad 必须大于 0，收到 {min_rotation_rad!r}")
        self.settle_s = settle_s
        self.min_rotation_rad = min_rotation_rad
        self.state = "wait_first"
        self.pose1: Optional[Tuple[float, float, float]] = None
        self.result: Optional[Tuple[float, float]] = None
        self._last: Optional[Tuple[float, float, float]] = None
        self._static_since: Optional[float] = None

    # ---------------------------------------------------------------- 采集
    def feed(self, pose, now: float) -> Optional[Tuple[float, float]]:
        if self.state == "done":
            return self.result
        x, y, yaw = _checked_pose(pose, "pose")
        if self._last is None:
            moved, turned = 0.0, 0.0
        else:
            moved = math.hypot(x - self._last[0], y - self._last[1])
            turned = abs(wrap_pi(yaw - self._last[2]))
        self._last = (x, y, yaw)
        static = moved < STATIC_POS_EPS and turned < STATIC_YAW_EPS
        if not static:
            self._static_since = None
        elif self._static_since is None:
            self._static_since = now

        if self.state == "wait_first":
            if self._static_since is not None and now - self._static_since >= self.settle_s:
                self.pose1 = (x, y, yaw)
                self.state = "rotating"
            return None

        if self.state == "rotating":
            rotation = abs(wrap_pi(yaw - self.pose1[2]))
            if rotation >= self.min_rotation_rad:
                self.state = "settle"
                self._static_since = None
            return None

        # settle：停稳够久才认第二个位姿
        if self._static_since is None or now - self._static_since < self.settle_s:
            return None
        self.result = solve_offset(self.pose1, (x, y, yaw))
        self.state = "done"
        return self.result

    # ---------------------------------------------------------------- 提示
    def progress_text(self) -> str:
        if self.state == "wait_first":
            return "等待静止起点：请把车放好、别动"
        if self.state == "rotating":
            rotation = 0.0
            if self._last is not None and self.pose1 is not None:
                rotation = abs(wrap_pi(self._last[2] - self.pose1[2]))
            return (
                f"已转 {math.degrees(rotation):.0f}°，"
                f"继续原地转到 ≥ {math.degrees(self.min_rotation_rad):.0f}°"
                f"（180° 最准），然后停稳"
            )
        if self.state == "settle":
            return "已转够角度，等待停稳后再采第二个位姿"
        return "标定完成"