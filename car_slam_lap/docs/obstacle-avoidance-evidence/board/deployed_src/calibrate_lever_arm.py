"""calibrate_lever_arm —— 原地转车 180°，反解 Odin1 相对转心的力臂。

为什么需要：位姿来自 `base_frame`（Odin1 的 imu 系），它偏离车体转心约
0.19–0.25 m。`follow_path` 的起点判据与回起点目标都该用**转心**位姿，
但 offset 只能实测——这台车上装在哪、朝哪边偏，靠猜会错符号。

本节点：只读 TF，**不发任何 cmd_vel**。流程是"静止采第一个位姿 → 人原地转
180° → 停稳 1 s 采第二个位姿 → 反解 → 打印启动参数"。转不够 120° 不给结果。

用法（车已重定位、周围 1 m 空旷、S2 保持可用急停、手/遥控转车）：
    ros2 run car_bridge calibrate_lever_arm
    # 若传感器不是 imu 或想改停稳时间：
    ros2 run car_bridge calibrate_lever_arm --ros-args \
        -p base_frame:=imu -p settle_s:=1.5

输出示例：
    -p base_offset_x:=0.2400 -p base_offset_y:=-0.0700
再把这两个参数加到 follow_path 的启动参数里（默认 0 = 不补偿）。
"""

from __future__ import annotations

import math
import signal
import sys
import time
from contextlib import suppress
from typing import Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

import tf2_ros

from .lever_arm import LeverArmCapture, result_lines, sanity_warnings
from .path_io import yaw_from_quaternion


class CalibrateLeverArm(Node):
    def __init__(self) -> None:
        super().__init__("calibrate_lever_arm")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "imu")
        self.declare_parameter("settle_s", 1.0)
        self.declare_parameter("min_rotation_deg", 120.0)
        self.declare_parameter("tf_timeout_s", 30.0)

        self.frame: str = str(self.get_parameter("map_frame").value)
        self.base_frame: str = str(self.get_parameter("base_frame").value)
        settle_s = float(self.get_parameter("settle_s").value)
        min_rotation_deg = float(self.get_parameter("min_rotation_deg").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout_s").value)
        self._capture = LeverArmCapture(
            settle_s=settle_s,
            min_rotation_rad=math.radians(min_rotation_deg),
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._done = False
        self._outcome = ""
        self._t0 = time.monotonic()
        self._last_status = 0.0
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"力臂标定：等待 TF {self.frame} -> {self.base_frame}。"
            "本节点不发任何 cmd_vel——请按提示手动/遥控把车原地转约 180°"
        )

    def _pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self._tf_buffer.lookup_transform(
                self.frame, self.base_frame, rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        try:
            qx, qy, qz, qw = float(q.x), float(q.y), float(q.z), float(q.w)
            values = (float(t.x), float(t.y), float(t.z), qx, qy, qz, qw)
        except (TypeError, ValueError):
            return None
        if not all(math.isfinite(value) for value in values):
            return None
        if math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) <= 1e-9:
            return None
        yaw = yaw_from_quaternion(qx, qy, qz, qw)
        if not math.isfinite(yaw):
            return None
        return values[0], values[1], yaw

    def _tick(self) -> None:
        if self._done:
            return
        now = time.monotonic()
        pose = self._pose()
        if pose is None:
            if now - self._t0 > self.tf_timeout:
                self._fail(
                    f"等待 TF {self.frame} -> {self.base_frame} 超时"
                    f"（{self.tf_timeout:.0f} s）：重定位成功了吗？"
                )
            return
        result = self._capture.feed(pose, now)
        if result is not None:
            self._finish(*result)
            return
        if now - self._last_status >= 2.0:
            self._last_status = now
            self.get_logger().info(self._capture.progress_text())

    def _finish(self, offset_x: float, offset_y: float) -> None:
        self.get_logger().info(
            f"力臂标定完成：offset = ({offset_x:+.4f}, {offset_y:+.4f}) m，"
            f"模长 {math.hypot(offset_x, offset_y):.3f} m"
        )
        for warning in sanity_warnings(offset_x, offset_y):
            self.get_logger().warn(warning)
        print()
        for line in result_lines(offset_x, offset_y):
            print(line)
        print()
        self._outcome = "success"
        self._done = True

    def _fail(self, reason: str) -> None:
        self.get_logger().error(reason)
        self._outcome = "failed"
        self._done = True


def main(args=None) -> int:
    rclpy.init(args=args)
    node: Optional[CalibrateLeverArm] = None
    code = 1
    interrupted = {"flag": False}

    def _on_sigint(signum, frame):  # noqa: ARG001
        interrupted["flag"] = True

    signal.signal(signal.SIGINT, _on_sigint)

    try:
        node = CalibrateLeverArm()
        while not node._done:
            if interrupted["flag"]:
                node.get_logger().warn("手动 Ctrl-C，标定未完成")
                node._outcome = "interrupted"
                break
            if not rclpy.ok():
                break
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        active_exc = sys.exc_info()[0] is not None
        if node is not None:
            with suppress(Exception):
                node.destroy_node()
            if interrupted["flag"] or node._outcome == "interrupted":
                code = 130
            elif node._outcome == "success":
                code = 0
            else:
                code = 1
        if rclpy.ok():
            if active_exc:
                with suppress(Exception):
                    rclpy.shutdown()
            else:
                rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())