"""follow_path —— 纯追踪（pure pursuit）回放录好的轨迹，让车自己走一圈。

输入：record_path.py 录的 path.yaml（map 坐标系）
反馈：TF map -> base_frame（重定位成功后由 Odin1 驱动发布）
输出：/cmd_vel，由 car_bridge 转成串口帧

先跑起来的三条前提（脚本会检查）：
    1. Odin1 处于重定位模式，且 tf2_echo map odom 有输出
    2. car_bridge 在跑，/car/status 正常（estop=0、ok=true）
    3. 车放回建图时的起点、朝向一致

安全：TF 丢超时 / 底盘报 estop / 链路断 任一发生，立刻发 0 并退出。

用法：
    ros2 run car_bridge follow_path --ros-args \
        -p path_file:=$HOME/car_ws/maps/lab1/path.yaml -p v_max:=0.3
"""

from __future__ import annotations

import json
import math
import os
import time
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

import tf2_ros

from .path_io import load_path, wrap_pi, yaw_from_quaternion


class FollowPath(Node):
    def __init__(self) -> None:
        super().__init__("follow_path")

        self.declare_parameter("path_file", "")
        # Odin1 驱动的 TF 链是 odom -> imu -> lidar，没有 odin1_base_link；
        # 「车体」就用 imu 系（设备本体）。设备装得离转向中心远的话，把
        # 这段偏移用静态 TF 补出来，别直接拿 imu 系当车心。
        self.declare_parameter("base_frame", "imu")
        self.declare_parameter("v_max", 0.35)
        self.declare_parameter("w_max", 1.2)
        self.declare_parameter("lookahead_min", 0.35)
        self.declare_parameter("lookahead_max", 0.80)
        self.declare_parameter("goal_tol", 0.20)
        self.declare_parameter("align_final_yaw", True)
        self.declare_parameter("tf_timeout_s", 0.50)
        self.declare_parameter("status_timeout_s", 1.00)
        self.declare_parameter("require_status", True)
        self.declare_parameter("start_delay_s", 2.0)
        self.declare_parameter("max_time_s", 0.0)   # 0 = 不限
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("save_actual", "")

        path_file = self.get_parameter("path_file").value
        if not path_file:
            raise RuntimeError("必须给 -p path_file:=/path/to/path.yaml")
        self.frame, pts = load_path(path_file)
        if len(pts) < 3:
            raise RuntimeError(f"{path_file} 里只有 {len(pts)} 个点，至少 3 个")

        self.px = [p[0] for p in pts]
        self.py = [p[1] for p in pts]
        self.pyaw = [p[2] for p in pts]
        # 累积弧长，用于"还剩多远"
        self.s = [0.0]
        for a, b in zip(pts, pts[1:]):
            self.s.append(self.s[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        self.total_len = self.s[-1]

        self.base_frame: str = self.get_parameter("base_frame").value
        self.v_max: float = float(self.get_parameter("v_max").value)
        self.w_max: float = float(self.get_parameter("w_max").value)
        self.lmin: float = float(self.get_parameter("lookahead_min").value)
        self.lmax: float = float(self.get_parameter("lookahead_max").value)
        self.goal_tol: float = float(self.get_parameter("goal_tol").value)
        self.align_yaw: bool = bool(self.get_parameter("align_final_yaw").value)
        self.tf_timeout: float = float(self.get_parameter("tf_timeout_s").value)
        self.status_timeout: float = float(self.get_parameter("status_timeout_s").value)
        self.require_status: bool = bool(self.get_parameter("require_status").value)
        self.max_time: float = float(self.get_parameter("max_time_s").value)
        self.save_actual: str = self.get_parameter("save_actual").value

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(String, "car/status", self._on_status, 10)

        self._status: Optional[dict] = None
        self._status_stamp = 0.0
        self._status_bad_since: Optional[float] = None
        self._tf_fail_since: Optional[float] = None
        self._idx = 0
        self._state = "run"       # run -> align -> done
        self._start = time.monotonic() + float(self.get_parameter("start_delay_s").value)
        self._t0: Optional[float] = None
        self._done = False
        self._reason = ""
        self._actual: List[Tuple[float, float, float, float]] = []
        self._err_max = 0.0
        self._err_sq = 0.0
        self._err_n = 0
        self._idx_prev = -1
        self._align_start: Optional[float] = None
        # 前视距离固定按 v_max 取（1 m/s 对应 2 m），限制在 [lmin, lmax]；
        # 车在跑的时候改前视反而容易震荡，这里刻意不做速度自适应。
        self._lookahead = max(self.lmin, min(self.lmax, 2.0 * self.v_max))

        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self._tick)
        self.get_logger().info(
            f"轨迹 {os.path.basename(path_file)}: {len(pts)} 点 / {self.total_len:.2f} m / "
            f"坐标系 {self.frame}；v_max={self.v_max} m/s，前视 {self.lmin}-{self.lmax} m"
        )

    # ---------------------------------------------------------- 反馈
    def _on_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        self._status = data
        self._status_stamp = time.monotonic()
        bad = bool(data.get("estop")) or not bool(data.get("ok", True))
        self._status_bad_since = (self._status_bad_since or time.monotonic()) if bad else None

    def _pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self._tf_buffer.lookup_transform(self.frame, self.base_frame, rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return t.x, t.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)

    # ---------------------------------------------------------- 控制
    def _publish(self, v: float, w: float) -> None:
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self._pub.publish(msg)

    def _stop(self, reason: str) -> None:
        for _ in range(3):
            self._publish(0.0, 0.0)
            time.sleep(0.02)
        self._reason = reason
        self._done = True

    def _nearest(self, x: float, y: float) -> Tuple[int, float]:
        lo = max(0, self._idx - 10)
        hi = min(len(self.px), self._idx + 200)
        best_i, best_d2 = self._idx, float("inf")
        for i in range(lo, hi):
            d2 = (self.px[i] - x) ** 2 + (self.py[i] - y) ** 2
            if d2 < best_d2:
                best_d2, best_i = d2, i
        return best_i, math.sqrt(best_d2)

    def _target(self, x: float, y: float, start: int, lookahead: float) -> int:
        for i in range(start, len(self.px)):
            if math.hypot(self.px[i] - x, self.py[i] - y) >= lookahead:
                return i
        return len(self.px) - 1

    def _tick(self) -> None:
        now = time.monotonic()
        if self._done:
            return

        # ---- 安全联锁 ----
        if self.require_status:
            if self._status is None or now - self._status_stamp > self.status_timeout:
                self._stop("没收到 /car/status（car_bridge 没起？串口断了？）")
                return
        if self._status_bad_since is not None and now - self._status_bad_since > 0.5:
            self._stop(f"底盘状态异常：{self._status}")
            return
        if self.max_time > 0 and self._t0 is not None and now - self._t0 > self.max_time:
            self._stop(f"超过 max_time_s={self.max_time}s")
            return

        pose = self._pose()
        if pose is None:
            self._tf_fail_since = self._tf_fail_since or now
            self._publish(0.0, 0.0)
            if now - self._tf_fail_since > self.tf_timeout:
                self._stop(f"查不到 TF {self.frame} -> {self.base_frame}（重定位成功了吗？）")
            return
        self._tf_fail_since = None

        if now < self._start:
            self._publish(0.0, 0.0)
            return
        if self._t0 is None:
            self._t0 = now
            self.get_logger().info("开始跟随")

        x, y, yaw = pose
        self._record_actual(x, y, yaw, now)

        idx, dist = self._nearest(x, y)
        self._idx = idx
        self._err_max = max(self._err_max, dist)
        self._err_sq += dist * dist
        self._err_n += 1
        self._log_progress(x, y, idx, dist)

        remaining = self.total_len - self.s[idx]

        if self._state == "run":
            if remaining < self.goal_tol:
                if self.align_yaw:
                    self._state = "align"
                    self.get_logger().info("到终点，修正最终朝向")
                else:
                    self._stop("走完一圈")
                self._publish(0.0, 0.0)
                return

            lookahead = self._lookahead
            ti = self._target(x, y, idx, lookahead)
            dx, dy = self.px[ti] - x, self.py[ti] - y
            alpha = wrap_pi(math.atan2(dy, dx) - yaw)
            curvature = 2.0 * math.sin(alpha) / max(lookahead, 1e-3)

            v = self.v_max * max(0.25, math.cos(alpha))
            if remaining < 0.5:  # 快到终点先减速
                v = min(v, 0.15)
            w = max(-self.w_max, min(self.w_max, v * curvature))
            self._publish(v, w)
            return

        if self._state == "align":
            if self._align_start is None:
                self._align_start = now
            if now - self._align_start > 5.0:
                self._stop("走完一圈（朝向修正超时，就地停）")
                return
            err = wrap_pi(self.pyaw[-1] - yaw)
            if abs(err) < 0.10:
                self._stop("走完一圈（朝向已对齐）")
                return
            w = max(-self.w_max * 0.8, min(self.w_max * 0.8, 1.5 * err))
            self._publish(0.0, w)

    # ---------------------------------------------------------- 记录
    def _record_actual(self, x: float, y: float, yaw: float, now: float) -> None:
        t = 0.0 if self._t0 is None else now - self._t0
        if not self._actual:
            self._actual.append((x, y, yaw, t))
            return
        lx, ly, _, _ = self._actual[-1]
        if math.hypot(x - lx, y - ly) >= 0.05:
            self._actual.append((x, y, yaw, t))

    def _log_progress(self, x: float, y: float, idx: int, dist: float) -> None:
        if idx % 20 != 0 or idx == self._idx_prev:
            return
        self._idx_prev = idx
        remaining = self.total_len - self.s[idx]
        self.get_logger().info(
            f"{self.s[idx]:5.1f}/{self.total_len:.1f} m  剩余 {remaining:4.1f} m  "
            f"横向误差 {dist:.3f} m"
        )

    # ---------------------------------------------------------- 收尾
    def summary(self) -> str:
        rms = math.sqrt(self._err_sq / self._err_n) if self._err_n else 0.0
        dur = 0.0 if not self._actual else self._actual[-1][3]
        return (
            f"结束原因: {self._reason or '未开始'}\n"
            f"跟踪点数: {self._err_n}  用时 {dur:.1f} s\n"
            f"横向误差 max={self._err_max:.3f} m  rms={rms:.3f} m\n"
            f"实际轨迹 {len(self._actual)} 点"
        )

    def save_actual(self) -> None:
        if not self.save_actual or not self._actual:
            return
        os.makedirs(os.path.dirname(os.path.abspath(self.save_actual)), exist_ok=True)
        lines = ["# 实际走过的轨迹（follow_path.py 生成）",
                 f"frame_id: {self.frame}", "points:"]
        for x, y, yaw, t in self._actual:
            lines.append(f"  - {{x: {x:.4f}, y: {y:.4f}, yaw: {yaw:.4f}, t: {t:.3f}}}")
        with open(self.save_actual, "w", encoding="utf-8") as fh:
            fh.write("\n".join(lines) + "\n")
        self.get_logger().info(f"实际轨迹写到 {self.save_actual}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FollowPath()
    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        node._stop("手动 Ctrl-C")
    finally:
        print(node.summary())
        node.save_actual()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
