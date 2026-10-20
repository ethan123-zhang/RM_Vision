"""record_path —— 建图那趟顺手把轨迹录下来，走圈时照着回放。

思路是"示教-复现"：人开着车把房间走一圈，这里把地图坐标系下的位姿记成
path.yaml；之后重定位成功、map->odom 有了，follow_path 就照着这条轨迹闭环跑。

轨迹点从 TF 取（world_frame <- base_frame），所以它跟着 Odin1 的 map 坐标系走，
即使 Odin1 的 odom 漂了也没关系——重定位会把 map 对齐回建图那趟的原点。

用法：
    ros2 run car_bridge record_path --ros-args \
        -p out_path:=$HOME/car_ws/maps/lab1/path.yaml
"""

from __future__ import annotations

import math
import os
import time
from typing import List, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

import tf2_ros

from .path_io import wrap_pi, write_path, yaw_from_quaternion


class RecordPath(Node):
    def __init__(self) -> None:
        super().__init__("record_path")
        self.declare_parameter("odom_topic", "/odin1/odometry")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("fallback_frame", "odom")  # map 查不到时的退路，见下
        self.declare_parameter("base_frame", "")  # 空 = 用 odom.child_frame_id（实测是 imu）
        self.declare_parameter("out_path", os.path.expanduser("~/car_ws/maps/path.yaml"))
        self.declare_parameter("min_dist", 0.05)      # m，移动超过才记点
        self.declare_parameter("min_yaw_deg", 5.0)    # deg，转动超过才记点
        self.declare_parameter("max_dt", 1.0)         # s，原地也要留时间戳
        self.declare_parameter("autosave_s", 10.0)
        self.declare_parameter("use_odom_pose", False)  # true = 不查 TF，直接用 /odin1/odometry

        self.world_frame: str = self.get_parameter("world_frame").value
        self.base_frame: str = self.get_parameter("base_frame").value
        self.fallback_frame: str = self.get_parameter("fallback_frame").value
        self.out_path: str = self.get_parameter("out_path").value
        self.min_dist: float = float(self.get_parameter("min_dist").value)
        self.min_yaw: float = math.radians(float(self.get_parameter("min_yaw_deg").value))
        self.max_dt: float = float(self.get_parameter("max_dt").value)
        self.use_odom_pose: bool = bool(self.get_parameter("use_odom_pose").value)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.points: List[Tuple[float, float, float, float]] = []  # x, y, yaw, t
        self._t0: Optional[float] = None
        self._tf_misses = 0
        self._last_save = time.monotonic()

        self.create_subscription(Odometry, self.get_parameter("odom_topic").value,
                                 self._on_odom, 20)
        autosave = float(self.get_parameter("autosave_s").value)
        if autosave > 0:
            self.create_timer(autosave, self._autosave)

        os.makedirs(os.path.dirname(os.path.abspath(self.out_path)), exist_ok=True)
        self.get_logger().info(
            f"录制轨迹: {self.world_frame} <- {self.base_frame or '(odom.child_frame_id)'} "
            f"-> {self.out_path}"
        )

    # ---------------------------------------------------------- 位姿
    def _lookup(self, odom_child: str) -> Optional[Tuple[float, float, float]]:
        frame = self.base_frame or odom_child
        try:
            # 用"最新可用"时刻，不用点云时间戳：Humble 上不会因外推失败而丢点
            tf = self._tf_buffer.lookup_transform(self.world_frame, frame, rclpy.time.Time())
        except Exception:  # noqa: BLE001 - TF 还没建起来是常态
            self._tf_misses += 1
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return t.x, t.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _on_odom(self, msg: Odometry) -> None:
        now = time.monotonic()
        if self._t0 is None:
            self._t0 = now

        if self.use_odom_pose:
            pos, ori = msg.pose.pose.position, msg.pose.pose.orientation
            pose = (pos.x, pos.y, yaw_from_quaternion(ori.x, ori.y, ori.z, ori.w))
        else:
            pose = self._lookup(msg.child_frame_id)
            if pose is None:
                # 建图模式下 map 与 odom 同原点。若 map 帧在 TF 树里压根没出现，
                # 就退回 odom 系（此时录出来的轨迹只能在同一趟里回放，不能跨会话）。
                if (not self.points and self.fallback_frame
                        and self.world_frame != self.fallback_frame
                        and self._tf_misses > 30):
                    self.get_logger().warn(
                        f"TF 树里没有 {self.world_frame} 帧，改用 {self.fallback_frame} 系录制。"
                        "注意：odom 系轨迹只能在同一会话里回放，跨会话走圈必须用 map 系"
                    )
                    self.world_frame = self.fallback_frame
                    return
                if self._tf_misses == 1 or self._tf_misses % 100 == 0:
                    self.get_logger().warn(
                        f"查不到 TF {self.world_frame} <- {msg.child_frame_id}（已失败 {self._tf_misses} 次）"
                        "：重定位成功之前 map->odom 是不存在的，建图那趟应该是 odom 系"
                    )
                return

        x, y, yaw = pose
        t = now - self._t0

        if self.points:
            px, py, pyaw, pt = self.points[-1]
            moved = math.hypot(x - px, y - py)
            turned = abs(wrap_pi(yaw - pyaw))
            if moved < self.min_dist and turned < self.min_yaw and (t - pt) < self.max_dt:
                return

        self.points.append((x, y, yaw, t))
        if len(self.points) % 20 == 0:
            self.get_logger().info(
                f"已录 {len(self.points)} 点  ({x:+.2f}, {y:+.2f}, yaw={math.degrees(yaw):+.0f}°)"
            )

    # ---------------------------------------------------------- 存盘
    def save(self, path: Optional[str] = None) -> None:
        path = path or self.out_path
        if not self.points:
            self.get_logger().warn("一个点都没录到，不写文件")
            return
        note = f"录制于 {time.strftime('%Y-%m-%d %H:%M:%S')}"
        write_path(path, self.world_frame, self.points, note=note)
        length = sum(
            math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(self.points, self.points[1:])
        )
        self.get_logger().info(
            f"已写 {path}（{len(self.points)} 点，{length:.2f} m，{self.points[-1][3]:.1f} s）"
        )

    def _autosave(self) -> None:
        if self.points and time.monotonic() - self._last_save > 1.0:
            self._last_save = time.monotonic()
            self.save()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RecordPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
