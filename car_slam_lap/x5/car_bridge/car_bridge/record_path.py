"""record_path —— 建图/定位示教时录轨迹，走圈时照着回放。

两种模式：

1. 旧建图模式（不给 ``map_file``）：保持原有行为，从 TF ``world_frame <- base_frame``
   录轨迹；map 帧查不到时可以退回 odom（只能同会话回放）。
2. 严格定位示教模式（给 ``map_file``）：必须 ``world_frame=map``、
   ``fallback_frame`` 为空、``use_odom_pose=False``。原始 /odin1/odometry 用
   OdomGuard + sensor_data QoS 校验；同时查 ``map<-base_frame`` 与
   ``odom<-map``，并要求 ``odom<-map`` 的 stamp 与原始消息 stamp 相差不超过
   0.5 s（两者都是设备时间域）。地图 TF 没就绪只等待、不录点；收到过有效
   定位后再出定位/原始数据故障就中止，返回非 0，且不写可正式运行的 bundle。

用法：
    # 旧建图录制
    ros2 run car_bridge record_path --ros-args \
        -p out_path:=$HOME/car_ws/maps/lab1/path.yaml

    # 严格定位示教（先手动启动旧地图重定位，再用 DT7 示教）
    ros2 run car_bridge record_path --ros-args \
        -p map_file:=$HOME/car_ws/maps/lab1/odin_map/xxx.bin \
        -p out_path:=$HOME/car_ws/maps/lab1/route_map.yaml \
        -p base_frame:=imu
"""

from __future__ import annotations

import math
import os
import signal
import sys
import time
from contextlib import suppress
from typing import List, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import tf2_ros

from .localization_guard import OdomGuard
from .path_io import path_length, wrap_pi, write_path, yaw_from_quaternion
from .route_bundle import file_sha256, write_bundle

MAP_FRAME = "map"
STRICT_ODOM_TIMEOUT_S = 0.5
MAP_TF_STAMP_TOL_S = 0.5
MAP_TF_WAIT_TIMEOUT_S = 30.0
LOCALIZATION_STALE_S = 1.0
_QUAT_NORM_EPS = 1e-9


class RecordPath(Node):
    def __init__(self) -> None:
        super().__init__("record_path")
        self.declare_parameter("odom_topic", "/odin1/odometry")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("fallback_frame", "odom")  # 旧建图模式的退路
        self.declare_parameter("base_frame", "")  # 空 = 用 odom.child_frame_id（实测是 imu）
        self.declare_parameter("out_path", os.path.expanduser("~/car_ws/maps/path.yaml"))
        self.declare_parameter("min_dist", 0.05)      # m，移动超过才记点
        self.declare_parameter("min_yaw_deg", 5.0)    # deg，转动超过才记点
        self.declare_parameter("max_dt", 1.0)         # s，原地也要留时间戳
        self.declare_parameter("autosave_s", 10.0)
        self.declare_parameter("use_odom_pose", False)  # 旧模式：直接用 /odin1/odometry
        # 给了 map_file 就进入严格定位示教模式。
        self.declare_parameter("map_file", "")

        self.world_frame: str = self.get_parameter("world_frame").value
        self.base_frame: str = self.get_parameter("base_frame").value
        raw_fallback = self.get_parameter("fallback_frame").value
        self.fallback_frame: str = "" if raw_fallback is None else str(raw_fallback)
        self.out_path: str = self.get_parameter("out_path").value
        self.min_dist: float = float(self.get_parameter("min_dist").value)
        self.min_yaw: float = math.radians(float(self.get_parameter("min_yaw_deg").value))
        self.max_dt: float = float(self.get_parameter("max_dt").value)
        self.use_odom_pose: bool = bool(self.get_parameter("use_odom_pose").value)
        self.map_file: str = str(self.get_parameter("map_file").value or "").strip()
        self.strict: bool = bool(self.map_file)

        if self.strict:
            self._validate_strict_params()

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.points: List[Tuple[float, float, float, float]] = []  # x, y, yaw, t
        self._t0: Optional[float] = None
        self._tf_misses = 0
        self._last_save = time.monotonic()

        # 严格模式状态
        self._odom_guard: Optional[OdomGuard] = None
        self._map_sha = ""
        self._wait_start = time.monotonic()
        self._localized = False
        self._last_localized_rx: Optional[float] = None
        self._resolved_base_frame = ""
        self._faulted = False
        self._done = False
        self._bundle_written = False
        self._strict_misses = 0

        if self.strict:
            try:
                self._map_sha = file_sha256(self.map_file)
            except OSError as exc:
                raise ValueError(f"读地图文件失败: {self.map_file}: {exc}") from exc
            self._odom_guard = OdomGuard(timeout_s=STRICT_ODOM_TIMEOUT_S)

        odom_topic = self.get_parameter("odom_topic").value
        if self.strict:
            # 原始里程计发布者常用 best effort；默认 reliable 订阅会收不到。
            self.create_subscription(Odometry, odom_topic, self._on_strict_odom,
                                     qos_profile_sensor_data)
        else:
            self.create_subscription(Odometry, odom_topic, self._on_odom, 20)

        autosave = float(self.get_parameter("autosave_s").value)
        if autosave > 0:
            self.create_timer(autosave, self._autosave)
        if self.strict:
            self.create_timer(0.2, self._watchdog)

        os.makedirs(os.path.dirname(os.path.abspath(self.out_path)), exist_ok=True)
        if self.strict:
            self.get_logger().info(
                f"严格定位示教: map <- {self.base_frame or '(odom.child_frame_id)'} "
                f"-> {self.out_path}；地图 {self.map_file}"
            )
            self.get_logger().info(
                f"地图 SHA256 {self._map_sha[:16]}...；"
                f"odom<-map stamp 容差 {MAP_TF_STAMP_TOL_S:.1f} s，"
                f"未就绪最多等 {MAP_TF_WAIT_TIMEOUT_S:.0f} s"
            )
        else:
            self.get_logger().info(
                f"录制轨迹: {self.world_frame} <- {self.base_frame or '(odom.child_frame_id)'} "
                f"-> {self.out_path}"
            )

    # ---------------------------------------------------------- 严格模式前置
    def _validate_strict_params(self) -> None:
        if self.world_frame != MAP_FRAME:
            raise ValueError(
                f"严格定位示教要求 world_frame=map，当前是 {self.world_frame!r}"
            )
        if str(self.fallback_frame).strip():
            raise ValueError(
                "严格定位示教要求 fallback_frame 为空，不能退回 odom"
            )
        if self.use_odom_pose:
            raise ValueError("严格定位示教要求 use_odom_pose=False")
        if not os.path.isfile(self.map_file):
            raise ValueError(f"地图文件不存在或不是文件: {self.map_file}")
        if os.path.exists(self.out_path) or os.path.exists(
                self.out_path + ".manifest.json"):
            raise RuntimeError(
                f"输出或 manifest 已存在，拒绝覆盖: {self.out_path}"
            )

    # ---------------------------------------------------------- 位姿
    @staticmethod
    def _stamp_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _pose_from_transform(self, tf) -> Optional[Tuple[float, float, float]]:
        t, q = tf.transform.translation, tf.transform.rotation
        try:
            values = (float(t.x), float(t.y), float(t.z),
                      float(q.x), float(q.y), float(q.z), float(q.w))
        except (TypeError, ValueError):
            return None
        if not all(math.isfinite(value) for value in values):
            return None
        qx, qy, qz, qw = values[3:]
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= _QUAT_NORM_EPS:
            return None
        yaw = yaw_from_quaternion(qx, qy, qz, qw)
        if not math.isfinite(yaw):
            return None
        return values[0], values[1], yaw

    def _strict_map_pose(self, msg: Odometry, child: str):
        """返回 (pose, error)。pose=None 表示地图 TF 还没就绪或明显陈旧。"""
        try:
            map_tf = self._tf_buffer.lookup_transform(
                MAP_FRAME, child, rclpy.time.Time())
        except Exception:  # noqa: BLE001 - 重定位成功前 map 帧不存在是常态
            return None, f"地图 TF 未就绪：查不到 {MAP_FRAME} <- {child}"
        pose = self._pose_from_transform(map_tf)
        if pose is None:
            return None, f"地图 TF 非法（{MAP_FRAME} <- {child} 含 NaN/Inf 或零四元数）"

        try:
            odom_map = self._tf_buffer.lookup_transform(
                "odom", MAP_FRAME, rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None, "地图 TF 未就绪：查不到 odom <- map"
        try:
            tf_stamp = self._stamp_seconds(odom_map.header.stamp)
            raw_stamp = self._stamp_seconds(msg.header.stamp)
        except (AttributeError, TypeError, ValueError):
            return None, "odom<-map 或原始里程计 stamp 非法"
        if not (math.isfinite(tf_stamp) and math.isfinite(raw_stamp)):
            return None, "odom<-map 或原始里程计 stamp 非有限数"
        diff = abs(tf_stamp - raw_stamp)
        if diff > MAP_TF_STAMP_TOL_S:
            return None, (
                f"odom<-map 与原始里程计 stamp 相差 {diff:.3f} s "
                f"> {MAP_TF_STAMP_TOL_S:.1f} s（地图变换陈旧，不能拼成有效定位）"
            )
        return pose, ""

    # ---------------------------------------------------------- 旧建图模式
    def _lookup(self, odom_child: str) -> Optional[Tuple[float, float, float]]:
        frame = self.base_frame or odom_child
        try:
            # 用"最新可用"时刻，不用点云时间戳：Humble 上不会因外推失败而丢点
            tf = self._tf_buffer.lookup_transform(self.world_frame, frame, rclpy.time.Time())
        except Exception:  # noqa: BLE001 - TF 还没建起来是常态
            self._tf_misses += 1
            return None
        return self._pose_from_transform(tf)

    def _on_odom(self, msg: Odometry) -> None:
        """旧建图模式：保持原有行为和 fallback。"""
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

        self._append_pose(pose[0], pose[1], pose[2], now - self._t0)

    # ---------------------------------------------------------- 严格模式
    def _on_strict_odom(self, msg: Odometry) -> None:
        if self._done:
            return
        now = time.monotonic()
        pose_msg = msg.pose.pose
        accepted = self._odom_guard.update(
            now,
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
            msg.header.frame_id,
            msg.child_frame_id,
            (pose_msg.position.x, pose_msg.position.y, pose_msg.position.z),
            (pose_msg.orientation.x, pose_msg.orientation.y,
             pose_msg.orientation.z, pose_msg.orientation.w),
        )
        if not accepted:
            if self._odom_guard.faulted:
                self._abort(f"原始里程计故障：{self._odom_guard.fault_reason}")
            else:
                self._strict_miss(f"忽略原始里程计：{self._odom_guard.last_reject}")
            return

        if not self._resolved_base_frame:
            self._resolved_base_frame = msg.child_frame_id
        child = self.base_frame or self._resolved_base_frame
        pose, reason = self._strict_map_pose(msg, child)
        if pose is None:
            if self._localized:
                self._abort(f"定位故障，中止示教：{reason}")
            else:
                self._strict_miss(f"{reason}；先等待，不录制")
            return

        if self._t0 is None:
            self._t0 = now
            self.get_logger().info("定位就绪，开始录制 map 坐标路线")
        self._localized = True
        self._last_localized_rx = now
        self._append_pose(pose[0], pose[1], pose[2], now - self._t0)

    def _strict_miss(self, reason: str) -> None:
        self._strict_misses += 1
        if self._strict_misses == 1 or self._strict_misses % 100 == 0:
            self.get_logger().warn(
                f"{reason}（累计 {self._strict_misses} 次）"
            )

    def _abort(self, reason: str) -> None:
        if self._done:
            return
        self._faulted = True
        self._done = True
        self.get_logger().error(f"示教中止：{reason}")

    def _watchdog(self) -> None:
        """严格模式的原始里程计/地图定位超时看守。"""
        if not self.strict or self._done:
            return
        now = time.monotonic()
        if not self._odom_guard.ready:
            if now - self._wait_start > MAP_TF_WAIT_TIMEOUT_S:
                self._abort(
                    f"{MAP_TF_WAIT_TIMEOUT_S:.0f} s 内未收到合格原始里程计/"
                    "地图 TF，放弃示教"
                )
            return
        if not self._odom_guard.is_fresh(now):
            age = self._odom_guard.age(now)
            self._abort(f"原始里程计停更 {age:.2f} s，中止示教")
            return
        if not self._localized:
            if now - self._wait_start > MAP_TF_WAIT_TIMEOUT_S:
                self._abort(
                    f"{MAP_TF_WAIT_TIMEOUT_S:.0f} s 内地图 TF 未就绪"
                    "（map<-base 或 odom<-map），未录制任何点"
                )
            return
        if (self._last_localized_rx is not None
                and now - self._last_localized_rx > LOCALIZATION_STALE_S):
            self._abort(
                f"地图定位超过 {LOCALIZATION_STALE_S:.1f} s 未刷新，中止示教"
            )

    # ---------------------------------------------------------- 记录
    def _append_pose(self, x: float, y: float, yaw: float, t: float) -> None:
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
        length = path_length(self.points)
        self.get_logger().info(
            f"已写 {path}（{len(self.points)} 点，{length:.2f} m，{self.points[-1][3]:.1f} s）"
        )

    def finish(self, certify: bool = True) -> bool:
        """收尾：始终保存诊断轨迹；只有正常、无故障且数据足够新鲜时才认证 bundle。"""
        self.save()
        if not self.strict:
            return True
        if not certify:
            self.get_logger().error("非正常退出，不写正式 bundle")
            return False
        if self._faulted:
            self.get_logger().error("定位/原始数据故障过，不写正式 bundle")
            return False
        if len(self.points) < 3:
            self.get_logger().error(
                f"只录到 {len(self.points)} 个点（至少 3 个），不写正式 bundle"
            )
            return False
        if path_length(self.points) <= 0.0:
            self.get_logger().error("路线长度为 0，不写正式 bundle")
            return False
        now = time.monotonic()
        if (self._odom_guard is None or not self._odom_guard.ready
                or not self._odom_guard.is_fresh(now)):
            # Ctrl-C 可能抢在 watchdog 定时器之前；收尾时必须同步再查一次。
            self.get_logger().error("收尾时原始里程计不新鲜，不写正式 bundle")
            return False
        if (self._last_localized_rx is None
                or now - self._last_localized_rx > LOCALIZATION_STALE_S):
            self.get_logger().error("收尾时地图定位已陈旧，不写正式 bundle")
            return False
        try:
            current_map_sha = file_sha256(self.map_file)
        except OSError as exc:
            self.get_logger().error(f"收尾时读地图失败，拒绝写 bundle: {exc}")
            return False
        if current_map_sha != self._map_sha:
            self.get_logger().error("地图文件在录制期间发生变化，拒绝写 bundle")
            return False
        base = str(self.base_frame or self._resolved_base_frame).strip()
        if not base:
            self.get_logger().error("无法确定 base_frame，拒绝写 bundle")
            return False
        manifest = self.out_path + ".manifest.json"
        if os.path.exists(manifest):
            self.get_logger().error(f"manifest 已存在，拒绝覆盖: {manifest}")
            return False
        try:
            manifest = write_bundle(self.out_path, self.map_file, base_frame=base)
        except (OSError, ValueError) as exc:
            self.get_logger().error(f"写 bundle 失败: {exc}")
            return False
        self._bundle_written = True
        self.get_logger().info(f"已写正式 bundle: {manifest}")
        return True

    def _autosave(self) -> None:
        # autosave 只保存诊断轨迹，不能提前认证最终 bundle。
        if self.points and time.monotonic() - self._last_save > 1.0:
            self._last_save = time.monotonic()
            self.save()


def main(args=None) -> int:
    rclpy.init(args=args)
    node: Optional[RecordPath] = None
    code = 1

    # Ctrl-C 视为正常结束示教：置标志 -> 收尾 -> 正常路径写 bundle。
    interrupted = {"flag": False}

    def _on_sigint(signum, frame):  # noqa: ARG001
        interrupted["flag"] = True

    signal.signal(signal.SIGINT, _on_sigint)

    try:
        node = RecordPath()
        while not node._done:
            if interrupted["flag"]:
                break
            if not rclpy.ok():
                break
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        interrupted["flag"] = True
    except ExternalShutdownException:
        # 外部关停不是用户 Ctrl-C，不能据此认证 bundle。
        pass
    finally:
        active_exc = sys.exc_info()[0] is not None
        if node is not None:
            if active_exc:
                # 任何未预期异常都只保留诊断轨迹，绝不认证 bundle；
                # 收尾本身也不能盖掉原异常。
                with suppress(Exception):
                    node.finish(certify=False)
            else:
                certify = bool(interrupted["flag"])
                ok = False
                try:
                    ok = node.finish(certify=certify)
                except Exception as exc:  # noqa: BLE001 - 收尾失败也不能算成功
                    node.get_logger().error(f"收尾失败: {exc}")
                if node.strict:
                    code = 0 if (
                        ok and certify and not node._faulted
                        and node._bundle_written
                    ) else 1
                else:
                    code = 0
            try:
                node.destroy_node()
            except Exception:  # noqa: BLE001
                if not active_exc:
                    raise
        if rclpy.ok():  # 正常退出时 context 还活着，需要自己关
            if active_exc:
                with suppress(Exception):
                    rclpy.shutdown()
            else:
                rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
