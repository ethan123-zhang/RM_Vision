"""follow_path —— 纯追踪（pure pursuit）回放录好的轨迹，让车自己走一圈。

输入：record_path.py 录的 path.yaml（默认 map 坐标系；历史 odom 系需显式授权）
      map 路线默认必须给 map_file，并通过 path+manifest+map 的内容 SHA256 配套校验。
反馈：TF map -> base_frame（地图坐标转换）+ 原始 /odin1/odometry（新鲜度检查）
输出：/cmd_vel，由 car_bridge 转成串口帧

为什么既看 TF 又看原始里程计：
    Odin1 驱动会缓存并重发 odom->imu，重发时连 TF 时间戳一起刷新（见
    include/host_sdk_sample.h:1950-1985）。所以"TF 查得到且很新"并不代表
    原始位姿还在更新。原始 Odometry 一停更就必须停车，不能靠 TF 续命。

先跑起来的三条前提（脚本会检查）：
    1. Odin1 处于重定位模式，且 tf2_echo map odom 有输出
    2. car_bridge 在跑，/car/status 正常（estop=0、ok=true）
    3. 车放回建图时的起点、朝向一致

安全：TF 丢超时 / 原始里程计未就绪或停更 / 底盘报 estop / 链路断，
任一发生立刻发 0 并退出；终点要求"进度到末段 + 实际位置到最后点
<= goal_tol + 朝向误差 <= 0.10 rad"，对齐超时算失败。

跟踪几何只沿路线弧长走：最近点被限制在当前进度附近的弧长窗口里，进度
只增不减且不超过实际里程 + PROGRESS_SLACK_M；前视目标按 s[idx]+lookahead
取。掉头/近邻回程段在空间上可能只隔十几厘米，不能靠"欧式距离扫到第一个
够远的点"，否则会跳过整段掉头去追回程点（实测 162 -> 192，夹角 167°）。

大转角采用有滞回的停走策略：目标方位误差超过 TRACK_ALIGN_ENTER_RAD 就
v=0、以 w_max 为上限原地对齐；缩到 TRACK_ALIGN_EXIT_RAD 以内才恢复跟踪。
对齐超时按失败停车，不无限旋转。整个过程仍然逐拍走全部原始
odom/TF/底盘状态安全门，并保持"只向前"——这里不是倒车支持。

默认流程（return_to_start=true）：
    启动宽限（只发 0）-> 定位/底盘安全门 -> 首次位姿起点范围检查
    -> 小范围回起点 -> 起始朝向对齐（只转不看位置）-> 位置不够就回起点补位
    -> 纯追踪 -> 终点对齐 -> done
回起点只适用于已确认空旷的小范围，不倒车、不绕障、不做任意地点返回。
回起点"边走边转"时角速度按 w <= v/RETURN_MIN_RADIUS_M 封顶（转弯半径不能
小于到位容差，否则会绕着起点打圈）。
起始对齐采用"先转正再补位"：位姿传感器偏离转心约 0.25 m，"位置在起点"
与"朝向对齐"在物理上互斥（转 140° 必然平移 0.47 m），所以对齐阶段只判朝向，
转正后位置超差就回 return 补位；往返轮次上限 START_RETURN_ROUNDTRIP_MAX，
共享 return_timeout_s 兜底，超过轮次按失败停车。
return_to_start=false 仅供已人工对齐的历史诊断，会打印警告。

用法：
    ros2 run car_bridge follow_path --ros-args \
        -p path_file:=$HOME/car_ws/maps/lab1/path.yaml -p v_max:=0.3

    历史 odom 系路线（仅供已人工对齐的历史诊断，不代表跨会话安全）：
        -p allow_legacy_odom:=true
"""

from __future__ import annotations

import json
import math
import os
import signal
import sys
import time
from contextlib import suppress
from typing import List, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

import tf2_ros

from .localization_guard import OdomGuard, check_route_frame
from .path_io import load_path, wrap_pi, yaw_from_quaternion
from .route_bundle import validate_bundle
from .tracking_geometry import (
    cumulative_arc_length,
    limit_progress_index,
    nearest_index,
    target_index_by_arc,
)

#: 成功 / 失败 / 用户中断；summary 与 main 的退出码都看它。
OUTCOME_SUCCESS = "success"
OUTCOME_FAILED = "failed"
OUTCOME_INTERRUPTED = "interrupted"

#: 终点朝向容差（rad）。
ALIGN_TOL = 0.10
#: 最终朝向对齐超时（s）；超时按失败处理，不写"走完一圈"。
ALIGN_TIMEOUT_S = 5.0
#: 路线进度索引允许比实际行驶距离超前这么多米，防止闭合路线起点被当成终点。
PROGRESS_SLACK_M = 0.10
#: 回起点时，方位误差超过这个角度就先原地转向，不发前进速度。
START_HEADING_TOL_RAD = 0.35

#: 回起点行驶中的转弯半径下限（m）。边走边转时用 w <= v/R 封顶：转弯半径
#: 小于到位容差时，车会绕着目标点打小圈而不是收敛（实车首测：R 只有 0.14 m，
#: 容差 0.10 m，结果绕了 194° 才"蹭"进容差圈，朝向完全不对）。
RETURN_MIN_RADIUS_M = 0.40

#: "先转正再补位"的最大往返轮次。位姿来自装在车上的传感器，它偏离转心约
#: 0.25 m（原地转实测 0.19–0.25 m），转正朝向必然带来等效平移；因此对齐阶段
#: 只判朝向，位置差回 return 补位。正常应每轮递减，超过这个轮次说明不收敛，
#: 失败停车而不是一直来回蹭。
START_RETURN_ROUNDTRIP_MAX = 3

#: 大转角停走对齐：目标方位误差超过进入阈值就停车原地转，缩到退出阈值内才
#: 恢复跟踪。进入阈值必须大于退出阈值，中间靠状态滞回，避免在阈值附近抖动。
TRACK_ALIGN_ENTER_RAD = math.radians(60.0)
TRACK_ALIGN_EXIT_RAD = math.radians(20.0)
#: 原地对齐超时（s）；按默认 w_max 转半圈还留余量，超时按失败停车，不无限旋转。
TRACK_ALIGN_TIMEOUT_S = 15.0
#: 最近点搜索的局部弧长窗口（m）：不允许为了凑欧式最近点跨过掉头/回程段。
NEAREST_BACK_M = 0.30
NEAREST_FORWARD_M = 0.30


def _finite_param(name: str, value, *, allow_zero: bool = False) -> float:
    """参数统一校验：必须有限；速度允许 0，其余必须大于 0。"""
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"参数 {name} 必须是数字，收到 {value!r}") from exc
    if not math.isfinite(number):
        raise ValueError(f"参数 {name} 必须是有限数，收到 {value!r}")
    if number < 0.0 or (number == 0.0 and not allow_zero):
        hint = "大于等于 0" if allow_zero else "大于 0"
        raise ValueError(f"参数 {name} 必须{hint}，收到 {value!r}")
    return number


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
        # 原始里程计：帧名/时间推进/超时检查。TF 新鲜不能替代它。
        self.declare_parameter("odom_topic", "/odin1/odometry")
        self.declare_parameter("odom_timeout_s", 0.5)
        # 历史 odom 系路线默认拒绝；显式打开也只是一次性诊断用途。
        self.declare_parameter("allow_legacy_odom", False)
        # map 路线必须与地图文件配套；只有隔离诊断才允许关掉。
        self.declare_parameter("map_file", "")
        self.declare_parameter("require_map_bundle", True)
        # 自动回起点 + 起始朝向对齐（保守软件初值，不是硬件验收值）。
        self.declare_parameter("return_to_start", True)
        self.declare_parameter("start_max_offset_m", 0.5)
        self.declare_parameter("start_pos_tol_m", 0.10)
        self.declare_parameter("start_yaw_tol_rad", 0.10)
        self.declare_parameter("return_v_max", 0.08)
        self.declare_parameter("return_w_max", 0.4)
        self.declare_parameter("return_timeout_s", 30.0)

        self.return_to_start = bool(self.get_parameter("return_to_start").value)
        self.start_max_offset_m = _finite_param(
            "start_max_offset_m", self.get_parameter("start_max_offset_m").value)
        self.start_pos_tol_m = _finite_param(
            "start_pos_tol_m", self.get_parameter("start_pos_tol_m").value)
        self.start_yaw_tol_rad = _finite_param(
            "start_yaw_tol_rad", self.get_parameter("start_yaw_tol_rad").value)
        self.return_v_max = _finite_param(
            "return_v_max", self.get_parameter("return_v_max").value, allow_zero=True)
        self.return_w_max = _finite_param(
            "return_w_max", self.get_parameter("return_w_max").value, allow_zero=True)
        self.return_timeout_s = _finite_param(
            "return_timeout_s", self.get_parameter("return_timeout_s").value)
        if not self.return_to_start:
            self.get_logger().warn(
                "return_to_start=False：跳过自动回起点与起始对齐，仅供已人工对齐的"
                "诊断模式，不代表跨会话安全"
            )

        path_file = self.get_parameter("path_file").value
        if not path_file:
            raise RuntimeError("必须给 -p path_file:=/path/to/path.yaml")
        raw_frame, pts = load_path(path_file, require_frame=True, require_finite=True)
        self.frame, is_legacy_route = check_route_frame(
            raw_frame, bool(self.get_parameter("allow_legacy_odom").value)
        )
        if is_legacy_route and self.return_to_start:
            raise ValueError(
                "路线参考系是历史 odom：自动回起点需要地图坐标，"
                "只能显式 return_to_start=False 做已人工对齐的诊断"
            )
        if is_legacy_route:
            self.get_logger().warn(
                "路线参考系是历史 odom：仅供已人工对齐的历史诊断，"
                "不代表跨会话安全；优先使用 map 系示教路线"
            )
        if len(pts) < 3:
            raise RuntimeError(f"{path_file} 里只有 {len(pts)} 个点，至少 3 个")

        self.px = [p[0] for p in pts]
        self.py = [p[1] for p in pts]
        self.pyaw = [p[2] for p in pts]
        # 累积弧长，用于"还剩多远"和沿弧长挑目标
        self.s = cumulative_arc_length(self.px, self.py)
        self.total_len = self.s[-1]

        self.base_frame: str = self.get_parameter("base_frame").value
        # 跟踪参数必须有限且为正：NaN/Inf/负数会让 cmd_vel 变成非法值。
        self.v_max: float = _finite_param(
            "v_max", self.get_parameter("v_max").value)
        self.w_max: float = _finite_param(
            "w_max", self.get_parameter("w_max").value)
        self.lmin: float = _finite_param(
            "lookahead_min", self.get_parameter("lookahead_min").value)
        self.lmax: float = _finite_param(
            "lookahead_max", self.get_parameter("lookahead_max").value)
        if self.lmin > self.lmax:
            raise ValueError(
                f"lookahead_min 必须 <= lookahead_max，"
                f"收到 {self.lmin} > {self.lmax}"
            )
        self.goal_tol: float = float(self.get_parameter("goal_tol").value)
        self.align_yaw: bool = bool(self.get_parameter("align_final_yaw").value)
        self.tf_timeout: float = float(self.get_parameter("tf_timeout_s").value)
        self.status_timeout: float = float(self.get_parameter("status_timeout_s").value)
        self.require_status: bool = bool(self.get_parameter("require_status").value)
        self.max_time: float = float(self.get_parameter("max_time_s").value)
        self._save_actual_path: str = self.get_parameter("save_actual").value
        self._odom_topic: str = self.get_parameter("odom_topic").value
        self._odom_timeout: float = float(self.get_parameter("odom_timeout_s").value)
        self._odom_guard = OdomGuard(timeout_s=self._odom_timeout)
        self._odom_rejects = 0
        self._odom_ready_logged = False
        self.map_file: str = str(self.get_parameter("map_file").value or "").strip()
        self.require_map_bundle: bool = bool(
            self.get_parameter("require_map_bundle").value)
        self._bundle_metadata = None
        if self.frame == "map" and self.require_map_bundle:
            if not self.map_file:
                raise ValueError(
                    "map 路线必须给 -p map_file:=/path/to/odin_map.bin；"
                    "只有隔离诊断才显式 require_map_bundle:=false"
                )
            try:
                self._bundle_metadata = validate_bundle(
                    path_file, self.map_file, base_frame=self.base_frame)
            except ValueError as exc:
                raise ValueError(f"地图路线 bundle 校验失败: {exc}") from exc
            self.get_logger().info(
                f"地图路线 bundle 校验通过：{self._bundle_metadata['point_count']} 点，"
                f"base_frame={self._bundle_metadata['base_frame']}，"
                f"map_sha256={self._bundle_metadata['map_sha256'][:16]}..."
            )
        elif not self.require_map_bundle:
            self.get_logger().warn(
                "require_map_bundle=False：跳过地图路线与地图文件的配套校验，"
                "仅供隔离诊断，不代表可正式运行"
            )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(String, "car/status", self._on_status, 10)
        # 必须是 sensor data QoS（best effort）：Odometry 发布者常用它，
        # 默认 reliable 订阅会收不到 best effort 的原始里程计。
        self.create_subscription(Odometry, self._odom_topic, self._on_odom,
                                 qos_profile_sensor_data)

        self._status: Optional[dict] = None
        self._status_stamp = 0.0
        self._status_bad_since: Optional[float] = None
        self._tf_fail_since: Optional[float] = None
        self._tf_invalid_reason = ""
        self._idx = 0
        # return/align_start/run/track_align/align
        self._state = "return" if self.return_to_start else "run"
        self._start_checked = False
        self._start_phase_t0: Optional[float] = None
        self._start_elapsed = 0.0
        self._start_roundtrips = 0
        self._start = time.monotonic() + float(self.get_parameter("start_delay_s").value)
        self._t0: Optional[float] = None
        self._done = False
        self._outcome: Optional[str] = None
        self._reason = ""
        self._actual: List[Tuple[float, float, float, float]] = []
        self._err_max = 0.0
        self._err_sq = 0.0
        self._err_n = 0
        self._idx_prev = -1
        self._align_start: Optional[float] = None
        # 跟踪阶段：当前沿弧长选到的目标点，以及大转角原地对齐的起始时刻。
        self._target_idx = 0
        self._track_align_t0: Optional[float] = None
        # 路线进度索引不允许比实际行驶距离跑得快，否则闭合路线起点会被
        # 最近的"终点"吸过去；同时记录实际走过的里程用于诊断。
        self._traveled = 0.0
        self._last_pose: Optional[Tuple[float, float]] = None
        # 终点判定：进度到末段、实际位置到最后点 <= goal_tol，再单独判朝向。
        self._near_end_m = max(0.5, 2.0 * self.goal_tol)
        # 闭合路线起点/终点重合：必须真的走过至少半程，才算"到达终点"。
        self._min_goal_travel = 0.5 * self.total_len
        # 前视距离固定按 v_max 取（1 m/s 对应 2 m），限制在 [lmin, lmax]；
        # 车在跑的时候改前视反而容易震荡，这里刻意不做速度自适应。
        self._lookahead = max(self.lmin, min(self.lmax, 2.0 * self.v_max))

        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self._tick)
        self.get_logger().info(
            f"轨迹 {os.path.basename(path_file)}: {len(pts)} 点 / {self.total_len:.2f} m / "
            f"坐标系 {self.frame}；v_max={self.v_max} m/s，前视 {self.lmin}-{self.lmax} m"
        )
        self.get_logger().info(
            f"原始里程计 {self._odom_topic}（sensor data QoS），"
            f"停更阈值 {self._odom_timeout:.2f} s"
        )
        if self.return_to_start:
            self.get_logger().info(
                f"自动回起点已启用：范围 {self.start_max_offset_m:.2f} m，"
                f"位置容差 {self.start_pos_tol_m:.2f} m，"
                f"朝向容差 {self.start_yaw_tol_rad:.2f} rad，"
                f"限速 {self.return_v_max:.2f} m/s / {self.return_w_max:.2f} rad/s，"
                f"总超时 {self.return_timeout_s:.1f} s"
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
        self._tf_invalid_reason = ""
        try:
            tf = self._tf_buffer.lookup_transform(self.frame, self.base_frame, rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        try:
            values = (float(t.x), float(t.y), float(t.z),
                      float(q.x), float(q.y), float(q.z), float(q.w))
            if not all(math.isfinite(value) for value in values):
                self._tf_invalid_reason = "TF 平移/四元数含 NaN/Inf"
                return None
            qx, qy, qz, qw = values[3:]
            norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
            if norm <= 1e-9:
                self._tf_invalid_reason = "TF 四元数范数为 0"
                return None
            yaw = yaw_from_quaternion(qx, qy, qz, qw)
        except (TypeError, ValueError):
            self._tf_invalid_reason = "TF 字段不是数字"
            return None
        if not math.isfinite(yaw):
            self._tf_invalid_reason = "TF 偏航角不是有限数"
            return None
        return values[0], values[1], yaw

    def _on_odom(self, msg: Odometry) -> None:
        """原始里程计只用来判"数据还在不在动"，位姿仍从 TF 取。"""
        pose = msg.pose.pose
        accepted = self._odom_guard.update(
            time.monotonic(),
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
            msg.header.frame_id,
            msg.child_frame_id,
            (pose.position.x, pose.position.y, pose.position.z),
            (pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w),
        )
        if accepted:
            if not self._odom_ready_logged:
                self._odom_ready_logged = True
                self.get_logger().info("原始里程计已就绪（时间戳开始严格递增）")
            return
        self._odom_rejects += 1
        if self._odom_rejects == 1 or self._odom_rejects % 100 == 0:
            self.get_logger().warn(
                f"忽略原始里程计：{self._odom_guard.last_reject}"
                f"（累计 {self._odom_rejects} 次）"
            )

    # ---------------------------------------------------------- 控制
    def _publish(self, v: float, w: float) -> None:
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self._pub.publish(msg)

    def _stop(self, reason: str, outcome: str = OUTCOME_FAILED) -> None:
        """先落定结果，再尽力发零。

        发布失败时不能还让主循环以为"运行正常"：状态已经置为 done/failed，
        并把失败原因追加到 summary 里。
        """
        self._reason = reason
        self._outcome = outcome
        self._done = True
        if self._state in ("return", "align_start") and self._start_phase_t0 is not None:
            # 安全门在回起点阶段触发时，summary 也要把这段时间算到回起点。
            self._start_elapsed = max(
                self._start_elapsed, time.monotonic() - self._start_phase_t0
            )
        publish_error: Optional[Exception] = None
        for _ in range(3):
            try:
                self._publish(0.0, 0.0)
            except Exception as exc:  # noqa: BLE001 - 串口/中间件都可能在这时抛
                publish_error = exc
            time.sleep(0.02)
        if publish_error is not None:
            self._outcome = OUTCOME_FAILED
            self._reason = f"{reason}；停车指令发送失败：{publish_error}"
            self.get_logger().error(self._reason)

    def _nearest(self, x: float, y: float) -> Tuple[int, float]:
        """只在当前进度附近的弧长窗口里找最近点，避免吸到回程/交叉支路。"""
        return nearest_index(
            self.s, self.px, self.py, x, y, self._idx,
            back_m=NEAREST_BACK_M, forward_m=NEAREST_FORWARD_M,
        )

    def _target_index(self, idx: int) -> int:
        """沿路线弧长取前视目标：只按 s 前进，不按欧式距离扫远端点。"""
        return target_index_by_arc(self.s, idx, self._lookahead)

    def _tick(self) -> None:
        now = time.monotonic()
        if self._done:
            return

        # 启动宽限期：只发 0，不做任何"没数据就退出"的判定。
        # DDS 发现 + tf2 缓冲填充通常需要 100~500 ms；原来在 start_delay 之前就判
        # "/car/status 没收到"，导致 follow_path 刚启动就退出（2026-10-26 实车踩到）。
        if now < self._start:
            self._publish(0.0, 0.0)
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

        # ---- 原始里程计前置（TF 新鲜不能替代）----
        if self._odom_guard.faulted:
            self._stop(f"原始里程计故障：{self._odom_guard.fault_reason}")
            return
        if not self._odom_guard.ready:
            self._stop(
                f"启动宽限期结束仍未收到合格的原始里程计 {self._odom_topic}"
                "（约定 frame=odom / child=imu，sensor data QoS）"
            )
            return
        if not self._odom_guard.is_fresh(now):
            age = self._odom_guard.age(now)
            self._stop(
                f"原始里程计停更 {age:.2f} s（阈值 {self._odom_timeout:.2f} s）"
                "：TF 可能只是驱动在重发旧位姿"
            )
            return

        pose = self._pose()
        if pose is None:
            if self._tf_invalid_reason:
                self._stop(f"TF 非法：{self._tf_invalid_reason}，不能带入控制")
                return
            self._tf_fail_since = self._tf_fail_since or now
            self._publish(0.0, 0.0)
            if now - self._tf_fail_since > self.tf_timeout:
                self._stop(f"查不到 TF {self.frame} -> {self.base_frame}（重定位成功了吗？）")
            return
        self._tf_fail_since = None

        x, y, yaw = pose
        if self._t0 is None:
            self._t0 = now
            self.get_logger().info("开始跟随")
        self._record_actual(x, y, yaw, now)

        # 首帧有效地图位姿先做一次"是否在允许的返回范围"检查，
        # 之后才可能进入自动回起点。累计里程不参与这个判定。
        if not self._start_checked:
            self._start_checked = True
            self._start_phase_t0 = now
            offset = math.hypot(x - self.px[0], y - self.py[0])
            if offset > self.start_max_offset_m:
                self._fail_start(
                    now,
                    f"起点偏差 {offset:.2f} m > start_max_offset_m="
                    f"{self.start_max_offset_m:.2f} m，拒绝自动回起点",
                )
                return
            if not self.return_to_start:
                self.get_logger().info(
                    "诊断模式：跳过自动回起点与起始对齐，直接从路线首段开始"
                )

        if self._state in ("return", "align_start"):
            self._tick_start(now, x, y, yaw)
            return

        if self._last_pose is not None:
            self._traveled += math.hypot(x - self._last_pose[0], y - self._last_pose[1])
        self._last_pose = (x, y)

        raw_idx, _ = self._nearest(x, y)
        idx = self._limit_progress(raw_idx)
        self._idx = idx
        # 横向误差必须按最终采用的索引重算：被进度封顶后，最近点可能不是它。
        dist = math.hypot(self.px[idx] - x, self.py[idx] - y)
        self._err_max = max(self._err_max, dist)
        self._err_sq += dist * dist
        self._err_n += 1
        self._log_progress(x, y, idx, dist)

        remaining = self.total_len - self.s[idx]
        goal_dist = math.hypot(self.px[-1] - x, self.py[-1] - y)
        yaw_err = abs(wrap_pi(self.pyaw[-1] - yaw))
        # "进度接近末段"和"真的走到终点"必须同时成立：闭合路线的起点
        # 也可能离最后一点很近，不能只看剩余弧长或最近点索引。
        near_end = (remaining <= self._near_end_m
                    and self._traveled >= self._min_goal_travel)

        if self._state in ("run", "track_align"):
            if near_end and goal_dist <= self.goal_tol:
                if not self.align_yaw or yaw_err <= ALIGN_TOL:
                    self._stop(
                        f"到达终点（位置误差 {goal_dist:.2f} m，"
                        f"朝向误差 {yaw_err:.2f} rad）",
                        OUTCOME_SUCCESS,
                    )
                    return
                self._state = "align"
                self._track_align_t0 = None
                self.get_logger().info(
                    f"位置到终点但朝向差 {yaw_err:.2f} rad，开始最终对齐"
                )
                self._publish(0.0, 0.0)
                return

            ti = self._target_index(idx)
            self._target_idx = ti
            dx, dy = self.px[ti] - x, self.py[ti] - y
            target_dist = math.hypot(dx, dy)
            alpha = wrap_pi(math.atan2(dy, dx) - yaw)

            if self._state == "run" and abs(alpha) >= TRACK_ALIGN_ENTER_RAD:
                # 目标在侧后方：先停车原地对齐，绝不带着大方位误差往前开。
                self._state = "track_align"
                self._track_align_t0 = now
                self.get_logger().info(
                    f"目标方位差 {math.degrees(abs(alpha)):.0f}°，"
                    "停车原地对齐（线速度为 0）"
                )

            if self._state == "track_align":
                if now - self._track_align_t0 > TRACK_ALIGN_TIMEOUT_S:
                    self._stop(
                        f"大转角原地对齐超时（{TRACK_ALIGN_TIMEOUT_S:.1f} s），"
                        "失败停车，未走完路线"
                    )
                    return
                if abs(alpha) <= TRACK_ALIGN_EXIT_RAD:
                    self._state = "run"
                    self._track_align_t0 = None
                else:
                    w = max(-self.w_max, min(self.w_max, 1.5 * alpha))
                    self._publish(0.0, w)
                    return

            # 曲率用"到实际目标的欧氏距离"（弦长），不能拿前视弧长当弦长。
            curvature = 2.0 * math.sin(alpha) / max(target_dist, 1e-3)

            v = self.v_max * max(0.25, math.cos(alpha))
            if remaining < 0.5:  # 快到终点先减速
                v = min(v, 0.15)
            w = max(-self.w_max, min(self.w_max, v * curvature))
            self._publish(v, w)
            return

        if self._state == "align":
            if self._align_start is None:
                self._align_start = now
            # 先看有没有漂离终点：对齐阶段也不能只凭 yaw 判成功。
            goal_dist = math.hypot(self.px[-1] - x, self.py[-1] - y)
            if goal_dist > self.goal_tol:
                self._stop(
                    f"最终对齐时已漂离终点 {goal_dist:.2f} m "
                    f"> goal_tol={self.goal_tol:.2f} m，失败"
                )
                return
            if now - self._align_start > ALIGN_TIMEOUT_S:
                self._stop("最终朝向对齐超时（失败，未走完）")
                return
            err = wrap_pi(self.pyaw[-1] - yaw)
            if abs(err) <= ALIGN_TOL:
                self._stop(
                    f"到达终点（位置与朝向均达标，朝向误差 {abs(err):.2f} rad）",
                    OUTCOME_SUCCESS,
                )
                return
            w = max(-self.w_max * 0.8, min(self.w_max * 0.8, 1.5 * err))
            self._publish(0.0, w)

    # ---------------------------------------------------------- 回起点
    def _tick_start(self, now: float, x: float, y: float, yaw: float) -> None:
        """return_start / align_start 阶段；与任务01安全门串联，不另开循环。"""
        if now - self._start_phase_t0 > self.return_timeout_s:
            self._fail_start(
                now,
                f"回起点/起始对齐总超时（{self.return_timeout_s:.1f} s），失败停车",
            )
            return

        target_x, target_y = self.px[0], self.py[0]
        dx, dy = target_x - x, target_y - y
        dist = math.hypot(dx, dy)

        if self._state == "return":
            if dist <= self.start_pos_tol_m:
                # 进入对齐前先发零，别把上一状态的残余速度带进去。
                self._publish(0.0, 0.0)
                self._state = "align_start"
                self.get_logger().info(
                    f"已到起点位置（误差 {dist:.2f} m），开始对齐起始朝向"
                )
                return

            heading_err = wrap_pi(math.atan2(dy, dx) - yaw)
            if abs(heading_err) > START_HEADING_TOL_RAD:
                # 方位差大就先原地转，绝不倒车。
                v = 0.0
                w = max(-self.return_w_max, min(self.return_w_max, 1.5 * heading_err))
            else:
                # 距离越近线速度越小；最大不超过路线起点允许范围。
                scale = max(0.2, min(1.0, dist / max(self.start_max_offset_m, 1e-9)))
                v = self.return_v_max * scale
                # 边走边转时按转弯半径下限压角速度，别绕着目标点打转。
                w_cap = min(self.return_w_max, v / RETURN_MIN_RADIUS_M)
                w = max(-w_cap, min(w_cap, 1.5 * heading_err))
            self._publish(v, w)
            return

        # align_start：只负责朝向。
        # 转正本身会因传感器偏离转心带来等效平移（0.25 m 力臂：转 140° 就扫
        # 0.47 m），所以转动期间不做位置判据；转正后再看位置差，需要的话回
        # return 补位，并按往返轮次防抖（方案 B）。
        yaw_err = wrap_pi(self.pyaw[0] - yaw)
        if abs(yaw_err) > self.start_yaw_tol_rad:
            w = max(-self.return_w_max, min(self.return_w_max, 1.5 * yaw_err))
            self._publish(0.0, w)
            return

        if dist <= self.start_pos_tol_m:
            # 位置和朝向都达标：先发零，重置路线跟踪状态，下一拍才正式 run。
            self._publish(0.0, 0.0)
            self._start_elapsed = now - self._start_phase_t0
            self._reset_route_tracking(x, y)
            self._state = "run"
            self.get_logger().info(
                f"起始位置/朝向已达标（yaw 误差 {abs(yaw_err):.2f} rad，"
                f"位置误差 {dist:.2f} m），开始跟踪路线"
            )
            return

        self._start_roundtrips += 1
        if self._start_roundtrips > START_RETURN_ROUNDTRIP_MAX:
            self._fail_start(
                now,
                f"起始补位往返超过 {START_RETURN_ROUNDTRIP_MAX} 轮仍未收敛"
                f"（位置差 {dist:.2f} m > start_pos_tol_m="
                f"{self.start_pos_tol_m:.2f} m），失败停车",
            )
            return
        self.get_logger().info(
            f"起始朝向已对齐（yaw 误差 {abs(yaw_err):.2f} rad），但位置差 "
            f"{dist:.2f} m > start_pos_tol_m={self.start_pos_tol_m:.2f} m："
            f"第 {self._start_roundtrips} 轮回起点补位"
        )
        self._publish(0.0, 0.0)
        self._state = "return"

    def _fail_start(self, now: float, reason: str) -> None:
        if self._start_phase_t0 is not None:
            self._start_elapsed = max(0.0, now - self._start_phase_t0)
        self._stop(reason)

    def _reset_route_tracking(self, x: float, y: float) -> None:
        """回起点走过的距离不能算进路线完成进度。"""
        self._idx = 0
        self._idx_prev = -1
        self._traveled = 0.0
        self._last_pose = (x, y)
        self._align_start = None
        self._target_idx = 0
        self._track_align_t0 = None

    def _limit_progress(self, candidate: int) -> int:
        """路线进度索引只增不减，且不能比实际行驶里程超前太多。

        闭合路线起点和终点在同一处、掉头两侧也贴得很近，最近点搜索可能一步
        跳到远处；用累计实际里程给进度封顶，保留 PROGRESS_SLACK_M 余量吸收
        横向偏差和拐角。
        """
        return limit_progress_index(
            self.s, self._idx, candidate, self._traveled, PROGRESS_SLACK_M)

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
        outcome_text = {
            OUTCOME_SUCCESS: "成功",
            OUTCOME_FAILED: "失败",
            OUTCOME_INTERRUPTED: "中断",
        }.get(self._outcome, "未结束")
        rms = math.sqrt(self._err_sq / self._err_n) if self._err_n else 0.0
        dur = 0.0 if not self._actual else self._actual[-1][3]
        route_dur = max(0.0, dur - self._start_elapsed)
        return (
            f"结果: {outcome_text}\n"
            f"结束原因: {self._reason or '未开始'}\n"
            f"回起点/起始对齐 {self._start_elapsed:.1f} s  "
            f"（补位往返 {self._start_roundtrips} 次）  "
            f"路线跟踪 {route_dur:.1f} s\n"
            f"跟踪点数: {self._err_n}  用时 {dur:.1f} s  行驶 {self._traveled:.2f} m\n"
            f"横向误差 max={self._err_max:.3f} m  rms={rms:.3f} m\n"
            f"原始里程计: 接受 {self._odom_guard.accepted} 帧 / "
            f"拒绝 {self._odom_guard.rejected} 帧\n"
            f"实际轨迹 {len(self._actual)} 点"
        )

    def save_actual(self) -> None:
        # 注意：参数值存在 _save_actual_path 里，不能叫 self.save_actual ——
        # 同名会把下面的方法整个遮住，退出时 node.save_actual() 会变成"调用字符串"
        # （TypeError: 'str' object is not callable，2026-10-26 实车踩到）。
        if not self._save_actual_path or not self._actual:
            return
        os.makedirs(os.path.dirname(os.path.abspath(self._save_actual_path)), exist_ok=True)
        lines = ["# 实际走过的轨迹（follow_path.py 生成）",
                 f"frame_id: {self.frame}", "points:"]
        for x, y, yaw, t in self._actual:
            lines.append(f"  - {{x: {x:.4f}, y: {y:.4f}, yaw: {yaw:.4f}, t: {t:.3f}}}")
        with open(self._save_actual_path, "w", encoding="utf-8") as fh:
            fh.write("\n".join(lines) + "\n")
        self.get_logger().info(f"实际轨迹写到 {self._save_actual_path}")


def main(args=None) -> int:
    rclpy.init(args=args)
    node: Optional[FollowPath] = None
    code = 1

    # 自己接管 SIGINT：rclpy 默认收到 SIGINT 会直接把 context 关掉，Humble 上随后
    # spin_once 抛的是 RuntimeError（"Unable to convert call argument to Python object"）
    # 而不是 ExternalShutdownException——看起来像崩溃（2026-10-26 实车踩到）。
    # 这里改成：置标志 → 正常发 0 → 存实际轨迹 → 退出（SIGINT 用 130）。
    interrupted = {"flag": False}

    def _on_sigint(signum, frame):  # noqa: ARG001
        interrupted["flag"] = True

    signal.signal(signal.SIGINT, _on_sigint)

    try:
        node = FollowPath()
        while not node._done:
            if interrupted["flag"]:
                node._stop("手动 Ctrl-C", OUTCOME_INTERRUPTED)
                break
            if not rclpy.ok():
                break
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        active_exc = sys.exc_info()[0] is not None
        if node is not None:
            def _cleanup() -> None:
                # 收尾每一步都做，前一步失败不挡住后面的停车/存盘。
                try:
                    if not node._done:
                        node._stop("异常中止（未完成）", OUTCOME_FAILED)
                finally:
                    try:
                        print(node.summary())
                    finally:
                        try:
                            node.save_actual()
                        finally:
                            node.destroy_node()

            if active_exc:
                # 原始异常优先：收尾本身出错也不能把它盖掉。
                with suppress(Exception):
                    _cleanup()
            else:
                _cleanup()

            if interrupted["flag"] or node._outcome == OUTCOME_INTERRUPTED:
                code = 130
            elif node._outcome == OUTCOME_SUCCESS:
                code = 0
            else:
                code = 1

        if rclpy.ok():  # 正常退出时 context 还活着，需要自己关
            if active_exc:
                # shutdown 本身也可能抛；不能把控制循环的原始异常盖掉。
                with suppress(Exception):
                    rclpy.shutdown()
            else:
                rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
