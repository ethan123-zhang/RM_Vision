"""follow_path 的安全前置与终点判定测试（PC 上用最小 ROS 替身，不需要装 ROS）。

跑法：
    python test_follow_safety.py
    python -m unittest discover -s x5/car_bridge/test -p 'test_follow_safety.py' -v

测试刻意只替身 rclpy / 消息类型 / TF 缓冲这几样，真正执行 follow_path 的
订阅回调与 _tick()，断言发布出去的 cmd_vel，而不是去搜源码字符串。
"""

from __future__ import annotations

import io
import json
import math
import os
import signal
import sys
import types
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from car_bridge import route_bundle as rb  # noqa: E402

ODOM_FRAME = "odom"
ODOM_CHILD = "imu"
ODOM_TOPIC = "/odin1/odometry"


# =====================================================================
# 最小 ROS 替身
# =====================================================================
class _FakeLogger:
    def __init__(self) -> None:
        self.infos: list[str] = []
        self.warns: list[str] = []
        self.errors: list[str] = []

    def info(self, message: str) -> None:
        self.infos.append(str(message))

    def warn(self, message: str) -> None:
        self.warns.append(str(message))

    def warning(self, message: str) -> None:
        self.warns.append(str(message))

    def error(self, message: str) -> None:
        self.errors.append(str(message))


class _RecordingPublisher:
    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.sent: list[tuple[float, float]] = []

    def publish(self, msg) -> None:
        self.sent.append((float(msg.linear.x), float(msg.angular.z)))


class _RecordingSubscription:
    def __init__(self, msg_type, topic, callback, qos) -> None:
        self.msg_type = msg_type
        self.topic = topic
        self.callback = callback
        self.qos = qos


class _RecordingTimer:
    def __init__(self, period, callback) -> None:
        self.period = period
        self.callback = callback


#: 测试在构造节点前往里塞参数覆盖值。
PARAM_OVERRIDES: dict = {}


class _FakeNode:
    def __init__(self, name: str) -> None:
        self.name = name
        self._params: dict = {}
        self._publishers: list[_RecordingPublisher] = []
        self._subscriptions: list[_RecordingSubscription] = []
        self._timers: list[_RecordingTimer] = []
        self._logger = _FakeLogger()
        self.destroyed = False

    def declare_parameter(self, name, default):
        self._params[name] = PARAM_OVERRIDES.get(name, default)

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self._params[name])

    def create_publisher(self, msg_type, topic, qos):
        pub = _RecordingPublisher(topic)
        self._publishers.append(pub)
        return pub

    def create_subscription(self, msg_type, topic, callback, qos):
        sub = _RecordingSubscription(msg_type, topic, callback, qos)
        self._subscriptions.append(sub)
        return sub

    def create_timer(self, period, callback):
        timer = _RecordingTimer(period, callback)
        self._timers.append(timer)
        return timer

    def get_logger(self):
        return self._logger

    def destroy_node(self):
        self.destroyed = True


def _yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class _FakeBuffer:
    """只实现 follow_path 用到的 lookup_transform。"""

    def __init__(self) -> None:
        self.pose: tuple[float, float, float] | None = None
        self.raw_quaternion: tuple[float, float, float, float] | None = None
        self.calls = 0

    def lookup_transform(self, target, source, when):
        self.calls += 1
        if self.pose is None:
            raise RuntimeError("测试里 TF 不可用")
        x, y, yaw = self.pose
        if self.raw_quaternion is None:
            quat = _yaw_to_quat(yaw)
        else:
            quat = self.raw_quaternion
        qx, qy, qz, qw = quat
        return types.SimpleNamespace(
            transform=types.SimpleNamespace(
                translation=types.SimpleNamespace(x=x, y=y, z=0.0),
                rotation=types.SimpleNamespace(x=qx, y=qy, z=qz, w=qw),
            )
        )


class _FakeTransformListener:
    def __init__(self, buffer, node) -> None:
        self.buffer = buffer
        self.node = node


_QOS_SENSOR_DATA = object()

_ORIGINAL_MODULES: dict[str, types.ModuleType | None] = {}


def _register(name: str, module) -> None:
    _ORIGINAL_MODULES[name] = sys.modules.get(name)
    sys.modules[name] = module


def _install_ros_stubs() -> types.ModuleType:
    rclpy = types.ModuleType("rclpy")
    rclpy.__path__ = []  # type: ignore[attr-defined]

    state = {"ok": True, "spin_hook": lambda node: None,
             "shutdown_hook": lambda: None}

    def init(args=None) -> None:
        state["ok"] = True

    def ok() -> bool:
        return bool(state["ok"])

    def shutdown() -> None:
        state["ok"] = False
        state["shutdown_hook"]()

    def spin_once(node, timeout_sec=0.05):
        return state["spin_hook"](node)

    rclpy.init = init
    rclpy.ok = ok
    rclpy.shutdown = shutdown
    rclpy.spin_once = spin_once
    rclpy.state = state  # type: ignore[attr-defined]

    executors = types.ModuleType("rclpy.executors")

    class ExternalShutdownException(Exception):
        pass

    executors.ExternalShutdownException = ExternalShutdownException

    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = _FakeNode

    qos_mod = types.ModuleType("rclpy.qos")
    qos_mod.qos_profile_sensor_data = _QOS_SENSOR_DATA

    time_mod = types.ModuleType("rclpy.time")

    class Time:
        def __init__(self) -> None:
            pass

    time_mod.Time = Time

    rclpy.executors = executors
    rclpy.node = node_mod
    rclpy.qos = qos_mod
    rclpy.time = time_mod

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs.__path__ = []  # type: ignore[attr-defined]
    geometry_msg = types.ModuleType("geometry_msgs.msg")

    class Twist:
        def __init__(self) -> None:
            self.linear = types.SimpleNamespace(x=0.0)
            self.angular = types.SimpleNamespace(z=0.0)

    geometry_msg.Twist = Twist
    geometry_msgs.msg = geometry_msg

    std_msgs = types.ModuleType("std_msgs")
    std_msgs.__path__ = []  # type: ignore[attr-defined]
    std_msg = types.ModuleType("std_msgs.msg")

    class String:
        def __init__(self, data: str = "") -> None:
            self.data = data

    std_msg.String = String
    std_msgs.msg = std_msg

    nav_msgs = types.ModuleType("nav_msgs")
    nav_msgs.__path__ = []  # type: ignore[attr-defined]
    nav_msg = types.ModuleType("nav_msgs.msg")

    class Odometry:
        pass

    nav_msg.Odometry = Odometry
    nav_msgs.msg = nav_msg

    tf2_ros = types.ModuleType("tf2_ros")
    tf2_ros.Buffer = _FakeBuffer
    tf2_ros.TransformListener = _FakeTransformListener

    for name, module in (
        ("rclpy", rclpy),
        ("rclpy.executors", executors),
        ("rclpy.node", node_mod),
        ("rclpy.qos", qos_mod),
        ("rclpy.time", time_mod),
        ("geometry_msgs", geometry_msgs),
        ("geometry_msgs.msg", geometry_msg),
        ("std_msgs", std_msgs),
        ("std_msgs.msg", std_msg),
        ("nav_msgs", nav_msgs),
        ("nav_msgs.msg", nav_msg),
        ("tf2_ros", tf2_ros),
    ):
        _register(name, module)

    return rclpy


def _restore_modules() -> None:
    for name, original in _ORIGINAL_MODULES.items():
        if original is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = original
    _ORIGINAL_MODULES.clear()


_ros = _install_ros_stubs()
try:
    from car_bridge import follow_path as follow
finally:
    _restore_modules()


# =====================================================================
# 测试夹具
# =====================================================================
class _FakeClock:
    def __init__(self, start: float = 1000.0) -> None:
        self.now = start

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


RECT = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0), (0.0, 1.0, 0.0)]
LINE = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 1.0)]
#: 起点 (0,0) 与终点 (0,0.02) 几乎重合的闭合路线。
LOOP = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0),
        (0.0, 1.0, 0.0), (0.0, 0.02, 0.0)]


def _interpolate_route(waypoints, step: float = 0.02):
    """把折线按固定步长插值成密集路线，yaw 取各段切向。

    真实示教路线点距约 4 cm；这里用同样量级的点距，避免稀疏折线掩盖
    "欧式前视扫到远处回程段" 的问题。
    """
    pts = [waypoints[0]]
    for a, b in zip(waypoints, waypoints[1:]):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        if seg <= 1e-12:
            pts.append(b)
            continue
        count = max(1, int(math.ceil(seg / step)))
        for k in range(1, count + 1):
            t = k / count
            pts.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    route = []
    for i, (x, y) in enumerate(pts):
        if i + 1 < len(pts):
            yaw = math.atan2(pts[i + 1][1] - y, pts[i + 1][0] - x)
        else:
            yaw = route[-1][2] if route else 0.0
        route.append((x, y, yaw))
    return route


def _uturn_route(x_end: float = 0.6, radius: float = 0.1, step: float = 0.02):
    """出程 -> 半圆掉头 -> 紧邻反向回程（两段平行、只隔 2*radius）。"""
    pts = []
    out_steps = max(1, int(round(x_end / step)))
    for i in range(out_steps + 1):
        pts.append((i * x_end / out_steps, 0.0))
    arc_steps = max(4, int(round(math.pi * radius / step)))
    for i in range(1, arc_steps + 1):
        theta = math.pi * i / arc_steps
        pts.append((x_end + radius * math.sin(theta),
                    radius - radius * math.cos(theta)))
    for i in range(1, out_steps + 1):
        pts.append((x_end - i * x_end / out_steps, 2.0 * radius))
    route = []
    for i, (x, y) in enumerate(pts):
        if i + 1 < len(pts):
            yaw = math.atan2(pts[i + 1][1] - y, pts[i + 1][0] - x)
        else:
            yaw = math.pi
        route.append((x, y, yaw))
    return route


def _figure_eight_route(step: float = 0.02):
    """自交路线：(1,1) 被走到两次，空间重合但弧长相差很远。"""
    return _interpolate_route(
        [(0.0, 0.0), (1.0, 1.0), (2.0, 0.0), (2.0, 2.0),
         (1.0, 1.0), (0.0, 2.0)],
        step,
    )


def _cross_track_m(pose, route) -> float:
    """位姿到折线的实际最短距离（只用于仿真报告，不参与控制）。"""
    x, y = pose[0], pose[1]
    best = float("inf")
    for a, b in zip(route, route[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        seg2 = dx * dx + dy * dy
        if seg2 <= 1e-12:
            t = 0.0
        else:
            t = ((x - a[0]) * dx + (y - a[1]) * dy) / seg2
            t = max(0.0, min(1.0, t))
        px, py = a[0] + t * dx, a[1] + t * dy
        best = min(best, math.hypot(x - px, y - py))
    return best



class FollowPathSafetyTest(unittest.TestCase):
    def setUp(self) -> None:
        # 用测试目录里的短命文件；只清理本测试创建的文件。
        self._tmp_dir = Path(__file__).resolve().parent
        self._created: list[Path] = []
        self.addCleanup(self._cleanup_created)
        self._seq = 0
        self.clock = _FakeClock()
        for patcher in (
            mock.patch("time.monotonic", self.clock),
            mock.patch("time.sleep", lambda _seconds: None),
        ):
            patcher.start()
            self.addCleanup(patcher.stop)
        PARAM_OVERRIDES.clear()
        self.addCleanup(PARAM_OVERRIDES.clear)
        _ros.state["ok"] = True
        _ros.state["spin_hook"] = lambda node: None
        _ros.state["shutdown_hook"] = lambda: None

    # ---------------------------------------------------------- 夹具
    def _cleanup_created(self) -> None:
        for path in self._created:
            try:
                path.unlink()
            except FileNotFoundError:
                pass
        self._created.clear()

    def _temp_path(self, suffix: str) -> str:
        self._seq += 1
        path = self._tmp_dir / f".follow_test_{os.getpid()}_{self._seq}{suffix}"
        self._created.append(path)
        return str(path)

    def _write_path(self, points, frame: str) -> str:
        path = self._temp_path(".yaml")
        frame_line = 'frame_id: ""' if frame == "" else f"frame_id: {frame}"
        lines = [frame_line, "points:"]
        for x, y, yaw in points:
            lines.append(f"  - {{x: {x!r}, y: {y!r}, yaw: {yaw!r}}}")
        Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")
        return path

    def _set_params(self, **overrides) -> None:
        params = {
            "start_delay_s": 0.0,
            "rate_hz": 20.0,
            "require_status": False,
            "allow_legacy_odom": False,
            "odom_timeout_s": 0.5,
            "goal_tol": 0.2,
            "align_final_yaw": True,
            "save_actual": "",
            # 任务01/02 的旧 helper 显式跳过地图 bundle 校验；
            # 新增的严格模式用例会显式打开。
            "require_map_bundle": False,
        }
        params.update(overrides)
        PARAM_OVERRIDES.clear()
        PARAM_OVERRIDES.update(params)
        self.addCleanup(PARAM_OVERRIDES.clear)

    def _node(self, points, frame: str = "map", **overrides):
        # 任务01回归用例只测旧跟踪安全行为，显式关闭自动回起点。
        overrides.setdefault("return_to_start", False)
        return self._node_from_path(self._write_path(points, frame), **overrides)

    def _node_default_start(self, points, frame: str = "map", **overrides):
        """走 ROS 默认参数（return_to_start=True），专门测新的回起点流程。"""
        return self._node_from_path(self._write_path(points, frame), **overrides)

    def _strict_node(self, points, map_bytes: bytes = b"TEST_MAP",
                     bundle_base_frame: str = "imu", frame: str = "map",
                     **overrides):
        """任务03：建一份真实 bundle，然后按跟随默认 require_map_bundle=True 构造。"""
        path_file = self._write_path(points, frame)
        map_file = self._temp_path(".bin")
        Path(map_file).write_bytes(map_bytes)
        manifest = Path(rb.write_bundle(
            path_file, map_file, base_frame=bundle_base_frame))
        self._created.append(manifest)
        overrides.setdefault("require_map_bundle", True)
        overrides.setdefault("map_file", map_file)
        node = self._node_from_path(path_file, **overrides)
        node._test_path_file = path_file
        node._test_map_file = map_file
        return node

    def _node_from_path(self, path_file: str, **overrides):
        self._set_params(path_file=path_file, **overrides)
        node = follow.FollowPath()
        node._tf_buffer.pose = (0.0, 0.0, 0.0)
        self.addCleanup(node.destroy_node)
        return node

    def _ros_default_node_from_path(self, path_file: str, **overrides):
        """不覆盖 require_map_bundle，验证 ROS 默认严格行为。"""
        self._set_params(path_file=path_file, **overrides)
        PARAM_OVERRIDES.pop("require_map_bundle", None)
        node = follow.FollowPath()
        node._tf_buffer.pose = (0.0, 0.0, 0.0)
        self.addCleanup(node.destroy_node)
        return node

    def _odom_msg(self, stamp: float, x: float = 0.0, y: float = 0.0, yaw: float = 0.0,
                  frame: str = ODOM_FRAME, child: str = ODOM_CHILD, orientation=None):
        sec = int(math.floor(stamp))
        nsec = int(round((stamp - sec) * 1e9))
        qx, qy, qz, qw = _yaw_to_quat(yaw)
        if orientation is not None:
            qx, qy, qz, qw = orientation
        return types.SimpleNamespace(
            header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(sec=sec, nanosec=nsec),
                frame_id=frame,
            ),
            child_frame_id=child,
            pose=types.SimpleNamespace(
                pose=types.SimpleNamespace(
                    position=types.SimpleNamespace(x=x, y=y, z=0.0),
                    orientation=types.SimpleNamespace(x=qx, y=qy, z=qz, w=qw),
                )
            ),
        )

    def _odom_sub(self, node):
        return next(s for s in node._subscriptions if s.topic == ODOM_TOPIC)

    def _feed(self, node, stamp: float, x: float = 0.0, y: float = 0.0, yaw: float = 0.0,
              **kwargs) -> None:
        self._odom_sub(node).callback(self._odom_msg(stamp, x, y, yaw, **kwargs))

    def _feed_status(self, node, payload: dict) -> None:
        sub = next(s for s in node._subscriptions if s.topic == "car/status")
        sub.callback(types.SimpleNamespace(data=json.dumps(payload)))

    def _prime_run(self, node, idx: int, pose, traveled=None) -> None:
        """把节点摆到"已沿路线走到 idx"的物理一致状态，单拍只测控制决策。"""
        x, y = pose[0], pose[1]
        node._state = "run"
        node._start_checked = True
        node._start = 0.0
        node._idx = idx
        node._traveled = node.s[idx] if traveled is None else traveled
        node._last_pose = (x, y)
        node._done = False
        node._outcome = None
        node._reason = ""
        node._align_start = None
        node._track_align_t0 = None
        node._tf_fail_since = None

    def _fixture_forward_route(self):
        """只读实测路线的前向派生视图：原索引 0..329（首次倒车段 329->330 之前）。"""
        fixture = (Path(__file__).resolve().parents[3]
                   / ".dsh" / "cache" / "task04-fixtures" / "route_map.yaml")
        if not fixture.exists():
            self.skipTest(f"没有只读实测路线夹具 {fixture}")
        _frame, pts = follow.load_path(
            str(fixture), require_frame=True, require_finite=True)
        return [(x, y, yaw) for x, y, yaw, _t in pts[:330]]

    def _tick_pose(self, node, stamp: float, pose, dt: float = 0.1) -> None:
        """喂一帧有效原始里程计 + 设置 TF 位姿 + 推进时钟 + tick。"""
        x, y, yaw = pose
        self._feed(node, stamp, x, y, yaw)
        self.clock.advance(dt)
        node._tf_buffer.pose = pose
        node._tick()

    def _commands(self, node):
        return [sent for pub in node._publishers for sent in pub.sent]

    def _assert_all_zero(self, node) -> None:
        commands = self._commands(node)
        self.assertTrue(commands, "应该至少发过一条 cmd_vel")
        for linear, angular in commands:
            self.assertEqual((linear, angular), (0.0, 0.0))

    def _assert_tail_zero(self, node, count: int = 3) -> None:
        commands = self._commands(node)
        self.assertGreaterEqual(len(commands), count)
        for linear, angular in commands[-count:]:
            self.assertEqual((linear, angular), (0.0, 0.0))

    # ---------------------------------------------------------- 路线参考系
    def test_map_route_loads(self) -> None:
        node = self._node(RECT, frame="map")
        self.assertEqual(node.frame, "map")
        # helper 显式关了 return_to_start，会有一条诊断警告；这里只关心没有历史路线警告。
        warnings = "\n".join(node.get_logger().warns)
        self.assertNotIn("路线参考系是历史", warnings)

    def test_legacy_odom_rejected_by_default(self) -> None:
        with self.assertRaises(ValueError) as ctx:
            self._node(RECT, frame="odom")
        self.assertIn("allow_legacy_odom", str(ctx.exception))

    def test_legacy_odom_allowed_with_warning(self) -> None:
        node = self._node(RECT, frame="odom", allow_legacy_odom=True)
        self.assertEqual(node.frame, "odom")
        warnings = "\n".join(node.get_logger().warns)
        self.assertIn("历史", warnings)
        self.assertIn("不代表跨会话安全", warnings)

    def test_unknown_route_frame_rejected(self) -> None:
        for frame in ("world", ""):
            with self.subTest(frame=frame):
                with self.assertRaises(ValueError):
                    self._node(RECT, frame=frame)

    def test_missing_frame_id_rejected(self) -> None:
        path = self._temp_path(".yaml")
        Path(path).write_text(
            "points:\n"
            "  - {x: 0.0, y: 0.0, yaw: 0.0}\n"
            "  - {x: 1.0, y: 0.0, yaw: 0.0}\n"
            "  - {x: 2.0, y: 0.0, yaw: 0.0}\n",
            encoding="utf-8",
        )
        with self.assertRaises(ValueError) as ctx:
            self._node_from_path(path)
        self.assertIn("frame_id", str(ctx.exception))

    def test_non_finite_route_values_rejected(self) -> None:
        bad_points = [
            (float("nan"), 0.0, 0.0),
            (float("inf"), 0.0, 0.0),
            (0.0, float("inf"), 0.0),
            (0.0, 0.0, float("nan")),
            (0.0, 0.0, float("inf")),
        ]
        for bad in bad_points:
            with self.subTest(point=bad):
                path = self._write_path([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), bad], "map")
                with self.assertRaises(ValueError) as ctx:
                    self._node_from_path(path)
                self.assertIn("NaN", str(ctx.exception))

        path = self._temp_path(".yaml")
        Path(path).write_text(
            "frame_id: map\npoints:\n"
            "  - {x: 0.0, y: 0.0}\n"
            "  - {x: 1.0, y: 0.0}\n"
            "  - {x: 2.0, y: 0.0, t: .nan}\n",
            encoding="utf-8",
        )
        with self.assertRaises(ValueError):
            self._node_from_path(path)

    # ---------------------------------------------------------- 订阅
    # ---------------------------------------------------------- 地图 bundle
    def test_ros_default_requires_map_bundle(self) -> None:
        path_file = self._write_path(RECT, "map")
        with self.assertRaises(ValueError) as ctx:
            self._ros_default_node_from_path(path_file)
        self.assertIn("map_file", str(ctx.exception))

    def test_ros_default_accepts_valid_bundle(self) -> None:
        path_file = self._write_path(RECT, "map")
        map_file = self._temp_path(".bin")
        Path(map_file).write_bytes(b"ROS_DEFAULT_MAP")
        manifest = Path(rb.write_bundle(path_file, map_file, base_frame="imu"))
        self._created.append(manifest)
        node = self._ros_default_node_from_path(path_file, map_file=map_file)
        self.assertTrue(node.require_map_bundle)
        self.assertIsNotNone(node._bundle_metadata)

    def test_strict_map_route_requires_valid_bundle(self) -> None:
        node = self._strict_node(RECT)
        self.assertTrue(node.require_map_bundle)
        self.assertIsNotNone(node._bundle_metadata)
        self.assertEqual(node._bundle_metadata["point_count"], len(RECT))
        self.assertEqual(node._bundle_metadata["base_frame"], "imu")

    def test_strict_map_route_rejects_missing_map_file(self) -> None:
        path_file = self._write_path(RECT, "map")
        with self.assertRaises(ValueError) as ctx:
            self._node_from_path(path_file, require_map_bundle=True)
        self.assertIn("map_file", str(ctx.exception))

    def test_strict_map_route_rejects_missing_manifest(self) -> None:
        path_file = self._write_path(RECT, "map")
        map_file = self._temp_path(".bin")
        Path(map_file).write_bytes(b"NO_MANIFEST")
        with self.assertRaises(ValueError) as ctx:
            self._node_from_path(path_file, require_map_bundle=True,
                                 map_file=map_file)
        self.assertIn("manifest", str(ctx.exception))

    def test_strict_map_route_rejects_changed_map(self) -> None:
        node = self._strict_node(RECT)
        map_file = node._test_map_file
        Path(map_file).write_bytes(b"CHANGED_AFTER_BUNDLE")
        with self.assertRaises(ValueError) as ctx:
            self._node_from_path(node._test_path_file, require_map_bundle=True,
                                 map_file=map_file)
        self.assertIn("map", str(ctx.exception).lower())

    def test_strict_map_route_rejects_changed_route(self) -> None:
        node = self._strict_node(RECT)
        path_file = Path(node._test_path_file)
        path_file.write_text(path_file.read_text(encoding="utf-8") + "\n",
                             encoding="utf-8")
        with self.assertRaises(ValueError) as ctx:
            self._node_from_path(str(path_file), require_map_bundle=True,
                                 map_file=node._test_map_file)
        self.assertIn("path", str(ctx.exception).lower())

    def test_isolation_mode_without_bundle_warns(self) -> None:
        node = self._node(RECT)
        warnings = "\n".join(node.get_logger().warns)
        self.assertIn("require_map_bundle=False", warnings)

    def test_strict_map_route_enforces_base_frame(self) -> None:
        with self.assertRaises(ValueError) as ctx:
            self._strict_node(RECT, base_frame="base_link")
        self.assertIn("base_frame", str(ctx.exception))

    def test_raw_odometry_uses_sensor_data_qos(self) -> None:
        node = self._node(RECT)
        sub = self._odom_sub(node)
        self.assertEqual(sub.topic, ODOM_TOPIC)
        self.assertIs(sub.qos, _QOS_SENSOR_DATA)

    # ---------------------------------------------------------- 回起点流程
    def test_default_return_to_start_flow(self) -> None:
        node = self._node_default_start(LINE)
        self.assertTrue(node.return_to_start)      # 没被测试 helper 关掉
        self.assertEqual(node.start_max_offset_m, 0.5)
        self.assertEqual(node.start_pos_tol_m, 0.10)
        self.assertEqual(node.start_yaw_tol_rad, 0.10)
        self.assertEqual(node.return_v_max, 0.08)
        self.assertEqual(node.return_w_max, 0.4)
        self.assertEqual(node.return_timeout_s, 30.0)
        self.assertEqual(node._state, "return")

        self._tick_pose(node, 1.0, (0.0, 0.0, 0.0))
        self.assertEqual(node._state, "align_start")
        self.assertEqual(self._commands(node)[-1], (0.0, 0.0))

        self._tick_pose(node, 1.1, (0.0, 0.0, 0.0))
        self.assertEqual(node._state, "run")
        self.assertEqual(self._commands(node)[-1], (0.0, 0.0))
        self.assertAlmostEqual(node._start_elapsed, 0.1, places=6)
        self.assertIn("回起点", node.summary())

        self._tick_pose(node, 1.2, (0.0, 0.0, 0.0))
        self.assertFalse(node._done)
        self.assertGreater(self._commands(node)[-1][0], 0.0)

    def test_return_approach_goes_forward_and_slows_down(self) -> None:
        node = self._node_default_start(LINE)
        self._tick_pose(node, 1.0, (-0.2, 0.0, 0.0))
        v_far, w_far = self._commands(node)[-1]
        self.assertEqual(node._state, "return")
        self.assertGreater(v_far, 0.0)
        self.assertAlmostEqual(w_far, 0.0, places=6)

        self._tick_pose(node, 1.1, (-0.12, 0.0, 0.0))
        v_near, _ = self._commands(node)[-1]
        self.assertEqual(node._state, "return")
        self.assertGreater(v_near, 0.0)
        self.assertLess(v_near, v_far)       # 距离越近越慢

        self._tick_pose(node, 1.2, (0.0, 0.0, 0.0))
        self.assertEqual(node._state, "align_start")
        self.assertEqual(self._commands(node)[-1], (0.0, 0.0))

    def test_return_rotates_in_place_for_large_heading_error(self) -> None:
        node = self._node_default_start(LINE)
        # 目标在车后方：方位误差接近 pi，只能原地转，不能倒车或前进。
        self._tick_pose(node, 1.0, (0.2, 0.0, 0.0))
        v, w = self._commands(node)[-1]
        self.assertEqual(node._state, "return")
        self.assertEqual(v, 0.0)
        self.assertGreater(w, 0.0)

    def test_start_align_uses_shortest_rotation_without_forward(self) -> None:
        for yaw, expected_sign in ((math.pi / 2, -1.0),
                                   (math.pi - 0.05, -1.0),
                                   (-math.pi + 0.05, 1.0)):
            with self.subTest(yaw=yaw):
                node = self._node_default_start(LINE)
                self._tick_pose(node, 1.0, (0.0, 0.0, yaw))
                self.assertEqual(node._state, "align_start")
                self._tick_pose(node, 1.1, (0.0, 0.0, yaw))
                v, w = self._commands(node)[-1]
                self.assertEqual(node._state, "align_start")
                self.assertEqual(v, 0.0)
                self.assertLessEqual(abs(w), node.return_w_max)
                self.assertEqual(math.copysign(1.0, w), expected_sign)

    def test_start_offset_beyond_limit_fails(self) -> None:
        node = self._node_default_start(LINE)
        self._tick_pose(node, 1.0, (0.51, 0.0, 0.0))
        self.assertTrue(node._done)
        self.assertEqual(node._outcome, follow.OUTCOME_FAILED)
        self.assertIn("起点偏差", node._reason)
        self._assert_all_zero(node)

    def test_return_does_not_enter_run_before_start_reached(self) -> None:
        node = self._node_default_start(LINE)
        for stamp, x in ((1.0, -0.3), (1.1, -0.25), (1.2, -0.2)):
            self._tick_pose(node, stamp, (x, 0.0, 0.0))
            self.assertEqual(node._state, "return")
            self.assertFalse(node._done)
            self.assertGreater(self._commands(node)[-1][0], 0.0)

    def test_start_position_but_heading_not_ready_enters_run(self) -> None:
        node = self._node_default_start(LINE)
        self._tick_pose(node, 1.0, (0.0, 0.0, 0.0))
        self.assertEqual(node._state, "align_start")

        self._tick_pose(node, 1.1, (0.0, 0.0, 0.2))
        v, w = self._commands(node)[-1]
        self.assertEqual(node._state, "align_start")
        self.assertEqual(v, 0.0)
        self.assertNotEqual(w, 0.0)

        self._tick_pose(node, 1.2, (0.0, 0.0, 0.05))
        self.assertEqual(node._state, "run")
        self.assertEqual(self._commands(node)[-1], (0.0, 0.0))

    def test_return_driving_turn_rate_limited_by_turn_radius(self) -> None:
        # 实车首测：边走边转的角速度太大（等效转弯半径 0.14 m < 到位容差
        # 0.10 m），车绕着起点打圈、到位时朝向 162° 反着。行驶中的角速度必须
        # 按 w <= v/RETURN_MIN_RADIUS_M 封顶。
        node = self._node_default_start(LINE)
        self._tick_pose(node, 1.0, (-0.30, -0.10, 0.0))
        v, w = self._commands(node)[-1]
        self.assertEqual(node._state, "return")
        self.assertGreater(v, 0.0)
        self.assertGreater(w, 0.0)
        self.assertLessEqual(abs(w), v / follow.RETURN_MIN_RADIUS_M + 1e-9)
        # 半径封顶必须真的起作用（未封顶时会是 1.5 * 方位误差）
        heading_err = math.atan2(0.10, 0.30)
        self.assertLess(abs(w), 1.5 * heading_err)

    def test_return_in_place_rotation_not_radius_limited(self) -> None:
        # 原地转（v=0）没有转弯半径概念，必须仍用满 return_w_max。
        node = self._node_default_start(LINE)
        self._tick_pose(node, 1.0, (0.2, 0.0, 0.0))
        v, w = self._commands(node)[-1]
        self.assertEqual(v, 0.0)
        self.assertAlmostEqual(abs(w), node.return_w_max, places=9)

    def test_start_align_ignores_position_while_rotating(self) -> None:
        # 方案 B：align_start 只负责朝向。传感器偏离转心约 0.25 m，原地转正
        # 本身就会让位姿点扫过几十厘米，所以转动期间绝不能拿位置判失败。
        node = self._node_default_start(LINE)
        self._tick_pose(node, 1.0, (0.0, 0.0, 0.0))
        self.assertEqual(node._state, "align_start")
        # 已漂 0.30 m，朝向还差 0.30 rad（仍在对齐过程中）
        self._tick_pose(node, 1.1, (0.30, 0.0, 0.30))
        self.assertFalse(node._done, node._reason)
        self.assertEqual(node._state, "align_start")
        v, w = self._commands(node)[-1]
        self.assertEqual(v, 0.0)
        self.assertNotEqual(w, 0.0)

    def test_start_align_returns_to_start_when_heading_done_but_off_position(self) -> None:
        # 朝向已对齐但位置差超过容差：不能直接进 run，要回 return 补位。
        node = self._node_default_start(LINE)
        self._tick_pose(node, 1.0, (0.0, 0.0, 0.0))
        self.assertEqual(node._state, "align_start")
        self._tick_pose(node, 1.1, (0.30, 0.0, 0.0))
        self.assertFalse(node._done, node._reason)
        self.assertEqual(node._state, "return")
        self.assertEqual(self._commands(node)[-1], (0.0, 0.0))
        self.assertEqual(node._start_roundtrips, 1)
        self.assertTrue(
            any("补位" in message for message in node.get_logger().infos),
            node.get_logger().infos,
        )

    def test_start_roundtrip_converges_and_runs(self) -> None:
        node = self._node_default_start(LINE)
        self._tick_pose(node, 1.0, (0.0, 0.0, 0.0))     # return -> align_start
        self._tick_pose(node, 1.1, (0.30, 0.0, 0.0))    # 朝向已对、位置差 -> return
        self.assertEqual(node._state, "return")
        self._tick_pose(node, 1.2, (0.02, 0.0, 0.02))   # 补位到容差内 -> align_start
        self.assertEqual(node._state, "align_start")
        self._tick_pose(node, 1.3, (0.02, 0.0, 0.02))   # 位置/朝向都达标 -> run
        self.assertEqual(node._state, "run")
        self.assertFalse(node._done)
        self.assertEqual(node._start_roundtrips, 1)
        self.assertIn("补位往返 1 次", node.summary())
        # 先转正再补位的全过程只允许前进或原地转，绝不倒车。
        for linear, _ in self._commands(node):
            self.assertGreaterEqual(linear, 0.0)

    def test_start_roundtrip_limit_fails(self) -> None:
        # 每轮都"转正→位置差→补位"，若始终不收敛，超轮次必须失败停车。
        node = self._node_default_start(LINE)
        self._tick_pose(node, 1.0, (0.0, 0.0, 0.0))
        for i in range(follow.START_RETURN_ROUNDTRIP_MAX + 1):
            self._tick_pose(node, 1.1 + 0.2 * i, (0.30, 0.0, 0.0))
            if node._done:
                break
            self._tick_pose(node, 1.2 + 0.2 * i, (0.0, 0.0, 0.0))
        self.assertTrue(node._done)
        self.assertEqual(node._outcome, follow.OUTCOME_FAILED)
        self.assertIn("往返", node._reason)
        self.assertEqual(node._start_roundtrips,
                         follow.START_RETURN_ROUNDTRIP_MAX + 1)
        self._assert_tail_zero(node)

    def _simulate_start_flow(self, r: float, along: float, yaw_deg: float,
                             seconds_budget: float = 90.0):
        """用"转心 + 传感器偏置"的差速车运动学跑一遍起始流程。

        返回 (node, entered_run, sensor, theta, trace)。r 是 base_frame 相对
        转心的距离，along 是传感器在路线起点 +x 方向的位置，yaw_deg 是初始
        车身朝向与路线起点朝向的差。
        """
        node = self._node_default_start(LINE, return_timeout_s=seconds_budget)
        theta = math.radians(yaw_deg)
        center = (along - r * math.cos(theta), -r * math.sin(theta))
        stamp = 1.0
        entered_run = False
        trace: list[str] = []
        for _ in range(int(seconds_budget * 10) + 50):
            sensor = (center[0] + r * math.cos(theta),
                      center[1] + r * math.sin(theta))
            self._tick_pose(node, stamp, (sensor[0], sensor[1], theta))
            stamp += 0.1
            if node._done or node._state == "run":
                entered_run = node._state == "run"
                break
            v, w = self._commands(node)[-1]
            trace.append(
                f"{node._state:11s} dist={math.hypot(sensor[0], sensor[1]):.3f} "
                f"yaw_err={follow.wrap_pi(-theta):+.3f} v={v:.3f} w={w:+.3f}")
            if len(trace) > 400:
                trace.pop(0)
            theta += w * 0.1
            center = (center[0] + v * math.cos(theta) * 0.1,
                      center[1] + v * math.sin(theta) * 0.1)
        sensor = (center[0] + r * math.cos(theta),
                  center[1] + r * math.sin(theta))
        return node, entered_run, sensor, theta, trace

    def test_start_roundtrip_converges_without_lever_arm(self) -> None:
        # 先证明状态机本身对：没有传感器偏置时，0.45 m / 140° 能收敛进 run。
        node, entered_run, sensor, theta, trace = self._simulate_start_flow(
            r=0.0, along=-0.45, yaw_deg=140.0)
        self.assertTrue(
            entered_run,
            f"没进 run，最终状态 {node._state}，原因 {node._reason}，"
            f"往返 {node._start_roundtrips} 次\n" + "\n".join(trace[-25:]),
        )
        self.assertLessEqual(node._start_roundtrips,
                             follow.START_RETURN_ROUNDTRIP_MAX)
        self.assertLessEqual(math.hypot(sensor[0], sensor[1]),
                             node.start_pos_tol_m + 1e-6)
        self.assertLessEqual(abs(follow.wrap_pi(-theta)),
                             node.start_yaw_tol_rad + 1e-6)
        self.assertLessEqual(node._start_elapsed, 30.0)
        for linear, _ in self._commands(node):
            self.assertGreaterEqual(linear, 0.0)
        self.assertTrue(any(v > 0.0 for v, _ in self._commands(node)))

    def test_strict_start_with_lever_arm_does_not_converge(self) -> None:
        # 已确认的物理缺陷（Odin1 偏离转心 0.19–0.25 m）会让"严格起点 + 先转正
        # 再补位"进入极限环：align 要把朝向转回路线方向，而 return 要把车头对准
        # 目标点，两个目标在同一几何下互相抵消（实测 dist 在 0.099~0.137 m 之间
        # 来回）。这条用例固定的是"失败必须安全"：有界往返、写零停车、绝不倒车。
        # 结论：严格判据在力臂存在时不可能达标 → 起步要用力臂补偿或宽松起点。
        node, entered_run, _sensor, _theta, trace = self._simulate_start_flow(
            r=0.25, along=-0.45, yaw_deg=140.0)
        self.assertFalse(entered_run)
        self.assertTrue(node._done)
        self.assertEqual(node._outcome, follow.OUTCOME_FAILED)
        self.assertIn("往返", node._reason)
        self.assertEqual(node._start_roundtrips,
                         follow.START_RETURN_ROUNDTRIP_MAX + 1)
        for linear, _ in self._commands(node):
            self.assertGreaterEqual(linear, 0.0)
        self._assert_tail_zero(node)

    def test_loose_start_converges_with_lever_arm(self) -> None:
        # 方案 A 的参数组合（0.50 m / 34°）：允许的转角小到"转正带来的等效平移"
        # 仍落在位置容差内，所以同一物理几何下能直接进 run，交给纯追踪收敛。
        for label, (along, yaw_deg) in (
            ("后方 0.45 m / 30°", (-0.45, 30.0)),
            ("侧方 0.30 m / 30°", (0.30, 30.0)),
        ):
            with self.subTest(case=label):
                node = self._node_default_start(
                    LINE,
                    start_pos_tol_m=0.50,
                    start_yaw_tol_rad=math.radians(34.0),
                    return_timeout_s=90.0,
                )
                theta = math.radians(yaw_deg)
                r = 0.25
                center = (along - r * math.cos(theta), -r * math.sin(theta))
                stamp = 1.0
                entered_run = False
                trace = []
                for _ in range(900):
                    sensor = (center[0] + r * math.cos(theta),
                              center[1] + r * math.sin(theta))
                    self._tick_pose(node, stamp, (sensor[0], sensor[1], theta))
                    stamp += 0.1
                    if node._done or node._state == "run":
                        entered_run = node._state == "run"
                        break
                    v, w = self._commands(node)[-1]
                    trace.append(f"{node._state:11s} "
                                 f"dist={math.hypot(sensor[0], sensor[1]):.3f} "
                                 f"yaw_err={follow.wrap_pi(-theta):+.3f}")
                    if len(trace) > 400:
                        trace.pop(0)
                    theta += w * 0.1
                    center = (center[0] + v * math.cos(theta) * 0.1,
                              center[1] + v * math.sin(theta) * 0.1)
                self.assertTrue(
                    entered_run,
                    f"{label}: 没进 run，最终状态 {node._state}，"
                    f"原因 {node._reason}，往返 {node._start_roundtrips} 次\n"
                    + "\n".join(trace[-25:]),
                )
                self.assertLessEqual(node._start_roundtrips,
                                     follow.START_RETURN_ROUNDTRIP_MAX)

    def test_return_and_start_align_share_one_timeout(self) -> None:
        for pose in ((-0.2, 0.0, 0.0), (0.0, 0.0, 0.0)):
            with self.subTest(pose=pose):
                node = self._node_default_start(LINE)
                self._tick_pose(node, 1.0, pose)
                state = node._state
                self.assertIn(state, ("return", "align_start"))
                self.clock.advance(30.1)
                self._feed(node, 2.0, *pose)   # 在 tick 时刻喂新鲜数据，专测总超时
                node._tf_buffer.pose = pose
                node._tick()
                self.assertTrue(node._done)
                self.assertEqual(node._outcome, follow.OUTCOME_FAILED)
                self.assertIn("超时", node._reason)

    def test_return_phase_still_honors_task01_safety_gates(self) -> None:
        # 原始里程计停更
        node = self._node_default_start(LINE)
        self._tick_pose(node, 1.0, (-0.2, 0.0, 0.0))
        self.clock.advance(0.6)
        node._tf_buffer.pose = (-0.2, 0.0, 0.0)
        node._tick()
        self.assertTrue(node._done)
        self.assertIn("停更", node._reason)

        # 有效帧之后来非法帧
        node = self._node_default_start(LINE)
        self._tick_pose(node, 1.0, (-0.2, 0.0, 0.0))
        self._feed(node, 2.0, x=float("nan"))
        self.clock.advance(0.1)
        node._tf_buffer.pose = (-0.2, 0.0, 0.0)
        node._tick()
        self.assertTrue(node._done)
        self.assertEqual(node._outcome, follow.OUTCOME_FAILED)
        self._assert_tail_zero(node)

        # TF 丢失（沿用任务01的 tf_timeout 失败）
        node = self._node_default_start(LINE)
        self._tick_pose(node, 1.0, (-0.2, 0.0, 0.0))
        stamp = 2.0
        for _ in range(8):
            self._feed(node, stamp, -0.2, 0.0, 0.0)
            stamp += 0.1
            self.clock.advance(0.1)
            node._tf_buffer.pose = None
            node._tick()
            if node._done:
                break
        self.assertTrue(node._done)
        self.assertEqual(node._outcome, follow.OUTCOME_FAILED)
        self.assertIn("TF", node._reason)
        self._assert_tail_zero(node)

    def test_return_distance_not_counted_as_route_progress(self) -> None:
        short_loop = [(0.0, 0.0, 0.0), (0.1, 0.0, 0.0), (0.1, 0.1, 0.0),
                      (0.0, 0.1, 0.0), (0.0, 0.0, 0.0)]
        node = self._node_default_start(short_loop)
        self._tick_pose(node, 1.0, (-0.2, 0.0, 0.0))   # 回起点走 0.2 m
        self.assertGreater(self._commands(node)[-1][0], 0.0)
        self._tick_pose(node, 1.1, (0.0, 0.0, 0.0))
        self.assertEqual(node._state, "align_start")
        self._tick_pose(node, 1.2, (0.0, 0.0, 0.0))
        self.assertEqual(node._state, "run")
        self.assertEqual(node._traveled, 0.0)           # 回起点里程已清掉

        self._tick_pose(node, 1.3, (0.0, 0.0, 0.0))
        self.assertFalse(node._done)
        self.assertIsNone(node._outcome)
        self.assertEqual(node._idx, 0)                  # 闭合路线从首段开始
        self.assertGreater(self._commands(node)[-1][0], 0.0)

    def test_closed_loop_runs_from_first_segment_after_return(self) -> None:
        route = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0),
                 (0.0, 1.0, 0.0), (0.05, 0.0, 0.0)]
        node = self._node_default_start(route)
        # 起点就在末点旁边（闭合路线），但还没走，必须从首段开始。
        self._tick_pose(node, 1.0, (0.05, 0.0, 0.0))
        self.assertEqual(node._state, "align_start")
        self._tick_pose(node, 1.1, (0.05, 0.0, 0.0))
        self.assertEqual(node._state, "run")
        self._tick_pose(node, 1.2, (0.05, 0.0, 0.0))
        self.assertEqual(node._idx, 0)
        self.assertFalse(node._done)
        self.assertGreater(self._commands(node)[-1][0], 0.0)

    def test_diagnostic_mode_skips_return_to_start(self) -> None:
        node = self._node(LINE)                         # helper 显式 False
        warnings = "\n".join(node.get_logger().warns)
        self.assertIn("return_to_start=False", warnings)
        self._tick_pose(node, 1.0, (-0.2, 0.0, 0.0))
        self.assertEqual(node._state, "run")
        self.assertGreater(self._commands(node)[-1][0], 0.0)

    def test_legacy_odom_cannot_auto_return(self) -> None:
        with self.assertRaises(ValueError) as ctx:
            self._node_default_start(RECT, frame="odom", allow_legacy_odom=True)
        self.assertIn("return_to_start=False", str(ctx.exception))

    def test_invalid_start_parameters_rejected(self) -> None:
        bad_cases = [
            ("start_max_offset_m", float("nan")),
            ("start_max_offset_m", float("inf")),
            ("start_max_offset_m", -0.1),
            ("start_max_offset_m", 0.0),
            ("start_pos_tol_m", -0.1),
            ("start_pos_tol_m", 0.0),
            ("start_yaw_tol_rad", float("nan")),
            ("start_yaw_tol_rad", 0.0),
            ("return_v_max", -0.01),
            ("return_v_max", float("inf")),
            ("return_w_max", -0.01),
            ("return_timeout_s", 0.0),
            ("return_timeout_s", float("nan")),
        ]
        for name, value in bad_cases:
            with self.subTest(name=name, value=value):
                with self.assertRaises(ValueError):
                    self._node(RECT, **{name: value})

        node = self._node_default_start(LINE, return_v_max=0.0, return_w_max=0.0)
        self.assertEqual(node.return_v_max, 0.0)
        self.assertEqual(node.return_w_max, 0.0)

    # ---------------------------------------------------------- 启动/停更
    def test_startup_grace_period_only_publishes_zero(self) -> None:
        node = self._node(RECT, start_delay_s=2.0)
        self._feed(node, 1.0)          # 宽限期内即使里程计已经来了
        self.clock.advance(0.5)
        node._tick()
        self.clock.advance(1.0)        # 仍在宽限期内
        node._tick()
        self.assertEqual(self._commands(node), [(0.0, 0.0), (0.0, 0.0)])
        self.assertFalse(node._done)

    def test_no_odometry_publishes_zero_and_fails(self) -> None:
        node = self._node(RECT)
        node._tf_buffer.pose = (0.0, 0.0, 0.0)   # TF 正常也没用
        node._tick()
        self.assertTrue(node._done)
        self.assertEqual(node._outcome, follow.OUTCOME_FAILED)
        self.assertIn("里程计", node._reason)
        self._assert_all_zero(node)

    def test_fresh_tf_but_stale_odometry_stops(self) -> None:
        node = self._node(RECT)
        self._feed(node, 1.0)
        self.clock.advance(0.1)
        node._tf_buffer.pose = (0.0, 0.0, 0.0)
        node._tick()
        self.assertTrue(any(v > 0.0 for v, _ in self._commands(node)),
                        "里程计新鲜时应该正常发非零指令")

        self.clock.advance(0.6)        # 距上次有效接收 0.6 s > 0.5 s
        node._tf_buffer.pose = (0.0, 0.0, 0.0)
        node._tick()
        self.assertTrue(node._done)
        self.assertEqual(node._outcome, follow.OUTCOME_FAILED)
        self.assertIn("停更", node._reason)
        self._assert_tail_zero(node)

    def test_repeated_stamp_does_not_keep_odometry_alive(self) -> None:
        node = self._node(RECT)
        self._feed(node, 1.0)
        self.clock.advance(0.1)
        node._tick()
        self.clock.advance(0.3)        # t = 1000.4
        self._feed(node, 1.0)          # 时间戳没变
        self.clock.advance(0.2)        # t = 1000.6，距最初接收 0.6 s
        node._tf_buffer.pose = (0.0, 0.0, 0.0)
        node._tick()
        self.assertTrue(node._done)
        self.assertEqual(node._outcome, follow.OUTCOME_FAILED)
        self.assertIn("停更", node._reason)

    def test_stationary_pose_with_advancing_stamps_stays_active(self) -> None:
        node = self._node(RECT)
        for stamp in (1.0, 1.1, 1.2):
            self._feed(node, stamp)     # 位姿不变，但 stamp 在推进
            self.clock.advance(0.1)
            node._tf_buffer.pose = (0.0, 0.0, 0.0)
            node._tick()
        self.assertFalse(node._done)
        self.assertGreater(self._commands(node)[-1][0], 0.0)

    # ---------------------------------------------------------- 拒绝坏数据
    def test_backward_stamp_faults_without_resuming(self) -> None:
        node = self._node(RECT)
        self._feed(node, 1.0)
        self.clock.advance(0.1)
        node._tick()
        self._feed(node, 0.9)          # 设备时间倒退
        node._tick()
        self.assertTrue(node._done)
        self.assertEqual(node._outcome, follow.OUTCOME_FAILED)
        self.assertIn("倒退", node._reason)
        self._assert_tail_zero(node)

        count = len(self._commands(node))
        self._feed(node, 2.0)          # 来了更新的时间戳也不自动恢复
        self.clock.advance(0.1)
        node._tick()
        self.assertEqual(len(self._commands(node)), count)

    def test_zero_stamp_does_not_refresh(self) -> None:
        node = self._node(RECT)
        self._feed(node, 1.0)
        self.clock.advance(0.1)
        node._tick()
        self.clock.advance(0.3)
        self._feed(node, 0.0)          # 零时间戳
        self.clock.advance(0.2)
        node._tf_buffer.pose = (0.0, 0.0, 0.0)
        node._tick()
        self.assertTrue(node._done)
        self.assertEqual(node._outcome, follow.OUTCOME_FAILED)

    def test_non_finite_position_rejected(self) -> None:
        for bad in (float("nan"), float("inf")):
            with self.subTest(bad=bad):
                node = self._node(RECT)
                self._feed(node, 1.0, x=bad)
                self.clock.advance(0.1)
                node._tick()
                self.assertTrue(node._done)
                self.assertEqual(node._outcome, follow.OUTCOME_FAILED)
                self.assertIn("有限", node._odom_guard.last_reject)

    def test_zero_norm_quaternion_rejected(self) -> None:
        node = self._node(RECT)
        self._feed(node, 1.0, orientation=(0.0, 0.0, 0.0, 0.0))
        self.clock.advance(0.1)
        node._tick()
        self.assertTrue(node._done)
        self.assertIn("四元数", node._odom_guard.last_reject)

    def test_invalid_odometry_after_valid_faults_immediately(self) -> None:
        """有效帧之后来一条坏帧：即使还没到停更阈值也不能继续发非零。"""
        cases = [
            ("nan position", {"x": float("nan")}),
            ("zero quaternion", {"orientation": (0.0, 0.0, 0.0, 0.0)}),
            ("wrong frame", {"frame": "map"}),
            ("wrong child", {"child": "base_link"}),
        ]
        for name, kwargs in cases:
            with self.subTest(case=name):
                node = self._node(RECT)
                self._feed(node, 1.0)          # 先建立有效基线
                self.clock.advance(0.1)
                node._tf_buffer.pose = (0.0, 0.0, 0.0)
                node._tick()
                self.assertFalse(node._done)
                self.assertGreater(self._commands(node)[-1][0], 0.0)

                self._feed(node, 1.1, **kwargs)  # 距上次有效接收仅 0.1 s
                self.clock.advance(0.1)
                node._tf_buffer.pose = (0.0, 0.0, 0.0)
                node._tick()
                self.assertTrue(node._done)
                self.assertEqual(node._outcome, follow.OUTCOME_FAILED)
                self._assert_tail_zero(node)

    def test_invalid_tf_is_zero_and_fails(self) -> None:
        cases = [
            ("nan translation", (float("nan"), 0.0, 0.0), None),
            ("inf translation", (float("inf"), 0.0, 0.0), None),
            ("nan quaternion", (0.0, 0.0, 0.0), (0.0, 0.0, float("nan"), 1.0)),
            ("zero quaternion", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0)),
        ]
        for name, pose, quat in cases:
            with self.subTest(case=name):
                node = self._node(RECT)
                self._feed(node, 1.0)
                self.clock.advance(0.1)
                node._tf_buffer.pose = pose
                node._tf_buffer.raw_quaternion = quat
                node._tick()
                self.assertTrue(node._done)
                self.assertEqual(node._outcome, follow.OUTCOME_FAILED)
                self.assertIn("TF", node._reason)
                self._assert_all_zero(node)

    def test_wrong_frame_or_child_rejected(self) -> None:
        cases = [
            {"frame": "map", "child": ODOM_CHILD},
            {"frame": ODOM_FRAME, "child": "base_link"},
            {"frame": "", "child": ODOM_CHILD},
        ]
        for kwargs in cases:
            with self.subTest(**kwargs):
                node = self._node(RECT)
                self._feed(node, 1.0, **kwargs)
                self.clock.advance(0.1)
                node._tick()
                self.assertTrue(node._done)
                self.assertEqual(node._outcome, follow.OUTCOME_FAILED)
                self.assertIn("不符", node._odom_guard.last_reject)

    # ---------------------------------------------------------- 终点判定
    def test_end_index_but_far_from_goal_is_not_success(self) -> None:
        node = self._node(LINE)
        for stamp, (x, y, yaw) in zip(
            (1.0, 1.1, 1.2),
            ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.5, 0.0, 0.0)),
        ):
            self._feed(node, stamp, x, y, yaw)
            self.clock.advance(0.1)
            node._tf_buffer.pose = (x, y, yaw)
            node._tick()
        # 最近点已经是终点索引，但实际离最后一点 0.5 m > goal_tol=0.2 m
        self.assertFalse(node._done)
        self.assertIsNone(node._outcome)
        # 车已冲过末点、目标在车后（方位差约 180°）：按新的大转角停走策略，
        # 线速度必须是 0、只允许有界原地对齐，所以这里检查"仍在控制中"（v 或 w 非零），
        # 不能再要求线速度 > 0（那正是掉头/回程误判要修掉的行为）。
        v, w = self._commands(node)[-1]
        self.assertEqual(v, 0.0)
        self.assertGreater(abs(w), 0.0)

    def test_closed_loop_start_is_not_completion(self) -> None:
        node = self._node(LOOP, goal_tol=0.2)
        # 起点就在最后一点上（闭合路线），但一步都还没走。
        self._feed(node, 1.0, 0.0, 0.02, 0.0)
        self.clock.advance(0.1)
        node._tf_buffer.pose = (0.0, 0.02, 0.0)
        node._tick()
        self.assertFalse(node._done)
        self.assertIsNone(node._outcome)
        self.assertEqual(node._idx, 0)
        self.assertGreater(abs(self._commands(node)[-1][0]), 0.0)
        # 横向误差必须按最终采用的 idx=0 重算，而不是封顶前的"终点"距离 0。
        self.assertAlmostEqual(node._err_max, 0.02, places=6)

    def test_success_requires_goal_position_and_heading(self) -> None:
        node = self._node(LINE)
        for stamp, (x, y, yaw) in zip(
            (1.0, 1.1, 1.2),
            ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.95, 0.0, 0.0)),
        ):
            self._feed(node, stamp, x, y, yaw)
            self.clock.advance(0.1)
            node._tf_buffer.pose = (x, y, yaw)
            node._tick()

        self.assertFalse(node._done)
        self.assertEqual(node._state, "align")   # 位置到了，朝向还差

        self._feed(node, 1.3, 1.95, 0.0, 0.95)
        self.clock.advance(0.1)
        node._tf_buffer.pose = (1.95, 0.0, 0.95)
        node._tick()
        self.assertTrue(node._done)
        self.assertEqual(node._outcome, follow.OUTCOME_SUCCESS)
        self.assertIn("成功", node.summary())
        self._assert_tail_zero(node)

    def test_align_drift_beyond_goal_tol_fails(self) -> None:
        node = self._node(LINE)
        for stamp, (x, y, yaw) in zip(
            (1.0, 1.1, 1.2),
            ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.95, 0.0, 0.0)),
        ):
            self._feed(node, stamp, x, y, yaw)
            self.clock.advance(0.1)
            node._tf_buffer.pose = (x, y, yaw)
            node._tick()
        self.assertEqual(node._state, "align")

        # 漂到离终点 2 m，但朝向已经正确：不能只看 yaw 就报成功。
        self._feed(node, 1.3, 4.0, 0.0, 1.0)
        self.clock.advance(0.1)
        node._tf_buffer.pose = (4.0, 0.0, 1.0)
        node._tick()
        self.assertTrue(node._done)
        self.assertEqual(node._outcome, follow.OUTCOME_FAILED)
        self.assertIn("漂离", node._reason)
        self.assertNotIn("成功", node.summary())
        self._assert_tail_zero(node)

    def test_alignment_timeout_is_failure(self) -> None:
        node = self._node(LINE)
        for stamp, (x, y, yaw) in zip(
            (1.0, 1.1, 1.2),
            ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.95, 0.0, 0.0)),
        ):
            self._feed(node, stamp, x, y, yaw)
            self.clock.advance(0.1)
            node._tf_buffer.pose = (x, y, yaw)
            node._tick()
        self.assertEqual(node._state, "align")

        stamp = 1.2
        for _ in range(60):               # 6 s，超过 5 s 对齐超时
            stamp += 0.1
            self._feed(node, stamp, 1.95, 0.0, 0.0)
            self.clock.advance(0.1)
            node._tf_buffer.pose = (1.95, 0.0, 0.0)
            node._tick()
            if node._done:
                break
        self.assertTrue(node._done)
        self.assertEqual(node._outcome, follow.OUTCOME_FAILED)
        self.assertIn("对齐", node._reason)
        self.assertIn("超时", node._reason)
        summary = node.summary()
        self.assertIn("失败", summary)
        self.assertNotIn("走完一圈", summary)

    # ---------------------------------------------------------- 退出收尾
    def test_publish_failure_marks_failed(self) -> None:
        node = self._node(RECT)

        class _BrokenPublisher:
            def publish(self, msg):
                raise RuntimeError("串口写失败")

        node._pub = _BrokenPublisher()
        node._stop("到达终点", follow.OUTCOME_SUCCESS)
        self.assertTrue(node._done)
        self.assertEqual(node._outcome, follow.OUTCOME_FAILED)
        self.assertIn("停车指令发送失败", node._reason)
        self.assertIn("失败", node.summary())

    def test_unexpected_exception_still_stops_and_saves_actual_path(self) -> None:
        path_file = self._write_path(RECT, "map")
        save_file = self._temp_path("_actual_exc.yaml")
        self._set_params(path_file=path_file, save_actual=save_file,
                         return_to_start=False)
        box = {}

        def hook(node):
            box["node"] = node
            self._feed(node, 1.0)
            self.clock.advance(0.1)
            node._tf_buffer.pose = (0.0, 0.0, 0.0)
            node._tick()
            raise RuntimeError("模拟控制循环异常")

        _ros.state["spin_hook"] = hook
        old_handler = signal.getsignal(signal.SIGINT)
        self.addCleanup(signal.signal, signal.SIGINT, old_handler)
        output = io.StringIO()
        with redirect_stdout(output):
            with self.assertRaises(RuntimeError):
                follow.main()

        node = box["node"]
        self.assertTrue(node.destroyed)
        self.assertTrue(os.path.exists(save_file), "异常退出也必须保存已有实际轨迹")
        saved = Path(save_file).read_text(encoding="utf-8")
        self.assertIn("frame_id: map", saved)
        self.assertIn("points:", saved)
        commands = self._commands(node)
        self.assertTrue(any(v > 0.0 for v, _ in commands))
        self._assert_tail_zero(node)
        self.assertIn("失败", output.getvalue())

    def test_shutdown_exception_does_not_mask_control_exception(self) -> None:
        path_file = self._write_path(RECT, "map")
        self._set_params(path_file=path_file, return_to_start=False)

        def hook(node):
            raise RuntimeError("控制循环异常")

        def bad_shutdown():
            raise RuntimeError("shutdown 异常")

        _ros.state["spin_hook"] = hook
        _ros.state["shutdown_hook"] = bad_shutdown
        old_handler = signal.getsignal(signal.SIGINT)
        self.addCleanup(signal.signal, signal.SIGINT, old_handler)
        output = io.StringIO()
        with redirect_stdout(output):
            with self.assertRaises(RuntimeError) as ctx:
                follow.main()
        self.assertIn("控制循环异常", str(ctx.exception))
        self.assertNotIn("shutdown", str(ctx.exception))

    def test_sigint_stops_saves_and_exits_130(self) -> None:
        path_file = self._write_path(RECT, "map")
        save_file = self._temp_path("_actual_sigint.yaml")
        self._set_params(path_file=path_file, save_actual=save_file,
                         return_to_start=False)
        box = {}

        def hook(node):
            box["node"] = node
            self._feed(node, 1.0)
            self.clock.advance(0.1)
            node._tf_buffer.pose = (0.0, 0.0, 0.0)
            node._tick()
            signal.raise_signal(signal.SIGINT)

        _ros.state["spin_hook"] = hook
        old_handler = signal.getsignal(signal.SIGINT)
        self.addCleanup(signal.signal, signal.SIGINT, old_handler)
        output = io.StringIO()
        with redirect_stdout(output):
            code = follow.main()

        self.assertEqual(code, 130)
        node = box["node"]
        self.assertTrue(node.destroyed)
        self.assertEqual(node._outcome, follow.OUTCOME_INTERRUPTED)
        self.assertTrue(os.path.exists(save_file))
        self._assert_tail_zero(node)

    # ---------------------------------------------------------- 掉头/回程目标
    def test_real_route_forward_view_does_not_target_past_uturn(self) -> None:
        """实测路线前向视图：旧代码在 162 会选到掉头后的 192（夹角 167°）还向前。"""
        forward = self._fixture_forward_route()
        path_file = self._write_path(forward, "map")
        node = self._node_from_path(
            path_file, v_max=0.08, w_max=0.4,
            lookahead_min=0.35, lookahead_max=0.35)
        idx = 162
        pose = forward[idx]
        self._prime_run(node, idx, pose)
        self._tick_pose(node, 1.0, pose)

        self.assertEqual(node._idx, idx)
        target = node._target_idx
        self.assertGreaterEqual(target, idx)
        self.assertLess(target, 191, "目标不能扫到掉头之后紧邻的回程点")
        gap = max(node.s[i + 1] - node.s[i] for i in range(len(node.s) - 1))
        self.assertLessEqual(node.s[target] - node.s[idx],
                             node._lookahead + gap + 1e-9)
        v, w = self._commands(node)[-1]
        self.assertEqual(node._state, "run")
        self.assertGreater(v, 0.5 * node.v_max)
        self.assertLessEqual(abs(w), node.w_max)

    def test_real_route_forward_view_sweep_bounded_by_arc_length(self) -> None:
        """前向视图每一拍：目标准前进 <= 前视 + 一个点距；大转角时 v=0、w 有界。"""
        forward = self._fixture_forward_route()
        path_file = self._write_path(forward, "map")
        node = self._node_from_path(
            path_file, v_max=0.08, w_max=0.4,
            lookahead_min=0.35, lookahead_max=0.35)
        gap = max(node.s[i + 1] - node.s[i] for i in range(len(node.s) - 1))
        enter = follow.TRACK_ALIGN_ENTER_RAD
        large_angle = 0
        stamp = 1.0
        for idx in range(0, len(forward) - 1):
            if node.total_len - node.s[idx] <= node._near_end_m:
                continue
            pose = forward[idx]
            self._prime_run(node, idx, pose)
            self._tick_pose(node, stamp, pose)
            stamp += 0.1
            self.assertEqual(node._idx, idx)
            target = node._target_idx
            self.assertGreaterEqual(target, idx)
            self.assertLessEqual(node.s[target] - node.s[idx],
                                 node._lookahead + gap + 1e-9)
            v, _w = self._commands(node)[-1]
            alpha = abs(follow.wrap_pi(
                math.atan2(node.py[target] - pose[1],
                           node.px[target] - pose[0]) - pose[2]))
            if alpha >= enter:
                large_angle += 1
                self.assertEqual(node._state, "track_align")
                self.assertEqual(v, 0.0)
            else:
                self.assertEqual(node._state, "run")
                self.assertGreater(v, 0.0)
        self.assertGreater(large_angle, 0, "真实掉头必须出现需要停车对齐的大转角")

    def test_synthetic_uturn_and_crossing_sweep(self) -> None:
        """合成紧 U 弯 / 自交路线：目标不跳段，大转角只原地转、线速度为 0。"""
        routes = {
            "tight_uturn": _uturn_route(x_end=0.6, radius=0.1),
            "figure_eight": _figure_eight_route(),
        }
        enter = follow.TRACK_ALIGN_ENTER_RAD
        for name, route in routes.items():
            with self.subTest(route=name):
                node = self._node(route, v_max=0.08, w_max=0.4,
                                  lookahead_min=0.35, lookahead_max=0.35)
                gap = max(node.s[i + 1] - node.s[i]
                          for i in range(len(node.s) - 1))
                stamp = 1.0
                for idx in range(0, len(route) - 1):
                    if node.total_len - node.s[idx] <= node._near_end_m:
                        continue
                    pose = route[idx]
                    self._prime_run(node, idx, pose)
                    self._tick_pose(node, stamp, pose)
                    stamp += 0.1
                    target = node._target_idx
                    self.assertGreaterEqual(target, idx)
                    self.assertLessEqual(node.s[target] - node.s[idx],
                                         node._lookahead + gap + 1e-9)
                    v, w = self._commands(node)[-1]
                    alpha = abs(follow.wrap_pi(
                        math.atan2(node.py[target] - pose[1],
                                   node.px[target] - pose[0]) - pose[2]))
                    if alpha >= enter:
                        self.assertEqual(node._state, "track_align")
                        self.assertEqual(v, 0.0)
                        self.assertGreater(abs(w), 0.0)
                        self.assertLessEqual(abs(w), node.w_max)
                    else:
                        self.assertEqual(node._state, "run")
                        self.assertGreater(v, 0.0)

    def test_target_behind_car_rotates_in_place(self) -> None:
        node = self._node(LINE, w_max=0.4)
        pose = (0.0, 0.0, math.pi)
        self._prime_run(node, 0, pose)
        self._tick_pose(node, 1.0, pose)
        v, w = self._commands(node)[-1]
        self.assertEqual(node._state, "track_align")
        self.assertEqual(v, 0.0)
        self.assertGreater(w, 0.0)
        self.assertLessEqual(abs(w), node.w_max)
        self.assertFalse(node._done)

    def test_180_degree_error_turns_deterministically(self) -> None:
        for yaw in (math.pi, -math.pi):
            with self.subTest(yaw=yaw):
                node = self._node(LINE, w_max=0.4)
                pose = (0.0, 0.0, yaw)
                self._prime_run(node, 0, pose)
                self._tick_pose(node, 1.0, pose)
                first = self._commands(node)[-1]
                self._tick_pose(node, 1.1, pose)
                second = self._commands(node)[-1]
                self.assertEqual(first, second)
                self.assertEqual(first[0], 0.0)
                self.assertGreater(first[1], 0.0)
                self.assertLessEqual(first[1], node.w_max)

    def test_tracking_align_hysteresis_and_recovery(self) -> None:
        node = self._node(LINE, w_max=0.4)
        enter = follow.TRACK_ALIGN_ENTER_RAD
        exit_ = follow.TRACK_ALIGN_EXIT_RAD
        self.assertTrue(0.0 < exit_ < enter <= math.pi)
        # 进入阈值以内：照常前进，不因接近阈值就停车。
        pose = (0.0, 0.0, -(enter - math.radians(1.0)))
        self._prime_run(node, 0, pose)
        self._tick_pose(node, 1.0, pose)
        v, _w = self._commands(node)[-1]
        self.assertEqual(node._state, "run")
        self.assertGreater(v, 0.0)

        # 超过进入阈值：停车原地转。
        pose = (0.0, 0.0, -(enter + math.radians(1.0)))
        self._prime_run(node, 0, pose)
        self._tick_pose(node, 1.1, pose)
        v, w = self._commands(node)[-1]
        self.assertEqual(node._state, "track_align")
        self.assertEqual(v, 0.0)
        self.assertGreater(w, 0.0)
        self.assertLessEqual(w, node.w_max)

        # 在 (退出, 进入) 区间里保持对齐状态：不因为回到 60° 以下就恢复跟踪。
        for stamp, alpha in ((1.2, exit_ + math.radians(30.0)),
                             (1.3, exit_ + math.radians(1.0))):
            pose = (0.0, 0.0, -alpha)
            self._tick_pose(node, stamp, pose)
            v, w = self._commands(node)[-1]
            self.assertEqual(node._state, "track_align")
            self.assertEqual(v, 0.0)
            self.assertLessEqual(abs(w), node.w_max)

        # 缩到退出阈值以内：恢复跟踪并重新发前进速度。
        pose = (0.0, 0.0, -(exit_ - math.radians(1.0)))
        self._tick_pose(node, 1.4, pose)
        v, _w = self._commands(node)[-1]
        self.assertEqual(node._state, "run")
        self.assertGreater(v, 0.0)

    def test_tracking_align_timeout_stops_and_fails(self) -> None:
        node = self._node(LINE, w_max=0.4)
        pose = (0.0, 0.0, -math.pi / 2.0)
        self._prime_run(node, 0, pose)
        self._tick_pose(node, 1.0, pose)
        self.assertEqual(node._state, "track_align")
        stamp = 1.0
        for _ in range(30):
            stamp += 1.0
            self.clock.advance(1.0)
            self._feed(node, stamp, *pose)
            node._tf_buffer.pose = pose
            node._tick()
            v, w = self._commands(node)[-1]
            self.assertEqual(v, 0.0)
            self.assertLessEqual(abs(w), node.w_max)
            if node._done:
                break
        self.assertTrue(node._done)
        self.assertEqual(node._outcome, follow.OUTCOME_FAILED)
        self.assertIn("对齐", node._reason)
        self.assertIn("超时", node._reason)
        self._assert_tail_zero(node)

    def test_tracking_align_still_honors_safety_gates(self) -> None:
        pose = (0.0, 0.0, -math.pi / 2.0)
        # 原始里程计停更
        node = self._node(LINE, w_max=0.4)
        self._prime_run(node, 0, pose)
        self._tick_pose(node, 1.0, pose)
        self.assertEqual(node._state, "track_align")
        self.clock.advance(0.6)
        node._tf_buffer.pose = pose
        node._tick()
        self.assertTrue(node._done)
        self.assertEqual(node._outcome, follow.OUTCOME_FAILED)
        self.assertIn("停更", node._reason)
        self._assert_tail_zero(node)

        # 对齐中来一条非法原始里程计
        node = self._node(LINE, w_max=0.4)
        self._prime_run(node, 0, pose)
        self._tick_pose(node, 1.0, pose)
        self.assertEqual(node._state, "track_align")
        self._feed(node, 1.1, x=float("nan"))
        self.clock.advance(0.1)
        node._tf_buffer.pose = pose
        node._tick()
        self.assertTrue(node._done)
        self.assertEqual(node._outcome, follow.OUTCOME_FAILED)
        self._assert_tail_zero(node)

        # 对齐中底盘急停
        node = self._node(LINE, w_max=0.4, require_status=True)
        self._feed_status(node, {"estop": 0, "ok": True})
        self._prime_run(node, 0, pose)
        self._tick_pose(node, 1.0, pose)
        self.assertEqual(node._state, "track_align")
        self._feed_status(node, {"estop": 1, "ok": True})
        self.clock.advance(0.6)
        self._feed(node, 1.1, *pose)
        node._tf_buffer.pose = pose
        node._tick()
        self.assertTrue(node._done)
        self.assertEqual(node._outcome, follow.OUTCOME_FAILED)
        self.assertIn("底盘", node._reason)
        self._assert_tail_zero(node)

        # 对齐中 TF 失效
        node = self._node(LINE, w_max=0.4)
        self._prime_run(node, 0, pose)
        self._tick_pose(node, 1.0, pose)
        self.assertEqual(node._state, "track_align")
        stamp = 1.0
        for _ in range(10):
            stamp += 0.1
            self._feed(node, stamp, *pose)
            self.clock.advance(0.1)
            node._tf_buffer.pose = None
            node._tick()
            if node._done:
                break
        self.assertTrue(node._done)
        self.assertEqual(node._outcome, follow.OUTCOME_FAILED)
        self.assertIn("TF", node._reason)
        self._assert_tail_zero(node)

    def test_curvature_uses_actual_target_distance(self) -> None:
        # 目标沿弧长 0.35 m，但空间上只有 0.25 m：曲率必须按实际距离算。
        route = [(0.0, 0.0, 0.0), (0.2, 0.0, 0.0), (0.2, 0.15, 0.0)]
        node = self._node(route, v_max=0.08, w_max=0.4,
                          lookahead_min=0.35, lookahead_max=0.35)
        self._tick_pose(node, 1.0, route[0])
        v, w = self._commands(node)[-1]
        target_dist = math.hypot(node.px[-1], node.py[-1])
        alpha = math.atan2(node.py[-1], node.px[-1])
        expected_v = node.v_max * max(0.25, math.cos(alpha))
        expected_w = expected_v * 2.0 * math.sin(alpha) / target_dist
        self.assertAlmostEqual(target_dist, 0.25, places=9)
        self.assertAlmostEqual(v, expected_v, places=9)
        self.assertAlmostEqual(w, expected_w, places=9)

    def test_progress_monotonic_and_capped_by_mileage(self) -> None:
        route = _uturn_route(x_end=0.6, radius=0.1)
        node = self._node(route, v_max=0.08, w_max=0.4,
                          lookahead_min=0.35, lookahead_max=0.35)
        idx = 12
        pose = route[idx]
        self._prime_run(node, idx, pose)
        # 位姿往回挪（噪声/横向偏差）：最近点会落在更低的索引，但有效进度不回退。
        self._tick_pose(node, 1.0, route[idx - 5])
        self.assertEqual(node._idx, idx)

        # 位姿跳到很远的回程支路：仍然只允许局部弧长窗口，不超实际里程 + 余量。
        node._done = False
        node._outcome = None
        node._reason = ""
        node._state = "run"
        node._track_align_t0 = None
        far = route[-1]
        node._traveled = node.s[idx]
        node._last_pose = (far[0], far[1])
        self._tick_pose(node, 1.1, far)
        self.assertLessEqual(node.s[node._idx],
                             node._traveled + follow.PROGRESS_SLACK_M + 1e-9)
        self.assertLess(node._idx, len(route) - 1, "不能一步跳到末端的回程支路")

    def test_duplicate_points_and_short_route_are_handled(self) -> None:
        duplicated = [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.05, 0.0, 0.0),
                      (0.05, 0.0, 0.0), (0.10, 0.0, 0.0), (0.10, 0.02, 0.0)]
        node = self._node(duplicated)
        self._tick_pose(node, 1.0, (0.0, 0.0, 0.0))
        v, w = self._commands(node)[-1]
        self.assertTrue(math.isfinite(v) and math.isfinite(w))
        self.assertFalse(node._done)
        self.assertGreaterEqual(node._target_idx, node._idx)

        # 剩余弧长比前视还短：目标退回末点，输出仍有限。
        short = [(0.0, 0.0, 0.0), (0.03, 0.0, 0.0), (0.06, 0.01, 0.0)]
        node = self._node(short)
        self._tick_pose(node, 1.0, (0.0, 0.0, 0.0))
        v, w = self._commands(node)[-1]
        self.assertTrue(math.isfinite(v) and math.isfinite(w))
        self.assertEqual(node._target_idx, len(short) - 1)
        self.assertGreaterEqual(v, 0.0)

    def test_invalid_tracking_parameters_rejected(self) -> None:
        bad_cases = [
            ("v_max", 0.0),
            ("v_max", -0.1),
            ("v_max", float("nan")),
            ("w_max", 0.0),
            ("w_max", float("inf")),
            ("lookahead_min", 0.0),
            ("lookahead_min", float("nan")),
            ("lookahead_max", -0.2),
            ("lookahead_max", float("inf")),
        ]
        for name, value in bad_cases:
            with self.subTest(name=name, value=value):
                with self.assertRaises(ValueError):
                    self._node(RECT, **{name: value})
        with self.assertRaises(ValueError):
            self._node(RECT, lookahead_min=0.5, lookahead_max=0.3)

    def _run_diff_drive_sim(self, route, *, dt: float = 0.1,
                            max_steps: int = 3000):
        """简单差速运动模型：只把 cmd_vel 积分成位姿，不接 ROS/硬件。"""
        node = self._node(route, v_max=0.08, w_max=0.4,
                          lookahead_min=0.35, lookahead_max=0.35,
                          goal_tol=0.2, return_to_start=True)
        pose = (route[0][0], route[0][1], route[0][2])
        stamp = 1.0
        steps = 0
        prev_idx = None
        max_jump = 0.0
        regressions = 0
        max_err = 0.0
        align_ticks = 0
        while not node._done and steps < max_steps:
            self._feed(node, stamp, *pose)
            self.clock.advance(dt)
            node._tf_buffer.pose = pose
            node._tick()
            steps += 1
            stamp += dt
            v, w = self._commands(node)[-1]
            x, y, yaw = pose
            pose = (x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt,
                    follow.wrap_pi(yaw + w * dt))
            if node._state in ("return", "align_start"):
                prev_idx = None
                continue
            if prev_idx is not None:
                jump = node.s[node._idx] - node.s[prev_idx]
                max_jump = max(max_jump, jump)
                if node._idx < prev_idx:
                    regressions += 1
            prev_idx = node._idx
            if node._state == "track_align":
                align_ticks += 1
            max_err = max(max_err, _cross_track_m(pose, route))
        return {
            "completed": node._outcome == follow.OUTCOME_SUCCESS,
            "outcome": node._outcome,
            "reason": node._reason,
            "steps": steps,
            "elapsed_s": steps * dt,
            "traveled_m": node._traveled,
            "max_cross_track_m": max_err,
            "max_progress_jump_m": max_jump,
            "progress_regressions": regressions,
            "align_ticks": align_ticks,
        }

    def test_diff_drive_completes_synthetic_forward_uturn(self) -> None:
        # 紧 U 弯（半径 0.1 m）会触发"停车原地对齐"，完整模型仍必须走完。
        route = _uturn_route(x_end=2.0, radius=0.1)
        metrics = self._run_diff_drive_sim(route)
        self.assertTrue(metrics["completed"], metrics)
        self.assertEqual(metrics["progress_regressions"], 0, metrics)
        self.assertLessEqual(metrics["max_progress_jump_m"], 0.35, metrics)
        self.assertLess(metrics["max_cross_track_m"], 0.6, metrics)
        self.assertGreater(metrics["align_ticks"], 0, metrics)
        self.assertIn("到达终点", metrics["reason"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
