"""record_path 严格定位示教模式的离线测试（PC 最小 ROS 替身）。

跑法：
    python test_record_map_route.py
    python -m unittest discover -s x5/car_bridge/test -p 'test_record_map_route.py' -v
"""

from __future__ import annotations

import io
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

ODOM_FRAME = "odom"
ODOM_CHILD = "imu"
ODOM_TOPIC = "/odin1/odometry"


# =====================================================================
# 最小 ROS 替身（只覆盖 record_path 用到的接口）
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


PARAM_OVERRIDES: dict = {}


class _FakeNode:
    def __init__(self, name: str) -> None:
        self.name = name
        self._params: dict = {}
        self._subscriptions: list[_RecordingSubscription] = []
        self._timers: list[_RecordingTimer] = []
        self._logger = _FakeLogger()
        self.destroyed = False

    def declare_parameter(self, name, default):
        self._params[name] = PARAM_OVERRIDES.get(name, default)

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self._params[name])

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


def _yaw_to_quat(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _stamp(seconds: float):
    sec = int(math.floor(seconds))
    nsec = int(round((seconds - sec) * 1e9))
    return types.SimpleNamespace(sec=sec, nanosec=nsec)


class _FakeBuffer:
    def __init__(self) -> None:
        self.transforms: dict[tuple[str, str], types.SimpleNamespace] = {}

    def set_transform(self, target: str, source: str, x: float, y: float,
                      yaw: float, stamp: float, orientation=None) -> None:
        qx, qy, qz, qw = _yaw_to_quat(yaw)
        if orientation is not None:
            qx, qy, qz, qw = orientation
        self.transforms[(target, source)] = types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=_stamp(stamp)),
            transform=types.SimpleNamespace(
                translation=types.SimpleNamespace(x=x, y=y, z=0.0),
                rotation=types.SimpleNamespace(x=qx, y=qy, z=qz, w=qw),
            ),
        )

    def lookup_transform(self, target, source, when):
        try:
            return self.transforms[(target, source)]
        except KeyError as exc:
            raise RuntimeError(f"no transform {target}<-{source}") from exc


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
    state = {"ok": True, "spin_hook": lambda node: None}

    def init(args=None) -> None:
        state["ok"] = True

    def ok() -> bool:
        return bool(state["ok"])

    def shutdown() -> None:
        state["ok"] = False

    def spin_once(node, timeout_sec=0.1):
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
    from car_bridge import record_path as record
finally:
    _restore_modules()


class _FakeClock:
    def __init__(self, start: float = 1000.0) -> None:
        self.now = start

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class RecordMapRouteTest(unittest.TestCase):
    def setUp(self) -> None:
        self.test_dir = Path(__file__).resolve().parent
        self._seq = 0
        self._created: list[Path] = []
        self.addCleanup(self._cleanup)
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

        self.map_file = self._map_bytes(b"FAKE_MAP_FOR_RECORDING")
        self.out_path = self._new_path(".yaml")
        self.manifest = Path(self.out_path + ".manifest.json")
        self._created.append(self.manifest)
        self.buffer = _FakeBuffer()

    def _cleanup(self) -> None:
        for path in self._created:
            try:
                path.unlink()
            except FileNotFoundError:
                pass
        self._created.clear()

    def _new_path(self, suffix: str) -> str:
        self._seq += 1
        path = self.test_dir / f".record_test_{os.getpid()}_{self._seq}{suffix}"
        self._created.append(path)
        return str(path)

    def _map_bytes(self, content: bytes) -> str:
        path = Path(self._new_path(".bin"))
        path.write_bytes(content)
        return str(path)

    def _params(self, **overrides) -> None:
        params = {
            "map_file": self.map_file,
            "out_path": self.out_path,
            "world_frame": "map",
            "fallback_frame": "",
            "base_frame": "imu",
            "use_odom_pose": False,
            "autosave_s": 0.0,
            "max_dt": 1.0,
        }
        params.update(overrides)
        PARAM_OVERRIDES.clear()
        PARAM_OVERRIDES.update(params)
        self.addCleanup(PARAM_OVERRIDES.clear)

    def _node(self, **overrides):
        self._params(**overrides)
        self.buffer = _FakeBuffer()
        node = record.RecordPath()
        node._tf_buffer = self.buffer
        self.addCleanup(node.destroy_node)
        return node

    def _set_location(self, stamp: float, pose, map_stamp_delta: float = 0.0) -> None:
        x, y, yaw = pose
        self.buffer.set_transform("map", "imu", x, y, yaw, stamp)
        self.buffer.set_transform("odom", "map", 0.0, 0.0, 0.0,
                                  stamp + map_stamp_delta)

    def _odom_msg(self, stamp: float, x: float = 0.0, y: float = 0.0,
                  yaw: float = 0.0, frame: str = ODOM_FRAME,
                  child: str = ODOM_CHILD, position=None, orientation=None):
        qx, qy, qz, qw = _yaw_to_quat(yaw)
        if position is None:
            position = (x, y, 0.0)
        if orientation is not None:
            qx, qy, qz, qw = orientation
        return types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=_stamp(stamp), frame_id=frame),
            child_frame_id=child,
            pose=types.SimpleNamespace(
                pose=types.SimpleNamespace(
                    position=types.SimpleNamespace(
                        x=position[0], y=position[1], z=position[2]),
                    orientation=types.SimpleNamespace(
                        x=qx, y=qy, z=qz, w=qw),
                )
            ),
        )

    def _feed(self, node, stamp: float, pose=(0.0, 0.0, 0.0),
              map_stamp_delta: float = 0.0, **kwargs) -> None:
        self.clock.advance(0.1)
        self._set_location(stamp, pose, map_stamp_delta)
        sub = node._subscriptions[0]
        sub.callback(self._odom_msg(stamp, *pose, **kwargs))

    def _manifest_path(self) -> Path:
        return self.manifest

    # ---------------------------------------------------------- 初始化
    def test_strict_mode_rejects_non_map_world_frame(self) -> None:
        with self.assertRaises(ValueError):
            self._node(world_frame="odom")

    def test_strict_mode_rejects_fallback_frame(self) -> None:
        with self.assertRaises(ValueError):
            self._node(fallback_frame="odom")

    def test_strict_mode_rejects_use_odom_pose(self) -> None:
        with self.assertRaises(ValueError):
            self._node(use_odom_pose=True)

    def test_strict_mode_rejects_missing_map(self) -> None:
        with self.assertRaises(ValueError):
            self._node(map_file=self._new_path(".bin"))

    def test_strict_mode_refuses_existing_output(self) -> None:
        Path(self.out_path).write_text("old", encoding="utf-8")
        with self.assertRaises(RuntimeError):
            self._node()

    def test_strict_mode_refuses_existing_manifest(self) -> None:
        self._manifest_path().write_text("{}", encoding="utf-8")
        with self.assertRaises(RuntimeError):
            self._node()

    def test_strict_subscription_uses_sensor_data_qos(self) -> None:
        node = self._node()
        sub = node._subscriptions[0]
        self.assertEqual(sub.topic, ODOM_TOPIC)
        self.assertIs(sub.qos, _QOS_SENSOR_DATA)

    # ---------------------------------------------------------- 等待/故障
    def test_map_tf_not_ready_records_nothing(self) -> None:
        node = self._node()
        self.clock.advance(0.1)
        self.buffer.set_transform("odom", "map", 0.0, 0.0, 0.0, 1.0)
        node._subscriptions[0].callback(self._odom_msg(1.0))
        self.assertEqual(node.points, [])
        self.assertFalse(node._faulted)

    def test_map_tf_wait_timeout_aborts_without_bundle(self) -> None:
        node = self._node()
        node._subscriptions[0].callback(self._odom_msg(1.0))
        node._wait_start = self.clock.now - 31.0
        node._watchdog()
        self.assertTrue(node._faulted)
        self.assertTrue(node._done)
        self.assertFalse(node.finish())
        self.assertFalse(self._manifest_path().exists())

    def test_odom_stale_after_localization_aborts(self) -> None:
        node = self._node()
        self._feed(node, 1.0, (0.0, 0.0, 0.0))
        self.assertEqual(len(node.points), 1)
        self.clock.advance(0.6)
        node._watchdog()
        self.assertTrue(node._faulted)
        self.assertTrue(node._done)
        self.assertFalse(node.finish())
        self.assertFalse(self._manifest_path().exists())

    def test_stale_map_transform_aborts_recording(self) -> None:
        node = self._node()
        self._feed(node, 1.0, (0.0, 0.0, 0.0))
        self._feed(node, 1.1, (0.2, 0.0, 0.0), map_stamp_delta=0.6)
        self.assertTrue(node._faulted)
        self.assertTrue(node._done)
        self.assertFalse(node.finish())
        self.assertFalse(self._manifest_path().exists())

    def test_invalid_raw_odom_after_localization_aborts(self) -> None:
        node = self._node()
        self._feed(node, 1.0, (0.0, 0.0, 0.0))
        self._feed(node, 1.1, (0.2, 0.0, 0.0),
                   position=(float("nan"), 0.0, 0.0))
        self.assertTrue(node._faulted)
        self.assertTrue(node._done)
        self.assertFalse(node.finish())
        self.assertFalse(self._manifest_path().exists())

    def test_strict_mode_never_falls_back_to_odom(self) -> None:
        node = self._node()
        # 只有 odom->map，没有 map<-imu：严格模式必须等待，不能退回 odom 姿态。
        self.clock.advance(0.1)
        self.buffer.set_transform("odom", "map", 0.0, 0.0, 0.0, 1.0)
        node._subscriptions[0].callback(self._odom_msg(1.0))
        self.assertEqual(node.points, [])
        self.assertFalse(node._faulted)

    # ---------------------------------------------------------- 正常录制
    def test_normal_map_teaching_writes_valid_bundle(self) -> None:
        node = self._node()
        poses = [(0.0, 0.0, 0.0), (0.2, 0.0, 0.0), (0.4, 0.0, 0.1)]
        for i, pose in enumerate(poses):
            self._feed(node, 1.0 + i * 0.1, pose)
        self.assertTrue(node.finish())
        self.assertTrue(Path(self.out_path).exists())
        self.assertTrue(self._manifest_path().exists())

        from car_bridge import route_bundle as rb
        metadata = rb.validate_bundle(self.out_path, self.map_file, base_frame="imu")
        self.assertEqual(metadata["start"]["x"], 0.0)
        self.assertEqual(metadata["start"]["y"], 0.0)
        self.assertEqual(metadata["start"]["yaw"], 0.0)
        self.assertEqual(metadata["point_count"], 3)

    def test_autosave_writes_diagnostic_path_without_bundle(self) -> None:
        node = self._node()
        for i in range(3):
            self._feed(node, 1.0 + i * 0.1, (0.2 * i, 0.0, 0.0))
        self.clock.advance(2.0)
        node._autosave()
        self.assertTrue(Path(self.out_path).exists())
        self.assertFalse(self._manifest_path().exists())

    def test_zero_length_teaching_does_not_write_bundle(self) -> None:
        node = self._node(max_dt=0.05)
        for i in range(4):
            self._feed(node, 1.0 + i * 0.1, (0.0, 0.0, 0.0))
        self.assertGreaterEqual(len(node.points), 3)
        self.assertFalse(node.finish())
        self.assertTrue(Path(self.out_path).exists())
        self.assertFalse(self._manifest_path().exists())

    def test_finish_rejects_stale_raw_odom(self) -> None:
        node = self._node()
        for i, pose in enumerate(((0.0, 0.0, 0.0), (0.2, 0.0, 0.0),
                                  (0.4, 0.0, 0.0))):
            self._feed(node, 1.0 + i * 0.1, pose)
        # watchdog 还没跑，但 Ctrl-C 不能抢在它前面认证陈旧数据。
        self.clock.advance(0.6)
        self.assertFalse(node.finish())
        self.assertTrue(Path(self.out_path).exists())
        self.assertFalse(self._manifest_path().exists())

    def test_legacy_mode_without_map_file_still_works(self) -> None:
        self._params(map_file="", out_path=self.out_path, use_odom_pose=True)
        node = record.RecordPath()
        self.addCleanup(node.destroy_node)
        node._tf_buffer = _FakeBuffer()
        sub = node._subscriptions[0]
        self.clock.advance(0.1)
        sub.callback(self._odom_msg(1.0, 0.0, 0.0, 0.0))
        self.clock.advance(0.1)
        sub.callback(self._odom_msg(1.1, 0.2, 0.0, 0.0))
        self.assertTrue(node.finish())
        self.assertTrue(Path(self.out_path).exists())
        self.assertFalse(self._manifest_path().exists())

    # ---------------------------------------------------------- exit code
    def test_main_returns_nonzero_on_fault(self) -> None:
        def hook(node):
            node._abort("模拟定位故障")

        _ros.state["spin_hook"] = hook
        self._params()
        old_handler = signal.getsignal(signal.SIGINT)
        self.addCleanup(signal.signal, signal.SIGINT, old_handler)
        with redirect_stdout(io.StringIO()):
            code = record.main()
        self.assertNotEqual(code, 0)
        self.assertFalse(self._manifest_path().exists())

    def test_main_unexpected_exception_never_writes_bundle(self) -> None:
        def hook(node):
            node._tf_buffer = self.buffer
            for i, pose in enumerate(((0.0, 0.0, 0.0), (0.2, 0.0, 0.0),
                                      (0.4, 0.0, 0.0))):
                self._feed(node, 1.0 + i * 0.1, pose)
            raise RuntimeError("模拟 spin_once 未预期异常")

        _ros.state["spin_hook"] = hook
        self._params()
        old_handler = signal.getsignal(signal.SIGINT)
        self.addCleanup(signal.signal, signal.SIGINT, old_handler)
        with redirect_stdout(io.StringIO()):
            with self.assertRaises(RuntimeError) as ctx:
                record.main()
        self.assertIn("spin_once", str(ctx.exception))
        self.assertFalse(self._manifest_path().exists())
        self.assertTrue(Path(self.out_path).exists())  # 诊断轨迹仍保留

    def test_main_sigint_with_stale_odom_does_not_bundle(self) -> None:
        def hook(node):
            node._tf_buffer = self.buffer
            for i, pose in enumerate(((0.0, 0.0, 0.0), (0.2, 0.0, 0.0),
                                      (0.4, 0.0, 0.0))):
                self._feed(node, 1.0 + i * 0.1, pose)
            self.clock.advance(0.6)  # watchdog 定时器还没触发
            signal.raise_signal(signal.SIGINT)

        _ros.state["spin_hook"] = hook
        self._params()
        old_handler = signal.getsignal(signal.SIGINT)
        self.addCleanup(signal.signal, signal.SIGINT, old_handler)
        with redirect_stdout(io.StringIO()):
            code = record.main()
        self.assertNotEqual(code, 0)
        self.assertFalse(self._manifest_path().exists())

    def test_main_sigint_after_points_writes_bundle_and_exit_zero(self) -> None:
        def hook(node):
            node._tf_buffer = self.buffer
            for i, pose in enumerate(((0.0, 0.0, 0.0), (0.2, 0.0, 0.0),
                                      (0.4, 0.0, 0.0))):
                self._feed(node, 1.0 + i * 0.1, pose)
            signal.raise_signal(signal.SIGINT)

        _ros.state["spin_hook"] = hook
        self._params()
        old_handler = signal.getsignal(signal.SIGINT)
        self.addCleanup(signal.signal, signal.SIGINT, old_handler)
        with redirect_stdout(io.StringIO()):
            code = record.main()
        self.assertEqual(code, 0)
        self.assertTrue(self._manifest_path().exists())


if __name__ == "__main__":
    unittest.main(verbosity=2)
