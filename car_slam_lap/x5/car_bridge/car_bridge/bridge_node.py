"""bridge_node —— 把 /cmd_vel 变成串口帧发给 C8T6，并回读底盘状态。

数据流：
    /cmd_vel (Twist, m/s & rad/s)
        -> 按 mps_at_max / radps_at_max 换算成 ±1000 指令单位
        -> 50 Hz 定时组帧（A5 5A ...）写串口
    C8T6 10 Hz 上行 status 帧
        -> 解析成 /car/status (std_msgs/String, JSON)

三层安全：
    1. 本节点：cmd_timeout_s 内没收到新 /cmd_vel 就发 0；
    2. 固件：串口 300 ms 无有效帧就发 0（见 main.c 仲裁）；
    3. 遥控器 s2 打到底 = 硬急停，压过串口（status 里带 ESTOP 位）。

用法：
    ros2 run car_bridge bridge_node --ros-args -p port:=/dev/car_link
    ros2 run car_bridge bridge_node --ros-args -p dry_run:=true   # 不开串口，只打日志
"""

from __future__ import annotations

import glob
import json
import os
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from . import protocol as p

try:  # pyserial 只在真机上需要
    import serial  # type: ignore
except ImportError:  # pragma: no cover
    serial = None


class BridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("car_bridge")

        # ---- 串口 ----
        # 默认走 RDK X5 40pin 的 UART：第 8 脚 TX / 第 10 脚 RX，设备节点 /dev/ttyS1。
        # 如果 udev 规则生效，/dev/car_link 会指向同一个节点，两个都能用。
        self.declare_parameter("port", "/dev/ttyS1")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("auto_fallback_usb", True)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("reconnect_period_s", 1.0)

        # ---- 频率与超时 ----
        self.declare_parameter("send_rate_hz", 50.0)
        self.declare_parameter("cmd_timeout_s", 0.30)
        self.declare_parameter("status_timeout_s", 0.50)
        self.declare_parameter("status_rate_hz", 10.0)

        # ---- 单位换算（at_max 由 calibrate_speed 实测填） ----
        self.declare_parameter("mps_at_max", 0.8)
        self.declare_parameter("radps_at_max", 2.0)
        self.declare_parameter("invert_speed", False)
        self.declare_parameter("invert_angular", True)  # ROS: +z=左转；协议: +angular=右转

        self._port_name: str = self.get_parameter("port").value
        self._baud: int = int(self.get_parameter("baud").value)
        self._dry_run: bool = bool(self.get_parameter("dry_run").value)
        self._auto_fallback: bool = bool(self.get_parameter("auto_fallback_usb").value)
        self._reconnect_period: float = float(self.get_parameter("reconnect_period_s").value)

        self._send_dt: float = 1.0 / float(self.get_parameter("send_rate_hz").value)
        self._cmd_timeout: float = float(self.get_parameter("cmd_timeout_s").value)
        self._status_timeout: float = float(self.get_parameter("status_timeout_s").value)
        self._status_dt: float = 1.0 / float(self.get_parameter("status_rate_hz").value)

        self._mps_at_max: float = float(self.get_parameter("mps_at_max").value)
        self._radps_at_max: float = float(self.get_parameter("radps_at_max").value)
        self._invert_speed: bool = bool(self.get_parameter("invert_speed").value)
        self._invert_angular: bool = bool(self.get_parameter("invert_angular").value)

        # ---- 运行时状态 ----
        self._lock = threading.Lock()
        self._cmd_speed = 0
        self._cmd_angular = 0
        self._cmd_stamp = 0.0
        self._last_status: Optional[bytes] = None
        self._last_status_stamp = 0.0
        self._status: Optional[tuple] = None  # (status, src)
        self._sent = 0
        self._recv_ok = 0
        self._crc_err = 0
        self._link_warned = False
        self._estop_warned = False

        self._ser = None
        self._open_stamp = 0.0
        self._decoder = p.FrameDecoder(p.SOF_UP)
        self._stop_event = threading.Event()

        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
        self._status_pub = self.create_publisher(String, "car/status", 10)

        self.create_timer(self._send_dt, self._on_send_timer)
        self.create_timer(self._status_dt, self._on_status_timer)

        if not self._dry_run:
            self._try_open()
        self.get_logger().info(
            f"car_bridge 启动: port={self._port_name} baud={self._baud} "
            f"send={1.0 / self._send_dt:.0f}Hz dry_run={self._dry_run} "
            f"mps_at_max={self._mps_at_max} radps_at_max={self._radps_at_max}"
        )

    # ------------------------------------------------------------ 串口
    def _resolve_port(self) -> str:
        if not self._dry_run and not self._auto_fallback:
            return self._port_name
        if os.path.exists(self._port_name):
            return self._port_name
        # 40pin UART 优先（X5 上就是 ttyS1），其次才是 USB-TTL 转接板
        candidates = sorted(glob.glob("/dev/ttyS1")) + sorted(glob.glob("/dev/ttyUSB*")) \
            + sorted(glob.glob("/dev/ttyACM*"))
        if candidates:
            self.get_logger().warn(
                f"{self._port_name} 不存在，回退到 {candidates[0]}（建议装 udev 规则固定名字）"
            )
            return candidates[0]
        return self._port_name

    def _try_open(self) -> bool:
        if serial is None:
            self.get_logger().error("没有 pyserial：sudo apt install python3-serial")
            return False
        port = self._resolve_port()
        try:
            self._ser = serial.Serial(port, self._baud, timeout=0.05, write_timeout=0.1)
            self._port_name = port
            self._link_warned = False
            self.get_logger().info(f"串口已打开: {port} @ {self._baud}")
            return True
        except Exception as exc:  # noqa: BLE001 - 打不开就等下一轮重试
            self._ser = None
            self._open_stamp = time.monotonic()
            self.get_logger().warn(f"打开 {port} 失败: {exc}（{self._reconnect_period}s 后重试）")
            return False

    def _ensure_open(self) -> bool:
        if self._dry_run:
            return True
        if self._ser is not None:
            return True
        if time.monotonic() - self._open_stamp < self._reconnect_period:
            return False
        return self._try_open()

    def _read_available(self) -> None:
        """把串口里已有的字节全部喂给解码器（在 IO 线程里跑）。"""
        ser = self._ser
        if ser is None:
            return
        try:
            pending = ser.in_waiting
            data = ser.read(pending if pending else 1)
        except Exception as exc:  # noqa: BLE001 - 掉线（拔线/复位）走重连
            self.get_logger().warn(f"串口读异常: {exc}，准备重连")
            try:
                ser.close()
            except Exception:  # noqa: BLE001
                pass
            with self._lock:
                self._ser = None
            self._open_stamp = time.monotonic()
            return
        if not data:
            return
        for frame in self._decoder.feed_bytes(data):
            try:
                status, src = p.parse_status(frame)
            except ValueError:
                continue  # 不是 status 帧，丢
            with self._lock:
                self._status = (status, src)
                self._last_status_stamp = time.monotonic()
                self._recv_ok = self._decoder.frames_ok
                self._crc_err = self._decoder.crc_errors

    def _io_loop(self) -> None:
        while rclpy.ok() and not self._stop_event.is_set():
            if self._ensure_open():
                self._read_available()
            else:
                time.sleep(0.05)
            time.sleep(0.002)

    # ------------------------------------------------------------ 指令
    def _on_cmd_vel(self, msg: Twist) -> None:
        speed_units = p.units_from_rate(msg.linear.x, self._mps_at_max)
        angular_units = p.units_from_rate(msg.angular.z, self._radps_at_max)
        if self._invert_speed:
            speed_units = -speed_units
        if self._invert_angular:
            angular_units = -angular_units
        with self._lock:
            self._cmd_speed = speed_units
            self._cmd_angular = angular_units
            self._cmd_stamp = time.monotonic()

    def _current_cmd(self) -> tuple:
        """取当前该发出去的指令；超时则归零。"""
        with self._lock:
            age = time.monotonic() - self._cmd_stamp
            if age > self._cmd_timeout:
                return 0, 0
            return self._cmd_speed, self._cmd_angular

    # ------------------------------------------------------------ 定时器
    def _on_send_timer(self) -> None:
        speed, angular = self._current_cmd()
        frame = p.build_set_vel(speed, angular)

        if self._dry_run:
            self._sent += 1
            if self._sent % 50 == 0:  # 1 Hz 打一行，别刷屏
                self.get_logger().info(f"[dry_run] {frame.hex(' ').upper()}")
            return

        if not self._ensure_open() or self._ser is None:
            return
        try:
            self._ser.write(frame)
            self._sent += 1
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"串口写异常: {exc}，准备重连")
            try:
                self._ser.close()
            except Exception:  # noqa: BLE001
                pass
            with self._lock:
                self._ser = None
            self._open_stamp = time.monotonic()

    def _on_status_timer(self) -> None:
        with self._lock:
            status = self._status
            age = time.monotonic() - self._last_status_stamp if status else 1e9
            recv_ok, crc_err, sent = self._recv_ok, self._crc_err, self._sent

        if status is None or age > self._status_timeout:
            if not self._link_warned:
                self._link_warned = True
                self.get_logger().warn(
                    "没收到 C8T6 上行 status（检查：TX/RX 是否交叉、波特率 115200、"
                    "固件是否装了 USART3 协议、GND 是否共地）"
                )
            payload = {"ok": False, "age_s": None if status is None else round(age, 3),
                       "src": "none", "flags": [], "sent": sent}
        else:
            self._link_warned = False
            flags = p.status_text(status[0])
            src = p.SRC_NAMES.get(status[1], f"?{status[1]}")
            if p.ST_ESTOP & status[0]:
                if not self._estop_warned:
                    self._estop_warned = True
                    self.get_logger().error("底盘报 ESTOP：遥控器 s2 打在急停档，车不会动")
            else:
                self._estop_warned = False
            payload = {
                "ok": True,
                "age_s": round(age, 3),
                "status": status[0],
                "flags": flags,
                "src": src,
                "estop": bool(status[0] & p.ST_ESTOP),
                "recv_ok": recv_ok,
                "crc_errors": crc_err,
                "sent": sent,
            }
        self._status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    # ------------------------------------------------------------ 退出
    def stop(self) -> None:
        try:
            if self._ser is not None and not self._dry_run:
                self._ser.write(p.build_set_vel(0, 0))
                time.sleep(0.02)
                self._ser.close()
        except Exception:  # noqa: BLE001
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BridgeNode()
    io_thread = threading.Thread(target=node._io_loop, daemon=True)
    io_thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_event.set()
        node.stop()  # 退出前补一帧 0，别让车带着最后一条指令继续跑
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
