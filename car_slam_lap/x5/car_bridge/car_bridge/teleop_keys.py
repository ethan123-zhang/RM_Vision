"""teleop_keys —— SSH 里直接用键盘开车（不依赖 ros-humble-teleop-twist-keyboard）。

为什么不用官方 teleop：板子上不一定装了那个 apt 包，而且我们要的是一套
"按键设定方向 + 松开一段时间自动停"的语义，比持续按住更稳。

按键：
    w / s        前进 / 后退
    a / d        原地左转 / 右转
    q / e        前进并左转 / 前进并右转
    空格         立即停
    1 / 2 / 3    速度档 0.15 / 0.25 / 0.35 m/s
    x            退出

安全：deadman 秒内没有按键就自动发 0（默认 2 s）；Ctrl-C 也会发 0 再退出。

用法（必须在真实 TTY 里跑，管道/重定向会失败）：
    ros2 run car_bridge teleop_keys
    ros2 run car_bridge teleop_keys --ros-args -p max_linear:=0.25 -p deadman_s:=3.0
"""

from __future__ import annotations

import select
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

try:
    import termios
    import tty
except ImportError:  # pragma: no cover - Windows 上不去跑它
    termios = None
    tty = None

HELP = """
按键速查（deadman {deadman:.1f}s 内没按键就自动停）：
  w/s 前进/后退   a/d 原地左/右转   q/e 前进左/右转
  空格 停         1/2/3 速度档 0.15/0.25/0.35 m/s    x 退出
当前档位: {speed:.2f} m/s
"""


class TeleopKeys(Node):
    def __init__(self) -> None:
        super().__init__("teleop_keys")
        self.declare_parameter("max_linear", 0.35)
        self.declare_parameter("max_angular", 1.2)
        self.declare_parameter("deadman_s", 2.0)
        self.declare_parameter("rate_hz", 20.0)

        self.v = float(self.get_parameter("max_linear").value)
        self.w = float(self.get_parameter("max_angular").value)
        self.deadman = float(self.get_parameter("deadman_s").value)
        rate = float(self.get_parameter("rate_hz").value)

        self._pub = self.create_publisher(Twist, "cmd_vel", 10)
        self._lin = 0.0
        self._ang = 0.0
        self._last_key = 0.0
        self._dt = 1.0 / rate
        self._quit = False

    def _set(self, lin: float, ang: float) -> None:
        self._lin, self._ang = lin, ang
        self._last_key = time.monotonic()

    def _handle(self, key: str) -> None:
        if key == "w":
            self._set(self.v, 0.0)
        elif key == "s":
            self._set(-self.v, 0.0)
        elif key == "a":
            self._set(0.0, self.w)
        elif key == "d":
            self._set(0.0, -self.w)
        elif key == "q":
            self._set(self.v, self.w * 0.6)
        elif key == "e":
            self._set(self.v, -self.w * 0.6)
        elif key == " ":
            self._set(0.0, 0.0)
        elif key in "123":
            self.v = {"1": 0.15, "2": 0.25, "3": 0.35}[key]
            print(HELP.format(deadman=self.deadman, speed=self.v))
        elif key == "x":
            self._quit = True
        else:
            return
        state = "停" if (self._lin == 0 and self._ang == 0) else "动"
        print(f"\r[{state}] v={self._lin:+.2f} m/s  w={self._ang:+.2f} rad/s  ", end="", flush=True)

    def publish_once(self) -> None:
        timed_out = (time.monotonic() - self._last_key) > self.deadman
        if timed_out and (self._lin or self._ang):
            self._lin = self._ang = 0.0
            print("\r[deadman] 超时自动停                                    ", flush=True)
        msg = Twist()
        msg.linear.x = self._lin
        msg.angular.z = self._ang
        self._pub.publish(msg)

    def run(self) -> int:
        if termios is None or not sys.stdin.isatty():
            print("需要一个真实终端（SSH 里直接跑，别用 nohup / 管道）", file=sys.stderr)
            return 2

        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        print(HELP.format(deadman=self.deadman, speed=self.v))
        try:
            tty.setcbreak(fd)
            while not self._quit and rclpy.ok():
                ready, _, _ = select.select([sys.stdin], [], [], 0.02)
                if ready:
                    key = sys.stdin.read(1)
                    self._handle(key)
                self.publish_once()
                time.sleep(self._dt)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            # 退出前补三帧 0，确保底盘收到
            for _ in range(3):
                self._pub.publish(Twist())
                time.sleep(0.05)
            print("\n已停止。")
        return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeleopKeys()
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 0
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
