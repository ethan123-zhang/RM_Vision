"""calibrate_speed —— 实测「指令打满 = 多少 m/s / rad/s」，让 /cmd_vel 变成真单位。

为什么必须标定：纯追踪算出来的是 m/s 和 rad/s，但底盘只认 ±1000 的无量纲指令。
不知道换算系数，就只能拍一个 0.8 m/s，跟实际速度差一倍——弯道会直接撞墙。

做法：发一个固定指令（比如让 bridge 认 0.5 m/s），跑 4 s，用 Odin1 的里程计
量实际位移/转角，反推出"打满 1000 对应多少物理量"。

用法（车要放在地上，前方留 2 m 空地）：
    ros2 run car_bridge calibrate_speed            # 直线
    ros2 run car_bridge calibrate_speed --ros-args -p axis:=angular
"""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .path_io import wrap_pi, yaw_from_quaternion


class CalibrateSpeed(Node):
    def __init__(self) -> None:
        super().__init__("calibrate_speed")
        self.declare_parameter("axis", "linear")        # linear | angular
        self.declare_parameter("command", 0.4)          # 发给 /cmd_vel 的物理量
        self.declare_parameter("mps_at_max", 0.8)       # bridge 当前用的换算系数
        self.declare_parameter("radps_at_max", 2.0)
        self.declare_parameter("settle_s", 1.0)
        self.declare_parameter("duration_s", 4.0)
        self.declare_parameter("odom_topic", "/odin1/odometry")

        self.axis: str = self.get_parameter("axis").value
        self.command: float = float(self.get_parameter("command").value)
        self.mps_at_max: float = float(self.get_parameter("mps_at_max").value)
        self.radps_at_max: float = float(self.get_parameter("radps_at_max").value)
        self.settle: float = float(self.get_parameter("settle_s").value)
        self.duration: float = float(self.get_parameter("duration_s").value)

        self._pub = self.create_publisher(Twist, "cmd_vel", 10)
        self._pose = None  # (x, y, yaw)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value,
                                 self._on_odom, 20)

        at_max = self.mps_at_max if self.axis == "linear" else self.radps_at_max
        self.units = int(max(-1000, min(1000, round(self.command / at_max * 1000))))
        if self.units == 0:
            raise RuntimeError("command 太小，指令四舍五入成 0 了，调大一点")

    def _on_odom(self, msg: Odometry) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self._pose = (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def _spin_for(self, seconds: float, twist: Twist) -> None:
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            self._pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.02)

    def run(self) -> int:
        zero = Twist()
        cmd = Twist()
        if self.axis == "linear":
            cmd.linear.x = self.command
        else:
            cmd.angular.z = self.command

        print(f"轴={self.axis}  指令 {self.command:.3f} -> {self.units} units")
        print(f"先静置 {self.settle:.1f}s ...")
        self._spin_for(self.settle, zero)
        start = self._pose
        if start is None:
            print("✗ 没收到里程计：Odin1 驱动起来了吗？（ros2 topic hz /odin1/odometry）")
            return 1

        print(f"跑 {self.duration:.1f}s ...（车会动，注意别撞东西）")
        self._spin_for(self.duration, cmd)
        end = self._pose
        self._spin_for(0.5, zero)  # 松油门，别让它继续溜
        if end is None:
            print("✗ 跑的过程中丢了里程计")
            return 1

        if self.axis == "linear":
            dist = math.hypot(end[0] - start[0], end[1] - start[1])
            measured = dist / self.duration
            suggested = measured * 1000.0 / abs(self.units)
            print(f"\n实测位移 {dist:.2f} m -> {measured:.3f} m/s")
            print(f"建议  -p mps_at_max:={suggested:.3f}")
            if measured < 0.02:
                print("✗ 基本没动：查急停档（s2=3）、电池、CAN 线、或把 --speed 调大")
                return 1
        else:
            dyaw = wrap_pi(end[2] - start[2])
            measured = abs(dyaw) / self.duration
            suggested = measured * 1000.0 / abs(self.units)
            print(f"\n实测转角 {math.degrees(dyaw):+.1f} deg -> {measured:.3f} rad/s")
            print(f"建议  -p radps_at_max:={suggested:.3f}")
            if measured < 0.02:
                print("✗ 基本没转：查急停档（s2=3）、MIX 模式是否支持原地转")
                return 1

        print("\n把上面的值写进 start_relocal.sh / drive_lap.sh 的 ros2 run 参数里（或直接改脚本默认值）")
        return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CalibrateSpeed()
    try:
        code = node.run()
    except (KeyboardInterrupt, ExternalShutdownException):  # SIGINT 时 spin() 抛的是后者
        code = 130
    finally:
        node._pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():  # SIGINT 时 rclpy 已自行关闭，重复调用会抛 RCLError
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
