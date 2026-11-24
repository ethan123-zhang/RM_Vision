#!/usr/bin/env python3
"""把 gz 传感器帧号修成 ROS 帧号：/scan_safety_raw -> /scan_safety。

为什么必须有这一层：gz-sim 的传感器消息把 frame 写成"模型/链接/传感器"的作用域名
（实测 `sentry/base_footprint/safety_lidar`），ros_gz_bridge 原样搬到 header.frame_id。
TF 里没有这个名字，于是两张 costmap 的 MessageFilter 把整帧丢掉，日志只有
`Message Filter dropping message: frame 'sentry/base_footprint/safety_lidar' ...` ——
症状和"雷达没起作用"一模一样，非常容易误判成传感器没发布。

为什么映射到 base_footprint 是精确的：雷达在 URDF 里挂在 base_link 的固定子关节上，
转 SDF 时固定关节被合并，传感器实际落在 base_footprint 上，x/y 偏移为 0
（z 抬高只在 2D 扫描里不影响）。所以不需要额外 TF，也不需要新增帧。
"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

import math


class SafetyScanRelay(Node):
    def __init__(self):
        super().__init__('safety_scan_relay',
                         parameter_overrides=[
                             Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.declare_parameter('input_topic', '/scan_safety_raw')
        self.declare_parameter('output_topic', '/scan_safety')
        self.declare_parameter('frame_id', 'base_footprint')
        self.declare_parameter('stale_after', 0.5)
        self.declare_parameter('publish_empty_when_stale', True)
        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.stale_after = float(self.get_parameter('stale_after').value)
        self.publish_empty = bool(self.get_parameter('publish_empty_when_stale').value)

        # 传感器源用 best-effort（gz 桥可能也是 best-effort），输出用 reliable：
        # reliable 发布者 + best-effort 订阅者、reliable 订阅者都能收到，两边都不挑。
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)
        out_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(LaserScan, out_topic, out_qos)
        self.sub = self.create_subscription(LaserScan, in_topic, self.on_scan, sensor_qos)
        self.count = 0
        self.last_rx = None
        self.silent_cycles = 0
        self.empty_sent = 0
        # 定时器兜底：雷达没数据时也要持续发"空扫描"，否则 collision_monitor 会
        # `Robot to stop due to invalid source` 把车永久钉住（2026-11-24 实测：
        # safety_lidar:=false 时整车 216 s 纹丝不动，故障现象和"导航坏了"一样）。
        self.timer = self.create_timer(0.1, self.on_timer)
        self.get_logger().info(f'{in_topic} -> {out_topic} (frame_id={self.frame_id})')

    def _empty_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = 2.0 * math.pi / 360.0
        msg.scan_time = 0.1
        msg.time_increment = 0.1 / 360.0
        msg.range_min = 0.05
        msg.range_max = 2.5
        msg.ranges = [float('inf')] * 360
        msg.intensities = []
        return msg

    def on_timer(self):
        if not self.publish_empty:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        stale = self.last_rx is None or (now - self.last_rx) > self.stale_after
        if not stale:
            self.silent_cycles = 0
            return
        self.pub.publish(self._empty_scan())
        self.empty_sent += 1
        self.silent_cycles += 1
        if self.silent_cycles % 50 == 1:
            self.get_logger().warn(
                f'低位安全雷达 {self.stale_after:.1f} s 无数据，改发空扫描'
                f'（安全层暂时是瞎的）；已发 {self.empty_sent} 帧')

    def on_scan(self, msg):
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)
        self.last_rx = self.get_clock().now().nanoseconds / 1e9
        self.count += 1
        if self.count % 100 == 0:
            self.get_logger().info(f'relayed {self.count} scans')


def main():
    rclpy.init()
    node = SafetyScanRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()