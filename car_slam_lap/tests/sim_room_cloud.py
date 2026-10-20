#!/usr/bin/env python3
"""sim_room_cloud —— 发布一个合成的"房间"点云，用来在没有 Odin1/没车的时候验证 map_from_cloud。

为什么需要：`map_from_cloud` 里有体素去重、numpy 变换、PCD 写出、cv2 俯视图一堆逻辑，
  真实数据一上来的话"没出图"既可能是相机没数据，也可能是自己的代码有 bug。
  用合成数据先把它跑通，真机阶段就只剩一个变量。

用：ubuntu 上直接跑（需要 rclpy + numpy）
    python3 sim_room_cloud.py            # 默认 6x6 m、2 m 高、5 cm 间距
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


def make_room(half_width: float = 3.0, height: float = 2.0, step: float = 0.05) -> np.ndarray:
    """四面墙（z 从 0 到 height），像一间 6x6 m 的屋子。"""
    pts = []
    horiz = np.arange(-half_width, half_width + step, step)
    vert = np.arange(0.0, height + step, step)
    for x in horiz:
        for z in vert:
            pts.append((x, -half_width, z))
            pts.append((x, half_width, z))
    for y in horiz:
        for z in vert:
            pts.append((-half_width, y, z))
            pts.append((half_width, y, z))
    return np.asarray(pts, dtype=np.float32)


class SimRoomCloud(Node):
    def __init__(self) -> None:
        super().__init__("sim_room_cloud")
        self.declare_parameter("topic", "/odin1/cloud_slam")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("half_width", 3.0)
        self.declare_parameter("rate_hz", 10.0)

        self.frame_id = self.get_parameter("frame_id").value
        self.pts = make_room(float(self.get_parameter("half_width").value))
        self.pub = self.create_publisher(PointCloud2, self.get_parameter("topic").value, 2)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self._tick)
        self.get_logger().info(
            f"发布 {len(self.pts)} 点，frame={self.frame_id}，"
            f"房间 {self.get_parameter('half_width').value * 2:.0f} m 见方"
        )

    def _tick(self) -> None:
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = len(self.pts)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(self.pts)
        msg.is_dense = True
        msg.data = self.pts.tobytes()
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = SimRoomCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
