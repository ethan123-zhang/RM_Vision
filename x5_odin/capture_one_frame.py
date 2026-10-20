#!/usr/bin/env python3
"""Grab one frame from each Odin1 topic and dump to /tmp/odin_demo/*.npz|jpg"""
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, PointCloud2

OUT = "/tmp/odin_demo"
os.makedirs(OUT, exist_ok=True)

DT = {1: "int8", 2: "uint8", 3: "int16", 4: "uint16", 5: "int32",
      6: "uint32", 7: "float32", 8: "float64"}


def pc2_to_dict(msg):
    n = msg.width * msg.height
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    out = {}
    for f in msg.fields:
        dt = np.dtype(DT[f.datatype])
        if dt.itemsize == 0:
            continue
        out[f.name] = np.ndarray(
            shape=(n,), dtype=dt, buffer=buf,
            offset=f.offset, strides=(msg.point_step,)).copy()
    return out, msg.point_step, msg.width, msg.height


class Grab(Node):
    def __init__(self):
        super().__init__("odin_grab")
        qos = QoSProfile(depth=5)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST
        self.got = set()
        self.create_subscription(PointCloud2, "/odin1/cloud_render",
                                 lambda m: self.cloud(m, "cloud_render"), qos)
        self.create_subscription(PointCloud2, "/odin1/cloud_raw",
                                 lambda m: self.cloud(m, "cloud_raw"), qos)
        self.create_subscription(CompressedImage, "/odin1/image/compressed",
                                 self.image, qos)
        self.deadline = self.get_clock().now().nanoseconds + 40 * 10**9

    def cloud(self, msg, tag):
        name = tag.replace("/", "_")
        if name in self.got:
            return
        d, step, w, h = pc2_to_dict(msg)
        np.savez_compressed(os.path.join(OUT, name + ".npz"), step=step,
                            width=w, height=h, **d)
        self.get_logger().info("%s: %d pts, fields=%s" % (tag, w * h, list(d)))
        self.got.add(name)

    def image(self, msg):
        if "image" in self.got:
            return
        with open(os.path.join(OUT, "rgb.jpg"), "wb") as f:
            f.write(bytes(msg.data))
        self.get_logger().info("image: %d bytes fmt=%s" % (len(msg.data), msg.format))
        self.got.add("image")


def main():
    rclpy.init()
    node = Grab()
    while rclpy.ok() and node.get_clock().now().nanoseconds < node.deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
        if len(node.got) >= 3:
            break
    node.get_logger().info("captured: %s" % sorted(node.got))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
