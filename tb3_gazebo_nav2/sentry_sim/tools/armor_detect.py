#!/usr/bin/env python3
"""装甲板检测（S3e）：前视相机 + HSV 阈值 → 颜色/方位/距离。

为什么用颜色而不是模板/训练模型：仿真里的装甲板是我们自己 spawn 的纯色块
（`spawn_obstacle.py --armor red|blue`），HSV 阈值足够、可复现、零训练成本；
实车/真实比赛要换成数字识别（装甲板上的数字），那一步不在仿真范围。

输出：`/armor/detections`（std_msgs/String，JSON），每条含
  color, cx_px, cy_px, w_px, h_px, bearing_rad（相机系、右正）, elev_rad, distance_m
距离口径：`d = fx * 板宽(0.135 m) / 像素宽`；方位口径：`atan2(cx - W/2, fx)`。

用法（栈已跑，且带 `camera:=true`）：
  python3 tools/armor_detect.py                       # 只检测
  python3 tools/armor_detect.py --save /tmp/frame.png # 顺带存一帧带框的图
  python3 tools/armor_detect.py --print               # 每秒打印一次检测结果
"""
import argparse
import json
import os
import time

import cv2
import numpy as np

os.environ.setdefault("ROS_DOMAIN_ID", "45")

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

PLATE_W_M = 0.135          # RM 小装甲板 135 mm 宽
PLATE_H_M = 0.055
MIN_AREA_PX = 25
ASPECT_RANGE = (1.6, 4.2)

HSV = {
    "red": [((0, 110, 50), (12, 255, 255)), ((168, 110, 50), (180, 255, 255))],
    "blue": [((95, 110, 40), (135, 255, 255))],
}

IMG_QOS = QoSProfile(depth=3, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST)
# ros_gz_bridge 的 camera_info 是 VOLATILE/RELIABLE；用 TRANSIENT_LOCAL 会收到
# "offering incompatible QoS (DURABILITY)" 然后一条都收不到（2026-11-25 踩）。
INFO_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST)


class ArmorDetect(Node):

    def __init__(self, save_path, do_print):
        super().__init__('armor_detect', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.bridge = CvBridge()
        self.save_path = save_path
        self.do_print = do_print
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.frames = 0
        self.seen = 0
        self.last_print = 0.0
        self.pub = self.create_publisher(String, '/armor/detections', 10)
        self.create_subscription(Image, '/camera/image', self.on_image, IMG_QOS)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, INFO_QOS)

    def on_info(self, msg):
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]

    def detect(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        out = []
        for color, ranges in HSV.items():
            mask = None
            for lo, hi in ranges:
                m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
                mask = m if mask is None else cv2.bitwise_or(mask, m)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                area = cv2.contourArea(c)
                if area < MIN_AREA_PX:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                ratio = w / float(h) if h else 0.0
                if not (ASPECT_RANGE[0] <= ratio <= ASPECT_RANGE[1]):
                    continue
                if self.fx is None:
                    continue
                dist = self.fx * PLATE_W_M / float(w)
                bearing = float(np.arctan2((x + w / 2.0) - self.cx, self.fx))
                elev = float(np.arctan2((y + h / 2.0) - self.cy, self.fy or self.fx))
                out.append({'color': color, 'cx_px': int(x + w // 2), 'cy_px': int(y + h // 2),
                            'w_px': int(w), 'h_px': int(h), 'area_px': int(area),
                            'bearing_rad': round(bearing, 4),
                            'elev_rad': round(elev, 4),
                            'distance_m': round(float(dist), 3),
                            'bbox': [int(x), int(y), int(w), int(h)]})
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 1)
        out.sort(key=lambda d: d['distance_m'])
        return out, img

    def on_image(self, msg):
        self.frames += 1
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        dets, annotated = self.detect(img)
        payload = {'stamp_sim': round(self.get_clock().now().nanoseconds * 1e-9, 3),
                   'frame_shape': [int(img.shape[1]), int(img.shape[0])],
                   'n': len(dets), 'detections': dets}
        self.pub.publish(String(data=json.dumps(payload)))
        if dets:
            self.seen += 1
        if self.save_path and self.frames % 20 == 0:
            cv2.imwrite(self.save_path.replace('.png', '_%03d.png' % (self.frames // 20 % 100)),
                        annotated)
        now = time.monotonic()
        if self.do_print and now - self.last_print >= 1.0:
            self.last_print = now
            top = dets[0] if dets else None
            print(f"[armor] frames={self.frames} n={len(dets)} top={top}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--save', default='', help='每 2 s 存一帧带框图到这个路径前缀')
    ap.add_argument('--print', dest='do_print', action='store_true')
    ap.add_argument('--duration', type=float, default=0.0, help='跑多久（秒，0=一直跑）')
    args = ap.parse_args()
    rclpy.init()
    node = ArmorDetect(args.save, args.do_print)
    t0 = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if args.duration and time.monotonic() - t0 > args.duration:
                break
    except KeyboardInterrupt:
        pass
    print(f"[armor] done: frames={node.frames} with_detection={node.seen}", flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())