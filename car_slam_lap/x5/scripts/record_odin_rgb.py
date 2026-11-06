#!/usr/bin/env python3
"""record_odin_rgb —— 把 /odin1/image/compressed 的 JPEG 帧原样落盘，用于建图过程录像。

为什么要单独写，而不是 ros2 bag / 网页 viewer：
  - ros2 bag 里是序列化消息，观众没法直接双识别播放；
  - odin_web_viewer 要解码 + 投影，建图期间额外吃 0.6~1.1 核；本机已经出现过
    "Odin1 回调队列溢出导至示教中途停更"（docs/demo-runbook.md 第 95 行）。
  - 本脚本只做 bytes -> 文件：不解码、不编码、不转格式，CPU 接近 0，
    剩盘代价就是磁盘写入（实测 1600x1296 JPEG 约 450 KB/帧 @10 Hz ≈ 4.5 MB/s）。

用法（板端，X5）：
  python3 record_odin_rgb.py --out ~/car_ws/maps/demo_x/video
  python3 record_odin_rgb.py --out ... --stride 2      # 隔一帧取一帧（跌到 5 fps）
  Ctrl-C 结束；退出时写 meta.json。

不要手动删/改 frames/ 里的文件；成片在 PC 上用 pc_frames_to_mp4.py 合成。
"""
import argparse
import json
import os
import shutil
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

# 与 x5_odin/capture_one_frame.py 一致：驱动发布侧用它自己能连上的最宽松配置。
# BEST_EFFORT 订阅既能接 BEST_EFFORT 也能接 RELIABLE 发布者，反过来不行。
QOS = QoSProfile(depth=10)
QOS.reliability = ReliabilityPolicy.BEST_EFFORT
QOS.history = HistoryPolicy.KEEP_LAST


class RgbRecorder(Node):
    def __init__(self, out_dir, topic, stride, max_seconds, min_free_mb):
        super().__init__("record_odin_rgb")
        self.frames_dir = os.path.join(out_dir, "frames")
        os.makedirs(self.frames_dir, exist_ok=True)
        self.topic = topic
        self.stride = max(1, int(stride))
        self.min_free_mb = min_free_mb
        self.max_seconds = float(max_seconds)

        self.seen = 0          # 收到的帧数（含被 stride 跳过的）
        self.written = 0       # 实际落盘的帧数
        self.bytes_written = 0
        self.start_wall = time.time()
        self.start_stamp = None
        self.last_stamp = None
        self.stop_reason = ""
        self.format_seen = set()

        self.create_subscription(CompressedImage, topic, self.on_image, QOS)
        self.get_logger().info(
            "订阅 %s -> %s（stride=%d）" % (topic, self.frames_dir, self.stride))

    # ------------------------------------------------------------------ 回调
    def on_image(self, msg):
        self.seen += 1
        self.format_seen.add(msg.format)
        if self.seen % self.stride != 0:
            return
        if self.max_seconds > 0 and (time.time() - self.start_wall) > self.max_seconds:
            self.request_stop("达到 --max-seconds")
            return
        if self.written % 200 == 0 and self.written:
            free_mb = shutil.disk_usage(self.frames_dir).free / (1024 * 1024)
            if free_mb < self.min_free_mb:
                self.request_stop("剩余磁盘 %.0f MB < %d MB" % (free_mb, self.min_free_mb))
                return

        path = os.path.join(self.frames_dir, "frame_%06d.jpg" % (self.written + 1))
        payload = bytes(msg.data)
        try:
            with open(path, "wb") as fh:
                fh.write(payload)
        except OSError as exc:
            self.request_stop("写盘失败: %s" % exc)
            return

        self.written += 1
        self.bytes_written += len(payload)
        if self.start_stamp is None:
            self.start_stamp = msg.header.stamp
        self.last_stamp = msg.header.stamp

        if self.written % 100 == 0:
            elapsed = time.time() - self.start_wall
            self.get_logger().info(
                "已录 %d 帧 / %.1f MB / %.1f fps"
                % (self.written, self.bytes_written / 1e6, self.written / max(elapsed, 1e-6)))

    def request_stop(self, reason):
        if not self.stop_reason:
            self.stop_reason = reason
            self.get_logger().warn("停止录制：%s" % reason)

    # ------------------------------------------------------------------ 收尾
    def write_meta(self):
        elapsed = time.time() - self.start_wall
        meta = {
            "topic": self.topic,
            "frames_dir": self.frames_dir,
            "frames_written": self.written,
            "messages_seen": self.seen,
            "stride": self.stride,
            "bytes_written": self.bytes_written,
            "wall_seconds": round(elapsed, 3),
            "measured_fps": round(self.written / elapsed, 3) if elapsed > 0 else 0.0,
            "formats": sorted(self.format_seen),
            "start_stamp_ns": self.start_stamp.sec * 10**9 + self.start_stamp.nanosec
            if self.start_stamp else None,
            "end_stamp_ns": self.last_stamp.sec * 10**9 + self.last_stamp.nanosec
            if self.last_stamp else None,
            "stop_reason": self.stop_reason or "Ctrl-C",
        }
        path = os.path.join(os.path.dirname(self.frames_dir), "meta.json")
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(meta, fh, ensure_ascii=False, indent=2)
        self.get_logger().info("meta -> %s" % path)
        return meta


def main(argv=None):
    ap = argparse.ArgumentParser(description="把 /odin1/image/compressed 落成 JPEG 帧序列")
    ap.add_argument("--out", required=True, help="输出目录（会建 frames/）")
    ap.add_argument("--topic", default="/odin1/image/compressed")
    ap.add_argument("--stride", type=int, default=1, help="每 N 帧存 1 帧，默认 1")
    ap.add_argument("--max-seconds", type=float, default=0.0, help="0 = 不限")
    ap.add_argument("--min-free-mb", type=int, default=500, help="磁盘低于此值自动停")
    args = ap.parse_args(argv)

    rclpy.init()
    node = RgbRecorder(args.out, args.topic, args.stride, args.max_seconds, args.min_free_mb)

    stop = {"flag": False}

    def _on_signal(signum, _frame):
        stop["flag"] = True
        node.request_stop("收到信号 %d" % signum)

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    try:
        while rclpy.ok() and not stop["flag"]:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        if node.seen == 0:
            node.get_logger().error(
                "一帧都没收到：确认驱动是建图/重定位模式且 sendrgbcompressed: 1，"
                "并用 ros2 topic hz %s 复核" % args.topic)
        meta = node.write_meta()
        print(json.dumps(meta, ensure_ascii=False, indent=2))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if meta["frames_written"] > 0 else 1


if __name__ == "__main__":
    sys.exit(main())