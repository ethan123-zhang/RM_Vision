#!/usr/bin/env python3
"""抓 Gazebo 相机图像并存 PNG（无 GUI，靠 headless rendering + llvmpipe）。

用法: python3 capture_camera.py <out.png> [topic] [timeout_s]
默认 topic=/overview/image
"""
import os
import struct
import sys
import time
import zlib

from gz.transport13 import Node
import gz.msgs10.image_pb2 as image_pb2

out = sys.argv[1] if len(sys.argv) > 1 else "/tmp/camera.png"
topic = sys.argv[2] if len(sys.argv) > 2 else "/overview/image"
timeout = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0

node = Node()
got = {}


def cb(msg):
    if "msg" not in got:
        got["msg"] = msg


node.subscribe(image_pb2.Image, topic, cb)
print(f"waiting {timeout}s for {topic} ...")
t0 = time.time()
while time.time() - t0 < timeout and "msg" not in got:
    time.sleep(0.2)
if "msg" not in got:
    print("!! 没收到图像（渲染没起来？）")
    sys.exit(2)

msg = got["msg"]
w, h = msg.width, msg.height
data = bytes(msg.data)
print(f"image {w}x{h} bytes={len(data)} fmt={msg.pixel_format_type} step={msg.step}")

# gz.msgs.Image pixel_format_type: 1=RGB_INT8, 3=RGBA_INT8, 4=BGRA?
if len(data) >= w * h * 3:
    ch = 3 if msg.pixel_format_type != 3 else 4
    ch = ch if len(data) >= w * h * ch else 3
else:
    print("!! 数据长度不够"); sys.exit(3)


def write_png(path, w, h, rgb):
    raw = b"".join(b"\x00" + rgb[y * w * 3:(y + 1) * w * 3] for y in range(h))

    def chunk(typ, payload):
        c = struct.pack(">I", len(payload)) + typ + payload
        return c + struct.pack(">I", zlib.crc32(typ + payload) & 0xFFFFFFFF)

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(raw, 6))
    png += chunk(b"IEND", b"")
    with open(path, "wb") as f:
        f.write(png)


rgb = bytearray()
for i in range(w * h):
    rgb += data[i * ch:i * ch + 3]
os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
write_png(out, w, h, bytes(rgb))
print("saved", out, os.path.getsize(out), "bytes")

# 像素统计：场地亮灰 vs 背景暗
px = rgb
vals = [(px[i * 3], px[i * 3 + 1], px[i * 3 + 2]) for i in range(0, w * h, 7)]
lum = [0.299 * r + 0.587 * g + 0.114 * b for r, g, b in vals]
n = len(lum)
bright = sum(1 for v in lum if v > 100)
mid = sum(1 for v in lum if 60 < v <= 100)
dark = sum(1 for v in lum if v <= 60)
print(f"亮度: 均值={sum(lum)/n:.1f} 亮(>100)={bright/n:.1%} 中={mid/n:.1%} 暗(<=60)={dark/n:.1%}")
# 场地是亮灰块，取亮像素 bbox 估计场地在画面里的占比
xs, ys = [], []
for i in range(w * h):
    r, g, b = px[i * 3], px[i * 3 + 1], px[i * 3 + 2]
    if 0.299 * r + 0.587 * g + 0.114 * b > 100:
        xs.append(i % w)
        ys.append(i // w)
if xs:
    print(f"亮区 bbox: x[{min(xs)},{max(xs)}] y[{min(ys)},{max(ys)}] "
          f"覆盖率={len(xs)/(w*h):.3f}")