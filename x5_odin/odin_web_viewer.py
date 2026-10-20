#!/usr/bin/env python3
"""Odin1 live web viewer for RDK X5.

ROS callbacks only stash the newest frame; one worker thread does all the
decoding / projection / JPEG encoding, so the executor never starves
(the first design dropped PointCloud2 frames while rendering blocked it).

Served on http://<board>:8080/
  /rgb.mjpg      RGB, rotated upright, 648x800
  /depth.mjpg    range image from /odin1/cloud_render (turbo colormap)
  /overlay.mjpg  depth blended over the RGB frame
  /stats         JSON counters
"""
import json
import os
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, PointCloud2

cv2.setNumThreads(2)

PORT = 8080
CALIB = os.path.expanduser("~/.ros/odin_ros_driver/calib.yaml")
OUT_W, OUT_H = 648, 800          # portrait: the device is mounted on its side
SC = 4                           # depth render downscale

TURBO = np.array([[48, 18, 59], [70, 107, 227], [62, 188, 195], [180, 222, 44],
                  [249, 160, 30], [232, 60, 20], [122, 4, 3]], dtype=np.float64)
_DT = {1: "int8", 2: "uint8", 3: "int16", 4: "uint16", 5: "int32", 6: "uint32",
       7: "float32", 8: "float64"}


def turbo(t):
    t = np.clip(t, 0, 1) * (len(TURBO) - 1)
    i = np.clip(t.astype(int), 0, len(TURBO) - 2)
    f = (t - i)[:, None]
    return (TURBO[i] * (1 - f) + TURBO[i + 1] * f)[:, ::-1].astype(np.uint8)  # BGR


def read_calib(path):
    txt = open(path, encoding="utf-8").read()

    def sc(k):
        return float(re.search(rf"^\s*{k}:\s*([-\d.eE+]+)", txt, re.M).group(1))

    tcl = re.search(r"Tcl_0:\s*\[(.*?)\]", txt, re.S).group(1)
    T = np.array([float(x) for x in tcl.replace("\n", " ").split(",")]).reshape(4, 4)
    return dict(T=T.astype(np.float32), fx=sc("A11"), fy=sc("A22"), skew=sc("A12"),
                cx=sc("u0"), cy=sc("v0"),
                k=[sc(k) for k in ("k2", "k3", "k4", "k5", "k6", "k7")],
                w=int(sc("image_width")), h=int(sc("image_height")))


class State:
    """Single-slot mailboxes: producers never block, consumers always see the newest."""

    def __init__(self):
        self.cond = threading.Condition()
        self.out = {"rgb": None, "depth": None, "overlay": None}
        self.seq = {"rgb": 0, "depth": 0, "overlay": 0}
        self.q = {"rgb": 0, "cloud": 0}
        self.inbox = {"rgb": None, "cloud": None}
        self.n = {"rgb": 0, "cloud": 0}
        self.pts = 0
        self.rng = (0, 0)
        self.ms = {"rgb": 0.0, "depth": 0.0}

    def push(self, key, payload):
        with self.cond:
            self.inbox[key] = payload
            self.q[key] += 1
            self.cond.notify_all()

    def take(self):
        with self.cond:
            self.cond.wait_for(lambda: self.q["rgb"] or self.q["cloud"], timeout=1.0)
            rgb, cloud = self.inbox["rgb"], self.inbox["cloud"]
            self.inbox["rgb"] = self.inbox["cloud"] = None
            self.q["rgb"] = self.q["cloud"] = 0
        return rgb, cloud

    def publish(self, key, data):
        with self.cond:
            self.out[key] = data
            self.seq[key] += 1
            self.cond.notify_all()

    def wait(self, key, last):
        with self.cond:
            self.cond.wait_for(lambda: self.seq[key] != last, timeout=2.0)
            return self.out[key], self.seq[key]


STATE = State()


class Viewer(Node):
    def __init__(self):
        super().__init__("odin_web_viewer")
        self.calib = read_calib(CALIB) if os.path.exists(CALIB) else None
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST
        self.create_subscription(CompressedImage, "/odin1/image/compressed", self.on_image, qos)
        self.create_subscription(PointCloud2, "/odin1/cloud_render", self.on_cloud, qos)
        self.get_logger().info("viewer up, calib=%s" % bool(self.calib))

    def on_image(self, msg):
        STATE.push("rgb", bytes(msg.data))

    def on_cloud(self, msg):
        STATE.push("cloud", (msg.data, list(msg.fields), msg.point_step, msg.width, msg.height))


class Worker(threading.Thread):
    daemon = True

    def __init__(self, calib):
        super().__init__()
        self.calib = calib
        self.bgr = None
        self.i = 0

    def run(self):
        while True:
            rgb, cloud = STATE.take()
            if rgb:
                self.render_rgb(rgb)
            if cloud:
                self.render_cloud(cloud)

    def render_rgb(self, data):
        t0 = time.time()
        # decode at 1/2 scale: 1600x1296 -> 800x648, still plenty for a preview,
        # and roughly 4x cheaper than a full-size decode + area resize
        arr = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_REDUCED_COLOR_2)
        if arr is None:
            return
        arr = cv2.rotate(arr, cv2.ROTATE_90_CLOCKWISE)      # -> 648x800
        self.bgr = arr
        ok, enc = cv2.imencode(".jpg", arr, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            STATE.publish("rgb", enc.tobytes())
            with STATE.cond:
                STATE.n["rgb"] += 1
                STATE.ms["rgb"] = (time.time() - t0) * 1e3

    def render_cloud(self, payload):
        if self.calib is None:
            return
        t0 = time.time()
        data, fields, step, width, height = payload
        c = self.calib
        n = width * height
        buf = np.frombuffer(data, np.uint8)
        cols = {f.name: np.ndarray((n,), np.dtype(_DT[f.datatype]), buf, f.offset, (step,))
                for f in fields}
        x = cols["x"].astype(np.float32)
        y = cols["y"].astype(np.float32)
        z = cols["z"].astype(np.float32)
        ok = (x * x + y * y + z * z) > 1e-8
        x, y, z = x[ok], y[ok], z[ok]
        if x.size == 0:
            return

        p = np.stack([x, y, z, np.ones_like(x)], 1) @ c["T"].T
        x1, y1, z1 = p[:, 0], p[:, 1], p[:, 2]
        r = np.hypot(x1, y1)
        norm = np.linalg.norm(p[:, :3], axis=1)
        good = (z1 > 0) & (r > 1e-6)
        th = np.arccos(np.clip(z1[good] / norm[good], -1, 1))
        k2, k3, k4, k5, k6, k7 = c["k"]
        td = th * (1 + th * (k2 + th * (k3 + th * (k4 + th * (k5 + th * (k6 + th * k7))))))
        s = td / r[good]
        u = (x1[good] * s * c["fx"] + y1[good] * s * c["skew"] + c["cx"]).astype(np.int32)
        v = (y1[good] * s * c["fy"] + c["cy"]).astype(np.int32)
        rng = np.sqrt(x[good] ** 2 + y[good] ** 2 + z[good] ** 2)
        keep = (u >= 0) & (u < c["w"]) & (v >= 0) & (v < c["h"])
        u, v, rng = u[keep], v[keep], rng[keep]
        if u.size == 0:
            return

        h2, w2 = c["h"] // SC, c["w"] // SC
        u2 = np.clip(u // SC, 0, w2 - 1)
        v2 = np.clip(v // SC, 0, h2 - 1)
        lo, hi = float(rng.min()), float(rng.max())
        img = np.zeros((h2, w2, 3), np.uint8)
        img[v2, u2] = turbo((rng - lo) / (hi - lo + 1e-9))
        img = cv2.dilate(img, np.ones((2, 2), np.uint8), iterations=1)
        img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        small = cv2.resize(img, (OUT_W, OUT_H), interpolation=cv2.INTER_LINEAR)
        ok_d, enc = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 72])
        if ok_d:
            STATE.publish("depth", enc.tobytes())
            with STATE.cond:
                STATE.n["cloud"] += 1
                STATE.pts = int(u.size)
                STATE.rng = (round(lo, 2), round(hi, 2))
                STATE.ms["depth"] = (time.time() - t0) * 1e3

        self.i += 1
        if self.i % 2 and self.bgr is not None:
            blend = cv2.addWeighted(self.bgr, 0.6, small, 0.4, 0)
            ok_o, eo = cv2.imencode(".jpg", blend, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok_o:
                STATE.publish("overlay", eo.tobytes())


PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><title>Odin1 @ RDK X5</title>
<style>
 body{margin:0;background:#0e0f12;color:#e6e6e6;font:14px/1.5 "Segoe UI",system-ui,sans-serif}
 header{padding:10px 16px;background:#17181d;display:flex;gap:22px;align-items:baseline;
        border-bottom:1px solid #262833;position:sticky;top:0;z-index:9;flex-wrap:wrap}
 h1{font-size:16px;margin:0;font-weight:600}
 .k{color:#8b90a0;font-size:12px}
 .v{color:#7ee0a1;font-weight:600;font-variant-numeric:tabular-nums}
 .wrap{display:grid;grid-template-columns:1fr 1fr;gap:12px;padding:12px}
 .card{background:#14161b;border:1px solid #24262f;border-radius:10px;overflow:hidden}
 .card h2{font-size:13px;margin:0;padding:8px 12px;color:#aab;
          border-bottom:1px solid #24262f;font-weight:500}
 .card img{display:block;width:100%;height:auto}
 .full{grid-column:1 / -1}
</style></head>
<body>
<header>
  <h1>Odin1 &rarr; RDK X5 实时画面</h1>
  <span class="k">RGB</span><span class="v" id="fps_rgb">-</span>
  <span class="k">点云</span><span class="v" id="fps_dep">-</span>
  <span class="k">点数</span><span class="v" id="pts">-</span>
  <span class="k">距离</span><span class="v" id="rng">-</span>
  <span class="k">渲染耗时</span><span class="v" id="ms">-</span>
</header>
<div class="wrap">
  <div class="card"><h2>① RGB 1600x1296 &rarr; 648x800（/odin1/image/compressed）</h2>
    <img src="/rgb.mjpg"></div>
  <div class="card"><h2>② 点云深度（/odin1/cloud_render，伪彩）</h2>
    <img src="/depth.mjpg"></div>
  <div class="card full"><h2>③ 深度叠加在 RGB 上（对齐检查）</h2>
    <img src="/overlay.mjpg"></div>
</div>
<script>
let last=null;
async function tick(){
  try{
    const s=await (await fetch('/stats')).json();
    if(last){
      const dt=s.t-last.t;
      document.getElementById('fps_rgb').textContent=((s.n_rgb-last.n_rgb)/dt).toFixed(1)+' Hz';
      document.getElementById('fps_dep').textContent=((s.n_cloud-last.n_cloud)/dt).toFixed(1)+' Hz';
    }
    last=s;
    document.getElementById('pts').textContent=s.pts;
    document.getElementById('rng').textContent=s.range[0]+' - '+s.range[1]+' m';
    document.getElementById('ms').textContent='rgb '+s.ms[0].toFixed(0)+' / depth '+s.ms[1].toFixed(0)+' ms';
  }catch(e){}
}
setInterval(tick,1000);
</script>
</body></html>
"""


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?")[0]
        if path == "/":
            self._send(200, PAGE.encode(), "text/html; charset=utf-8")
        elif path == "/stats":
            with STATE.cond:
                body = json.dumps({"t": time.time(), "n_rgb": STATE.n["rgb"],
                                   "n_cloud": STATE.n["cloud"], "pts": STATE.pts,
                                   "range": STATE.rng,
                                   "ms": [STATE.ms["rgb"], STATE.ms["depth"]]}).encode()
            self._send(200, body, "application/json")
        elif path.endswith(".mjpg"):
            self.stream(path[1:-5])
        elif path.endswith(".jpg"):
            key = path[1:-4]
            with STATE.cond:
                data = STATE.out.get(key)
            if not data:
                self._send(503, b"no frame", "text/plain")
            else:
                self._send(200, data, "image/jpeg")
        else:
            self._send(404, b"not found", "text/plain")

    def stream(self, key):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        last = -1
        try:
            while True:
                data, last = STATE.wait(key, last)
                if not data:
                    continue
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                 + str(len(data)).encode() + b"\r\n\r\n")
                self.wfile.write(data)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass


def main():
    rclpy.init()
    node = Viewer()
    Worker(node.calib).start()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    srv = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    srv.daemon_threads = True
    node.get_logger().info("http://0.0.0.0:%d/" % PORT)
    srv.serve_forever()


if __name__ == "__main__":
    main()

