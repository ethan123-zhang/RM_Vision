#!/usr/bin/env python3
"""遥控"敌方车"窗口：你按 W/S/A/D 把敌车开过来，哨兵车那边用"点地图"自己走过去。

为什么不能直接发 cmd_vel：敌车是一个 static 干扰模型（没有底盘插件），所以这个窗口在
自身里做运动学积分（按仿真时钟），再每 50 ms 用 `gz set_pose` 把它摆到新位置。
这样位置完全可控、也不参与物理求解，WSL2 的 llvmpipe 下不会把 GUI 拖死。

按键（窗口要有焦点，先点一下窗口；没有窗口管理器时用下面的按钮按住）：
  W/↑前进  S/↓后退  A/←左移  D/→右移  Q左转  E右转  空格停
  敌车只能走在"车能站"的自由格上（净空 >= --clearance），撞墙/出界会被挡住并提示 BLOCKED。

用法：
  python3 tools/enemy_teleop.py --start 2.0 1.0          # 在 (2.0,1.0) 生成并开始遥控
  python3 tools/enemy_teleop.py --start 2.0 1.0 --speed 0.6
"""
import argparse
import math
import os
import signal
import subprocess
import sys
import threading
import time

os.environ.setdefault("ROS_DOMAIN_ID", "45")

_EARLY = []


def _early_sig(signum, _frame):
    _EARLY.append(signum)


signal.signal(signal.SIGINT, _early_sig)
signal.signal(signal.SIGTERM, _early_sig)

import numpy as np                                                 # noqa: E402
from PIL import Image                                              # noqa: E402
from scipy import ndimage                                          # noqa: E402

import rclpy                                                       # noqa: E402
from rclpy.executors import SingleThreadedExecutor                # noqa: E402
from rclpy.node import Node                                        # noqa: E402
from rclpy.parameter import Parameter                              # noqa: E402
from rclpy.signals import SignalHandlerOptions                     # noqa: E402
from geometry_msgs.msg import PoseStamped                          # noqa: E402
from nav_msgs.msg import Odometry                                  # noqa: E402

from gz.msgs10.boolean_pb2 import Boolean                          # noqa: E402
from gz.msgs10.pose_pb2 import Pose                                # noqa: E402
from gz.transport13 import Node as GzNode                          # noqa: E402

import tkinter as tk                                               # noqa: E402

WORLD = "/world/rmul2027_3v3"
DEFAULT_MAP = "/mnt/e/RM/tb3_gazebo_nav2/rm_field/maps/rmul2027_3v3_up.yaml"
HERE = os.path.dirname(os.path.abspath(__file__))
CONTACT_M = 0.47          # 0.5x0.5 哨兵 + 0.4x0.4 敌车：面贴面 0.45，取 0.47 留 2 cm
HEIGHT = 0.25


def load_map(path):
    cfg = {}
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if ":" in line:
                k, v = line.split(":", 1)
                cfg[k.strip()] = v.strip()
    res = float(cfg["resolution"])
    ox, oy = [float(x) for x in cfg["origin"].strip("[] ").split(",")[:2]]
    occ = np.array(Image.open(os.path.join(os.path.dirname(path), cfg["image"]))) < 80
    clear = ndimage.distance_transform_edt(~occ) * res
    return res, ox, oy, occ, clear


class Backend:
    def __init__(self, name, start, map_path, clearance):
        self.name = name
        self.pos = [float(start[0]), float(start[1])]
        self.yaw = 0.0
        self.clearance = clearance
        res, ox, oy, occ, clear = load_map(map_path)
        self.res, self.ox, self.oy, self.occ, self.clear = res, ox, oy, occ, clear
        self.sentry = None
        self.min_dist = 999.0
        self.contacts = 0
        self._in_contact = False
        self.blocked = 0
        self.last_t = None

        self.node = Node("enemy_teleop",
                         parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.node.create_subscription(Odometry, "/odin1/odometry", self._on_sentry, 10)
        self.pose_pub = self.node.create_publisher(PoseStamped, "/enemy_pose", 10)
        self.gz = GzNode()
        self._stop = threading.Event()
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self.node)
        self._th = threading.Thread(target=self._spin, daemon=True)
        self._th.start()

    def _spin(self):
        while not self._stop.is_set():
            try:
                self._exec.spin_once(timeout_sec=0.05)
            except Exception:                                       # noqa: BLE001
                if self._stop.is_set():
                    break
                raise

    def _on_sentry(self, m):
        p = m.pose.pose.position
        self.sentry = (p.x, p.y)
        d = math.hypot(self.pos[0] - p.x, self.pos[1] - p.y)
        self.min_dist = min(self.min_dist, d)
        if d < CONTACT_M and not self._in_contact:
            self._in_contact = True
            self.contacts += 1
            print(f"CONTACT 敌车({self.pos[0]:.2f},{self.pos[1]:.2f}) "
                  f"哨兵({p.x:.2f},{p.y:.2f}) 中心距 {d:.2f} m", flush=True)
        elif d > CONTACT_M + 0.10:
            self._in_contact = False

    def clear_at(self, x, y):
        H, W = self.occ.shape
        r = H - 1 - int(round((y - self.oy) / self.res))
        c = int(round((x - self.ox) / self.res))
        if not (0 <= r < H and 0 <= c < W):
            return -1.0
        return float(self.clear[r, c])

    def shift(self, dx, dy):
        """把敌车按世界坐标增量挪一下（带自由区校验）。返回是否成功。"""
        nx, ny = self.pos[0] + dx, self.pos[1] + dy
        if self.clear_at(nx, ny) < self.clearance:
            self.blocked += 1
            return False
        self.pos[0], self.pos[1] = nx, ny
        self._apply()
        return True

    def step(self, vx, vy, wz):
        """按仿真时钟积分一帧；返回 (vx, vy, wz) 实际执行量。"""
        now = self.node.get_clock().now().nanoseconds / 1e9
        dt = 0.05 if self.last_t is None else max(0.0, min(0.2, now - self.last_t))
        self.last_t = now
        if wy := wz:
            self.yaw = (self.yaw + wy * dt + math.pi) % (2 * math.pi) - math.pi
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        wx = (vx * c - vy * s) * dt
        wy_ = (vx * s + vy * c) * dt
        if abs(wx) > 1e-6 or abs(wy_) > 1e-6:
            if not self.shift(wx, wy_):
                return 0.0, 0.0, wz
        else:
            self._apply()
        return vx, vy, wz

    def _apply(self):
        req = Pose(name=self.name)
        req.position.x = self.pos[0]
        req.position.y = self.pos[1]
        req.position.z = HEIGHT / 2.0
        half = self.yaw / 2.0
        req.orientation.z = math.sin(half)
        req.orientation.w = math.cos(half)
        self.gz.request(WORLD + "/set_pose", req, Pose, Boolean, 800)
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x = self.pos[0]
        msg.pose.position.y = self.pos[1]
        msg.pose.orientation.z = req.orientation.z
        msg.pose.orientation.w = req.orientation.w
        self.pose_pub.publish(msg)

    def shutdown(self):
        self._stop.set()
        if self._th.is_alive():
            self._th.join(timeout=2.0)
        try:
            self._exec.shutdown()
        except Exception:                                           # noqa: BLE001
            pass
        try:
            self.node.destroy_node()
        except Exception:                                           # noqa: BLE001
            pass


class Win(tk.Tk):
    def __init__(self, backend, speed):
        super().__init__()
        self.backend = backend
        self.speed = speed
        self.keys = set()
        self.closing = False
        self.title("遥控敌车（W/↑前进 S/↓后退 A/← D/→ 平移 Q/E 转 空格停）")
        self.protocol("WM_DELETE_WINDOW", self._close)
        self.status = tk.StringVar(value="点一下窗口拿焦点，再按 W 开；或鼠标按住下面按钮")
        tk.Label(self, textvariable=self.status, anchor="w", justify="left",
                 font=("Consolas", 11), width=95, height=2, bg="#f0f0f0").pack(fill="x")
        tk.Label(self, text="W/↑ 前进   S/↓ 后退   A/← 左移   D/→ 右移   Q 左转   E 右转   空格 停"
                            "（敌车只能走在净空够的自由格上，撞墙会显示 BLOCKED）",
                 font=("Consolas", 10), bg="#e8e8e8").pack(fill="x")
        bar = tk.Frame(self)
        bar.pack(fill="x")
        for text, key in (("▲ 前进", "w"), ("▼ 后退", "s"), ("◀ 左移", "a"), ("▶ 右移", "d"),
                          ("↺ 左转", "q"), ("↻ 右转", "e"), ("■ 停", "space")):
            b = tk.Button(bar, text=text, width=8, font=("Consolas", 10))
            b.bind("<ButtonPress-1>", lambda _e, k=key: self.keys.add(k))
            b.bind("<ButtonRelease-1>", lambda _e, k=key: self.keys.discard(k))
            b.pack(side="left", padx=2, pady=2)
        self.bind("<KeyPress>", self._press)
        self.bind("<KeyRelease>", self._release)
        self.canvas = tk.Canvas(self, width=520, height=250, bg="#fafafa", highlightthickness=1)
        self.canvas.pack()
        self.canvas.bind("<Button-1>", lambda _e: self.focus_force())
        self.after(50, self._tick)
        self.geometry("880x390")
        self.focus_force()

    def _press(self, ev):
        self.keys.add(ev.keysym.lower())
        return "break"

    def _release(self, ev):
        self.keys.discard(ev.keysym.lower())
        return "break"

    def _vector(self):
        vx = vy = wz = 0.0
        s = self.speed
        if "w" in self.keys or "up" in self.keys:
            vx += s
        if "s" in self.keys or "down" in self.keys:
            vx -= s
        if "a" in self.keys or "left" in self.keys:
            vy += s * 0.7
        if "d" in self.keys or "right" in self.keys:
            vy -= s * 0.7
        if "q" in self.keys:
            wz += 1.2
        if "e" in self.keys:
            wz -= 1.2
        if "space" in self.keys:
            vx = vy = wz = 0.0
        return vx, vy, wz

    def _tick(self):
        if self.closing:
            return
        vx, vy, wz = self._vector()
        ex = self.backend.step(vx, vy, wz)
        b = self.backend
        text = (f"敌车 ({b.pos[0]:+.2f},{b.pos[1]:+.2f}) yaw {math.degrees(b.yaw):+.0f}°  "
                f"指令 ({ex[0]:+.2f},{ex[1]:+.2f},{ex[2]:+.2f})  速度 {self.speed:.2f}")
        if b.sentry is not None:
            d = math.hypot(b.pos[0] - b.sentry[0], b.pos[1] - b.sentry[1])
            text += (f" | 哨兵 ({b.sentry[0]:+.2f},{b.sentry[1]:+.2f}) 距离 {d:.2f} m"
                     f" | 最近 {b.min_dist:.2f} m 碰撞 {b.contacts} 次")
        if b.blocked:
            text += f" | BLOCKED×{b.blocked}"
        self.status.set(text)
        self._draw()
        self.after(50, self._tick)

    def _draw(self):
        c, b = self.canvas, self.backend
        c.delete("all")
        c.create_text(8, 8, anchor="nw", font=("Consolas", 10), fill="#555",
                      text="红=敌车 蓝=哨兵（示意，不按真实比例）")
        cx, cy = 260, 125
        c.create_rectangle(cx - 12, cy - 12, cx + 12, cy + 12, outline="#d02020", width=3)
        if b.sentry is not None:
            dx, dy = b.sentry[0] - b.pos[0], b.sentry[1] - b.pos[1]
            d = math.hypot(dx, dy)
            k = min(90.0, d * 40.0)
            sx, sy = cx + dx / (d or 1) * k, cy - dy / (d or 1) * k
            c.create_oval(sx - 12, sy - 12, sx + 12, sy + 12, outline="#2060ff", width=2)
            c.create_text(cx, 216, text=f"距离 {d:.2f} m", font=("Consolas", 10), fill="#333")

    def _close(self):
        self.closing = True
        self.quit()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", nargs=2, type=float, default=(2.0, 1.5), metavar=("X", "Y"))
    ap.add_argument("--speed", type=float, default=0.6, help="敌车速度 m/s（默认 0.6）")
    ap.add_argument("--clearance", type=float, default=0.55, help="敌车允许走的净空 m")
    ap.add_argument("--name", default="enemy_robot")
    ap.add_argument("--map", default=DEFAULT_MAP)
    args = ap.parse_args()

    subprocess.run([sys.executable, os.path.join(HERE, "spawn_obstacle.py"), "remove",
                    "--name", args.name], capture_output=True)
    rc = subprocess.run([sys.executable, os.path.join(HERE, "spawn_obstacle.py"), "spawn",
                         f"{args.start[0]:.3f}", f"{args.start[1]:.3f}",
                         "--name", args.name]).returncode
    if rc != 0:
        print(f"spawn 失败 rc={rc}", flush=True)
        return 4

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    if _EARLY:
        rclpy.shutdown()
        return 0
    backend = Backend(args.name, args.start, args.map, args.clearance)
    backend._apply()
    win = Win(backend, args.speed)
    try:
        win.mainloop()
    except KeyboardInterrupt:
        pass
    backend.shutdown()
    if rclpy.ok():
        rclpy.shutdown()
    print(f"ENEMY_TELEOP done | 最近距离 {backend.min_dist:.2f} m | 碰撞 {backend.contacts} 次",
          flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())