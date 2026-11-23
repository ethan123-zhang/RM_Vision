#!/usr/bin/env python3
"""键盘遥控窗口：你手动开车，配合 wander_obstacle.py 的随机敌车做碰撞测试。

控制量进的是 `cmd_vel_nav`（velocity_smoother 的输入）→ 和自主导航同一条安全链：
smoother 限加速度 → collision_monitor 预判碰撞会减速/停 → `/cmd_vel` → 底盘。
所以"我撞敌车"时能不能撞上，就是这条链的真实表现。

按键（窗口要有焦点，先点一下窗口）：
  W / ↑   前进          S / ↓   后退
  A / ←   左平移        D / →   右平移
  Q       左转          E       右转
  空格    停车（清空指令）

状态栏显示：指令速度 / 底盘实际速度 / 车位置 / 离敌车中心距 / 碰撞计数 / collision_monitor 状态。

用法：
  python3 tools/teleop_gui.py                # 默认 topic cmd_vel_nav
  python3 tools/teleop_gui.py --topic cmd_vel_smoothed
"""
import argparse
import math
import os
import signal
import sys
import threading
import time

os.environ.setdefault("ROS_DOMAIN_ID", "45")

_EARLY_SIGNALS = []


def _early_signal(signum, _frame):
    _EARLY_SIGNALS.append(signum)


signal.signal(signal.SIGINT, _early_signal)
signal.signal(signal.SIGTERM, _early_signal)

import rclpy                                                        # noqa: E402
from rclpy.executors import SingleThreadedExecutor                  # noqa: E402
from rclpy.node import Node                                         # noqa: E402
from rclpy.parameter import Parameter                               # noqa: E402
from rclpy.signals import SignalHandlerOptions                      # noqa: E402
from geometry_msgs.msg import PoseStamped, Twist                    # noqa: E402
from nav_msgs.msg import Odometry                                   # noqa: E402
from nav2_msgs.msg import CollisionMonitorState                     # noqa: E402
from rcl_interfaces.msg import Parameter as RosParameter            # noqa: E402
from rcl_interfaces.msg import ParameterValue, ParameterType        # noqa: E402
from rcl_interfaces.srv import SetParameters                        # noqa: E402
import tkinter as tk                                                # noqa: E402

CONTACT_M = 0.47          # 0.5x0.5 车 + 0.4x0.4 敌车：面贴面时中心距 0.45，取 0.47 留 2 cm


class RosBackend:
    def __init__(self, topic):
        self.node = Node("teleop_gui",
                         parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        # 安全模式：进 cmd_vel_nav（smoother + collision_monitor → /cmd_vel）。
        # "裸模式"不是去 deactivate collision_monitor —— 那会断 bond，lifecycle_manager 判定
        # 节点失联后把整套导航节点一起关掉（2026-11-23 实测，车就彻底不动了）。
        # 正确做法：只把碰撞保护多边形 FootprintApproach.enabled 设成 false（可动态改，
        # 节点保持 active、bond 不断，命令原样直通）。
        self.pub_safe = self.node.create_publisher(Twist, topic, 10)
        self.raw = False
        self.param_client = self.node.create_client(SetParameters,
                                                    "/collision_monitor/set_parameters")
        self.car = None
        self.enemy = None
        self.base_cmd = (0.0, 0.0, 0.0)
        self.state = "?"
        self.min_dist = 999.0
        self.contacts = 0
        self._in_contact = False
        self.node.create_subscription(Odometry, "/odin1/odometry", self._on_odom, 10)
        self.node.create_subscription(PoseStamped, "/enemy_pose", self._on_enemy, 10)
        self.node.create_subscription(Twist, "/cmd_vel", self._on_cmd, 20)
        self.node.create_subscription(CollisionMonitorState, "/collision_monitor_state",
                                      self._on_mon, 10)
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self.node)
        self._stop = threading.Event()
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

    def _on_odom(self, m):
        p = m.pose.pose.position
        self.car = (p.x, p.y)

    def _on_enemy(self, m):
        p = m.pose.position
        self.enemy = (p.x, p.y)
        if self.car is not None:
            d = math.hypot(self.car[0] - p.x, self.car[1] - p.y)
            self.min_dist = min(self.min_dist, d)
            if d < CONTACT_M and not self._in_contact:
                self._in_contact = True
                self.contacts += 1
                print(f"CONTACT 车({self.car[0]:.2f},{self.car[1]:.2f}) "
                      f"敌车({p.x:.2f},{p.y:.2f}) 中心距 {d:.2f} m", flush=True)
            elif d > CONTACT_M + 0.10:
                self._in_contact = False

    def _on_cmd(self, m):
        self.base_cmd = (m.linear.x, m.linear.y, m.angular.z)

    def _on_mon(self, m):
        self.state = {0: "STOP", 1: "SLOWDOWN", 2: "APPROACH", 3: "PASS"}.get(
            int(m.action_type), str(int(m.action_type)))

    def send(self, vx, vy, wz):
        t = Twist()
        t.linear.x = float(vx)
        t.linear.y = float(vy)
        t.linear.z = 0.0
        t.angular.z = float(wz)
        self.pub_safe.publish(t)

    def set_guard(self, enabled):
        """开关 collision_monitor 的碰撞保护（FootprintApproach.enabled，动态参数）。

        不用 lifecycle deactivate：那会断 bond，lifecycle_manager 认为节点失联后会把整套
        导航节点一起停掉（实测车完全不动）。这里节点保持 active，命令只是不再被拦。
        """
        if not self.param_client.service_is_ready():
            if not self.param_client.wait_for_service(timeout_sec=2.0):
                print("SET_GUARD 服务不可用", flush=True)
                return False
        req = SetParameters.Request()
        req.parameters.append(RosParameter(
            name="FootprintApproach.enabled",
            value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=enabled)))
        fut = self.param_client.call_async(req)

        def _done(f):
            try:
                res = f.result().results[0]
                print(f"SET_GUARD enabled={enabled} -> {res.successful}", flush=True)
            except Exception as exc:                                # noqa: BLE001
                print(f"SET_GUARD 失败 {exc}", flush=True)

        fut.add_done_callback(_done)
        return True

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
        self.title("键盘遥控（W/↑前进 S/↓后退 A/← D/→ 平移 Q/E 转 空格停）")
        self.protocol("WM_DELETE_WINDOW", self._close)
        self.status = tk.StringVar(value="点一下窗口（或按住下面按钮）就能开：W/↑ 前进 …")
        lbl = tk.Label(self, textvariable=self.status, justify="left", anchor="w",
                       font=("Consolas", 11), width=95, height=2, bg="#f0f0f0")
        lbl.pack(fill="x")
        self.lbl = lbl
        tk.Label(self, text="W/↑ 前进   S/↓ 后退   A/← 左移   D/→ 右移   Q 左转   E 右转   空格 停"
                            "（没有窗口管理器时键盘可能不进焦点 → 用下面的按钮按住也能开）",
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
        self.raw_var = tk.BooleanVar(value=False)
        tk.Checkbutton(self, text="关碰撞保护（裸模式：能真撞上；取消勾选恢复安全模式）",
                       variable=self.raw_var, command=self._toggle_raw,
                       font=("Consolas", 10)).pack(fill="x")
        self.canvas = tk.Canvas(self, width=520, height=260, bg="#fafafa",
                                highlightthickness=1)
        self.canvas.pack()
        self.canvas.bind("<Button-1>", lambda _e: self.focus_force())
        self.after(50, self._tick)
        self.focus_force()

    def _press(self, ev):
        k = ev.keysym.lower() if len(ev.keysym) > 1 else ev.keysym.lower()
        self.keys.add(k)
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
            vy += s * 0.6
        if "d" in self.keys or "right" in self.keys:
            vy -= s * 0.6
        if "q" in self.keys:
            wz += 1.0
        if "e" in self.keys:
            wz -= 1.0
        return vx, vy, wz

    def _toggle_raw(self):
        want = bool(self.raw_var.get())
        if self.backend.set_guard(not want):
            self.backend.raw = want
            print(f"RAW {'ON（monitor 关闭，能撞）' if want else 'OFF（安全模式）'}", flush=True)

    def _tick(self):
        if self.closing:
            return
        vx, vy, wz = self._vector()
        if "space" in self.keys:
            vx = vy = wz = 0.0
        self.backend.send(vx, vy, wz)
        car = self.backend.car
        ene = self.backend.enemy
        dist = (math.hypot(car[0] - ene[0], car[1] - ene[1])
                if (car and ene) else None)
        bx, by, bw = self.backend.base_cmd
        mode = "裸模式(能撞)" if self.backend.raw else "安全模式"
        text = (f"指令 vx={vx:+.2f} vy={vy:+.2f} wz={wz:+.2f} "
                f"| 底盘收到 ({bx:+.2f},{by:+.2f},{bw:+.2f}) | {mode}")
        if car is not None:
            text += f" | 车 {car[0]:+.2f},{car[1]:+.2f}"
        else:
            text += " | 等车位置 ..."
        if dist is not None:
            text += (f" | 离敌车 {dist:.2f} m  最近 {self.backend.min_dist:.2f} m "
                     f"碰撞 {self.backend.contacts} 次 | 监控 {self.backend.state}")
        self.status.set(text)
        self._draw(car, ene, dist)
        self.after(50, self._tick)

    def _draw(self, car, ene, dist):
        c = self.canvas
        c.delete("all")
        c.create_text(8, 8, anchor="nw", font=("Consolas", 10), fill="#444",
                      text="蓝=我方向前 红=敌车（示意，不按真实比例）")
        cx, cy = 260, 190
        c.create_oval(cx - 12, cy - 12, cx + 12, cy + 12, outline="#2060ff", width=2)
        c.create_line(cx, cy, cx, cy - 26, fill="#2060ff", width=3, arrow=tk.LAST)
        if ene is not None and car is not None and dist is not None:
            dx, dy = ene[0] - car[0], ene[1] - car[1]
            k = min(60.0, dist * 45.0)
            ex, ey = cx + dx / (dist or 1) * k, cy - dy / (dist or 1) * k
            col = "#d02020" if dist < CONTACT_M else "#ffb020"
            c.create_rectangle(ex - 10, ey - 10, ex + 10, ey + 10, outline=col, width=2)
            c.create_text(cx, 245, text=f"距离 {dist:.2f} m", font=("Consolas", 10),
                          fill="#333")

    def _close(self):
        self.closing = True
        self.quit()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="cmd_vel_nav",
                    help="cmd_vel_nav（默认，走 smoother+collision_monitor）或 cmd_vel_smoothed")
    ap.add_argument("--speed", type=float, default=0.5, help="前进速度 m/s（默认 0.5）")
    args = ap.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()

    def _onsig(_s, _f):
        stop.set()

    for s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(s, _onsig)
    if _EARLY_SIGNALS:
        rclpy.shutdown()
        return 0
    backend = RosBackend(args.topic)
    backend.set_guard(True)          # 开局强制安全模式（碰撞保护开着），要撞自己勾裸模式
    time.sleep(0.5)
    win = Win(backend, args.speed)
    try:
        win.mainloop()
    except KeyboardInterrupt:
        pass
    if backend.raw:                       # 退出前把安全模式恢复回来
        backend.set_guard(True)
        time.sleep(1.0)
    backend.shutdown()
    if rclpy.ok():
        rclpy.shutdown()
    print(f"TELEOP done | 最近距离 {backend.min_dist:.2f} m | 碰撞 {backend.contacts} 次", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())