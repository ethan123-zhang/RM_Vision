#!/usr/bin/env python3
"""地图点击窗口：在一张 2D 地图上点一下，机器人就导航过去（不依赖 RViz 的工具选择）。

为什么做这个：RViz 的"点地图→导航"要先在工具栏里选中 Nav2 Goal 工具，选中状态不明显；
用户实测"在 RViz 上点了没反应"（点击只是平移了视角）。这个窗口把"地图 + 点击"做成唯一交互，
点哪去哪，不会点错工具。

界面：
  - 左键点地图 = 发导航目标（直发 NavigateToPose；不用 /goal_pose 桥）
  - 右键点地图 = 给 AMCL 初始位姿（重定位，发 /initialpose）
  - 中键 = 取消当前目标
  - 绿色三角 = 机器人当前位置（TF map→base_footprint）；蓝叉 = 最近一次点的目标；
    灰线 = 当前全局路径（/plan）
  - 底部状态栏显示 ROS 状态、点击坐标、目标结果

用法：
  source /opt/ros/jazzy/setup.bash
  python3 map_click.py --map maps/rmul2027_3v3_up.yaml --ppm 70
（nav_gui.sh 会自动起它；栈没起也没关系，窗口会显示"等 action server"并自动重试。）
"""
import argparse
import math
import os
import sys
import threading
import time

os.environ.setdefault("ROS_DOMAIN_ID", "42")

import rclpy                                                    # noqa: E402
from rclpy.action import ActionClient                           # noqa: E402
from rclpy.node import Node                                     # noqa: E402
from rclpy.parameter import Parameter                           # noqa: E402
from rclpy.time import Time                                     # noqa: E402
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped  # noqa: E402
from nav_msgs.msg import Path                                   # noqa: E402
from nav2_msgs.action import NavigateToPose                     # noqa: E402
from tf2_ros import Buffer, TransformListener                   # noqa: E402

import tkinter as tk                                            # noqa: E402
import numpy as np                                              # noqa: E402
from PIL import Image, ImageTk                                  # noqa: E402
from scipy import ndimage                                       # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
# 点击落点离墙太近（车体半径 0.22 放不下）时，自动吸附到最近的可站格子；
# 否则 nav2 会"目标在致命区"→反复重规划/恢复行为，看起来就是绕圈。
SNAP_MIN_CLEAR = 0.26      # m，robot_radius 0.22 + 4 cm 余量
SNAP_MAX_DIST = 1.5        # m，吸附搜索上限


def yaml_load(path):
    """只解析地图 yaml 里的 resolution/origin/image 三个字段（不引入 pyyaml 依赖）。"""
    cfg = {}
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if not line or ":" not in line:
                continue
            k, v = line.split(":", 1)
            cfg[k.strip()] = v.strip()
    res = float(cfg["resolution"])
    origin = [float(x) for x in cfg["origin"].strip("[] ").split(",")[:3]]
    img = os.path.join(os.path.dirname(path), cfg["image"])
    return res, origin, img


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class RosBackend:
    """rclpy 侧：发目标、取消、TF 位姿、路径、状态。"""

    def __init__(self):
        self.node = Node("map_click",
                         parameter_overrides=[Parameter("use_sim_time",
                                                       Parameter.Type.BOOL, True)])
        self.client = ActionClient(self.node, NavigateToPose, "navigate_to_pose")
        self.goal_handle = None
        self.status = "等 action server ..."
        self.result = ""
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self.node)
        self.path = None
        self.lock = threading.Lock()
        self.truth = None                  # Gazebo 真值（仅仿真；用于显示定位误差/一键重定位）
        self.last_goal = None              # 最近一次目标（重定位后要用它重发，让路径按新位姿重算）
        self.goal_active = False
        self.node.create_subscription(Path, "/plan", self._on_plan, 10)
        threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True).start()
        try:
            from gz.msgs10.pose_v_pb2 import Pose_V
            from gz.transport13 import Node as GzNode
            self.gz = GzNode()          # 必须存成成员，局部变量会被回收→收不到真值
            self.gz.subscribe(Pose_V, "/world/rmul2027_3v3/pose/info", self._on_truth)
        except Exception as e:                                   # noqa: BLE001
            print(f"（收不到 Gazebo 真值，定位误差显示不可用: {e}）", flush=True)

    def _on_truth(self, msg):
        for p in msg.pose:
            if "waffle" in p.name and "wheel" not in p.name:
                self.truth = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _on_plan(self, msg):
        with self.lock:
            self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def robot_pose(self):
        try:
            tr = self.tf.lookup_transform("map", "base_footprint", Time())
        except Exception:                                       # noqa: BLE001
            return None
        t = tr.transform.translation
        return t.x, t.y, yaw_of(tr.transform.rotation)

    def wait_server(self, timeout=2.0):
        return self.client.wait_for_server(timeout_sec=timeout)

    def send_goal(self, x, y):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.last_goal = (float(x), float(y))
        self.goal_active = True
        self.result = ""
        self.status = f"发送目标 ({x:.2f}, {y:.2f}) ..."
        fut = self.client.send_goal_async(goal)
        fut.add_done_callback(self._accepted)

    def _accepted(self, fut):
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.status = "目标被拒绝"
            return
        self.goal_handle = gh
        self.status = "导航中 ..."
        gh.get_result_async().add_done_callback(self._result)

    def _result(self, fut):
        r = fut.result()
        st = int(r.status)
        name = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}.get(st, f"STATUS_{st}")
        self.status = f"结果: {name}"
        self.goal_handle = None
        self.goal_active = False
        if name in ("SUCCEEDED", "CANCELED"):
            # 正常结束或被用户取消 → 不再自动重发；ABORTED（如定位被设错导致跟丢）
            # 保留 last_goal，等重定位后自动重发，路径才会按新位姿重算。
            self.last_goal = None

    def cancel(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.goal_active = False
        self.last_goal = None
        self.status = "已取消目标"

    def relocalize(self, x, y, yaw=0.0, resend_goal=True):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance = [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0,
                               0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0.0685, 0, 0,
                               0, 0, 0, 0, 0.0685, 0, 0, 0, 0, 0, 0, 0.0685]
        pub = self.node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        for _ in range(3):                       # 发 3 次：刚创建的 publisher 可能还没被发现
            pub.publish(msg)
            time.sleep(0.15)
        self.status = f"已发重定位 ({x:.2f}, {y:.2f})"
        # 重定位后必须让 nav2 按新位姿重新规划：取消当前目标并在 1 s 后重发同一个目标。
        # 不这样做的话，旧路径还是按错误位姿算的 → 机器人会沿着"断掉"的路径走或原地不动。
        if resend_goal and self.last_goal is not None:
            self.status = "已重定位，正在按新位姿重发目标（路径会重算）..."
            print(f"（重定位后重发目标 {self.last_goal}）", flush=True)
            threading.Timer(1.0, self._resend_goal).start()

    def _resend_goal(self):
        if self.last_goal is None:
            return
        print(f"RESEND goal {self.last_goal} after relocalize", flush=True)
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        x, y = self.last_goal
        self.send_goal(x, y)

    def relocalize_to_truth(self):
        """仿真专用：把 AMCL 初始位姿设成 Gazebo 真值（定位跟丢时的急救）。"""
        if self.truth is None:
            self.status = "拿不到仿真真值，无法一键重定位"
            return False
        self.relocalize(*self.truth)
        self.status = f"已按仿真真值重定位 ({self.truth[0]:.2f},{self.truth[1]:.2f})"
        print(f"RELOCALIZE truth ({self.truth[0]:.3f},{self.truth[1]:.3f})", flush=True)
        return True


class MapWindow(tk.Tk):
    def __init__(self, yaml_path, ppm):
        super().__init__()
        self.title("地图点选 → 机器人导航（左键=去这里 / Shift+左键=只报坐标 / 右键双击=重定位 / T=按真值重定位 / 中键=取消）")
        self.res_m, (ox, oy, _), img_path = yaml_load(yaml_path)
        self.ox, self.oy = ox, oy
        im = Image.open(img_path)
        if im.mode not in ("L", "RGB"):
            im = im.convert("L")
        self.im = im
        self.occ = np.array(im) < 80
        self.clear = ndimage.distance_transform_edt(~self.occ) * self.res_m
        self.w_m = im.width * self.res_m
        self.h_m = im.height * self.res_m
        self.ppm = ppm
        self.cw = int(round(self.w_m * ppm))
        self.ch = int(round(self.h_m * ppm))
        self.canvas = tk.Canvas(self, width=self.cw, height=self.ch, bg="#202020",
                                highlightthickness=0)
        self.canvas.pack()
        self.status_var = tk.StringVar(value="启动中 ...")
        tk.Label(self, textvariable=self.status_var, anchor="w", bg="#f0f0f0",
                 font=("Consolas", 10)).pack(fill="x")
        self.tkimg = ImageTk.PhotoImage(self._map_image())
        self.canvas.create_image(0, 0, anchor="nw", image=self.tkimg, tags="map")
        self._draw_grid()
        self.goal_xy = None
        self.clicked_xy = None
        self.backend = RosBackend()
        self.canvas.bind("<Button-1>", self._on_left)
        self.canvas.bind("<Button-3>", self._on_right)
        self.canvas.bind("<Button-2>", self._on_middle)
        self.bind_all("<KeyPress-t>", self._on_truth_key)
        self.bind_all("<KeyPress-T>", self._on_truth_key)
        self.after(100, self._tick)
        self.after(800, self._print_geom)

    def _on_truth_key(self, _ev):
        """按 T：用 Gazebo 真值一键重定位（仿真专用急救）。"""
        self.backend.relocalize_to_truth()
        return "break"

    def _self_test(self):
        """无鼠标时自检：右键两次（应触发确认重定位）→ 再按 T（按真值重定位）。"""
        print("SELFTEST start", flush=True)
        cx, cy = int(round((1.0 - self.ox) * self.ppm)), int(round((self.oy + self.h_m - 0.0) * self.ppm))
        self.canvas.event_generate("<Button-3>", x=cx, y=cy)
        self.after(400, lambda: self.canvas.event_generate("<Button-3>", x=cx, y=cy))
        self.after(1500, lambda: self._on_truth_key(None))
        self.after(3500, lambda: (print(f"SELFTEST truth={self.backend.truth}", flush=True),
                                  self._on_truth_key(None)))
        self.after(5000, lambda: print("SELFTEST done", flush=True))

    def _print_geom(self):
        """窗口映射后才有真实屏坐标；自动化点击（xdotool）靠这行换算。"""
        print(f"GEOM canvas_root=({self.canvas.winfo_rootx()},{self.canvas.winfo_rooty()}) "
              f"size=({self.cw}x{self.ch}) ppm={self.ppm}", flush=True)

    # ---- 坐标换算 -------------------------------------------------------
    def w2c(self, x, y):
        return (x - self.ox) * self.ppm, (self.oy + self.h_m - y) * self.ppm

    def c2w(self, cx, cy):
        return self.ox + cx / self.ppm, self.oy + self.h_m - cy / self.ppm

    # ---- 落点吸附 -------------------------------------------------------
    def _clear_at(self, x, y):
        """世界坐标 → 该格离障碍的距离（m）；出界返回 -1。"""
        H, W = self.occ.shape
        r = H - 1 - int(round((y - self.oy) / self.res_m))
        c = int(round((x - self.ox) / self.res_m))
        if not (0 <= r < H and 0 <= c < W):
            return -1.0
        return float(self.clear[r, c])

    def snap(self, x, y):
        """把点击点挪到最近"车体能站下"的格子。

        返回 (gx, gy, clicked_clear, moved)；附近 SNAP_MAX_DIST 内找不到就返回 None。
        车体半径 0.22 m：中心离障碍 <0.22 的格子对 nav2 是致命区，目标点会被拒/反复绕。
        """
        H, W = self.occ.shape
        cr = H - 1 - int(round((y - self.oy) / self.res_m))
        cc = int(round((x - self.ox) / self.res_m))
        if not (0 <= cr < H and 0 <= cc < W):
            return None
        clicked_clear = float(self.clear[cr, cc])
        if clicked_clear >= SNAP_MIN_CLEAR:
            return x, y, clicked_clear, False
        rad = int(SNAP_MAX_DIST / self.res_m) + 1
        r0, r1 = max(0, cr - rad), min(H, cr + rad + 1)
        c0, c1 = max(0, cc - rad), min(W, cc + rad + 1)
        win = self.clear[r0:r1, c0:c1]
        ok = win >= SNAP_MIN_CLEAR
        if not ok.any():
            return None
        rr, ccs = np.nonzero(ok)
        d2 = (rr + r0 - cr) ** 2 + (ccs + c0 - cc) ** 2
        k = int(np.argmin(d2))
        if math.sqrt(d2[k]) * self.res_m > SNAP_MAX_DIST:
            return None
        r, c = int(rr[k] + r0), int(ccs[k] + c0)
        gx = self.ox + c * self.res_m
        gy = self.oy + (H - 1 - r) * self.res_m
        return gx, gy, clicked_clear, True

    # ---- 静态图 ---------------------------------------------------------
    def _map_image(self):
        out = Image.new("RGB", (int(round(self.w_m * self.ppm)),
                                int(round(self.h_m * self.ppm))), "#f5f5f5")
        src = self.im
        sx = out.width / src.width
        sy = out.height / src.height
        px = src.load()
        opt = out.load()
        for r in range(src.height):
            for c in range(src.width):
                v = px[c, r]
                if v < 80:
                    col = (40, 40, 40)          # 障碍
                elif v > 230:
                    col = (245, 245, 245)       # 自由
                else:
                    col = (170, 170, 170)       # 未知
                x0, y0 = int(c * sx), int(r * sy)
                x1, y1 = int((c + 1) * sx), int((r + 1) * sy)
                for yy in range(y0, min(y1, out.height)):
                    for xx in range(x0, min(x1, out.width)):
                        opt[xx, yy] = col
        return out

    def _draw_grid(self):
        for m in range(0, int(self.w_m) + 1):
            cx, _ = self.w2c(self.ox + m, 0)
            self.canvas.create_line(cx, 0, cx, self.ch, fill="#d0d0d0", dash=(1, 4))
        for m in range(0, int(self.h_m) + 1):
            _, cy = self.w2c(0, self.oy + m)
            self.canvas.create_line(0, cy, self.cw, cy, fill="#d0d0d0", dash=(1, 4))

    # ---- 事件 -----------------------------------------------------------
    def _on_left(self, ev):
        x, y = self.c2w(ev.x, ev.y)
        clr = self._clear_at(x, y)
        if ev.state & 0x0001:           # 按住 Shift：只报坐标，不导航（用来指认"洞"的位置）
            self.clicked_xy = (x, y)
            self.goal_xy = None
            print(f"PROBE ({x:.3f}, {y:.3f}) clearance={clr:.2f} m", flush=True)
            self.backend.status = f"探针点 ({x:.2f},{y:.2f}) 净空 {clr:.2f} m（未导航）"
            return
        g = self.snap(x, y)
        if g is None:
            print(f"REJECT ({x:.3f}, {y:.3f}) 附近 {SNAP_MAX_DIST} m 内没有能站车的位置", flush=True)
            self.backend.status = "点在障碍/太窄处，附近没有可站的位置，没发目标"
            return
        gx, gy, clicked_clear, moved = g
        self.clicked_xy = (x, y) if moved else None
        self.goal_xy = (gx, gy)
        if moved:
            msg = (f"点击 ({x:.2f},{y:.2f}) 离障碍仅 {clicked_clear:.2f} m"
                   f"（车要 0.22 m）→ 自动挪到 ({gx:.2f},{gy:.2f})")
            print(f"CLICK go ({gx:.3f}, {gy:.3f}) [clicked ({x:.3f},{y:.3f}) clear={clicked_clear:.2f} snapped]", flush=True)
            self.backend.status = msg
        else:
            print(f"CLICK go ({gx:.3f}, {gy:.3f})", flush=True)
        self.backend.send_goal(gx, gy)

    def _on_right(self, ev):
        """右键重定位：必须 1.5 s 内在同一处（±0.3 m）再点一次才生效，防误触把定位设飞。"""
        x, y = self.c2w(ev.x, ev.y)
        now = time.monotonic()
        prev = getattr(self, "_pending_reloc", None)
        if (prev is not None and now - prev[2] < 1.5
                and math.hypot(x - prev[0], y - prev[1]) < 0.30):
            self._pending_reloc = None
            print(f"CLICK relocalize ({x:.3f}, {y:.3f}) confirmed", flush=True)
            self.backend.relocalize(x, y)
            return
        self._pending_reloc = (x, y, now)
        self.backend.status = (f"重定位待确认 ({x:.2f},{y:.2f})：1.5 s 内再右键同一处才生效"
                               f"（误触会把 AMCL 设飞）")

    def _on_middle(self, _ev):
        print("CLICK cancel", flush=True)
        self.backend.cancel()

    # ---- 刷新 -----------------------------------------------------------
    def _tick(self):
        if self.backend.client.server_is_ready():
            if self.backend.status.startswith("等 action"):
                self.backend.status = "就绪：左键点地图派目标"
        else:
            self.backend.status = "等 action server ...（栈没起来？）"
        self.canvas.delete("dyn")
        with self.backend.lock:
            path = list(self.backend.path or [])
        if len(path) > 1:
            pts = []
            for x, y in path:
                pts.extend(self.w2c(x, y))
            self.canvas.create_line(*pts, fill="#9a9a9a", width=2, tags="dyn")
        if self.clicked_xy is not None:
            cx0, cy0 = self.w2c(*self.clicked_xy)
            self.canvas.create_oval(cx0 - 3, cy0 - 3, cx0 + 3, cy0 + 3,
                                    outline="#d0a000", width=2, tags="dyn")
        if self.goal_xy is not None:
            gx, gy = self.w2c(*self.goal_xy)
            self.canvas.create_line(gx - 6, gy - 6, gx + 6, gy + 6, fill="#2060ff",
                                    width=3, tags="dyn")
            self.canvas.create_line(gx - 6, gy + 6, gx + 6, gy - 6, fill="#2060ff",
                                    width=3, tags="dyn")
        pose = self.backend.robot_pose()
        if pose is not None:
            rx, ry, ryaw = pose
            px, py = self.w2c(rx, ry)
            for ln, col in ((14, "#20a020"),):
                self.canvas.create_line(px, py, px + ln * math.cos(ryaw),
                                        py - ln * math.sin(ryaw), fill=col, width=3,
                                        arrow=tk.LAST, tags="dyn")
                self.canvas.create_oval(px - 4, py - 4, px + 4, py + 4, fill=col,
                                        outline="", tags="dyn")
            loc = ""
            t = self.backend.truth
            if t is not None:
                e = math.hypot(rx - t[0], ry - t[1])
                loc = f" | 定位误差 {e:.2f} m{'  ⚠ 跟丢了，按 T 重定位' if e > 0.4 else ''}"
            self.status_var.set(
                f"{self.backend.status} | 机器人 ({rx:.2f}, {ry:.2f}) yaw {math.degrees(ryaw):.0f}°{loc} "
                f"| 左键=目标 Shift+左键=报坐标 右键双击=重定位 T=按真值重定位")
        else:
            self.status_var.set(f"{self.backend.status} | 等 TF map->base_footprint ...")
        self.after(100, self._tick)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default=os.path.join(HERE, "maps", "rmul2027_3v3_up.yaml"))
    ap.add_argument("--ppm", type=int, default=70, help="每米多少像素（默认 70）")
    ap.add_argument("--self-test", action="store_true",
                    help="启动后自动触发一次右键重定位确认 + T 重定位（无鼠标环境自检）")
    args = ap.parse_args()
    rclpy.init()
    win = MapWindow(args.map, args.ppm)
    if args.self_test:
        win.after(1500, win._self_test)
    try:
        win.mainloop()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())