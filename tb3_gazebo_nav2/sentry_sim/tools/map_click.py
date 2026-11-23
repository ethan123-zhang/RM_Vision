#!/usr/bin/env python3
"""地图点选窗口：在地图上点一下，哨兵车导航到那里（不用 RViz 的工具选择）。

为什么做这个：RViz 要先把工具栏切到 Nav2 Goal 工具再点，工具选错时"点了没反应"；
Gazebo 自带的右键 Move To 只改模型位姿，不通知 Nav2，还会把 odom/TF 搞乱。
这个窗口把"地图 + 点击"做成唯一交互。

界面：
  - 左键点地图 = 发 NavigateToPose（先吸附到车体能站下的空格，见 --min-clear）
  - Shift+左键 = 探针：只报世界坐标和净空，不发目标
  - 右键 / 中键 = 取消当前目标
  - 绿色箭头 = 机器人当前位置（TF map->base_footprint）；蓝叉 = 目标；灰线 = /plan
  - 状态栏 = ROS 状态 / 机器人位姿 / 与 Gazebo 真值(/gt_odom) 的偏差 / 目标结果

与 rm_field/map_click.py 的区别：这套栈没有 AMCL，所以没有"右键重定位"和"按真值重定位"
（位姿由 odin1_sim_bridge 用真值 + 可注入的 map 偏移直接发），右键改作取消目标。

用法：
  source /opt/ros/jazzy/setup.bash
  python3 tools/map_click.py --map /mnt/e/RM/tb3_gazebo_nav2/rm_field/maps/rmul2027_3v3_up.yaml
  # 或者把窗口挂到整栈上： ./run_sentry_nav.sh gui:=true map_click:=true
"""
import argparse
import math
import os
import signal
import sys
import threading
import time

os.environ.setdefault("ROS_DOMAIN_ID", "45")

# 进程刚起、ROS 库还在 import 时收到 SIGINT/SIGTERM：默认处置是立即终止（exit 143，跳过清理）。
# 先接管成置位，main() 里再换成真正的 handler。
_EARLY_SIGNALS = []


def _early_signal(signum, _frame):
    _EARLY_SIGNALS.append(signum)


signal.signal(signal.SIGINT, _early_signal)
signal.signal(signal.SIGTERM, _early_signal)

import rclpy                                                    # noqa: E402
from rclpy.action import ActionClient                           # noqa: E402
from rclpy.executors import SingleThreadedExecutor              # noqa: E402
from rclpy.node import Node                                     # noqa: E402
from rclpy.parameter import Parameter                           # noqa: E402
from rclpy.signals import SignalHandlerOptions                  # noqa: E402
from rclpy.time import Time                                     # noqa: E402
from nav_msgs.msg import OccupancyGrid, Odometry, Path      # noqa: E402
from nav2_msgs.action import NavigateToPose                     # noqa: E402
from rclpy.qos import (DurabilityPolicy, QoSProfile,            # noqa: E402
                       ReliabilityPolicy)
from tf2_ros import Buffer, TransformListener                   # noqa: E402

import tkinter as tk                                            # noqa: E402
import numpy as np                                              # noqa: E402
from PIL import Image, ImageTk                                  # noqa: E402
from scipy import ndimage                                       # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MAP = "/mnt/e/RM/tb3_gazebo_nav2/rm_field/maps/rmul2027_3v3_up.yaml"
SNAP_MAX_DIST = 1.0        # m，吸附搜索上限（再远就不猜了，直接拒绝，免得"点这里去了那里"）


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
    """rclpy 侧：发目标、取消、TF 位姿、路径、真值偏差、结果状态。"""

    def __init__(self, truth_topic='/gt_odom'):
        self.node = Node("sentry_map_click",
                         parameter_overrides=[Parameter("use_sim_time",
                                                       Parameter.Type.BOOL, True)])
        self.client = ActionClient(self.node, NavigateToPose, "navigate_to_pose")
        self.goal_handle = None
        self.status = "等 action server ..."
        self.result_status = None        # SUCCEEDED / ABORTED / CANCELED
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self.node)
        self.path = None
        self.lock = threading.Lock()
        self.truth = None                # Gazebo 真值（/gt_odom），用来显示定位链路偏差
        self.last_goal = None
        self.node.create_subscription(Path, "/plan", self._on_plan, 10)
        self.node.create_subscription(Odometry, truth_topic, self._on_truth, 10)
        # S3: 临时障碍（敌车/箱子/锥桶）只存在于代价地图里，静态图上看不出来。抢一份全局
        # 代价地图，点目标时先查它，否则会照着敌车发目标 → 车顶着敌车挤。
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.costmap = None
        self.costmap_t = 0.0
        self.node.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                                      self._on_costmap, qos)
        self._stop = threading.Event()
        self._shut = False
        self._timers = []
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        # 不用 rclpy.spin()：它只能靠 rcl_shutdown 停，context 一关 spin 还在 wait set 里就
        # RCLError / C++ terminate。用 executor.spin_once 循环可以按退出的顺序收尾。
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

    def _spin(self):
        while not self._stop.is_set():
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception:                                   # noqa: BLE001
                if self._stop.is_set():
                    break                                       # 退出中，executor 已关
                raise                                           # 真异常不吞，暴露出来

    def _later(self, delay, fn):
        """可取消的定时器：shutdown 后再触发会去碰已经销毁的 node。"""
        t = threading.Timer(delay, fn)
        t.daemon = True
        t.start()
        self._timers.append(t)
        return t

    def shutdown(self):
        """退出顺序：先让 spin 线程退出，再销毁 node；rclpy.shutdown() 由调用方最后调。

        反过来（先关 context 再让 spin 醒来）会得到 RCLError / terminate。
        """
        if self._shut:
            return
        self._shut = True
        for t in self._timers:
            t.cancel()
            if t.is_alive():                                    # 已触发但还没跑完：等它走完再销毁 node
                t.join(timeout=1.0)
        self._timers = []
        self._stop.set()
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=3.0)
        try:
            self._executor.shutdown()
        except Exception:                                       # noqa: BLE001
            pass                                                # 退出路径上不能再抛
        try:
            self.node.destroy_node()
        except Exception:                                       # noqa: BLE001
            pass

    def _on_truth(self, msg):
        p = msg.pose.pose
        self.truth = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _on_plan(self, msg):
        with self.lock:
            self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _on_costmap(self, msg):
        """保存最新一帧全局代价地图（map 系，0.05 m/格，0=空 100=致命 -1=未知）。"""
        info = msg.info
        self.costmap = (msg.data, info.origin.position.x, info.origin.position.y,
                        info.resolution, info.width, info.height)
        self.costmap_t = time.monotonic()
        if not getattr(self, "_costmap_seen", False):
            self._costmap_seen = True
            print(f"COSTMAP ready {info.width}x{info.height} res={info.resolution:.3f}", flush=True)

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
        self.result_status = None
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
        if name == "ABORTED":
            # 实测坑：车停在离障碍 < robot_radius(0.40) 的位置时，全局规划器把起点判成
            # 致命区，之后点哪都规划失败（日志 "Failed to create plan ... tolerance"）。
            self.status = ("结果: ABORTED（规划失败）—— 车可能贴障碍太近；"
                           "点一个远离墙/箱子的目标，或先停栈重起")
        self.result_status = name
        self.goal_handle = None
        if name in ("SUCCEEDED", "CANCELED"):
            self.last_goal = None

    def cancel(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.last_goal = None
        self.status = "已取消目标"


class MapWindow(tk.Tk):
    def __init__(self, yaml_path, ppm, min_clear, truth_topic, stop_event=None):
        super().__init__()
        self._stop_event = stop_event
        self._closing = False
        self._shut = False
        self._selftest_done = None       # True/False，只有 --click 自检会设
        # 点窗口的 X 也走统一退出路径；不接管的话 Tk 直接 destroy，ROS 侧没人收尾
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.title("哨兵地图点选（左键=去这里 / Shift+左键=报坐标 / 右键或中键=取消）")
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
        self.min_clear = min_clear
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
        self.backend = RosBackend(truth_topic)
        self.canvas.bind("<Button-1>", self._on_left)
        self.canvas.bind("<Button-2>", self._on_middle)
        self.canvas.bind("<Button-3>", self._on_right)
        self.after(100, self._tick)
        self.after(800, self._print_geom)

    def _print_geom(self):
        """窗口映射后才有真实屏坐标；自动化点击（xdotool）靠这行换算。"""
        print(f"GEOM canvas_root=({self.canvas.winfo_rootx()},{self.canvas.winfo_rooty()}) "
              f"size=({self.cw}x{self.ch}) ppm={self.ppm} min_clear={self.min_clear}",
              flush=True)

    # ---- 退出 -----------------------------------------------------------
    def _on_close(self):
        """窗口关闭：只退 mainloop，ROS/Tk 的销毁留给 main() 按顺序做。"""
        self._closing = True
        self.quit()

    def shutdown(self):
        """统一退出：ROS 先干净退出（停 spin → 销毁 node → rclpy.shutdown），再销毁窗口。"""
        if self._shut:
            return
        self._shut = True
        self._closing = True
        self.backend.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        try:
            self.destroy()
        except tk.TclError:
            pass

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
        车体 0.5×0.5（外接半径 0.40）：中心离障碍 <0.40 的格子对 Nav2 是致命区，
        目标落在这里会被拒/反复重规划转圈，所以先吸附。
        """
        H, W = self.occ.shape
        cr = H - 1 - int(round((y - self.oy) / self.res_m))
        cc = int(round((x - self.ox) / self.res_m))
        if not (0 <= cr < H and 0 <= cc < W):
            return None
        clicked_clear = float(self.clear[cr, cc])
        if clicked_clear >= self.min_clear:
            return x, y, clicked_clear, False
        rad = int(SNAP_MAX_DIST / self.res_m) + 1
        r0, r1 = max(0, cr - rad), min(H, cr + rad + 1)
        c0, c1 = max(0, cc - rad), min(W, cc + rad + 1)
        win = self.clear[r0:r1, c0:c1]
        ok = win >= self.min_clear
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
        for m in range(0, int(self.w_m) + 1, 5):
            cx, _ = self.w2c(self.ox + m, 0)
            self.canvas.create_line(cx, 0, cx, self.ch, fill="#b0b0b0")
            self.canvas.create_text(cx + 2, 8, text=str(int(self.ox + m)), anchor="n",
                                    fill="#808080", font=("Consolas", 7))

    # ---- 事件 -----------------------------------------------------------
    def _on_left(self, ev):
        x, y = self.c2w(ev.x, ev.y)
        clr = self._clear_at(x, y)
        if ev.state & 0x0001:           # 按住 Shift：只报坐标，不导航（用来指认障碍位置）
            self.clicked_xy = (x, y)
            self.goal_xy = None
            print(f"PROBE ({x:.3f}, {y:.3f}) clearance={clr:.2f} m", flush=True)
            self.backend.status = f"探针点 ({x:.2f},{y:.2f}) 净空 {clr:.2f} m（未导航）"
            return
        g = self.snap(x, y)
        if g is None:
            print(f"REJECT ({x:.3f}, {y:.3f}) 附近 {SNAP_MAX_DIST} m 内没有净空 ≥{self.min_clear:.2f} m 的位置", flush=True)
            self.backend.status = (f"点在障碍/太窄处：附近 {SNAP_MAX_DIST} m 内找不到净空 "
                                   f"≥{self.min_clear:.2f} m 的位置，没发目标")
            return
        gx, gy, clicked_clear, moved = g
        dg = self.avoid_dynamic_goal(gx, gy)
        if dg is None:
            print(f"REJECT ({x:.3f}, {y:.3f}) 目标在临时障碍上（敌车/箱子），1 m 内没有空地", flush=True)
            self.backend.status = "目标在临时障碍上（敌车/箱子/锥桶），附近没有空位，没发目标"
            return
        gx, gy, dyn_cost, dyn_moved = dg
        self.clicked_xy = (x, y) if (moved or dyn_moved) else None
        self.goal_xy = (gx, gy)
        if moved or dyn_moved:
            why = []
            if moved:
                why.append(f"离静态障碍仅 {clicked_clear:.2f} m")
            if dyn_moved:
                why.append(f"目标是临时障碍（代价 {dyn_cost}）")
            msg = (f"点击 ({x:.2f},{y:.2f}) " + "、".join(why) +
                   f" → 自动挪到 ({gx:.2f},{gy:.2f})")
            print(f"CLICK go ({gx:.3f}, {gy:.3f}) [clicked ({x:.3f},{y:.3f}) "
                  f"static_clear={clicked_clear:.2f} dyn_cost={dyn_cost} snapped]", flush=True)
            self.backend.status = msg
        else:
            print(f"CLICK go ({gx:.3f}, {gy:.3f})", flush=True)
        self.backend.send_goal(gx, gy)

    # ---- 临时障碍避让：目标不能落在 /global_costmap 的高代价格上 --------------
    def _dyn_cost(self, x, y):
        cm = self.backend.costmap
        if cm is None:
            return None
        data, ox, oy, res, w, h = cm
        c = int((x - ox) / res)
        r = int((y - oy) / res)
        if not (0 <= c < w and 0 <= r < h):
            return -1
        return int(data[r * w + c])

    def avoid_dynamic_goal(self, gx, gy):
        """目标落在临时障碍上（代价 ≥85，即内切/致命区）就找附近空地，找不到返回 None。

        静态图上的吸附（snap）看不见敌车/箱子 —— 它们只在代价地图里；不查这一步，
        点"敌车后面"就会发一个被障碍占据的目标，车会顶着敌车挤。
        """
        cost = self._dyn_cost(gx, gy)
        if cost is None or cost < 85:
            return gx, gy, cost, False
        # 代价地图必须是新鲜的：旧面板进程可能只收到过一帧旧图（实测把空地误判成 99），
        # 超过 2 s 没更新就不做临时障碍判断，避免"点空地也被挪走"。
        if time.monotonic() - getattr(self.backend, "costmap_t", 0.0) > 2.0:
            print("WARN 代价地图超过 2 s 没更新，跳过临时障碍检查", flush=True)
            return gx, gy, cost, False
        data, ox, oy, res, w, h = self.backend.costmap
        c0 = int((gx - ox) / res)
        r0 = int((gy - oy) / res)
        rad = int(1.0 / res)
        best = None
        for dr in range(-rad, rad + 1):
            for dc in range(-rad, rad + 1):
                r, c = r0 + dr, c0 + dc
                if not (0 <= r < h and 0 <= c < w):
                    continue
                if data[r * w + c] > 20:
                    continue
                nx, ny = ox + c * res, oy + r * res
                if self._clear_at(nx, ny) < self.min_clear:      # 别挪进静态图的窄缝
                    continue
                d2 = dr * dr + dc * dc
                if best is None or d2 < best[0]:
                    best = (d2, nx, ny)
        if best is None:
            return None
        return best[1], best[2], cost, True

    def _on_right(self, _ev):
        print("CLICK cancel", flush=True)
        self.backend.cancel()

    def _on_middle(self, _ev):
        print("CLICK cancel", flush=True)
        self.backend.cancel()

    # ---- 自检（无鼠标环境：按世界坐标点一次，等结果，报最终位姿）----------
    def programmatic_click(self, x, y):
        cx, cy = self.w2c(x, y)
        print(f"SELFTEST click world=({x:.3f},{y:.3f}) canvas=({cx:.0f},{cy:.0f})", flush=True)
        self.canvas.event_generate("<Button-1>", x=int(cx), y=int(cy))

    def watch_result(self, gx, gy, deadline):
        rs = self.backend.result_status
        if rs is not None:
            pose = self.backend.robot_pose()
            dist = None if pose is None else math.hypot(pose[0] - gx, pose[1] - gy)
            print(f"SELFTEST result={rs} goal=({gx:.3f},{gy:.3f}) pose={pose} "
                  f"dist_to_goal={dist}", flush=True)
            self._selftest_done = (rs == "SUCCEEDED" and dist is not None and dist <= 0.4)
            self.after(200, self._on_close)
            return
        if time.monotonic() > deadline:
            print(f"SELFTEST TIMEOUT status={self.backend.status}", flush=True)
            self._selftest_done = False
            self.after(200, self._on_close)
            return
        self.after(500, lambda: self.watch_result(gx, gy, deadline))

    # ---- 刷新 -----------------------------------------------------------
    def _tick(self):
        if self._closing:
            return
        # 信号 handler 只置位，主循环在这里看到再收尾（信号 handler 里碰 Tk/ROS 不安全）
        if self._stop_event is not None and self._stop_event.is_set():
            print("收到退出信号，关闭窗口", flush=True)
            self._on_close()
            return
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
            self.canvas.create_line(px, py, px + 14 * math.cos(ryaw),
                                    py - 14 * math.sin(ryaw), fill="#20a020", width=3,
                                    arrow=tk.LAST, tags="dyn")
            self.canvas.create_oval(px - 4, py - 4, px + 4, py + 4, fill="#20a020",
                                    outline="", tags="dyn")
            dev = ""
            t = self.backend.truth
            if t is not None:
                dev = f" | 真值偏差 {math.hypot(rx - t[0], ry - t[1]):.2f} m"
            # 车自己贴到障碍 <0.40 m 时，全局规划器会把起点当致命区 → 之后点哪都不动，
            # 状态栏先把这个情况亮出来，不然就是"点了没反应"。
            here = self._clear_at(rx, ry)
            stuck = ""
            if 0.0 <= here < 0.45:
                stuck = f" | 净空 {here:.2f} m ⚠ 太贴障碍，规划会失败"
            self.status_var.set(
                f"{self.backend.status} | 机器人 ({rx:.2f}, {ry:.2f}) yaw {math.degrees(ryaw):.0f}°{dev}{stuck} "
                f"| 左键=去这里 Shift+左键=报坐标 右键=取消")
        else:
            self.status_var.set(f"{self.backend.status} | 等 TF map->base_footprint ...")
        self.after(100, self._tick)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default=DEFAULT_MAP)
    ap.add_argument("--ppm", type=int, default=70, help="每米多少像素（默认 70）")
    ap.add_argument("--min-clear", type=float, default=0.65,
                    help="落点最小净空 m（默认 0.65：goal 容差 0.2，停位净空仍 ≥0.45 > robot_radius 0.40）")
    ap.add_argument("--truth-topic", default="/gt_odom", help="仿真真值话题（只用于显示偏差）")
    ap.add_argument("--click", nargs=2, type=float, metavar=("X", "Y"),
                    help="自检：启动后在该世界坐标点一次，等结果后打印 FINAL 并退出")
    ap.add_argument("--click-timeout", type=float, default=240.0,
                    help="自检等结果的上限秒数（默认 240）")
    args = ap.parse_args()

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    # 信号只置位，不在 handler 里碰 ROS/Tk：rclpy 默认 handler 会直接 rcl_shutdown，
    # 后台 spin 还在 wait set 里 → RCLError / C++ terminate
    stop = threading.Event()

    def _on_signal(_signum, _frame):
        stop.set()

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, _on_signal)
    if _EARLY_SIGNALS:                  # 启动阶段就收到信号：不建窗口，直接干净退出
        print("启动时收到退出信号，直接退出", flush=True)
        rclpy.shutdown()
        return 0
    win = MapWindow(args.map, args.ppm, args.min_clear, args.truth_topic, stop)
    if args.click:
        gx, gy = args.click

        def _fire():
            win.programmatic_click(gx, gy)
            win.after(800, lambda: win.watch_result(gx, gy,
                                                    time.monotonic() + args.click_timeout))

        win.after(2500, _fire)
    try:
        if not stop.is_set():
            win.mainloop()
    except KeyboardInterrupt:
        pass
    win.shutdown()
    if args.click:
        ok = win._selftest_done
        print(f"SELFTEST exit ok={ok}", flush=True)
        return 0 if ok else 3
    return 0


if __name__ == "__main__":
    sys.exit(main())