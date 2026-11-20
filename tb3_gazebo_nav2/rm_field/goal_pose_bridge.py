#!/usr/bin/env python3
"""把"在 RViz 地图上点的坐标"变成导航目标：订阅 /goal_pose → 调 NavigateToPose。

为什么需要它：RViz 的 **2D Goal Pose** 工具只会往 `/goal_pose` 发一个 PoseStamped，
不会直接发 Nav2 的 action；Nav2 Goal 工具（nav2_rviz_plugins）才直接发 action。
两种点法都让用户能用，代价是这个几十行的桥：
  - 点 `2D Goal Pose` → /goal_pose → 本节点 → /navigate_to_pose（可脚本化测试）
  - 点 `Nav2 Goal`   → 直接 /navigate_to_pose（不经过本节点）
再点一个新坐标 = **换目标**：先取消上一个 action，再发新的（不会排队）。

用法（由 nav_gui.sh / start_goal_bridge.sh 拉起）：
  python3 goal_pose_bridge.py            # 默认 ROS_DOMAIN_ID=42、等待 action server
stdout：`READY`（就绪）→ 每次点击 `GOAL [...]` → `CANCELLED`/`ACCEPTED`/`SUCCEEDED`/`ABORTED`
"""
import math
import os
import signal
import sys
import threading
import time

os.environ.setdefault("ROS_DOMAIN_ID", "42")

# 进程刚起、ROS 库还在 import 时收到 SIGINT/SIGTERM：默认处置是立即终止（exit 143，跳过清理）。
# 先接管成置位，main() 里再换成真正的 handler。
_EARLY_SIGNALS = []


def _early_signal(signum, _frame):
    _EARLY_SIGNALS.append(signum)


signal.signal(signal.SIGINT, _early_signal)
signal.signal(signal.SIGTERM, _early_signal)

import rclpy                                            # noqa: E402
from rclpy.action import ActionClient                   # noqa: E402
from rclpy.executors import SingleThreadedExecutor      # noqa: E402
from rclpy.node import Node                             # noqa: E402
from rclpy.parameter import Parameter                   # noqa: E402
from rclpy.signals import SignalHandlerOptions          # noqa: E402
from geometry_msgs.msg import PoseStamped                # noqa: E402
from nav2_msgs.action import NavigateToPose             # noqa: E402


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class GoalPoseBridge(Node):
    def __init__(self):
        super().__init__("goal_pose_bridge",
                         parameter_overrides=[Parameter("use_sim_time",
                                                       Parameter.Type.BOOL, True)])
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_subscription(PoseStamped, "/goal_pose", self._on_click, 10)
        self.goal_handle = None
        self.n = 0
        self.goal_xy = None
        self.last_feedback_t = 0.0
        # 去抖：同一个点 1 s 内重复发多次（脚本连发/工具重发）只算一次点击
        self.last_click = None          # (wall_t, x, y, yaw)
        self.get_logger().info("goal_pose_bridge 已启动（订阅 /goal_pose）")

    # ------------------------------------------------------------------ 点击
    def _on_click(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        yaw = yaw_of(q)
        now = time.monotonic()
        if self.last_click is not None:
            dt, lx, ly, lyaw = self.last_click
            if (now - dt < 1.0 and math.hypot(p.x - lx, p.y - ly) < 0.10
                    and abs(yaw - lyaw) < 0.10):
                print("IGNORED 1 s 内的同一点重复点击（去抖）", flush=True)
                return
        self.last_click = (now, p.x, p.y, yaw)
        if msg.header.frame_id not in ("map", ""):
            print(f"WARN 目标框架是 {msg.header.frame_id}，按 map 用（未做 TF 变换）", flush=True)
        if self.goal_handle is not None:
            self.get_logger().info("收到新点击：取消上一个目标")
            print("CANCELLED 上一个目标", flush=True)
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.n += 1
        goal = NavigateToPose.Goal()
        goal.pose = msg
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        self.goal_xy = (p.x, p.y, yaw)
        print(f"GOAL #{self.n} ({p.x:.2f}, {p.y:.2f}, yaw={yaw:.2f})", flush=True)
        fut = self.client.send_goal_async(goal, feedback_callback=self._fb)
        fut.add_done_callback(self._accepted)

    def _accepted(self, fut):
        gh = fut.result()
        if gh is None or not gh.accepted:
            print("REJECTED 导航拒绝了这个目标", flush=True)
            return
        self.goal_handle = gh
        print("ACCEPTED", flush=True)
        gh.get_result_async().add_done_callback(self._result)

    def _fb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_feedback_t < 1.0:
            return
        self.last_feedback_t = now
        print(f"FEEDBACK 剩余 {msg.feedback.distance_remaining:.2f} m / "
              f"恢复 {msg.feedback.number_of_recoveries} 次", flush=True)

    def _result(self, fut):
        r = fut.result()
        st = int(r.status)
        # rclpy GoalStatus：0 unknown 1 accepted 2 executing 3 canceling
        #                   4 succeeded 5 canceled 6 aborted
        name = {0: "UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING", 3: "CANCELING",
                4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}.get(st, f"STATUS_{st}")
        ec = int(r.result.error_code) if hasattr(r, "result") else -1
        print(f"RESULT {name} status={st} error_code={ec}", flush=True)
        if name == "SUCCEEDED" and self.goal_xy:
            print(f"ARRIVED ({self.goal_xy[0]:.2f}, {self.goal_xy[1]:.2f})", flush=True)
        self.goal_handle = None


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    # 信号只置位，不在 handler 里 shutdown：rclpy 默认 handler 会直接 rcl_shutdown，
    # 主循环还在 wait set 里 → RCLError（logs/goal_pose_bridge.log 那次就是这么崩的）
    stop = threading.Event()

    def _on_signal(_signum, _frame):
        stop.set()

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, _on_signal)
    if _EARLY_SIGNALS:                  # 启动阶段就收到信号：不建节点，直接干净退出
        print("启动时收到退出信号，直接退出", flush=True)
        rclpy.shutdown()
        return 0
    node = GoalPoseBridge()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        print("等待 /navigate_to_pose action server ...", flush=True)
        # 自己轮询而不用 wait_for_server：它一阻塞就是 90 s，期间 SIGTERM 退不出来
        t0 = time.monotonic()
        while not stop.is_set() and not node.client.server_is_ready():
            if time.monotonic() - t0 > 90.0:
                break
            time.sleep(0.1)
        if not stop.is_set() and not node.client.server_is_ready():
            print("!! 等不到导航 action server（栈没起来？）", flush=True)
            return 2
        # 等仿真时钟就绪，避免刚启动时 use_sim_time 还是 0
        while not stop.is_set() and node.get_clock().now().nanoseconds == 0:
            executor.spin_once(timeout_sec=0.1)
        if stop.is_set():
            print("收到退出信号，直接退出", flush=True)
            return 0
        print("READY 可以在地图上点坐标了（2D Goal Pose 或 Nav2 Goal）", flush=True)
        while not stop.is_set():
            executor.spin_once(timeout_sec=0.1)
        print("收到退出信号，正在关闭 ...", flush=True)
        return 0
    finally:
        # 顺序：先停 executor（不再进 wait set），再销毁 node，最后 shutdown
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())