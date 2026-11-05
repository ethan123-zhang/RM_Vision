"""Odin1 实时点云安全门与局部绕行；由 follow_path 的同一执行器驱动。

此模块本身不创建速度发布者。所有指令最终由 follow_path._publish 审查后发出。
"""
from __future__ import annotations

import json
import math
import time
from collections import deque

import numpy as np

from .obstacle_grid import LocalGrid, follow_detour
from .path_io import yaw_from_quaternion


def decode_xyz(msg):
    """支持 Odin1 19 B 混合字段、行填充及大小端，不依赖同类型字段假设。"""
    fields = {f.name: f for f in msg.fields}
    if msg.width <= 0 or msg.height <= 0 or msg.point_step <= 0:
        raise ValueError("空点云布局")
    offsets = []
    for name in ('x', 'y', 'z'):
        f = fields.get(name)
        if f is None or f.datatype != 7 or f.count != 1 or not 0 <= f.offset <= msg.point_step - 4:
            raise ValueError(f"点云 {name} 必须是合法 FLOAT32 字段")
        offsets.append(f.offset)
    if msg.row_step < msg.point_step * msg.width or len(msg.data) < msg.row_step * msg.height:
        raise ValueError("点云数据长度/行步长非法")
    dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': [('>' if msg.is_bigendian else '<') + 'f4'] * 3,
                      'offsets': offsets, 'itemsize': msg.point_step})
    records = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                         strides=(msg.row_step, msg.point_step))
    return np.stack([records[name].ravel() for name in ('x', 'y', 'z')], axis=1)


def transform_matrix(tf):
    t, q = tf.transform.translation, tf.transform.rotation
    values = [t.x, t.y, t.z, q.x, q.y, q.z, q.w]
    if not all(math.isfinite(v) for v in values):
        raise ValueError("点云 TF 含 NaN/Inf")
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if norm < 1e-9:
        raise ValueError("点云 TF 四元数非法")
    x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
    result = np.eye(4)
    result[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    result[:3, 3] = values[:3]
    return result


def filter_obstacles(points, matrix, imu_pose, *, length, width, sensor_height,
                     offset_x, offset_y, min_height, max_height, front_extent=None):
    """使用重力对齐 odom 的高度带，以及相对转心的车身矩形去除自遮挡。"""
    p = np.asarray(points, dtype=float).reshape(-1, 3)
    valid = np.isfinite(p).all(axis=1) & (np.linalg.norm(p, axis=1) > .05)
    p = p[valid]
    # 小宽矩阵无需唤醒 BLAS 线程池，板端按列广播更稳定。
    p = (p[:, 0, None] * matrix[:3, 0] + p[:, 1, None] * matrix[:3, 1]
         + p[:, 2, None] * matrix[:3, 2] + matrix[:3, 3])
    ix, iy, iz, yaw = imu_pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = p[:, 0] - ix, p[:, 1] - iy
    bx = c * dx + s * dy + offset_x
    by = -s * dx + c * dy + offset_y
    front = length / 2 if front_extent is None else front_extent
    rear = length - front
    body = (bx <= front + .02) & (bx >= -rear - .02) & (np.abs(by) <= width / 2 + .02)
    height = p[:, 2] - (iz - sensor_height)
    good = ~body & (height >= min_height) & (height <= max_height)
    return p[good, :2]


class CloudHealth:
    def __init__(self, timeout_s):
        self.timeout = timeout_s
        self.last_stamp = None
        self.last_rx = None
        self.fault = ''

    def update(self, stamp, now, odom_stamp):
        if self.fault:
            return False
        if not math.isfinite(stamp) or stamp <= 0:
            return False
        if self.last_stamp is not None:
            if stamp < self.last_stamp:
                self.fault = '原始点云时间戳倒退，不自动恢复运动'
                return False
            if stamp == self.last_stamp:
                return False
        if odom_stamp is None or abs(stamp - odom_stamp) > self.timeout:
            return False
        self.last_stamp, self.last_rx = stamp, now
        return True

    def reason(self, now, odom_stamp):
        if self.fault:
            return self.fault
        if self.last_rx is None:
            return '未收到有效的原始点云，拒绝运动'
        if now - self.last_rx > self.timeout:
            return f'原始点云停更 {now-self.last_rx:.2f} s，停车'
        if odom_stamp is None or abs(self.last_stamp - odom_stamp) > self.timeout:
            return '原始点云与里程计设备时间不一致，停车'
        return None


class ObstacleAvoidance:
    def __init__(self, node):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2

        self.n = node
        self.ros_time = rclpy.time.Time
        defaults = dict(robot_length_m=.65, robot_width_m=.55, sensor_height_m=.23,
                        obstacle_margin_m=.10, obstacle_min_height_m=.06,
                        obstacle_max_height_m=.80, obstacle_resolution_m=.10,
                        obstacle_range_m=4., cloud_timeout_s=.6,
                        obstacle_wait_timeout_s=20., detour_timeout_s=90.,
                        detour_rejoin_ahead_m=2., detour_v_max=.12,
                        obstacle_cloud_topic='/odin1/cloud_raw')
        self.p = {}
        for name, default in defaults.items():
            node.declare_parameter(name, default)
            value = node.get_parameter(name).value
            if isinstance(default, float):
                value = float(value)
                if not math.isfinite(value) or value <= 0:
                    raise ValueError(f'{name} 必须是有限正数')
            elif not str(value).strip():
                raise ValueError(f'{name} 不能为空')
            self.p[name] = value
        p = self.p
        node.declare_parameter('robot_front_m', p['robot_length_m']/2)
        p['robot_front_m'] = float(node.get_parameter('robot_front_m').value)
        if not math.isfinite(p['robot_front_m']) or not 0 < p['robot_front_m'] < p['robot_length_m']:
            raise ValueError('robot_front_m 必须是转心到车头的正距离，且小于总车长')
        if p['obstacle_min_height_m'] >= p['obstacle_max_height_m']:
            raise ValueError('障碍高度带上下限顺序错误')
        if node.frame != 'map' or node.base_frame != 'imu':
            raise ValueError('避障要求 map 路线和已核实的 imu 车体参考系')
        extent = max(p['robot_front_m'], p['robot_length_m'] - p['robot_front_m'])
        self.radius = math.hypot(extent, p['robot_width_m']/2) + p['obstacle_margin_m']
        self.grid = LocalGrid(p['obstacle_resolution_m'], self.radius, p['obstacle_range_m'],
                              body=(p['robot_length_m'], p['robot_width_m'],
                                    p['robot_front_m'], p['obstacle_margin_m']))
        node.declare_parameter('cloud_processing_period_s', 0.0)
        self.processing_period = float(node.get_parameter('cloud_processing_period_s').value)
        if not math.isfinite(self.processing_period) or not 0 <= self.processing_period <= p['cloud_timeout_s']/2:
            raise ValueError('点云处理周期必须有限、非负且不超过超时的一半')
        self._last_processing_started = float('-inf')
        node.declare_parameter('obstacle_min_points_per_cell', 1)
        self.min_points = node.get_parameter('obstacle_min_points_per_cell').value
        if isinstance(self.min_points, bool) or not isinstance(self.min_points, int) or not 1 <= self.min_points <= 10:
            raise ValueError('obstacle_min_points_per_cell 必须是 1–10 的整数')
        node.declare_parameter('robot_self_mask_file', '')
        self.self_mask = None
        mask_file = str(node.get_parameter('robot_self_mask_file').value).strip()
        if mask_file:
            from .self_filter import SelfMask
            self.self_mask = SelfMask(mask_file)
            node.get_logger().info(f'已加载经转动验证的自身回波掩码：{mask_file}')
        self.health = CloudHealth(p['cloud_timeout_s'])
        self.view = None
        self.cloud_frames = 0
        self.last_points = 0
        self.last_processing_s = 0.
        self.max_processing_s = 0.
        self.input_error = ''
        self.control_pose = None
        self.pending = False
        self.blocked_since = None
        self.wait_since = None
        self.detour_started = None
        self.next_plan = 0.
        self.path = None
        self.path_index = 0
        self.goal_idx = None
        self.goal_xy = None
        self.last_candidates = []
        self.detours = 0
        self.rejoins = 0
        self.goal_cursor = 0
        self.scan_origin = None
        self.scan_step = 0
        self.scan_started = None
        self.last_heal = float('-inf')
        self.motion_rejects = 0
        self._last_cloud_log = 0.
        self._cloud_queue = deque(maxlen=4)
        node.create_subscription(PointCloud2, p['obstacle_cloud_topic'], self._on_cloud, qos_profile_sensor_data)
        # 原始点云常比 odom/TF 提前几十毫秒。短队列等待该帧的精确 TF，
        # 绝不用 latest TF 代替采样时刻，也不阻塞里程计回调。
        node.create_timer(.02, self._drain_cloud)
        node.get_logger().info(f'实时避障已启用：车身 {p["robot_length_m"]:.2f}×{p["robot_width_m"]:.2f} m，'
                               f'外接圆+余量半径 {self.radius:.2f} m，点云 {p["obstacle_cloud_topic"]}')

    def _on_cloud(self, msg):
        self._cloud_queue.append(msg)
        self._drain_cloud()

    def _drain_cloud(self):
        if not self._cloud_queue:
            return
        now = time.monotonic()
        if now - self._last_processing_started < self.processing_period:
            return
        # 跳过已积压的旧帧，优先处理设备时间不晚于已收到 odom 的最新一帧。
        odom_stamp = self.n._odom_guard.last_stamp
        if odom_stamp is not None:
            eligible = [m for m in self._cloud_queue
                        if m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 <= odom_stamp]
            if eligible:
                selected = eligible[-1]
                while self._cloud_queue[0] is not selected:
                    self._cloud_queue.popleft()
        msg = self._cloud_queue[0]
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.health.last_stamp is not None and stamp == self.health.last_stamp:
            self._cloud_queue.popleft()
            return
        if self.health.last_stamp is not None and stamp < self.health.last_stamp:
            self.health.fault = '原始点云时间戳倒退，不自动恢复运动'
            self._cloud_queue.popleft()
            return
        try:
            if msg.header.frame_id != 'lidar':
                raise ValueError('原始点云参考系必须是 lidar')
            when = self.ros_time.from_msg(msg.header.stamp)
            try:
                sensor_tf = self.n._tf_buffer.lookup_transform('odom', 'lidar', when)
                imu = self.n._tf_buffer.lookup_transform('odom', 'imu', when)
            except Exception:
                # 没有该帧精确 TF 时留待下一拍；长期缺失由 cloud_timeout 关闭运动。
                odom_stamp = self.n._odom_guard.last_stamp
                if odom_stamp is not None and odom_stamp - stamp > self.health.timeout:
                    self._cloud_queue.popleft()
                return
            self._cloud_queue.popleft()
            self._last_processing_started = now
            sensor = transform_matrix(sensor_tf)
            im = transform_matrix(imu)
            q = imu.transform.rotation
            yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
            pose = (*im[:3, 3], yaw)
            points = decode_xyz(msg)
            if self.self_mask is not None:
                points = self.self_mask.filter(points)
            count = np.count_nonzero(np.isfinite(points).all(axis=1) & (np.linalg.norm(points, axis=1) > .05))
            if count < 16:
                self.input_error = '原始点云有效回波不足 16 点'
                return
            p = self.p
            pts = filter_obstacles(points, sensor, pose,
                                   length=p['robot_length_m'], width=p['robot_width_m'],
                                   sensor_height=p['sensor_height_m'],
                                   offset_x=self.n.base_offset_x, offset_y=self.n.base_offset_y,
                                   min_height=p['obstacle_min_height_m'], max_height=p['obstacle_max_height_m'],
                                   front_extent=p['robot_front_m'])
            center = self.n._rotation_center(pose[0], pose[1], yaw)
            if not self.health.update(stamp, now, self.n._odom_guard.last_stamp):
                return
            # 3D 高回波投影的二维射线可能越过低矮箱体。首版处理静止障碍：
            # 已检测占据格锁存到移出窗口/重启，不用这类射线证明旧箱已消失。
            # 车被陈旧占据格冻住时自愈：连续 2 s 所有运动都被拒绝，且当前帧在
            # 碰撞半径内没有任何真实回波 → 判定位姿历史留下的陈旧格，按当前
            # 射线重建。有真实近距回波时绝不重建，盲区障碍不会被当成不存在。
            near = float(np.min(np.linalg.norm(pts - center, axis=1))) if len(pts) else float('inf')
            heal = (self.wait_since is not None and now - self.wait_since > 2.
                    and now - self.last_heal > 5. and near > self.radius + .05)
            if heal:
                self.last_heal = now
                self.n.get_logger().warn(
                    f'碰撞半径内无回波但运动持续被拒：按当前点云重建局部网格（最近回波 {near:.2f} m）')
            self.grid.observe(pts, sensor[:2, 3], center, now, clear_occupied=heal,
                              min_points=self.min_points)
            self.grid.mark_footprint(center, yaw, p['robot_length_m'], p['robot_width_m'], p['robot_front_m'])
            self.view = self.grid.view(center, now)
            self.cloud_frames += 1
            self.last_points = len(pts)
            self.input_error = ''
            self.last_processing_s = time.monotonic() - now
            self.max_processing_s = max(self.max_processing_s, self.last_processing_s)
        except Exception as exc:
            if self._cloud_queue and self._cloud_queue[0] is msg:
                self._cloud_queue.popleft()
            self.input_error = f'原始点云解析/TF 失败：{exc}'
            if now - self._last_cloud_log > 2.:
                self.n.get_logger().warn(self.input_error)
                self._last_cloud_log = now

    def ready_reason(self, now):
        return self.input_error or self.health.reason(now, self.n._odom_guard.last_stamp)

    def set_pose(self):
        """控制位姿用 odom<-imu；不将历史障碍随 map 重定位修正一起移动。"""
        tf = self.n._tf_buffer.lookup_transform('odom', 'imu', self.ros_time())
        m = transform_matrix(tf)
        q = tf.transform.rotation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        cx, cy = self.n._rotation_center(m[0, 3], m[1, 3], yaw)
        self.control_pose = (cx, cy, yaw)

    def guard(self, v, w):
        now = time.monotonic()
        if self.ready_reason(now) or self.view is None or self.control_pose is None:
            return False
        # 始终审查实际发布的速度，包括 return / align_start / align。
        if self.view.motion_free(self.control_pose, v, w):
            return True
        if not self.pending:
            self.n.get_logger().warn('运动预测范围遇到障碍或未知区域：停车并尝试局部绕行')
        self.pending = True
        self.blocked_since = self.blocked_since or now
        self.wait_since = self.wait_since or now
        return False

    def _target_odom(self, x, y):
        tf = self.n._tf_buffer.lookup_transform('odom', self.n.frame, self.ros_time())
        m = transform_matrix(tf)
        point = m @ np.array([x, y, 0., 1.])
        return (float(point[0]), float(point[1]))

    def _goals(self):
        n = self.n
        if n._state == 'return':
            return [(self._target_odom(n._start_cx, n._start_cy), None)]
        if n._state not in ('run', 'track_align'):
            return []  # 起点/终点原地对齐遇到障碍时保持停车，不跳过任务判据。
        cap = n.s[n._idx] + self.p['detour_rejoin_ahead_m']
        indices = []
        for i in range(n._idx + 1, len(n.s)):
            turn = (n.pyaw[i] - n.pyaw[n._idx] + math.pi) % (2*math.pi) - math.pi
            if n.s[i] > cap or abs(turn) >= math.pi/2:
                break
            if n.s[i] >= n.s[n._idx] + .5 or i == len(n.s)-1:
                indices.append(i)
        # 固定同一弧长区间，每 .4 m 一个候选，禁止按空间最近跳到回程支路。
        picked = []
        last_s = float('inf')
        for i in reversed(indices):
            if last_s - n.s[i] < .4:
                continue
            cx, cy = n._rotation_center(n.px[i], n.py[i], n.pyaw[i])
            target = self._target_odom(cx, cy)
            # 短闭环终点可能就在身边；绝不能零运动就宣布跳过一圈。
            if math.dist(self.control_pose[:2], target) <= .35:
                continue
            picked.append((target, i))
            last_s = n.s[i]
            if len(picked) == 4:
                break
        return picked

    def _observe_sides(self, now):
        """前向雷达看不到侧面通道时，限时原地观察；旋转仍须通过完整碰撞审查。"""
        if self.n._state not in ('run', 'track_align', 'return'):
            return False
        if self.scan_origin is None:
            self.scan_origin = self.control_pose[2]
            self.scan_started = now
            self.scan_step = 0
            self.n.get_logger().info('侧面可见空间不足：限速原地观察左右，再尝试规划')
        if self.scan_step >= 3 or now - self.scan_started > 20.:
            return False
        target = self.scan_origin + (.65, -.65, 0.)[self.scan_step]
        error = (target - self.control_pose[2] + math.pi) % (2*math.pi) - math.pi
        if abs(error) < .05:
            self.scan_step += 1
            self.n._publish(0., 0.)
            return True
        limit = min(.15, self.n.return_w_max if self.n._state == 'return' else self.n.w_max)
        w = max(-limit, min(limit, 1.5*error))
        if not self.view.motion_free(self.control_pose, 0., w):
            return False
        self.n._publish(0., w)
        return True

    def tick(self, now):
        """返回 True 表示本拍已接管；原始任务状态保持不变。"""
        n = self.n
        if not self.pending and self.path is None:
            return False
        if self.detour_started and now - self.detour_started > self.p['detour_timeout_s']:
            n._stop('局部绕行总超时，未完成路线')
            return True
        if self.wait_since and now - self.wait_since > self.p['obstacle_wait_timeout_s']:
            n._stop('障碍/未知区域持续阻塞，无安全可行局部路径')
            return True
        if n._state in ('return', 'align_start') and now - n._start_phase_t0 > n.return_timeout_s:
            n._fail_start(now, '绕行期间回起点/起始对齐总超时')
            return True
        if self.path is not None:
            limits = (min(n.return_v_max if n._state == 'return' else n.v_max, self.p['detour_v_max']),
                      min(n.return_w_max if n._state == 'return' else n.w_max, .4))
            v, w, self.path_index, done = follow_detour(self.path, self.path_index,
                                                       self.control_pose, *limits, self.view)
            if done:
                actual_goal = self._goals()[0][0] if self.goal_idx is None else self._target_odom(
                    *n._rotation_center(n.px[self.goal_idx], n.py[self.goal_idx], n.pyaw[self.goal_idx]))
                if math.dist(self.control_pose[:2], actual_goal) <= .12:
                    if self.goal_idx is not None:
                        n._route_progress_credit = max(n._route_progress_credit,
                                                       n.s[self.goal_idx] - n._traveled)
                        n._idx = n._limit_progress(self.goal_idx)
                        n._state = 'run'
                        n._track_align_t0 = None
                    self.rejoins += 1
                    self.path = None
                    self.pending = False
                    self.detour_started = None
                    self.goal_xy = None
                    self.goal_idx = None
                    self.goal_cursor = 0
                    self.scan_origin = None
                    self.wait_since = None
                    self.blocked_since = None
                    n.get_logger().info('局部绕行已实际到达接回点，继续原任务')
                    n._publish(0., 0.)
                    return True
                # 只到达观察边界，停住后根据新点云续规划，不提前推进原路线。
                self.path = None
                self.pending = True
                self.next_plan = now
            elif self.view.motion_free(self.control_pose, v, w):
                n._publish(v, w)
                self.motion_rejects = 0
                if v > 0:
                    self.wait_since = None
                elif self.wait_since is None:
                    self.wait_since = now
                return True
            else:
                # 反活锁：同一条候选路径连续多拍都执行不了一动（典型是路径第一拍就
                # 撞上未观测格），重规划只会得到同一条路。实车 run6 就是这样原地
                # 循环 30 次规划、舵轮不动的。此时换下一个候选，全部换完再由等待超时兜底。
                self.path = None
                self.pending = True
                self.motion_rejects += 1
                if self.motion_rejects >= 3:
                    self.motion_rejects = 0
                    self.goal_cursor += 1
                    self.goal_xy = None
                    self.goal_idx = None
                    self.next_plan = now
                    n.get_logger().warn('同一条局部路径连续执行失败，换下一个候选接回点')
                else:
                    self.next_plan = now + .3
                self.wait_since = self.wait_since or now
        n._publish(0., 0.)
        if now < self.next_plan:
            self._observe_sides(now)
            return True
        self.next_plan = now + .5
        if self.goal_xy is not None:
            # map->odom 可能被重定位微调；重规划和到位检查都使用当前变换。
            self.goal_xy = self._target_odom(*(n._rotation_center(
                n.px[self.goal_idx], n.py[self.goal_idx], n.pyaw[self.goal_idx])
                if self.goal_idx is not None else (n._start_cx, n._start_cy)))
        if self.goal_xy is not None and not self.view.free(self.goal_xy, require_known=False, planning=True):
            self.goal_xy = None
            self.goal_idx = None
            self.goal_cursor += 1
        goals = [(self.goal_xy, self.goal_idx)] if self.goal_xy is not None else self._goals()
        self.last_candidates = list(goals)
        free_goals = [g for g in goals if self.view.free(g[0], require_known=False, planning=True)]
        # 只被"未知"挡住的候选仍值得走到观测边界再观察；被已知障碍挡住的候选不能这么处理，
        # 否则车会沿着障碍横向漂移。run4 的 local_plans=0 属于前一类（箱子挡住了路线后段）。
        occluded = [g for g in goals if self.view.free(g[0], require_known=False, planning=True)
                    and not self.view.free(g[0], require_known=True, planning=True)]
        goals = free_goals
        if not goals and self.scan_step >= 3:
            # 侧向原地观察已经做完仍没有自由候选：退回未过滤候选，靠 plan(allow_partial)
            # 先走到观测边界、停车续规划（run4 的 local_plans=0 正是这里直接放弃）。
            goals = list(self.last_candidates)
        # 本拍只搜索一个候选，限制计算总时间，不挤占原始里程计安全回调。
        if goals:
            target, index = goals[self.goal_cursor % len(goals)]
            path = self.view.plan(self.control_pose[:2], target, budget_s=.035, allow_partial=True)
            if (path is not None and self.scan_step < 3
                    and math.dist(path[-1], target) > .10
                    and math.dist(self.control_pose[:2], path[-1]) < .6):
                # 只够推进不到 0.6 m 的部分路径不值得放弃侧向观察：先转起来看清楚。
                path = None
            if path is not None:
                self.path, self.path_index = path, 0
                self.goal_xy, self.goal_idx = target, index
                self.pending = False
                self.detour_started = self.detour_started or now
                self.detours += 1
                self.scan_origin = None
                n.get_logger().info(f'局部路径 {len(path)} 点，接回索引 {index}；'
                                    f'距最终接回点 {math.dist(path[-1], target):.2f} m')
                return True
            self.goal_cursor += 1
            self.goal_xy = None
            self.goal_idx = None
        self.wait_since = self.wait_since or now
        self._observe_sides(now)
        return True

    def save_snapshot(self, path):
        """导出本次 odom 网格诊断；全部为数值/字符串数组，无 pickle 对象。"""
        if self.view is None or self.control_pose is None:
            return
        occupied = np.asarray([key for key, hit in self.grid.cells.items() if hit],
                              dtype=float).reshape(-1, 2)
        occupied = (occupied + .5) * self.grid.resolution
        goal = np.asarray([] if self.goal_xy is None else [self.goal_xy],
                          dtype=float).reshape(-1, 2)
        candidates = np.asarray([point for point, _ in self.last_candidates],
                                dtype=float).reshape(-1, 2)
        candidate_indices = np.asarray([index if index is not None else -1
                                        for _, index in self.last_candidates], dtype=int)
        np.savez_compressed(
            path, known=self.view.known, blocked=self.view.blocked,
            planning_blocked=self.view.planning_blocked, origin=self.view.origin,
            resolution=self.view.resolution, pose=np.asarray(self.control_pose),
            radius=self.radius, occupied=occupied, goal_xy=goal,
            candidate_xy=candidates, candidate_indices=candidate_indices,
            goal_index=-1 if self.goal_idx is None else self.goal_idx,
            scan_step=self.scan_step,
            scan_origin=float('nan') if self.scan_origin is None else float(self.scan_origin),
            route_index=self.n._idx,
            params_json=np.asarray(json.dumps(self.p, ensure_ascii=False, sort_keys=True)))

    def summary(self):
        return dict(cloud_frames=self.cloud_frames, obstacle_points=self.last_points,
                    local_plans=self.detours, rejoins=self.rejoins,
                    cloud_max_processing_s=round(self.max_processing_s, 4))
