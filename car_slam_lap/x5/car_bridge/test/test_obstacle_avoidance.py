"""点云过滤、新鲜度与跟踪器实际速度发布集成测试。"""
import json
import math
import struct
import sys
import types
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import test_follow_safety as safety
from test_obstacle_grid import room_points

try:
    from car_bridge.obstacle_avoidance import CloudHealth, filter_obstacles, decode_xyz
except ImportError:
    CloudHealth = None
    filter_obstacles = None
    decode_xyz = None


class PerceptionTest(unittest.TestCase):
    def test_mixed_dtype_and_row_padding_are_decoded(self):
        self.assertIsNotNone(decode_xyz)
        fields = [types.SimpleNamespace(name=name, offset=off, datatype=7, count=1)
                  for name, off in [('x', 0), ('y', 4), ('z', 8)]]
        fields.append(types.SimpleNamespace(name='confidence', offset=13, datatype=4, count=1))
        data = struct.pack('<fffBHf', 1., 2., 3., 100, 35, .01) + b'12345'
        data += struct.pack('<fffBHf', 4., 5., 6., 100, 35, .02) + b'12345'
        msg = types.SimpleNamespace(fields=fields, point_step=19, row_step=24,
                                    width=1, height=2, is_bigendian=False, data=data)
        np.testing.assert_equal(decode_xyz(msg), [[1., 2., 3.], [4., 5., 6.]])

    def test_repeated_cloud_does_not_keep_motion_alive(self):
        self.assertIsNotNone(CloudHealth, "需要原始点云新鲜度安全门")
        h = CloudHealth(.6)
        self.assertTrue(h.update(10., 1., 10.1))
        self.assertFalse(h.update(10., 1.5, 10.2))
        self.assertIsNotNone(h.reason(1.7, 10.3))

    def test_backward_stamp_latches_fault(self):
        self.assertIsNotNone(CloudHealth)
        h = CloudHealth(.6)
        h.update(10., 1., 10.)
        self.assertFalse(h.update(9., 1.1, 10.))
        self.assertFalse(h.update(11., 1.2, 11.))
        self.assertIsNotNone(h.reason(1.2, 11.))

    def test_queued_cloud_with_old_device_stamp_rejected(self):
        self.assertIsNotNone(CloudHealth)
        h = CloudHealth(.6)
        self.assertFalse(h.update(10., 1., 12.))
        self.assertIsNotNone(h.reason(1.1, 12.))

    def test_floor_body_and_invalid_returns_filtered(self):
        self.assertIsNotNone(filter_obstacles)
        points = np.array([[1., 0., -.23], [1., 0., -.10], [.05, 0., 0.],
                           [0., 0., 0.], [math.nan, 0., 0.], [1., 0., 1.5]])
        result = filter_obstacles(points, np.eye(4), (0., 0., 0., 0.),
                                  length=.65, width=.55, sensor_height=.23,
                                  offset_x=0., offset_y=0., min_height=.06, max_height=.8)
        np.testing.assert_allclose(result, [[1., 0.]])

    def test_body_filter_uses_rotation_center_offset(self):
        self.assertIsNotNone(filter_obstacles)
        # imu 位于转心前 26 cm；x=-0.50 仍在长 65 cm 的车体中。
        result = filter_obstacles(np.array([[-.50, 0., 0.], [1., 0., 0.]]),
                                  np.eye(4), (0., 0., 0., 0.),
                                  length=.65, width=.55, sensor_height=.23,
                                  offset_x=.26, offset_y=0., min_height=.06, max_height=.8)
        np.testing.assert_allclose(result, [[1., 0.]])

    def test_asymmetric_body_uses_front_extent_from_rotation_center(self):
        self.assertIsNotNone(filter_obstacles)
        result = filter_obstacles(np.array([[.15, 0., 0.], [.6, 0., 0.]]),
                                  np.eye(4), (0., 0., 0., 0.),
                                  length=.65, width=.55, sensor_height=.23,
                                  offset_x=.2626, offset_y=0., min_height=.06, max_height=.8,
                                  front_extent=.50)
        np.testing.assert_allclose(result, [[.6, 0.]])


class IntegrationTest(unittest.TestCase):
    def setUp(self):
        self.h = safety.FollowPathSafetyTest()
        self.h.setUp()
        self.addCleanup(self.h.doCleanups)
        sensor = types.ModuleType('sensor_msgs.msg')
        sensor.PointCloud2 = type('PointCloud2', (), {})
        pc = types.SimpleNamespace(read_points_numpy=lambda msg, **kwargs: msg.points)
        self.addCleanup(mock.patch.stopall)
        mock.patch.dict(sys.modules, {'rclpy': safety._ros,
                                     'rclpy.qos': safety._ros.qos,
                                     'sensor_msgs': types.ModuleType('sensor_msgs'),
                                     'sensor_msgs.msg': sensor,
                                     'sensor_msgs_py': types.SimpleNamespace(point_cloud2=pc)}).start()
        mock.patch.object(safety._ros.time.Time, 'from_msg', classmethod(lambda cls, msg: cls()), create=True).start()
        # 字节布局在上面的真实解析用例覆盖；这里用点阵专注测真实 tick 和发布行为。
        mock.patch('car_bridge.obstacle_avoidance.decode_xyz', lambda msg: msg.points).start()

    def node(self, **kwargs):
        return self.h._node([(0., 0., 0.), (2., 0., 0.), (3., 0., 0.)],
                            obstacle_avoidance=True, **kwargs)

    def cloud(self, node, stamp, points=None):
        if points is None:
            points = [[x, y, 0.] for x, y in room_points(box=False)]
        sec = int(stamp)
        node._obstacles._on_cloud(types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=sec, nanosec=int((stamp-sec)*1e9)), frame_id='lidar'),
            points=np.array(points)))

    def test_enabled_mode_requires_cloud_before_moving(self):
        n = self.node()
        self.h._tick_pose(n, 10., (0., 0., 0.))
        self.assertTrue(n._done, "未接收到原始点云必须失败停车")
        self.h._assert_all_zero(n)

    def test_save_actual_exports_pickle_free_obstacle_snapshot(self):
        path = self.h._temp_path('.yaml')
        snapshot = Path(path + '.obstacles.npz')
        self.h._created.append(snapshot)
        n = self.node(save_actual=path)
        self.h._feed(n, 10.)
        self.cloud(n, 10.)
        n._tick()
        self.assertTrue(n._actual)
        a = n._obstacles
        n.save_actual()
        self.assertTrue(Path(path).is_file())
        self.assertTrue(snapshot.is_file())
        with np.load(snapshot, allow_pickle=False) as saved:
            self.assertEqual(int(saved['route_index']), n._idx)
            np.testing.assert_array_equal(saved['known'], a.view.known)
            np.testing.assert_array_equal(saved['blocked'], a.view.blocked)
            np.testing.assert_array_equal(saved['planning_blocked'], a.view.planning_blocked)
            np.testing.assert_array_equal(saved['origin'], a.view.origin)
            np.testing.assert_allclose(saved['pose'], a.control_pose)
            self.assertAlmostEqual(float(saved['resolution']), a.view.resolution)
            self.assertAlmostEqual(float(saved['radius']), a.radius)
            expected = [(np.asarray(key) + .5) * a.grid.resolution
                        for key, hit in a.grid.cells.items() if hit]
            np.testing.assert_allclose(saved['occupied'], expected)
            self.assertTrue(saved['blocked'].any())
            self.assertEqual(saved['goal_xy'].shape, (0, 2))
            self.assertEqual(saved['candidate_xy'].shape, (0, 2))
            self.assertEqual(saved['candidate_indices'].shape, (0,))
            self.assertEqual(saved['candidate_indices'].dtype.kind, 'i')
            self.assertEqual(int(saved['goal_index']), -1)
            self.assertEqual(saved['goal_index'].dtype.kind, 'i')
            self.assertEqual(int(saved['scan_step']), a.scan_step)
            self.assertEqual(saved['scan_step'].dtype.kind, 'i')
            self.assertTrue(np.isnan(saved['scan_origin']))
            self.assertEqual(saved['scan_origin'].dtype.kind, 'f')
            self.assertEqual(json.loads(str(saved['params_json'])), a.p)
            for key in saved.files:
                self.assertFalse(saved[key].dtype.hasobject, key)

    def test_save_actual_without_obstacle_view_still_writes_yaml(self):
        path = self.h._temp_path('.yaml')
        snapshot = Path(path + '.obstacles.npz')
        self.h._created.append(snapshot)
        n = self.node(save_actual=path)
        n._record_actual(0., 0., 0., self.h.clock())
        self.assertIsNone(n._obstacles.view)
        n.save_actual()
        self.assertIn('points:', Path(path).read_text(encoding='utf-8'))
        self.assertFalse(snapshot.exists())

    def test_snapshot_export_failure_preserves_actual_yaml(self):
        path = self.h._temp_path('.yaml')
        n = self.node(save_actual=path)
        self.h._feed(n, 10.)
        self.cloud(n, 10.)
        n._tick()
        with mock.patch.object(n._obstacles, 'save_snapshot', side_effect=OSError('disk full')):
            n.save_actual()
        self.assertIn('points:', Path(path).read_text(encoding='utf-8'))
        self.assertTrue(any('disk full' in message for message in n.get_logger().warns))

    def test_cloud_loss_protects_return_to_start(self):
        n = self.node(return_to_start=True)
        self.h._feed(n, 10.)
        self.cloud(n, 10.)
        self.assertIsNotNone(n._obstacles.health.last_rx)
        self.h.clock.advance(.7)
        self.h._tick_pose(n, 10.7, (.3, 0., math.pi))
        self.assertTrue(n._done)
        self.h._assert_all_zero(n)

    def test_obstacle_intercepts_forward_command(self):
        n = self.node()
        self.h._feed(n, 10.)
        self.cloud(n, 10., [[.5, 0., 0.]] * 40)
        self.h._tick_pose(n, 10.1, (0., 0., 0.))
        self.h._assert_all_zero(n)
        self.assertEqual(n._idx, 0)

    def test_empty_invalid_cloud_cannot_enable_motion(self):
        n = self.node()
        self.h._feed(n, 10.)
        self.cloud(n, 10., [[0., 0., 0.]] * 40)
        self.h._tick_pose(n, 10.1, (0., 0., 0.))
        self.assertTrue(n._done)
        self.h._assert_all_zero(n)

    def _simulate_navigation(self, box=True, start_pose=(0., 0., 0.), return_to_start=False, sensor_tuning=False):
        route = safety._interpolate_route([(0., 0.), (2., 0.)], step=.04)
        params = dict(obstacle_min_points_per_cell=8,
                      cloud_processing_period_s=.20) if sensor_tuning else {}
        n = self.h._node(route, obstacle_avoidance=True, v_max=.15, w_max=.35,
                         detour_timeout_s=120., return_to_start=return_to_start,
                         return_timeout_s=60., **params)
        lookup = n._tf_buffer.lookup_transform
        def lookup_frames(target, source, when):
            if target == 'odom' and source == 'map':
                old = n._tf_buffer.pose
                n._tf_buffer.pose = (0., 0., 0.)
                result = lookup(target, source, when)
                n._tf_buffer.pose = old
                return result
            return lookup(target, source, when)
        n._tf_buffer.lookup_transform = lookup_frames
        pose = start_pose
        world = room_points(box=box)
        if sensor_tuning:
            # 模拟多条高度扫描线投影到同一平面格，不用稀疏单层夹具替代真实密集点云。
            world_z = np.tile([-.05, 0., .05, .10], len(world))
            world = np.repeat(world, 4, axis=0)
        else:
            world_z = np.zeros(len(world))
        max_lateral = 0.
        for k in range(2000):
            stamp = 10 + k * .1
            x, y, yaw = pose
            n._tf_buffer.pose = pose
            self.h._feed(n, stamp, *pose)
            delta = world - [x, y]
            c, s = math.cos(yaw), math.sin(yaw)
            raw = np.column_stack((c * delta[:, 0] + s * delta[:, 1],
                                   -s * delta[:, 0] + c * delta[:, 1], world_z))
            self.cloud(n, stamp, raw)
            before = n._idx
            had_path = n._obstacles.path is not None
            n._tick()
            if had_path and n._obstacles.path is not None:
                self.assertEqual(n._idx, before, '绕行期间不得偷推进原路线')
            if n._done:
                break
            v, w = self.h._commands(n)[-1]
            self.assertGreaterEqual(v, 0.)
            if abs(w) > 1e-8:
                pose = (x + v/w*(math.sin(yaw+w*.1)-math.sin(yaw)),
                        y - v/w*(math.cos(yaw+w*.1)-math.cos(yaw)), yaw+w*.1)
            else:
                pose = (x+v*math.cos(yaw)*.1, y+v*math.sin(yaw)*.1, yaw)
            max_lateral = max(max_lateral, abs(pose[1]))
            self.h.clock.advance(.1)
        self.assertTrue(n._done, f'导航未结束 pose={pose}, state={n._state}, idx={n._idx}, logs={n.get_logger().infos[-6:]}')
        self.assertEqual(n._outcome, 'success', n._reason)
        return n, pose, max_lateral

    def test_real_tick_detours_and_finishes_without_skipping_route(self):
        n, pose, max_lateral = self._simulate_navigation()
        self.assertGreater(max_lateral, .7)
        self.assertGreaterEqual(n._obstacles.rejoins, 1)
        self.assertLess(math.hypot(pose[0]-2., pose[1]), .21)

    def test_obstacle_mode_preserves_return_alignment_and_clear_route(self):
        n, pose, _ = self._simulate_navigation(box=False, start_pose=(.3, 0., math.pi), return_to_start=True)
        self.assertGreater(n._start_elapsed, 1.)
        self.assertEqual(n._obstacles.rejoins, 0)
        self.assertLess(math.hypot(pose[0]-2., pose[1]), .21)

    def test_hardware_sensor_tuning_still_detours_and_finishes(self):
        n, pose, lateral = self._simulate_navigation(sensor_tuning=True)
        self.assertGreater(lateral, .7)
        self.assertGreaterEqual(n._obstacles.rejoins, 1)

    def test_rejoin_completion_drops_old_goal_for_next_obstacle(self):
        n = self.node()
        self.h._feed(n, 10.)
        self.cloud(n, 10., [[2., t, 0.] for t in np.arange(-2, 2, .05)])
        a = n._obstacles
        a.set_pose()
        a.path = [(0., 0.)]
        a.goal_xy = (0., 0.)
        a.goal_idx = 0
        n._start_checked = True
        a.tick(self.h.clock())
        self.assertIsNone(a.goal_xy, '下一个障碍必须重新选前方接回点')

    def test_raw_cloud_tf_failure_cannot_publish_motion(self):
        n = self.node()
        self.h._feed(n, 10.)
        n._tf_buffer.pose = None
        self.cloud(n, 10., [[2., t, 0.] for t in np.arange(-2, 2, .05)])
        self.h._tick_pose(n, 10.1, (0., 0., 0.))
        self.assertTrue(n._done)
        self.h._assert_all_zero(n)

    def test_cloud_waits_for_its_timestamp_tf_without_latest_fallback(self):
        n = self.node()
        self.h._feed(n, 10.)
        cloud = [[2., t, 0.] for t in np.arange(-2, 2, .05)]
        self.cloud(n, 10., cloud)
        lookup = n._tf_buffer.lookup_transform
        n._tf_buffer.lookup_transform = mock.Mock(side_effect=RuntimeError('future TF'))
        self.h._feed(n, 10.1)
        self.cloud(n, 10.1, cloud)
        self.assertEqual(n._obstacles.health.last_stamp, 10.)
        self.assertIsNone(n._obstacles.ready_reason(self.h.clock()))
        n._tf_buffer.lookup_transform = lookup
        n._obstacles._drain_cloud()
        self.assertAlmostEqual(n._obstacles.health.last_stamp, 10.1)

    def test_processing_throttle_uses_latest_frame_without_queue_backlog(self):
        n = self.node(cloud_processing_period_s=.2)
        self.h._feed(n, 10.)
        self.cloud(n, 10.)
        self.h._feed(n, 10.1)
        self.cloud(n, 10.1)
        self.assertEqual(n._obstacles.cloud_frames, 1)
        self.h.clock.advance(.21)
        self.h._feed(n, 10.2)
        self.cloud(n, 10.2)
        self.assertEqual(n._obstacles.cloud_frames, 2)
        self.assertAlmostEqual(n._obstacles.health.last_stamp, 10.2)

    def test_unknown_side_space_is_observed_by_bounded_stationary_scan(self):
        n = self.node()
        self.h._feed(n, 10.)
        self.cloud(n, 10., [[.9, t, 0.] for t in np.arange(-2, 2.01, .02)])
        a = n._obstacles
        a.set_pose()
        a.pending = True
        a.tick(self.h.clock())
        v, w = self.h._commands(n)[-1]
        self.assertEqual(v, 0.)
        self.assertGreater(abs(w), 0., '视野不足时应先安全原地观察侧面，而不是盲目前进')
        self.assertLessEqual(abs(w), .15)
        self.assertEqual(n._idx, 0)

    def test_fully_blocked_route_fails_instead_of_claiming_success(self):
        n = self.node(obstacle_wait_timeout_s=1.)
        wall = [[.8, t, 0.] for t in np.arange(-4, 4, .03)]
        for k in range(30):
            self.h._feed(n, 10+k*.1)
            self.cloud(n, 10+k*.1, wall)
            n._tick()
            if n._done:
                break
            self.h.clock.advance(.1)
        self.assertTrue(n._done)
        self.assertEqual(n._outcome, 'failed')
        self.assertEqual(n._idx, 0)
        self.h._assert_tail_zero(n)

    def test_scan_completes_left_right_origin_then_stays_stopped(self):
        n = self.node()
        a = n._obstacles
        origin = .4
        # 保持在重规划间隔内，隔离真实扫描控制与碰撞审查。
        a.pending = True
        a.next_plan = self.h.clock() + 100.
        for k, (yaw, direction) in enumerate([
                (origin, 1), (origin + .65, 0),
                (origin + .65, -1), (origin - .65, 0),
                (origin - .65, 1), (origin, 0), (origin, 0)]):
            with self.subTest(step=k):
                self.h._feed(n, 10. + k * .1, yaw=yaw)
                n._tf_buffer.pose = (0., 0., yaw)
                self.cloud(n, 10. + k * .1)
                a.set_pose()
                a.tick(self.h.clock())
                v, w = self.h._commands(n)[-1]
                self.assertEqual(v, 0.)
                if direction:
                    self.assertGreater(w * direction, 0.)
                    self.assertLessEqual(abs(w), .15)
                else:
                    self.assertEqual(w, 0.)
                self.assertFalse(n._done)
                self.assertEqual(n._idx, 0)
                self.h.clock.advance(.1)
        self.assertEqual(a.scan_step, 3)
        self.assertTrue(all(v == 0. for v, _ in self.h._commands(n)))

    def test_scan_time_budget_stops_rotation_before_obstacle_wait_timeout(self):
        n = self.node(obstacle_wait_timeout_s=35.)
        a = n._obstacles
        a.pending = True
        a.next_plan = self.h.clock() + 100.
        started = self.h.clock()
        a.wait_since = started
        for elapsed in (0., 20.1, 25., 34., 35.1):
            self.h.clock.advance(started + elapsed - self.h.clock())
            self.h._feed(n, 10. + elapsed)
            self.cloud(n, 10. + elapsed)
            a.set_pose()
            a.tick(self.h.clock())
            v, w = self.h._commands(n)[-1]
            self.assertEqual(v, 0.)
            if elapsed == 0.:
                self.assertGreater(w, 0.)
            else:
                self.assertEqual(w, 0.)
            self.assertEqual(n._done, elapsed > 35.)
        self.assertEqual(n._outcome, 'failed')
        self.h._assert_tail_zero(n)

    def test_body_rectangle_judges_rotation_near_obstacle(self):
        n = self.node()
        self.h._feed(n, 10.)
        # 点在车体过滤框外、外接碰撞圆内：小角度扫掠矩形没碰到车身时允许做观察性转动，
        # 但大角度转向（>90°，可能扫满一圈）仍必须按外接圆拒绝。
        self.cloud(n, 10., [[.45, .35, 0.]] * 40)
        a = n._obstacles
        a.set_pose()
        self.assertFalse(a.view.free(a.control_pose, require_known=False))
        self.assertTrue(a.view.motion_free(a.control_pose, 0., .15),
                        '小车距 0.5 m 以上、真实车身没碰到时应允许限速观察转动')
        self.assertFalse(a.view.motion_free(a.control_pose, 0., 1.),
                         '大角度转向可能扫满一圈，仍必须按外接圆拒绝')

    def test_real_tick_stops_started_scan_when_cloud_stales_despite_fresh_odom(self):
        n = self.node()
        self.h._feed(n, 10.)
        self.cloud(n, 10., [[.9, t, 0.] for t in np.arange(-2, 2.01, .02)])
        n._obstacles.pending = True
        n._tick()
        v, w = self.h._commands(n)[-1]
        self.assertEqual(v, 0.)
        self.assertGreater(abs(w), 0., '必须先确认真实 tick 已开始扫描')
        self.assertIsNotNone(n._obstacles.scan_started)
        self.assertFalse(n._done)
        before = len(self.h._commands(n))
        self.h.clock.advance(.7)
        self.h._feed(n, 10.7)
        n._tick()
        self.assertTrue(n._done)
        self.assertEqual(n._outcome, 'failed')
        after = self.h._commands(n)[before:]
        self.assertTrue(after)
        self.assertTrue(all(command == (0., 0.) for command in after))
        self.assertEqual(n._idx, 0)

    def test_blocked_farthest_rejoin_uses_nearer_candidate(self):
        route = safety._interpolate_route([(0., 0.), (3., 0.)], step=.04)
        n = self.h._node(route, obstacle_avoidance=True)
        self.h._feed(n, 10.)
        world = room_points(box=False)
        self.cloud(n, 10., [[x, y, 0.] for x, y in world])
        self.h._feed(n, 10.1)
        self.cloud(n, 10.1, [[2., t, 0.] for t in np.arange(-.2, .21, .01)])
        a = n._obstacles
        a.set_pose()
        a.pending = True
        a.tick(self.h.clock())
        self.assertIsNotNone(a.path)
        self.assertTrue(a.view.free(a.goal_xy, require_known=False, planning=True),
                        '不能永远锁定落在障碍中的最远接回点')

    def test_high_wall_ray_does_not_erase_previously_seen_low_box(self):
        n = self.node()
        self.h._feed(n, 10.)
        self.cloud(n, 10., [[1., 0., -.08]] * 40)
        self.h._feed(n, 10.1)
        self.cloud(n, 10.1, [[3., 0., .47]] * 40)
        # 探针从 0.4 m 移到 0.7 m：矩形车队里 0.4 m 处本来就该能走（车头 0.725 < 箱 1.0），
        # 旧断言只是靠外接圆 0.667 顺带成立；0.7 m 处车头越过箱面，必须仍然拦停。
        self.assertFalse(n._obstacles.view.motion_free((.7, 0., 0.), .1, 0.),
                         '越过低箱顶部的高墙射线不能证明箱子已消失')

    def test_real_rejoin_advances_progress_without_faking_actual_distance(self):
        route = safety._interpolate_route([(0., 0.), (.5, .25), (1., 0.), (1.5, .25), (2., 0.)], step=.04)
        n = self.h._node(route, obstacle_avoidance=True)
        self.h._feed(n, 10.)
        self.cloud(n, 10., [[x, y, 0.] for x, y in room_points(box=False)])
        n._tf_buffer.pose = (1., 0., 0.)
        a = n._obstacles
        a.set_pose()
        a._target_odom = lambda x, y: (x, y)  # 本例仅隔离恒等 map->odom 外部接口。
        a.goal_idx = 28
        a.goal_xy = (1., 0.)
        a.path = [(1., 0.)]
        n._traveled = 1.
        a.tick(self.h.clock())
        self.assertEqual(n._idx, 28)
        self.assertEqual(n._traveled, 1., '不能用路线弧长伪造实际里程')
        self.assertEqual(n._limit_progress(29), 29, '接回后不能继续被旧里程上限卡住')
        n._reset_route_tracking(0., 0.)
        self.assertEqual(n._route_progress_credit, 0.)

    def test_rejoin_window_stops_before_uturn_and_rejects_zero_length_loop(self):
        n = self.h._node(safety._uturn_route(), obstacle_avoidance=True)
        n._obstacles.control_pose = (0., 0., 0.)
        goals = n._obstacles._goals()
        self.assertTrue(goals)
        for point, index in goals:
            self.assertLess(abs(n.pyaw[index]), math.pi/2)
            self.assertGreater(math.dist((0., 0.), point), .35)

    def test_locked_free_but_unreachable_goal_is_released(self):
        n = self.node()
        self.h._feed(n, 10.)
        self.cloud(n, 10.)
        self.h._feed(n, 10.1)
        self.cloud(n, 10.1, [[.8, t, 0.] for t in np.arange(-4, 4.01, .02)])
        a = n._obstacles
        a.set_pose()
        self.assertTrue(a.view.free((2., 0.), planning=True))
        a.goal_xy, a.goal_idx, a.pending = (2., 0.), 1, True
        a.tick(self.h.clock())
        self.assertIsNone(a.path)
        self.assertIsNone(a.goal_xy, '无连通路径时必须释放锁定，下拍才能尝试其它目标')


if __name__ == '__main__':
    unittest.main()
