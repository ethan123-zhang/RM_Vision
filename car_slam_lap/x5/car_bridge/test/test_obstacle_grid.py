"""新增障碍物、未知区域、车身扫掠和绕行闭环；不依赖 ROS。"""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
try:
    from car_bridge.obstacle_grid import LocalGrid, follow_detour
except ImportError:
    LocalGrid = None
    follow_detour = None


def room_points(box=True, blocked=False):
    """四周墙 + 后加的 40 cm 方箱；单位 m。"""
    points = []
    for t in np.arange(-3, 3.01, .04):
        points.extend([(t, -2), (t, 2), (-2, t), (3, t)])
    if box:
        for t in np.arange(-.20, .201, .03):
            points.extend([(1 + t, -.2), (1 + t, .2), (.8, t), (1.2, t)])
    if blocked:
        points.extend([(.8, t) for t in np.arange(-2, 2.01, .03)])
    return np.asarray(points)


class GridTest(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(LocalGrid, "需要实现 LocalGrid 实时障碍网格")
        self.grid = LocalGrid(resolution=.10, radius=.53, half_extent=4.)

    def observe_room(self, **kwargs):
        # 来到此处时之前观察过空房间；新增箱子必须覆盖原有空闲观测。
        self.grid.observe(room_points(box=False), (0., 0.), (0., 0.), .5)
        self.grid.observe(room_points(**kwargs), (0., 0.), (0., 0.), 1.)

    def test_clear_route_stays_straight(self):
        self.observe_room(box=False)
        view = self.grid.view((0., 0.), 1.)
        self.assertTrue(view.segment_free((0., 0.), (2., 0.)))

    def test_isolated_noise_rejected_but_supported_obstacle_stops_motion(self):
        self.grid.observe(np.array([[1., 0.], [1.01, 0.]]), (0., 0.), (0., 0.), 1., min_points=3)
        self.assertTrue(self.grid.view((0., 0.), 1.).motion_free((.4, 0., 0.), .1, 0.))
        self.grid.observe(np.array([[1., 0.], [1.01, 0.], [1.02, 0.]]), (0., 0.), (0., 0.), 2., min_points=3)
        self.assertFalse(self.grid.view((0., 0.), 2.).motion_free((.4, 0., 0.), .1, 0.))

    def test_single_supported_hit_remains_protected_after_side_observation(self):
        g = LocalGrid()
        g.observe(np.array([[1., 0.]]), (0., 0.), (0., 0.), 1., clear_occupied=False)
        self.assertFalse(g.view((0., 0.), 1.).motion_free((.4, 0., 0.), .1, 0.))
        g.observe(np.array([[-2., 1.]]), (0., 0.), (0., 0.), 2., clear_occupied=False)
        self.assertFalse(g.view((0., 0.), 2.).motion_free((.4, 0., 0.), .1, 0.))

    def test_new_box_blocks_motion_and_has_detour(self):
        self.observe_room()
        view = self.grid.view((0., 0.), 1.)
        self.assertFalse(view.segment_free((0., 0.), (2., 0.)))
        path = view.plan((0., 0.), (2., 0.), budget_s=1.)
        self.assertIsNotNone(path)
        self.assertGreater(max(abs(p[1]) for p in path), .7)
        for a, b in zip(path, path[1:]):
            self.assertTrue(view.segment_free(a, b))

    def test_can_leave_extra_planning_margin_without_entering_collision_area(self):
        self.observe_room()
        view = self.grid.view((0., 0.), 1.)
        start = (.15, -.10)
        self.assertTrue(view.free(start, require_known=False))
        self.assertFalse(view.free(start, planning=True))
        path = view.plan(start, (2., 0.), budget_s=1.)
        self.assertIsNotNone(path, '仅在额外规划余量内时应允许安全地向外脱离')
        for a, b in zip(path, path[1:]):
            self.assertTrue(view.segment_free(a, b))

    def test_rectangle_body_passes_123cm_gap_that_circle_cannot(self):
        """run4 场地实测通道 1.23 m：车体矩形过得去，外接圆过不去。

        这也是改几何的全部理由：圆要 0.526 净空 + 约一格跟踪余量 ≈1.27 m。
        """
        pts = [(x, 1.38) for x in np.arange(-2, 2.51, .04)]
        pts += [(x, -1.12) for x in np.arange(-2, 2.51, .04)]
        for x in np.arange(.75, 1.251, .04):
            pts += [(x, -.45), (x, .15)]
        for y in np.arange(-.45, .151, .04):
            pts += [(.75, y), (1.25, y)]
        pts = np.asarray(pts)
        body = (.65, .55, .325, .10)
        rectangle = LocalGrid(.10, .53, 4., body=body)
        circle = LocalGrid(.10, .53, 4.)
        for grid in (rectangle, circle):
            grid.observe(pts, (0., 0.), (0., 0.), 1.)
        start, goal = (-.26, .03), (2.2, .75)
        view = rectangle.view(start, 1.)
        path = view.plan(start, goal, budget_s=1., allow_partial=True)
        self.assertIsNotNone(path, '车体矩形必须能通过 1.23 m 通道')
        self.assertGreater(math.dist(path[-1], start), .8, '必须实打实穿过这段 1.23 m 通道')
        for point in path[1:]:
            self.assertTrue(view.free(point, planning=True), '路径中心必须在已观测的空闲格内')
        circle_path = circle.view(start, 1.).plan(start, goal, budget_s=1., allow_partial=True)
        advance = 0. if circle_path is None else math.dist(circle_path[-1], start)
        self.assertLess(advance, .7,
                        '外接圆模型只能挤进 0.7 m 就地卡死——这就是换矩形的原因，不是随便改阈值')

    def test_blocked_passage_has_no_path(self):
        self.observe_room(blocked=True)
        self.assertIsNone(self.grid.view((0., 0.), 1.).plan((0., 0.), (2., 0.), budget_s=1.))

    def test_unknown_is_not_a_shortcut(self):
        self.grid.observe(np.array([[1., 0.]]), (0., 0.), (0., 0.), 1.)
        self.assertIsNone(self.grid.view((0., 0.), 1.).plan((0., 0.), (0., 2.), budget_s=1.))

    def test_occluded_goal_uses_only_observed_partial_path(self):
        self.grid.observe(room_points(), (0., 0.), (0., 0.), 1.)
        view = self.grid.view((0., 0.), 1.)
        self.assertIsNone(view.plan((0., 0.), (2., 0.), budget_s=1.))
        path = view.plan((0., 0.), (2., 0.), budget_s=1., allow_partial=True)
        self.assertIsNotNone(path)
        self.assertGreater(math.dist(path[0], path[-1]), .3)
        self.assertGreater(math.dist(path[-1], (2., 0.)), .1)
        self.assertLess(math.dist(path[-1], (2., 0.)), 1.5)
        for a, b in zip(path, path[1:]):
            self.assertTrue(view.segment_free(a, b))

    def test_obstacle_out_of_view_is_retained(self):
        self.observe_room()
        self.grid.observe(np.array([[-2., 0.]]), (0., 0.), (0., 0.), 2.)
        self.assertFalse(self.grid.view((0., 0.), 2.).segment_free((0., 0.), (2., 0.)))

    def test_observed_empty_ray_clears_removed_obstacle(self):
        self.observe_room()
        self.grid.observe(room_points(box=False), (0., 0.), (0., 0.), 2.)
        self.assertTrue(self.grid.view((0., 0.), 2.).segment_free((0., 0.), (2., 0.)))

    def test_rotation_checks_full_body_radius(self):
        self.grid.observe(np.array([[0., .40]]), (0., 0.), (0., 0.), 1.)
        self.assertFalse(self.grid.view((0., 0.), 1.).motion_free((0., 0., 0.), 0., .3))

    def test_arc_is_checked_not_only_endpoint(self):
        self.observe_room()
        view = self.grid.view((0., 0.), 1.)
        self.assertFalse(view.motion_free((.2, 0., 0.), .3, .01, horizon_s=3.))

    def test_one_side_wall_forces_other_side(self):
        self.observe_room()
        wall = np.array([(x, .85) for x in np.arange(-1, 2.6, .03)])
        self.grid.observe(wall, (0., 0.), (0., 0.), 1.)
        path = self.grid.view((0., 0.), 1.).plan((0., 0.), (2., 0.), budget_s=1.)
        self.assertIsNotNone(path)
        self.assertLess(min(p[1] for p in path), -.7)

    def test_detour_controller_reaches_goal_without_cutting_box(self):
        self.observe_room()
        view = self.grid.view((0., 0.), 1.)
        path = view.plan((0., 0.), (2., 0.), budget_s=1.)
        pose = (0., 0., 0.)
        index = 0
        for _ in range(2500):
            v, w, index, done = follow_detour(path, index, pose, .12, .35, view)
            if done:
                break
            self.assertTrue(view.motion_free(pose, v, w), f"危险指令 {pose, v, w}")
            x, y, yaw = pose
            pose = (x + v * math.cos(yaw) * .05, y + v * math.sin(yaw) * .05, yaw + w * .05)
        else:
            self.fail("绕行控制器未在预算内到达接回点")
        self.assertLess(math.hypot(pose[0] - 2, pose[1]), .13)


if __name__ == "__main__":
    unittest.main()
