"""定位 run4 卡死时的占据格来源：自身回波累积 vs 真实障碍。"""
import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).parent
RES = 0.1


def load_body_geom():
    z = np.load(HERE / "sources_before_clear.npz", allow_pickle=False)
    return z


def main():
    run = np.load(HERE / "live_run4_actual.yaml.obstacles.npz", allow_pickle=False)
    pose = run["pose"]
    occ_run = run["occupied"]          # 米制格中心（累积 224 帧）
    src = load_body_geom()
    center = src["center_pose"]
    print("run4 pose      =", np.round(pose, 3))
    print("capture center =", np.round(center, 3), " (同一帧控制位姿)")
    print("累积占据格数 =", len(occ_run))

    d = np.linalg.norm(occ_run - pose[:2], axis=1)
    for r in (0.30, 0.53, 0.67, 1.00, 1.50, 2.00):
        print(f"  距位姿 <= {r:.2f} m 的占据格: {int((d <= r).sum())}")
    near = occ_run[d <= 1.2]
    print("1.2 m 内占据格 bbox: x[%.2f,%.2f] y[%.2f,%.2f]"
          % (near[:, 0].min(), near[:, 0].max(), near[:, 1].min(), near[:, 1].max()))

    # 单帧重放得到的占据（只含这一帧看到的障碍）
    params = json.loads(str(src["params_json"]))
    radius = math.hypot(max(0.325, 0.65 - 0.325), 0.55 / 2) + params["obstacle_margin_m"]
    sys_path = r"E:\RM\car_slam_lap\x5\car_bridge"
    import sys
    sys.path.insert(0, sys_path)
    from car_bridge.obstacle_grid import LocalGrid

    grid = LocalGrid(RES, radius, 4.0)
    grid.observe(src["obstacles_xy"], src["odom_lidar"][:2, 3], center[:2], 0.0,
                 clear_occupied=False, min_points=params["min_points_per_cell"])
    occ_frame = np.asarray([((k[0] + .5) * RES, (k[1] + .5) * RES)
                            for k, hit in grid.cells.items() if hit])
    print("单帧占据格数 =", len(occ_frame))
    df = np.linalg.norm(occ_frame - pose[:2], axis=1) if len(occ_frame) else np.array([9.])
    print("  单帧最近占据格 = %.2f m" % df.min())

    # 累积里那些单帧看不到、又靠近车体的格子（= 历史锁存）
    set_run = {(round(p[0], 2), round(p[1], 2)) for p in occ_run}
    set_frame = {(round(p[0], 2), round(p[1], 2)) for p in occ_frame}
    latched_only = np.asarray(sorted(set_run - set_frame))
    print("只在累积里出现（已锁存、本帧看不到）的占据格 =", len(latched_only))
    if len(latched_only):
        dlat = np.linalg.norm(latched_only - pose[:2], axis=1)
        print("  其中距位姿 <= 0.67 m（足以阻挡起点） =", int((dlat <= 0.67).sum()))
        print("  距位姿 <= 0.67 m 的锁存格坐标（前 20 个）:")
        sel = latched_only[dlat <= 0.67][:20]
        imu = src["imu_pose"]
        print("    imu_pose(x,y,z,yaw) =", np.round(imu, 4))
        for p in sel:
            print("      (%.2f, %.2f)  距转心 %.2f m" % (p[0], p[1], math.dist(p, pose[:2])))
        # 这些点相对车体的位置
        if len(sel):
            c, s = math.cos(imu[3]), math.sin(imu[3])
            dx, dy = sel[:, 0] - imu[0], sel[:, 1] - imu[1]
            bx = c * dx + s * dy
            by = -s * dx + c * dy
            print("  相对 imu 车体系 (bx 前+, by 左+):")
            for p in np.stack([bx, by], axis=1):
                print("      (%.2f, %.2f)" % (p[0], p[1]))

    # 车体是否在占据里：车身矩形内的占据格
    imu = src["imu_pose"]
    c, s = math.cos(imu[3]), math.sin(imu[3])
    all_occ = occ_run
    dx, dy = all_occ[:, 0] - imu[0], all_occ[:, 1] - imu[1]
    bx = c * dx + s * dy
    by = -s * dx + c * dy
    inside = (np.abs(bx) <= 0.35) & (np.abs(by) <= 0.30)
    print("落在车身矩形(±0.35 x ±0.30)内的累积占据格 =", int(inside.sum()))
    if inside.any():
        print("   这些格相对 imu:", [(round(float(a), 2), round(float(b), 2)) for a, b in
                                     np.stack([bx[inside], by[inside]], axis=1)][:15])


if __name__ == "__main__":
    main()