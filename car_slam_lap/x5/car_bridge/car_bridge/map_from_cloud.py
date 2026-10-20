"""map_from_cloud —— 把 /odin1/cloud_slam 累积成地图文件（建模的可视交付物）。

输出三样东西到 out_dir：
    map.pcd    点云（ascii，可直接拖进 CloudCompare / Open3D）
    map_bev.png 俯视图（白底黑点，一眼能看出房间形状）
    map_info.txt 点数、包围盒尺寸、点云覆盖面积

为什么不用 Odin1 自己存的 .bin：那个是给设备重定位用的专有格式，人看不了；
这里的 PCD/PNG 用来核对"我建出来的图是不是这个房间"，两者互补。

用法（建图那趟和驱动一起跑，Ctrl-C 时落盘）：
    ros2 run car_bridge map_from_cloud --ros-args \
        -p out_dir:=$HOME/car_ws/maps/lab1
"""

from __future__ import annotations

import math
import os
import time
from typing import Dict, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

import tf2_ros

try:
    from sensor_msgs_py import point_cloud2 as pc2
except ImportError:  # pragma: no cover
    pc2 = None


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
        ],
        dtype=np.float64,
    )


class MapFromCloud(Node):
    def __init__(self) -> None:
        super().__init__("map_from_cloud")
        self.declare_parameter("cloud_topic", "/odin1/cloud_slam")
        # cloud_slam 的 header.frame_id 就是 odom；建图模式下 map 与 odom 同原点，
        # 所以默认在 odom 里累积，完全不需要查 TF（少一个失败点）。
        # 想在重定位那趟顺手累积到 map 系时，把它改成 map。
        self.declare_parameter("world_frame", "odom")
        self.declare_parameter("out_dir", os.path.expanduser("~/car_ws/maps"))
        self.declare_parameter("voxel", 0.05)
        self.declare_parameter("max_points", 400000)
        self.declare_parameter("z_min", -0.30)
        self.declare_parameter("z_max", 2.00)
        self.declare_parameter("png_res", 0.05)
        self.declare_parameter("autosave_s", 0.0)  # 0 = 只在退出时落盘

        self.world_frame: str = self.get_parameter("world_frame").value
        self.out_dir: str = self.get_parameter("out_dir").value
        self.voxel: float = float(self.get_parameter("voxel").value)
        self.max_points: int = int(self.get_parameter("max_points").value)
        self.z_min: float = float(self.get_parameter("z_min").value)
        self.z_max: float = float(self.get_parameter("z_max").value)
        self.png_res: float = float(self.get_parameter("png_res").value)

        os.makedirs(self.out_dir, exist_ok=True)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._vox: Dict[Tuple[int, int, int], Tuple[float, float, float]] = {}
        self._frames = 0
        self._tf_miss = 0
        self._warned_identity = False
        self._warned_full = False
        self._t_start = time.monotonic()

        self.create_subscription(PointCloud2, self.get_parameter("cloud_topic").value,
                                 self._on_cloud, 5)
        autosave = float(self.get_parameter("autosave_s").value)
        if autosave > 0:
            self.create_timer(autosave, self.save)
        self.get_logger().info(
            f"累积点云 -> {self.out_dir}（voxel={self.voxel} m，z 带 "
            f"[{self.z_min},{self.z_max}]，上限 {self.max_points} 点）"
        )

    # ---------------------------------------------------------- 变换
    def _transform(self, stamp, frame_id: str) -> Optional[np.ndarray]:
        if not frame_id or frame_id == self.world_frame:
            return np.eye(4)
        try:
            tf = self._tf_buffer.lookup_transform(self.world_frame, frame_id, rclpy.time.Time())
        except Exception:  # noqa: BLE001
            self._tf_miss += 1
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        m = np.eye(4)
        m[:3, :3] = quat_to_matrix(q.x, q.y, q.z, q.w)
        m[:3, 3] = (t.x, t.y, t.z)
        return m

    # ---------------------------------------------------------- 回调
    def _on_cloud(self, msg: PointCloud2) -> None:
        if pc2 is None:
            self.get_logger().error("没有 sensor_msgs_py.point_cloud2")
            return
        try:
            pts = pc2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        except AttributeError:  # 老版本只有 read_points
            pts = np.array(
                list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)),
                dtype=np.float32,
            )
        if pts is None or len(pts) == 0:
            return
        pts = np.asarray(pts, dtype=np.float64).reshape(-1, 3)

        m = self._transform(msg.header.stamp, msg.header.frame_id)
        if m is None:
            if not self._warned_identity:
                self._warned_identity = True
                self.get_logger().warn(
                    f"查不到 TF {self.world_frame} <- {msg.header.frame_id}，本帧先丢弃"
                    "（建图模式下 cloud_slam 一般在 odom 系，可直接跑）"
                )
            return

        pts = pts @ m[:3, :3].T + m[:3, 3]

        keys = np.floor(pts / self.voxel).astype(np.int64)
        uniq, first = np.unique(keys, axis=0, return_index=True)
        sel = pts[first]
        added = 0
        for k, p in zip(uniq, sel):
            key = (int(k[0]), int(k[1]), int(k[2]))
            if key in self._vox:
                continue
            if len(self._vox) >= self.max_points:
                if not self._warned_full:
                    self._warned_full = True
                    self.get_logger().warn(f"点数到上限 {self.max_points}，之后只更新不新增")
                break
            self._vox[key] = (float(p[0]), float(p[1]), float(p[2]))
            added += 1

        self._frames += 1
        if self._frames % 20 == 0:
            self.get_logger().info(
                f"帧 {self._frames}: {len(pts)} 点 -> +{added} 体素，累计 {len(self._vox)}"
            )

    # ---------------------------------------------------------- 落盘
    def save(self) -> None:
        if not self._vox:
            self.get_logger().warn("还没收到任何点云，不写文件")
            return
        arr = np.array(list(self._vox.values()), dtype=np.float64)
        pcd = os.path.join(self.out_dir, "map.pcd")
        self._write_pcd(pcd, arr)

        info = [
            f"累计帧数     : {self._frames}",
            f"点数（体素后）: {len(arr)}   voxel={self.voxel} m",
            f"耗时         : {time.monotonic() - self._t_start:.1f} s",
        ]
        band = arr[(arr[:, 2] >= self.z_min) & (arr[:, 2] <= self.z_max)]
        if len(band):
            lo, hi = band.min(axis=0), band.max(axis=0)
            size = hi - lo
            cells = len({(int(x / self.png_res), int(y / self.png_res))
                         for x, y in band[:, :2]})
            area = cells * self.png_res * self.png_res
            info += [
                f"水平包围盒   : X {size[0]:.2f} m  Y {size[1]:.2f} m  高 {size[2]:.2f} m",
                f"点云覆盖面积 : {area:.2f} m^2（{cells} 个 {self.png_res} m 栅格）",
            ]
            self._write_bev(os.path.join(self.out_dir, "map_bev.png"), band)

        txt = os.path.join(self.out_dir, "map_info.txt")
        with open(txt, "w", encoding="utf-8") as fh:
            fh.write("car_slam_lap 建图结果\n" + "\n".join(info) + "\n")
        self.get_logger().info("已写 " + ", ".join([pcd, txt]) + f"（{len(arr)} 点）")
        for line in info:
            print("  " + line)

    @staticmethod
    def _write_pcd(path: str, pts: np.ndarray) -> None:
        with open(path, "w", encoding="ascii") as fh:
            fh.write(
                "# .PCD v0.7 - Point Cloud Data file format\n"
                "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
                f"WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
                f"POINTS {len(pts)}\nDATA ascii\n"
            )
            np.savetxt(fh, pts, fmt="%.4f %.4f %.4f")

    def _write_bev(self, path: str, pts: np.ndarray) -> None:
        try:
            import cv2
        except ImportError:  # pragma: no cover
            self.get_logger().warn("没有 cv2，跳过俯视图（PCD 仍然生成了）")
            return
        res = self.png_res
        lo = pts[:, :2].min(axis=0) - 0.3
        hi = pts[:, :2].max(axis=0) + 0.3
        w = int(math.ceil((hi[0] - lo[0]) / res))
        h = int(math.ceil((hi[1] - lo[1]) / res))
        if w <= 0 or h <= 0 or w * h > 40_000_000:
            self.get_logger().warn(f"俯视图尺寸异常 {w}x{h}，跳过")
            return
        img = np.full((h, w), 255, dtype=np.uint8)
        # 图像坐标 y 向下，世界坐标 y 向上 -> 翻转
        for x, y in pts[:, :2]:
            cx = int((x - lo[0]) / res)
            cy = h - 1 - int((y - lo[1]) / res)
            if 0 <= cx < w and 0 <= cy < h:
                img[cy, cx] = 0
        img = cv2.medianBlur(img, 3)
        cv2.putText(img, f"{w * res:.1f} x {h * res:.1f} m", (6, 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, 100, 1, cv2.LINE_AA)
        cv2.imwrite(path, img)
        self.get_logger().info(f"俯视图 {path}（{w}x{h} px @ {res} m/px）")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapFromCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
