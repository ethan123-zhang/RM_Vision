# Odin1 演示：项目参数与操作入口

本文件只放**本项目的当前取值与命令**。原理与判据见知识库
`aiVault/wiki/concepts/SLAM_Map_Save_and_Recording_Contention.md`
（建图回传与录像争用）、`Pose_Sensor_Lever_Arm.md`（力臂）、
`Differential_Drive_Point_Arrival.md`（追点半径）、`Chassis_Command_Unit_Calibration.md`（速度标定）。

## 当前参数（2026-11-06，连续 2 次成功）

```
VMAX=0.2 W_MAX=0.6 RETURN_W_MAX=0.6 RETURN_TIMEOUT_S=90 START_POS_TOL_M=0.2
BASE_OFFSET_X=0.2626 BASE_OFFSET_Y=-0.0471 MAX_TIME_S=400
```

建图/存图：`VIDEO=1 VIDEO_STRIDE=2 DT7=1 start_mapping.sh` + `SAVE_TIMEOUT=300 save_map.sh`。

## 操作入口

```bash
# 建图（每趟必须新目录）
MAP=~/car_ws/maps/demo-$(date +%m%d-%H%M)
VIDEO=1 VIDEO_STRIDE=2 DT7=1 bash ~/car_slam_lap/x5/scripts/start_mapping.sh "$MAP"

# 存图（先停录像）
pkill -INT -f "record_odin_rgb[.]py$"; sleep 3
SAVE_TIMEOUT=300 bash ~/car_slam_lap/x5/scripts/save_map.sh "$MAP"
ls -lh "$MAP"/odin_map/*.bin

# 重定位 → 示教（桥接必须停；一次 Ctrl-C）→ 自主
bash ~/car_slam_lap/x5/scripts/start_relocal.sh "$MAP"
bash ~/car_slam_lap/x5/scripts/teach_map_route.sh "$MAP"
VMAX=0.2 W_MAX=0.6 RETURN_W_MAX=0.6 RETURN_TIMEOUT_S=90 START_POS_TOL_M=0.2 \
BASE_OFFSET_X=0.2626 BASE_OFFSET_Y=-0.0471 MAX_TIME_S=400 \
bash ~/car_slam_lap/x5/scripts/drive_lap.sh "$MAP/route_map.yaml"

# 推送：先停后删（详见 docs/reset-and-restart.md）
bash ~/car_slam_lap/x5/scripts/reset_run.sh "$MAP" --teach-only|--full|--purge
```

## 本机事实

- 板端：`sunrise@10.77.156.239`，盘 29 GB；每趟建图约 2 GB 帧 + 0.25 GB 图。
- 驱动暂存：`~/odin_ws/src/odin_ros_driver/map/`（与 `~/car_ws/maps/` 各存一份）。
- 默认地图指针：`~/car_ws/.current_map_dir`。

## 故障速查

| 现象 | 处理 |
| :--- | :--- |
| 遥控推杆不动 | 桥接在跑 → `pkill -f "lib/car_bridge/bridge_node$"` |
| 录像 0 帧 | `sendrgb` 与 `sendrgbcompressed` 都为 1 |
| 存图目录空、日志停在 waiting | 停录像后重存；不动则重启驱动重跑 |
| 存图 150 s 超时 | `SAVE_TIMEOUT=300` |
| mp4 0 字节 / `Slice parameters invalid` | `scale=W:-2`（偶数高度） |
| 示教无 manifest | 只按一次 Ctrl-C 重录 |
| 自主抽搐 / 回不来 | 没传力臂，或 `RETURN_W_MAX` 未设 0.6 |
| 板端看录像 | `ffmpeg -i x.mp4 -pix_fmt rgb24 -f caca -` |
| 磁盘不足 | 清 `~/car_ws/maps/*`、`~/odin_ws/src/odin_ros_driver/map/*`、`~/.cache`、`~/.ros/log` |