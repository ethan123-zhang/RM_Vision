# 项目记忆索引（E:\RM）

> 这是**索引入口**，不存正文。项目状态的唯一"现在进行时"是 [PROGRESS.md](PROGRESS.md)；可迁移的原理/方法/实测结论进 aiVault（见下），项目里只留摘要与指针。

## 1. 当前状态

- 现在做什么、卡在哪、下一步：[PROGRESS.md](PROGRESS.md)（顶部区块，最新的在最前）。
- 最近一次交接：[docs/handoffs/2026-11-24-sentry-safety-lidar.md](docs/handoffs/2026-11-24-sentry-safety-lidar.md)（a 低位安全雷达 + c 5 Hz 重规划/缝窄判死路均完成；5 点巡逻回归 5/5，1 Hz 归因对照已做）。
- 上一轮：[docs/handoffs/2026-11-23-sentry-sim-nav-tuning-and-limits.md](docs/handoffs/2026-11-23-sentry-sim-nav-tuning-and-limits.md)（哨兵仿真交互化 + 导航调参）。

## 2. 交接记录（docs/handoffs/）

按会话追加，新文件优先于旧文件：

- [2026-11-23 晚 哨兵仿真交互化+调参+"贴脸未识别"根因（下一步 a+c）](docs/handoffs/2026-11-23-sentry-sim-nav-tuning-and-limits.md)
- [2026-11-23 收尾：哨兵仿真闭环 + S100 迁移准备](docs/handoffs/2026-11-23-sentry-sim-session-closeout.md) / [S100 实车迁移清单与上车判据](docs/handoffs/2026-11-23-sentry-s100-migration.md)
- [2026-11-21 清场 domain 隔离 + 动态箱三档 + 0.62 m 超窄门（2.5 cm 栅格变体）](docs/handoffs/2026-11-21-domain-scope-cleanup-and-acceptance-tiers.md)
- [2026-11-20 未建图障碍物把 AMCL 带偏（窄缝后漂走）根因+beam skip 修复+gate_deep 回归](docs/handoffs/2026-11-20-amcl-unmapped-obstacle-drift.md)
- [2026-11-20 绕障+动态避障验收（副线 Gazebo）](docs/handoffs/2026-11-20-obstacle-avoidance-acceptance.md)
- [2026-11-20 翻转后场地 Nav2 地图+三目标闭环](docs/handoffs/2026-11-20-nav2-map-on-flipped-field.md) / [2026-11-19 碰撞 box+键盘遥控](docs/handoffs/2026-11-19-collision-box-and-keyboard-teleop.md) / [场地翻转+配色](docs/handoffs/2026-11-19-3v3-field-flip-and-colors.md)
- [2026-11-04 标定与首次实车回放收尾](docs/handoffs/2026-11-04-car-slam-lap-calibration-replay.md)
- [2026-11-03 路线复现收尾（任务03）](docs/handoffs/2026-11-03-car-slam-lap-closeout.md)
- [2026-11-02 路线讨论](docs/handoffs/2026-11-02-car-slam-lap-route-discussion.md) / [执行](docs/handoffs/2026-11-02-car-slam-lap-execution.md)
- [2026-10-29 C 板调速闭环](docs/handoffs/2026-10-29-cboard-speed-loop.md) / [阶段1→2](docs/handoffs/2026-10-29-stage1-to-stage2-next.md)

## 3. 实验与决策

- 实验记录（含实测数据与结论）：[docs/experiments/](docs/experiments/)；小车 ROS 控制三条线在 [2026-11-03-car-ros-control/README.md](docs/experiments/2026-11-03-car-ros-control/README.md)。
- 决策：[docs/decisions/](docs/decisions/)；问题复盘：[docs/issues/](docs/issues/)。
- 任务卡（带验收用例的实施规格）：[car_slam_lap/docs/tasks/](car_slam_lap/docs/tasks/)。

## 4. 知识库（aiVault，跨项目可迁移结论）

路径：`E:/Document/Ethan'vault/aiVault/wiki/`。项目里不重复正文，只引用。

- [Pose_Sensor_Lever_Arm](E:/Document/Ethan'vault/aiVault/wiki/concepts/Pose_Sensor_Lever_Arm.md) — 力臂使"位置+朝向"起点判据互斥
- [Differential_Drive_Point_Arrival](E:/Document/Ethan'vault/aiVault/wiki/concepts/Differential_Drive_Point_Arrival.md) — 追点转弯半径必须大于到位容差
- [Chassis_Command_Unit_Calibration](E:/Document/Ethan'vault/aiVault/wiki/concepts/Chassis_Command_Unit_Calibration.md) — 指令单位标定口径与死区
- [Nav2_Map_And_Localization_Verification](E:/Document/Ethan'vault/aiVault/wiki/concepts/Nav2_Map_And_Localization_Verification.md) — 自建栅格地图方向/定位一致性；未建图障碍物把 AMCL 拉走 2.9 m 的机理与 beam skip 修法、全样本定位判据
- [2D_Mapping_Low_Obstacle_BlindSpot](E:/Document/Ethan'vault/aiVault/wiki/concepts/2D_Mapping_Low_Obstacle_BlindSpot.md) — 3D 雷达建 2D 图漏低矮障碍（0.2 m 平台只检出 2–15%）；低矮按 CAD 先验
- [SLAM_Map_Frame_Alignment_Check](E:/Document/Ethan'vault/aiVault/wiki/concepts/SLAM_Map_Frame_Alignment_Check.md) — 设备端 SLAM 的 map 帧与 CAD 图对齐：到"边界面"距离场 + 粗搜精修，mm 级
- [ROS2_Humble_Jazzy_Compat_Traps](E:/Document/Ethan'vault/aiVault/wiki/concepts/ROS2_Humble_Jazzy_Compat_Traps.md) — 同一份代码双版本：tf2_sensor_msgs 纯头文件、C++17、ament_python、Nav2 参数不可跨版本搬
- [Nav2_Costmap_MPPI_Tuning_Traps](E:/Document/Ethan'vault/aiVault/wiki/concepts/Nav2_Costmap_MPPI_Tuning_Traps.md) — 起点在致命区/图层职责/MPPI temperature；补盲雷达高度口径、gz 帧名与 invalid-source 冻车、"过不去就停"用路径对内切区净空判定

## 5. 证据与产物

- 探针脚本与本机实验数据：`_probe/`（不入库，见 `.gitignore`）。
- 板端证据：`sunrise@10.77.156.239:~/car_ws/logs/<实验组>/`；部署副本 `~/car-deployments/`。
- 关键参数总表：[car_slam_lap/README.md](car_slam_lap/README.md)（串口/协议/坐标/速度比例）。