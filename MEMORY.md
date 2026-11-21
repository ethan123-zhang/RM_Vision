# 项目记忆索引（E:\RM）

> 这是**索引入口**，不存正文。项目状态的唯一"现在进行时"是 [PROGRESS.md](PROGRESS.md)；可迁移的原理/方法/实测结论进 aiVault（见下），项目里只留摘要与指针。

## 1. 当前状态

- 现在做什么、卡在哪、下一步：[PROGRESS.md](PROGRESS.md)（顶部区块，最新的在最前）。
- 最近一次交接：[docs/handoffs/2026-11-21-domain-scope-cleanup-and-acceptance-tiers.md](docs/handoffs/2026-11-21-domain-scope-cleanup-and-acceptance-tiers.md)。

## 2. 交接记录（docs/handoffs/）

按会话追加，新文件优先于旧文件：

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

## 5. 证据与产物

- 探针脚本与本机实验数据：`_probe/`（不入库，见 `.gitignore`）。
- 板端证据：`sunrise@10.77.156.239:~/car_ws/logs/<实验组>/`；部署副本 `~/car-deployments/`。
- 关键参数总表：[car_slam_lap/README.md](car_slam_lap/README.md)（串口/协议/坐标/速度比例）。