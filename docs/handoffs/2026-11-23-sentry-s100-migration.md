# 2026-11-23 哨兵实车迁移准备（RDK S100 + Odin1 + Mid-360）

> 上游：仿真栈见 [tb3_gazebo_nav2/sentry_sim](../../tb3_gazebo_nav2/sentry_sim/README.md)，
> 决策与验证记录见 [计划 v4](../superpowers/plans/2026-11-22-sentry-simulation.md)。
> 本文只写"怎么搬上车"和"上车前还缺什么"，不含机械/发射机构。

## 0. 一个先要明确的事实：车上要跑的代码比仿真少

| 模块 | 仿真用 | 实车用 | 说明 |
|---|---|---|---|
| `sentry_description`（麦轮车 xacro + gz 插件） | ✅ | ❌ | 实车是 C 板底盘，模型不参与运行 |
| `odin1_sim_bridge`（Odin1 接口替身） | ✅ | ❌ | 实车换官方 `odin_ros_driver` |
| `pcd_to_pgm`（点云建图实验） | ✅ 实验 | ❌ | 已判定不用于导航 |
| `map_align_check` | ✅ | **✅** | 帧对齐校验，实车必须用 |
| `sentry_bringup`（launch / yaml / 巡逻点） | ✅ | **✅** | 参数要按实车重标 |
| `pointcloud_to_laserscan` | ✅ | **✅** | Mid-360 点云 → `/scan` |
| Nav2（Humble） | ✅ Jazzy | **✅ Humble** | 无 AMCL，定位由 Odin1 给 |

→ 真正需要"双版本兼容"的只有 `map_align_check` + launch/yaml + 少量工具；纯 Python 且依赖面窄。

## 1. S100 适配清单（按依赖顺序）

| # | 项 | 要求 | 验证命令 / 判据 |
|---|---|---|---|
| 1 | 系统 | Ubuntu 22.04（官方 S100 镜像） | `cat /etc/os-release` |
| 2 | ROS | **Humble**（官方 S100/S100P 对应 Humble；S600 才是 Jazzy） | `echo $ROS_DISTRO` = humble |
| 3 | Nav2 | `ros-humble-navigation2`、`ros-humble-nav2-bringup` | `ros2 pkg list \| grep nav2_bringup` |
| 4 | 存储 | 64 GB eMMC；工作区 `~/sentry_ws` | `df -h /` ≥10 GB 余量 |
| 5 | Odin1 | USB3 口 + **12 V/2 A 独立供电**；厂商驱动 `manifoldsdk/odin_ros_driver`（aarch64 直接编，X5 上实测 32 min） | `lsusb \| grep 2207:0019`；`ros2 topic hz /odin1/cloud_raw` ≈10.26 Hz |
| 6 | Mid-360 | **网口**（静态 IP + PTP），12 V 供电；`livox_ros_driver2` Humble 分支 + Livox SDK2 | `ros2 topic hz /livox/lidar` ≈10 Hz；`/livox/imu` ≈200 Hz |
| 7 | 网口分配 | S100 只有一个网口 → Mid-360 用它（或 USB 千兆网卡），SSH 走 Wi-Fi | `ip a`；`ping` 雷达 IP |
| 8 | 时钟统一 | 两路都接到主机 ROS 时间（Odin1 `use_host_ros_time: 1`；Livox 驱动用主机时间） | `ros2 topic echo /odin1/odometry --field header.stamp` 与 `/clock`/系统时间同源 |
| 9 | 依赖 | `tf2_ros`、`numpy`、`ros-humble-pointcloud-to-laserscan`（或源码） | `colcon build` 0 错误 |
| 10 | 供电预算 | Odin1 ≈15 W + Mid-360 ≈6.5 W + S100 + 底盘；两路 12 V 单独走线 | 空载电压、峰值电流实测 |

## 2. 上车步骤（6 阶段，每阶段有判据）

### 阶段 1 板级
1. 烧官方 S100 镜像 → 确认 Ubuntu 22.04 + ROS Humble
2. `lsusb | grep 2207:0019`（Odin1）、Mid-360 网口 `ping` 通
3. 判据：两路传感器枚举正常、无掉线（`dmesg` 无 USB reset）

### 阶段 2 传感器
```bash
ros2 topic hz /odin1/cloud_raw      # ≈10.26 Hz，49152 点/帧
ros2 topic hz /livox/lidar          # ≈10 Hz
ros2 topic hz /livox/imu            # ≈200 Hz
```
判据：三路都稳定；点云不是空的；`/odin1/cloud_raw` 前向 FOV 约 129°（侧后盲区记住）

### 阶段 3 定位（Odin1）
1. 车放在**场地原点、车头朝场地 +x**（这是 map 帧与场地帧对齐的前提）
2. 开始建图（官方 `start_mapping` 等价流程），绕场一周 → `save_map` 拿 `.bin`
3. 重定位：`start_relocal`，等到 `Custom map mode = 2` 且 `map←imu` 稳定
4. 判据：重定位成功、静止 1 min 位姿波动 ≤2 cm / 0.2°

### 阶段 4 地图与帧对齐
1. 部署 `maps/rmul2027_3v3_up.{pgm,yaml}`（与场地 CAD 逐格一致，来自 `rm_field`）
2. 车停在**有角结构**处（长直墙前会退化），跑：
```bash
ros2 run map_align_check map_align_check --frames 5 --max-points 3000 \
    --out /tmp/align.json --fix-yaml /tmp/align_fix.yaml
```
3. 判据：估计 `|dx|,|dy| ≤ 0.05 m`、`|dyaw| ≤ 0.5°`、`confidence.gap_m` 不塌（仿真实测 0.021–0.024 m）
4. 把 `correction` 写进 Odin1 的 map 修正（或加静态 TF 抵消）

### 阶段 5 导航（Nav2 Humble，无 AMCL）
1. 起栈：`/scan` 10 Hz、`/map` 正常、`map→odom→base_footprint` TF 通
2. 发 1 个目标（`tools/send_nav_goal.py`）
3. 判据（与仿真同口径）：到点 ≤0.3 m、最小离障 ≥0.35 m、不出界、`/odom` 无 NaN

### 阶段 6 巡逻
5 点巡逻（`patrol_points.yaml` 需按实车场地重新挑点：净空 ≥0.45 m 且同一连通域）
判据：5/5 到达、误差 ≤0.3 m、碰撞 0。

## 3. Odin1 大场地图测试方案（最大未知项）

背景：Odin1 只在室内房间（观测 0.2–5.6 m）验证过；比赛场地 **12.6×8.6 m**，量程/建图能力未知。

| 测试 | 做法 | 判据 |
|---|---|---|
| T1 建图覆盖 | 绕场一周建图，统计点云覆盖场地边界的比例；记录 `.bin` 大小 | 边界覆盖 ≥95% |
| T2 重定位成功率 | 4 角 + 中心共 5 点 × 每点 5 次（含冷启动） | 成功率 ≥90%，失败样本要留日志 |
| T3 静止漂移 | 单点静止 5 min | 位置 ≤5 cm、yaw ≤0.5° |
| T4 长距离误差 | 沿走道往返，与地面标记比对 | ≤10 cm |
| T5 帧对齐稳定性 | 4 个拐角各跑一次 `map_align_check` | 估计值彼此差 ≤5 cm / 0.5° |

**不达标回退**：改用 Mid-360 + FAST-LIO 做定位（已验证静止 60 s 漂移 11.6 mm / yaw 2.24 mrad，
配置与坑记录在计划 §0.2）。切回 LIO 只需换定位源，Nav2 / 地图 / 对齐工具都不变。

## 4. Humble 双版本验证 ✅（2026-11-23 实测）

方式：Docker `ros:humble-ros-base`（容器内 Ubuntu 22.04.5 / Python 3.10.12 / GCC 11.4 / CMake 3.22.1，与 S100 同构）。

**结果：6/6 包编译通过。** 4 个 python 包 + `sentry_bringup`/`sentry_description` 零改动；
唯一编译阻塞是 `pointcloud_to_laserscan` 的 CMake。已修入代码：

| # | 问题 | 修法 |
|---|---|---|
| 1 | Humble 的 `tf2_sensor_msgs` 是**纯头文件**（无 CMake 目标），Jazzy 才有 INTERFACE 目标 → 原 CMake 直连目标名在 Humble configure 失败 | CMake 加 `if(TARGET ...)` 守卫 + 补 `${sensor_msgs_TARGETS}`（Jazzy 也定义，双兼容） |
| 2 | 反向节点（laserscan→pointcloud）依赖 `laser_geometry`，我们的栈不用它，且该包在受限网络下装不上 | 改成 `option(BUILD_LASERSCAN_TO_POINTCLOUD ... OFF)` 默认不编 → **依赖里彻底去掉 laser_geometry** |
| 3 | `map_align_check/grid.py` 的 `backend='auto'` 在无 scipy 时直接崩（文档承诺的 numpy 回退走不到；`package.xml` 里 scipy 是注释掉的） | 已修回 numpy wavefront 回退；自检含"模拟 scipy 缺失"通过 |
| 4 | 两个 python 包缺 `<buildtool_depend>ament_python</buildtool_depend>` | 已补 |

Jazzy 回归：4 包重编通过（14.9 s），`test_estimator.py`、`pcd_to_pgm/test/test_offline.py` 全 PASS。

**仍未验证**：Nav2 参数在 Humble 的真实字段（容器是 ros-base，apt 被网络策略拦，装不了 nav2）。

### 4.1 Nav2 参数表必须按 Humble 重写（高风险，未实测）

`sentry_nav2.yaml` 由 Jazzy 生成，含 Humble 不存在的 8 处：
`navigators`、`error_code_names`、`route_server`、`docking_server`、
`stamp_smoothed_velocity_with_smoothing_time`、`near_collision_cost`、
`use_path_orientations`、`max_path_occupancy_ratio`。

Humble 的 `bt_navigator` 用 `plugin_lib_names` + `default_nav_to_pose_bt_xml`，不是 `navigators`。
→ 上 S100 时以[上游 humble 分支的 nav2_params.yaml](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/params/nav2_params.yaml)为底，
只覆盖我们的数值（`robot_radius 0.40`、`inflation 0.55`、MPPI `Omni`、`vx/vy 0.4`、`wz 1.2`、`/scan`），不要整表照搬。

## 5. 风险与回退

| 风险 | 影响 | 回退 |
|---|---|---|
| Odin1 大场地重定位/建图不行 | 定位失效 | Mid-360 + FAST-LIO（已存档） |
| Humble 编译不过 | 上不了车 | 只保留纯 Python 的 `map_align_check` + launch，Nav2 直接用官方 Humble 包 |
| 网口只一个 | Mid-360 与 SSH 冲突 | USB 千兆网卡 / 交换机 |
| 双传感器时钟不同源 | 运动中 costmap 拖影 | 统一主机时间；仍不行则 Odin1 只做定位、`/scan` 加滤波 |
| 供电不足（Odin1 曾因 12 V 电流不够掉线） | 传感器掉线 | 独立 12 V/2 A 航插，先量峰值电流 |

## 6. 下次进入点

```bash
# 板上（S100）
source /opt/ros/humble/setup.bash
lsusb | grep 2207:0019
ros2 topic hz /odin1/cloud_raw ; ros2 topic hz /livox/lidar ; ros2 topic hz /livox/imu
ros2 run map_align_check map_align_check --frames 5 --max-points 3000 --fix-yaml /tmp/fix.yaml
python3 tools/send_nav_goal.py 3 0        # 单点验收
```

## 7. 复跑 Humble 验证（照抄即可）

```powershell
# 宿主机：Docker Desktop 装在 D:\Docker（不在 Program Files）
Start-Process "D:\Docker\Docker Desktop.exe"
docker pull ros:humble-ros-base          # 1.16 GB，约 8 min
docker run --rm -it -v "E:\RM\tb3_gazebo_nav2\sentry_sim\src":/ws/src ros:humble-ros-base bash
```
```bash
# 容器内
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select odin1_sim_bridge map_align_check pcd_to_pgm
colcon build --symlink-install --packages-select pointcloud_to_laserscan
python3 /ws/src/map_align_check/test/test_estimator.py     # SELF_TEST PASS
python3 /ws/src/pcd_to_pgm/test/test_offline.py            # PASS
```

> 注意：本机 WSL 到 GitHub 的直连时通时断（2026-11-23 出现 `gnutls_handshake` 失败），
> 拉镜像/补 apt 前先确认网络；Docker Hub 与 `raw.githubusercontent.com` 都可能被拦。