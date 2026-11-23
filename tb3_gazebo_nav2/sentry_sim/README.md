# sentry_sim —— RM 哨兵仿真（Odin1 定位 + Mid-360 感知 + Nav2）

> 2026-11-22/23 搭建。目标：在没有实车、没有地图的情况下，用**哨兵的真实技术路线**跑通
> 「传感器 → 定位 → 2D 感知 → Nav2 导航 → 巡逻」，并把结论做成可迁移到实车的形式。
> 场地与栅格图**复用** [`../rm_field`](../rm_field)：同一张 1:1 CAD 场地地图、同一套清场工具。

## 0. 一句话架构

```
Gazebo Harmonic (rm_field 3V3 场地)
  ├─ sentry_description : 4 麦轮 0.5×0.5×0.12 m 哨兵车 + Mid-360（livox_gz_sim）
  ├─ /livox/lidar ──► pointcloud_to_laserscan ──► /scan          （2D 感知）
  ├─ /livox/imu  200 Hz（当前无消费者，留给以后 LIO/融合）
  ├─ odin1_sim_bridge : Gazebo 真值 ──► /odin1/odometry + TF map→odom   （定位替身）
  └─ Nav2（不启 AMCL，map 由 map_server 加载 rm_field 的 CAD 栅格图）
```

关键决定（2026-11-22 用户确认）：
- **定位主力是 Odin1**（实车为官方驱动，仿真为接口替身），不跑 FAST-LIO，算力留给 Nav2；
- **2D 栅格图用场地 CAD 先验**，不用点云现场建图（原因见 §4）；
- 实车计划：**RDK S100 + Odin1 + Mid-360**，IMU 用 Mid-360 内置 6 轴 200 Hz。

## 1. 前置依赖

| 依赖 | 说明 |
|---|---|
| WSL Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic 8.15 | 本机已验证 8.15.0 |
| [`livox_gz_sim`](https://github.com/mathaimp/livox_gz_sim)（另建 `~/lidar_ws`） | Mid-360/HAP 的 Harmonic 插件，输出与 `livox_ros_driver2` 一致 |
| `livox_ros_driver2` 消息包 | 官方包在 Jazzy 上要 sudo 装 Livox-SDK2 → 用**消息-only 替身包**（同包名，只编 msg） |
| `rm_field` 的场地世界与地图 | `../rm_field/worlds/rmul2027_3v3_up_nav2.sdf.xacro`、`../rm_field/maps/rmul2027_3v3_up.{pgm,yaml}` |

> 源码在本目录；编译工作区在 WSL 的 `~/sentry_ws`（`colcon build --symlink-install`），
> 本目录是 source of truth，改完再 `rsync` 过去。

## 2. 快速开始

```bash
# WSL 内
source /opt/ros/jazzy/setup.bash && source ~/lidar_ws/install/setup.bash && source ~/sentry_ws/install/setup.bash
export ROS_DOMAIN_ID=45                    # 与旧 TB3 栈（42）隔离

cd ~/sentry_ws
./run_sentry_sim.sh                        # 只起仿真 + 传感器 + Odin1 替身 + /scan
./run_sentry_nav.sh                        # 仿真 + Nav2（无 AMCL）
./run_sentry_nav.sh gui:=true map_click:=true   # 上面 + Gazebo 窗口 + 地图点选窗口（点哪去哪）
./run_sentry_nav.sh map_offset_x:=0.30 map_offset_y:=-0.20 map_offset_yaw:=0.05236   # 注入帧偏移
./run_sentry_nav.sh stop                   # 停；status 看状态
```

发目标（回归用）：`python3 tools/send_nav_goal.py 0 3`（或 `ros2 action send_goal`）。

## 2.5 点地图导航（S3，2026-11-23 加）

窗口脚本：`tools/map_click.py`（`run_sentry_nav.sh ... map_click:=true` 自动起，也可单独
`./tools/run_map_click.sh`）。交互只有"地图 + 点击"，不用切 RViz 工具：

| 操作 | 效果 |
|---|---|
| 左键点地图 | 发 `NavigateToPose`；落点先吸附到净空 ≥ `--min-clear`（默认 **0.65 m**）的最近格子 |
| Shift+左键 | 探针：只报世界坐标 + 净空，不动车（指认障碍用） |
| 右键 / 中键 | 取消当前目标 |
| 画面 | 绿色箭头=机器人（TF `map→base_footprint`），蓝叉=目标，黄圈=原始点击（被吸附过时），灰线=当前 `/plan` |

状态栏显示：ROS 状态 / 机器人位姿 / 与 `/gt_odom` 真值的偏差 / 当前位置净空（<0.45 m 会亮警告）。

**为什么落点要 0.65 m 净空**：`goal_checker.xy_goal_tolerance = 0.2`，目标点净空 0.65 时停位仍有
≥0.45 m，高于 `robot_radius 0.40`。若车停在离障碍 <0.40 m 处（自己开过去 / 中途取消），
`global_costmap` 会把**规划起点**判成致命区，之后点哪都规划失败（日志
`Failed to create plan ... tolerance 0.3`）—— 2026-11-23 实测踩到，表现就是"点了没反应"。
真踩到就点一个远离墙的目标把车开出来，或 `./run_sentry_nav.sh stop` 重起（车回到原点）。

自检（无鼠标环境）：`python3 tools/map_click.py --click 3 -2.5` 会按世界坐标点一次、等结果、
打印 `SELFTEST result=... dist_to_goal=...`，到点 ≤0.4 m 且 SUCCEEDED 才返回 0。

## 2.6 临时障碍 / 敌方机器人测试（S3）

**不要用 Gazebo GUI 的 Insert 插模型**：WSL2 的 GUI 是 llvmpipe 软渲染，GUI 里插动态模型（球/箱）
会让 GUI 持续重绘 —— 实测 `gz sim gui` 冲到 **349% CPU**、RTF 掉到 **0.10**，表现就是整个仿真"死机"，
而且模型落下会砸/推车，车被推到墙边后就出现"点哪都不动"。用下面这套脚本（server 侧生成 static 模型）：

```bash
cd ~/sentry_ws
python3 tools/spawn_obstacle.py spawn 1.5 -1.25          # 生成 0.40 m 敌方机器人（可 --size/--yaw/--name）
python3 tools/spawn_obstacle.py remove                   # 删掉
python3 tools/verify_detour.py --goal 3 -2.5 --obstacle 1.5 -1.25     # 验收：绕开而不是撞
```

`verify_detour.py` 判据（全过才返回 0，报告写到 `results/report_detour_*.json`）：
到点 SUCCEEDED 且 ≤0.30 m / 绕障**面距** ≥0.05 m / 静态净空 ≥0.05 m（不蹭墙）/
横向偏移 ≥0.40 m（证明真绕了）/ 最长停滞 ≤5 s（没有"规划失败→恢复"循环）。
它同时打印 **用时 / 路径长度 / 最大速度**，可以直接当速度回归用。

**会动的敌车**（`--moving x1 y1 x2 y2`，沿线段往返，`--moving-speed` m/s）：

```bash
# 敌车在 (1.5,0.65)↔(1.5,-0.15) 来回横穿，车从 (0,3) 去 (3,-2.5)
python3 tools/moving_obstacle.py --from 1.5 0.65 --to 1.5 -0.15 --speed 0.3   # 单独演示用
python3 tools/verify_detour.py --moving 1.5 0.65 1.5 -0.15 --moving-speed 0.3 --goal 3 -2.5
```
移动敌车用 `gz set_pose` 推（`--dynamic` 模型），位置完全可控，验收时按墙钟插值算"当时敌车在哪"。

> 注意：横穿段如果**横扫过车当前停留的位置**，任何规划器都会被撞到——那是"敌车穿过车"
> 而不是"车没躲"。验收要用"扫过路线但不扫过车身"的几何（上面的例子就是）。

## 2.7 随机漫游敌车 + 键盘遥控（碰撞测试，S3）

```bash
cd ~/sentry_ws
# 终端 A：敌车在地图自由区随机漫游（只挑净空 >=0.9 m 的格子；
#           默认 --car-clear 0 = 不管我方车，要它别撞你就加 --car-clear 1.2）
python3 tools/wander_obstacle.py --speed 0.45

# 终端 B：键盘/按钮遥控（W/S 前后、A/D 平移、Q/E 转、空格停；没有窗管时用窗口里的按钮按住）
python3 tools/teleop_gui.py --speed 0.5
```

窗口显示：指令速度 / 底盘实际速度 / 车位置 / 离敌车距离 / 最近距离 / **碰撞次数** /
collision_monitor 状态。两种模式：

| 模式 | 链路 | 行为 |
|---|---|---|
| **安全模式**（默认） | `cmd_vel_nav → velocity_smoother → collision_monitor → /cmd_vel` | 前方有障碍时 collision_monitor 把车停住：实测按住前进 6 s，`cmd_vel_nav` 0.50 / `cmd_vel_smoothed` 0.50 / `/cmd_vel` **0.00**，车不动 |
| **裸模式**（勾选） | 把 `FootprintApproach.enabled` 设成 false（动态参数，节点仍 active、bond 不断），命令原样直通 | 才能真撞上做碰撞测试；窗口打印 `CONTACT 车(...) 敌车(...) 中心距 0.46 m` |

启动时窗口会**强制把碰撞保护设回 true**（安全模式），退出时如果还处于裸模式也会恢复。

退出遥控窗口会自动把 collision_monitor 恢复成 active。

> collision_monitor 只能拦"我方车开向障碍"，拦不住"别的物体撞向我方车"。
> `wander_obstacle.py` 默认 `--car-clear 0` 就是会撞上来的 —— 这正是这个测试要看的；
> 想让它躲着你开，就加 `--car-clear 1.2`。

## 2.8 双人模式：点地图开哨兵 + 遥控敌车（用户要的玩法）

| 谁 | 窗口 | 怎么操作 |
|---|---|---|
| **哨兵车（自己走）** | 点地图窗口 `tools/map_click.py` | **左键点地图任意位置 → 它自己规划走过去**（落点自动避开墙/敌车） |
| **敌车（你开）** | 遥控敌车窗口 `tools/enemy_teleop.py` | W/S/A/D + Q/E 转，或鼠标按住窗口按钮；只能走在净空 ≥0.55 m 的自由格，撞墙显示 `BLOCKED` |

```bash
cd ~/sentry_ws
python3 tools/map_click.py --map /mnt/e/RM/tb3_gazebo_nav2/rm_field/maps/rmul2027_3v3_up.yaml
python3 tools/enemy_teleop.py --start 0.5 2.5 --speed 0.6     # 敌车生成在 (0.5,2.5)
```

敌车是 static 干扰模型（没有底盘插件）：窗口内部按仿真时钟做运动学积分，每 50 ms 用
`gz set_pose` 摆到新位置 —— 位置可控、不参与物理求解、也不拖 GUI 渲染。
两个窗口都显示：位置 / 离对方距离 / **最近距离** / **碰撞次数**（中心距 <0.47 m 记一次）。
`wander_obstacle.py`（自动随机漫游）保留，做"没人开敌车"时的自动回归用。

> 点地图窗口的代价地图检查有新鲜度保护：超过 2 s 没收到新图就跳过"临时障碍"判断
> （旧进程只收到一帧旧图时会把空地误判成致命格，表现为"点空地也被挪走"）。

## 3. 已验证读数（全部可复跑）

| 项 | 读数 |
|---|---|
| 全向运动（各 2 s @20 Hz） | 前进 0.598 m（0.38%）、横移 0.630 m（4.98%）、自转 0.959 rad（4.08%） |
| `/livox/lidar` / `/livox/imu` / `/scan` | 10.0 Hz（2812–2891 点）/ 200.0 Hz / 10.0 Hz（720 束） |
| 传感器数据与真值 | 点云远点 7.23 m 为真实光线求交；`/scan` <1 m 回波 0 |
| Nav2 单点 | (3,−2.5) SUCCEEDED 0.197 m；(0,3) SUCCEEDED 0.196 m（真值 0.194） |
| 5 点巡逻 | **5/5 SUCCEEDED**，误差 0.005–0.197 m，最小离障 **0.371 m**，碰撞 0，RTF 0.864 |
| 点地图导航（S3） | 点 (3,−2.5) → `Goal succeeded`，停位 (2.977,−2.339) = **0.163 m** 误差 |
| 绕障·敌方机器人在路上（S3） | (0,0)→(3,−2.5)、敌车 (1.5,−1.25)：**5/5 判据过**，到点 0.107 m，绕障面距 **+0.236 m**，横向偏移 0.708 m，静态净空 0.861 m，8.4 s，峰值 0.98 m/s |
| 绕障·敌车横穿路线（S3） | 敌车 (1.5,0.65)↔(1.5,−0.15) @0.3 m/s，(0,3)→(3,−2.5)：**5/5 判据过**，到点 0.169 m，与移动敌车最小中心距 1.082 m（面距 +0.632 m），静态净空 0.843 m，12.3 s，峰值 1.38 m/s |
| 安全模式顶敌车（S3） | 按住前进 6 s：`cmd_vel_nav` 0.50 → `cmd_vel_smoothed` 0.50 → `/cmd_vel` **0.00**（monitor 停住），车不前进 |
| 碰撞检测（S3） | 敌车漫游贴上车：`CONTACT 车(1.28,−0.37) 敌车(1.55,−0.74) 中心距 0.46 m`（裸模式/敌车主动靠近） |
| 双人模式：点地图开哨兵（S3） | 点 (0,3)/(3,−2.5) → `CLICK go` 坐标精确（0.000/3.000、3.000/−2.500）→ `Goal succeeded`，到点 0.08 m |
| 双人模式：遥控敌车（S3） | 按住前进 3 s：敌车 (3.00,−2.50)→(4.32,−2.50)；开歪撞墙被拦（`BLOCKED`）；贴到哨兵旁触发 `CONTACT ... 0.46 m` |
| 绕障·敌车在返程线上（S3） | (2.9,−2.4)→(0,3)、敌车 (1.5,0.25)：**5/5 判据过**，到点 0.189 m，绕障面距 +0.519 m，横向偏移 1.450 m，静态净空 0.675 m，30.6 s，峰值 1.05 m/s |
| 地图/SDF 一致性 | pgm 与 48 块 box 投影**逐格一致**（0/180224 格不同），边界偏差 0.000 cm |
| 帧对齐校验（S2c） | 注入 (0,0,0)/(+0.30,−0.20,+3°)/(−0.15,+0.25,−5°) → 恢复误差 ≤2.3 mm / ≤0.063°，单次 4.4–4.7 s |

验证报告：`results/report_sentry_patrol.json`、`results/report_map_sdf.json`、
`results/align_case{1,2,3}.json`；证据日志在 WSL `/tmp/sentry_nav/evidence.log`。

## 4. 为什么不用点云现场建图（`src/pcd_to_pgm` 降级为实验工具）

实测（636 帧、63 s，Mid-360 装在 0.39 m 高）：

| 区域 | CAD 真值 | 点云建图 |
|---|---|---|
| box_011 西平台（1.85×1.85，高 0.2 m） | 100% 占据 | 仅 2.3% 占据（58% 自由 / 39% 未知） |
| box_013 东平台（1.85×1.8，高 0.2 m） | 100% | 10.7% |
| box_014 / box_009 | 100% | 15.3% / 2.3% |

原因：雷达高于低矮平台，顶面只在 ~1.5 m 内可见；打向远端的光线**从平台上空掠过**，
2D 投影把平台格擦成自由 → 平台中心永远观测不到。调 `min_occ_hits/block_hits` 只改边界。
结论：**低矮障碍必须靠 CAD 先验**。`pcd_to_pgm` 保留为实验工具（支持 dump/replay，12 s 级离线调参）。

## 5. 帧对齐（实车上线前的关键一步）

工具：`src/map_align_check`

```bash
# 车停在有角结构处（长直墙前会退化），跑一次
ros2 run map_align_check map_align_check --sim-time --frames 5 --max-points 3000 \
    --out /tmp/align.json --fix-yaml /tmp/align_fix.yaml
# 离线自检
python3 src/map_align_check/test/test_estimator.py
```

符号约定：`p_map = R(dyaw) · p_field + (dx, dy)`。输出含残差与 `confidence.gap_m`（越大越可信）。

**实车顺序**：建图（起点在场地原点、固定朝向）→ 重定位 → 停到拐角处跑校验 →
把 `correction` 覆盖到 Odin1 的 map 修正。仿真里用 `run_sentry_nav.sh map_offset_*` 注入偏移回归。

## 6. 目录

```
src/sentry_description   # 麦轮哨兵车 xacro（12 个 45° 被动滚子/轮）+ Mid-360 宏
src/sentry_bringup       # launch（sim / nav2）、sentry_nav2.yaml、patrol_points.yaml
src/odin1_sim_bridge     # Odin1 接口替身：真值 → /odin1/odometry + TF；可注入 map_offset_*
src/map_align_check      # 点云 ↔ CAD 图的帧对齐估计 + 修正 yaml
src/pcd_to_pgm           # 点云 → pgm 实验工具（dump/replay/compare）
src/pointcloud_to_laserscan  # 第三方包，vendored：[ros-perception/pointcloud_to_laserscan@442b332](https://github.com/ros-perception/pointcloud_to_laserscan/tree/442b33258b3af50e2981f338e1eac3ba259f8afa)（jazzy 分支，BSD）
tools/                   # 地图/SDF 复核、巡逻点挑选、巡逻执行、发目标、点地图窗口（map_click.py）、
                         # 敌方机器人障碍（spawn_obstacle.py）、移动/漫游敌车（moving_obstacle.py /
                         # wander_obstacle.py）、键盘遥控（teleop_gui.py）、绕障验收（verify_detour.py）
results/                 # 验收 JSON 证据
```

## 7. 坑（都已踩过）

1. **麦轮不能用圆柱轮**：四轮力抵消，横移/自转位移为 0。每轮 12 个 45° 被动滚子（revolute，轴 `0 0 1`），
   手性必须 FL+BR 一组、FR+BL 一组；接反会把横移变倒车。
2. **`<max_jerk>` 会翻车**：gz-math SpeedLimiter 在停车穿零时反向加速，0.3 m/s 一停整车 roll=π。
3. **话题冲突**：gz `MecanumDrive` 默认就用 `/cmd_vel`、`/odom`，必须显式换名，否则和 `OdometryPublisher` 抢。
4. **插件保持最后一条 Twist**：脚本分段发 `cmd_vel` 后必须补 0 速，否则多走。
5. **livox 插件直接发 ROS 话题**（rclcpp），不要再进 `ros_gz_bridge`；bridge 只桥 `/clock`、`/cmd_vel`、`/odom`。
6. **`GZ_SIM_RESOURCE_PATH` 要指到 `install/livox_gz_sim/share`**（少一层会丢 `mid360.dae`）。
7. **`pointcloud_to_laserscan` 参数**：`target_frame=base_footprint`、高度带 `0.05–0.60`，
   否则地面回波会在 3–4 m 处形成一圈假墙。
8. **TF 不能双父**：`map→imu_link` 与 `odom→base_footprint` 同时挂 base 会断 TF；必须标准链
   `map→odom→base_footprint`。
9. **costmap 的 `width/height` 必须是 int**（写 `3.0` 会让 controller_server 崩）。
10. **Jazzy 速度链**：`controller→cmd_vel_nav→velocity_smoother→cmd_vel_smoothed→collision_monitor→cmd_vel`，
    改错输出点车就不动。
11. **局部代价地图必须加 `static_layer`**：只吃 `/scan` 时墙在局部图里是空的（雷达装在 0.39 m 高、
    墙矮、又被车身挡住），躲临时障碍时 MPPI 就贴着墙切过去 → 撞墙 / 停在离墙 0.375 m。
    一旦停在离墙 < `robot_radius` 0.40 的位置，`global_costmap` 把**规划起点**判成致命区，
    之后点哪都 `Failed to create plan`（"点了没反应"的元凶）。
12. **全局代价地图必须加 `obstacle_layer`**：否则 `/scan` 看到的临时障碍只在局部图里，全局路径照样
    从障碍中间穿过，控制器只能硬挤。加上之后 NavFn 才会重规划绕行 —— 实测横向偏移 0.65–1.45 m。
13. **MPPI 的速度总闸是 `temperature`，不是 `vx_max`**：`vx_max` 0.4→2.0 但 `temperature` 保持 0.3 时，
    实际只跑 ~0.2 m/s（softmax 把采样轨迹平均掉了）；降到 **0.05** 后峰值 1.05–1.35 m/s，同一路线
    22.3 s → **8.2 s**。另外 `time_steps 56 / batch_size 2000` 在本机（RTF≈0.7）控制器只到 0.9 Hz，
    降到 **40 / 1000** 后回到 15 Hz。
14. **别在 Gazebo GUI 里插模型**：llvmpipe 软渲染 + 动态模型持续重绘会让 GUI 冲到 349% CPU、
    RTF 掉到 0.10（= 死机），落下的模型还会把车推/砸到墙边。用 `tools/spawn_obstacle.py`（server 侧
    static 模型）。
15. **`robot_radius` 0.40 → 0.60 是"别从缝里挤"的总闸**：0.40 时 NavFn 会把 0.8~1.1 m 的缝当路
    （车擦着过/顶着障碍过），表现为"不绕、硬挤、撞敌车"。0.60 让它把窄于 ~1.2 m 的缝判成死路，
    只能绕；实测 (3,−2.5)/(0,3)/(−4.1,−3.4) 仍然可达（连通性用地图离线算过）。
16. **点目标要查代价地图**：点地图窗口的静态吸附（`min_clear`）看不见敌车/箱子 —— 它们只在
    `/global_costmap/costmap` 里。窗口现在订阅该图，目标落在高代价格（≥85，内切/致命区）就自动
    挪到 1 m 内最近空地（实测点敌车正中心 → 代价 99 → 挪到 0.6 m 外），挪不开就拒绝发目标。
17. **别用 lifecycle 去 deactivate `collision_monitor`**：它和 lifecycle_manager 有 bond，
    手动 deactivate 会被判定"节点失联"→ 管理器把 bt_navigator 等一起停掉，整套导航下线、
    `/cmd_vel` 没人发，表现就是**车完全不动**（2026-11-23 踩过，遥控"打不开"就是这个）。
    要临时关掉碰撞保护，改动态参数 `FootprintApproach.enabled=false`（节点保持 active、bond 不断）。

## 8. Humble / RDK S100 兼容

代码按 **Humble + Jazzy 双兼容**写，2026-11-23 在 `ros:humble-ros-base` 容器（Ubuntu 22.04 / Python 3.10，与 S100 同构）
实测 **6/6 包编译通过**。为此改了三处（都已在代码里）：

1. `pointcloud_to_laserscan/CMakeLists.txt`：Humble 的 `tf2_sensor_msgs` 是纯头文件（无 CMake 目标），
   脚本加 `if(TARGET ...)` 守卫、补 `${sensor_msgs_TARGETS}`；反向节点依赖 `laser_geometry`，
   改为 `option(BUILD_LASERSCAN_TO_POINTCLOUD OFF)` **默认不编**（我们的栈只用 pointcloud→laserscan）。
2. `map_align_check/grid.py`：`backend='auto'` 在无 scipy 时曾直接崩（没走到文档承诺的 numpy 回退）→ 已修。
3. 两个 python 包 `package.xml` 补 `<buildtool_depend>ament_python</buildtool_depend>`。

**仍待实车核对**：`sentry_nav2.yaml` 由 Jazzy 生成，Humble 不认其中 8 个字段
（`navigators`、`error_code_names`、`route_server`、`docking_server` 等；Humble 的 bt_navigator 用
`plugin_lib_names` + `default_nav_to_pose_bt_xml`）。上 S100 时以
[上游 humble 的 nav2_params.yaml](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/params/nav2_params.yaml)
为底，只覆盖数值。详见 [迁移文档](../../docs/handoffs/2026-11-23-sentry-s100-migration.md)。

## 9. 已知限制 / 未做

- 未上实车/真 Odin1；Humble 编译已验证（§8），但 Nav2 参数与实车运行**未验证**。
- Odin1 在 12.6×8.6 m 大场地的量程/建图能力未实测（仿真按假设参数化）。
- Mid-360 的 `/livox/imu` 加速度单位是 **g**，未接 EKF/Nav2；不跑 LIO 时暂无消费者。
- `map_align_check` 连续模式只做过 6 帧冒烟；`--fix-yaml` 未自动回灌到 Odin1。
- **规划起点贴障碍的陷阱**：车停在离障碍 <0.40 m（`robot_radius`）处时 `global_costmap` 把起点判成
  致命区，NavFn 一律失败、恢复行为（spin/wait）也救不回来。点地图窗口用"落点净空 0.65 m + 状态栏
  净空警告 + ABORTED 提示"缓解，但**中途取消 / 手动开进窄缝仍可能触发**；真触发只能先把车开出来或重起栈。
  （2026-11-23 加了局部 `static_layer` 后，车自己不会再开进这种位置。）
- **开 GUI 时 RTF 只有 0.67–0.79**：`gz sim gui` 单进程吃 3.3 核（llvmpipe 软渲染），这也是"看起来慢"
  的一半原因；要跑最快速度/最高 RTF 就 `gui:=false`。
- 临时障碍测试：**静态敌车**与**慢速横穿敌车**（0.3–0.4 m/s）已验收；**追击/迎面而来的敌车**没做
  （`set_pose` 推的敌车不会决策；横穿段若扫过车身所在位置，任何规划器都会被撞 —— 见 §2.6 注）。
- 比赛要素（敌方车、装甲板/灯条、遭遇让行）尚未做。