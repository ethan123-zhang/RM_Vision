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
| 左键点地图 | 发 `NavigateToPose`；落点先吸附到净空 ≥ `--min-clear`（默认 **0.85 m**）的最近格子 |
| Shift+左键 | 探针：只报世界坐标 + 净空，不动车（指认障碍用） |
| 右键 / 中键 | 取消当前目标 |
| **脱困按钮 / R 键** | 车贴墙（净空 <0.72 m）时朝"净空变大"的方向倒出来，再重发目标；点地图时若发现贴墙会**自动先脱困** |
| 画面 | 绿色箭头=机器人（TF `map→base_footprint`），蓝叉=目标，黄圈=原始点击（被吸附过时），灰线=当前 `/plan` |

**为什么落点要 0.85 m 净空 / 为什么要脱困**：`robot_radius` 现在是 **0.60**（S3 为了不让车从缝里挤），
而 `goal_checker.xy_goal_tolerance = 0.2` —— 目标点净空 0.65 时车可能停在净空 0.45 处，
**低于 0.60 就被 `global_costmap` 判成"起点在致命区"**：之后点哪都 `Failed to create plan`
（实测 42 次失败，车"卡在墙里"出不来）。所以：
1. 落点门槛提到 0.85（停位净空 ≥0.65 > 0.60）；
2. 真卡住了：面板检测到当前净空 <0.72，就先按"净空梯度"方向倒车（速度 0.25 m/s，走 `cmd_vel_nav`
   同一条安全链），脱困后再发目标；目标被 ABORTED 也会自动脱困重发一次（最多 2 次）。
   实测：车卡在 (−3.90,0.33) 净空 0.35 → `RESCUE ... done clear=0.80` → 目标 (0,3) 重新发出并正常过去。

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

## 2.9 低位 2D 安全雷达（S3a，2026-11-24）

**要解决的问题**：敌车高 0.25 m，Mid-360 装在 0.30 m 且 `lidar_slices:=1`（单层）→ 0.25 m 高的
物体在 1.2 m 处就从 `/scan` 里消失（实测把敌车摆到车头 1.2 m：`/scan` 最小有限量程仍是 1.426 m
的墙；`/scan_safety` 读到 **1.000 m** = 敌车正面）。于是"敌车半路突然出现在车前"只能面贴面
（旧报告 `gap_face -0.000 m`）。

**四处改动**：

1. `sentry_description/urdf/sentry.urdf.xacro`：`base_link` 上加 `gpu_lidar`，
   `<pose>0 0 0.09 0 0 0</pose>`（离地 **0.18 m**）、360°/360 线/10 Hz/量程 0.05–2.5 m，
   gz 话题 `/scan_safety_raw`。**为什么不取"离地 0.10 m"**：底盘占 z 0.03–0.15 m，装在 0.10 m
   等于把 360° 每一束先打在自己壳上（满格自身回波 → NoTouch 永久停车）。0.18 m 掠过车顶 3 cm，
   又比敌车顶面 0.25 m 低 7 cm，正好横切敌车侧面；x/y 偏移为 0。
2. `sentry_bringup/launch/sentry_sim.launch.py`：桥 `/scan_safety_raw`，并起 `safety_scan_relay.py`。
3. `config/sentry_nav2.yaml`：两张 costmap 的 `obstacle_layer` 都加 `scan_safety` 源（量程 2.5 m →
   `obstacle_max_range 2.4`）；`collision_monitor.observation_sources` 加 `scan_safety`（`min_height 0`，
   因为 LaserScan 的点在自身帧里 z=0，旧 `scan` 源的 0.15 会把它整帧过滤掉）。
   顺带把 `FootprintApproach.time_before_collision` 1.0 → **0.4**：1.0 s 在 1 m/s 时等于"前方 1.6 m
   有障碍就开始减速"，一趟 200 s 的跑法在 approach/continue 之间切了 1099 次、车被压到 0.05–0.2 m/s。
4. `scripts/safety_scan_relay.py`：gz 传感器把帧名写成作用域名 `sentry/base_footprint/safety_lidar`，
   TF 里没有 → 两张 costmap 只报 `Message Filter dropping message: frame '...'`，症状和"雷达没工作"
   一模一样。relay 只做 `header.frame_id = base_footprint`（传感器 x/y 偏移 0，所以精确）。
   兜底：raw 超 0.5 s 没数据就发**空扫描** —— 否则 `collision_monitor` 报
   `Robot to stop due to invalid source` 把车永久钉住（实测 `safety_lidar:=false` 时整车 216 s 不动）。

```bash
./run_sentry_nav.sh gui:=false                      # 默认就带安全雷达
python3 tools/check_scan_safety.py --frames 8       # 体检 5 项：频率/束数/量程/无自身回波/TF
./run_sentry_nav.sh gui:=false safety_lidar:=false  # 回到旧行为（复现"贴脸看不见"）
```

`safety_lidar:=false` 时 costmap 仍列着 `scan_safety`，由 relay 发空扫描顶上 →
语义是"安全层是瞎的、但导航不冻"，正好当 A/B 对照。

## 2.10 5 Hz 重规划 + 缝窄判死路（S3c，2026-11-24）

**为什么要 5 Hz**：Nav2 默认 BT 是 **1 Hz** 重规划。车速 1.3 m/s 时 1 s 能跑 1.3 m，等于
"路径永远慢一拍"：临时障碍进代价地图后，全局路径最多要等 1 s 才更新。
`src/sentry_bringup/bt/sentry_navigate_to_pose.xml` 把 `RateController hz` 改成 **5.0**，
由 `sentry_nav2.launch.py` 以 `default_nav_to_pose_bt_xml` 注入。实测 `/plan` 平均 **4.48 Hz**。

**为什么还要"判死路"**：5 Hz 只能让路径跟上代价地图，跟随路径的还是 MPPI。当路径要从
一条 1.2–1.3 m 的缝里挤过去时，footprint 半径 0.6 的 inscribed 区已经把两侧都吃掉，
路径只能贴着内切边界走（实测路径最小净空 **0.05 m**）—— MPPI 于是原地小幅转/爬
**200 s**、10–11 次 `Failed to make progress`，既不撞也不走。这时正确行为是**判死路、取消、停住**。

判定口径（`tools/path_guard.py`）：把全局代价地图里 `>=99`（99=内切/100=致命）的格子当障碍做
距离变换，沿 `/plan`（去掉起点 0.6 m 与终点附近 0.4 m）取**最小净空**：

| 场景 | 路径最小净空 | 结论 |
|---|---|---|
| 开阔：goal (3,−2.5) | **0.43–0.65 m** | 能过 |
| 窄缝：敌车 0.4 m 卡在 1.3 m 走廊里、goal (0,2.6) | **0.05 m** | 判死路 |

阈值 **0.20 m**（对上面两组读数有 3 倍以上裕度），连续 2 帧命中才认（防单帧噪声）。

**为什么判定在客户端**：ROS 2 没有"按条件取消别人目标"的公开 API，取消要发起方手上的
goal handle。所以判定挂在点地图窗口（`map_click.py` 的 `RosBackend.dead_end()`）和验收脚本
（`tools/verify_dead_end.py`）里 —— 自己发目标、自己判、自己取消。**直接 `ros2 action send_goal`
发的目标不受这个保护**（没有客户端在盯路径）。

```bash
# 死路：判出来就取消并停车（0.8 s 内判定，取消后位移 0.00 m，面距 +0.665 m）
python3 tools/verify_dead_end.py --goal 0 2.6 --obstacle 0 1.2 --spawn --spawn-after 1.5 --remove
# 反向回归：开阔场地不该误判
python3 tools/verify_dead_end.py --goal 3 -2.5 --obstacle 6 6 --expect-no-dead-end
```

## 2.11 footprint 换 0.62 方形：1.2–1.3 m 走廊真能过去（S3d，2026-11-25）

**问题**：§2.10 把"1.2–1.3 m 走廊"按死路处理，根因不是感知也不是 inflation，是 **footprint**：
半径 0.60 的圆（inscribed 0.60）要求 1.2 m 净宽，走廊里路径净空只剩 **0.05 m** < 判死路阈值 0.20，
MPPI 只能原地磨。把 footprint 换成**真实车体 0.5×0.5 + 每侧 0.06 m 安全边 = 0.62 方形**
（inscribed 0.31 / circumscribed 0.438）后，同一场景路径净空回到 **0.30–0.32 m**，13.8–15.1 s 到点。
inflation(1.5) / CostCritic / MPPI 参数全部没动。

**为什么不是纯 0.5 方形**：纯 0.5（inscribed 0.25）也能过走廊（净空 0.35），但会**放行 0.8 m 缝**，
而且开阔路线擦墙角时净空 0.18 m 会被判死路（误判）。加 0.06 m 安全边把两边都拉开。

**边界是换算出来的，不是拍脑袋**：判死路阈值仍是 0.20 m（`path_guard.py` 未改），
它量的"到内切区边界距离"随 inscribed 半径平移，所以**能过的缝宽 = 2×(0.31+0.20) = 1.02 m**。
三档实测（两块 wall 夹出精确缝宽，每档重启栈）：

| 缝宽 | 路径最小净空 | 结果 |
|---|---|---|
| 0.9 m | 0.13 m | 0.2 s 判死路、取消后位移 0.00 m |
| 1.0 m | 0.18 m | 0.4 s 判死路 |
| 1.1 m | 0.20 m（正好压线） | 不判、`SUCCEEDED` 11.8 s、面距 +0.293 m |
| 1.2–1.3 m（敌车挡路） | 0.30–0.32 m | `SUCCEEDED` 13.8–15.1 s、面距 +0.40 m |

```bash
# 应该走得通（guard 照常生效，判据是"不误判+到点+不撞"）
python3 tools/verify_dead_end.py --goal 0 2.6 --obstacle 0 1.2 --spawn --spawn-after 1.5 --remove --expect-pass
# 应该判死路（0.75 m 缝：两块 1.25 m wall 在 ±1.0）
python3 tools/spawn_obstacle.py spawn -1.0 1.2 --size 1.25 --name wall_l
python3 tools/spawn_obstacle.py spawn 1.0 1.2 --size 1.25 --name wall_r
python3 tools/verify_dead_end.py --goal 0 2.6 --obstacle -1.0 1.2 --obstacle-size 1.25 --timeout 60
```

实测：走廊 `verify_detour` 5/5（到点 0.188 m、面距 +0.402 m、横向偏移 0.919 m、最长停滞 0.3 s、14.2 s）；
开阔 (0,0)→(3,−2.5) `SUCCEEDED` 13.9 s、净空 0.46 m、不误判；5 点巡逻仍 5/5（114.7 s、最小离障 1.095 m）。

## 2.12 动力学敌车（`tools/enemy_drive.py`，2026-11-25）

之前"会动的敌车"都是 `set_pose` 瞬移（`moving_obstacle.py` / `wander_obstacle.py` / `enemy_teleop.py`）：
落点精确、但不产生物理接触，也测不了速度/惯性。现在 `spawn_obstacle.py --dynamic` 生成的模型
带 `<inertial>` + **gz-sim-velocity-control-system**，接受 `/model/<name>/cmd_vel`（`gz.msgs.Twist`），
由物理积分、与我们的车真实碰撞：

```bash
# 生成 + 0.5 m/s 直行 2.5 s（结束后自动停住；--remove 顺带删除）
python3 tools/enemy_drive.py --spawn -1.0 1.2 --yaw 0 --linear 0.5 --duration 2.5 --remove
# 只驱动已存在的敌车（可加 --angular）
python3 tools/enemy_drive.py --linear 0.4 --angular 0.5 --duration 2.5
```

自检判据（对 `/world/<w>/pose/info` 采样，1 s 窗口测速、逐帧查瞬移）：

| 指令 | 1 s 窗口速度（中位） | 逐帧最大 | 位移 |
|---|---|---|---|
| 0.5 m/s × 2.5 s | 0.468 m/s | 0.491 m/s（无瞬移尖峰） | 1.12 m |
| 0.3 m/s × 3.0 s | 0.291 m/s | 0.291 m/s | 0.81 m |
| 0.4 m/s 撞向静止哨兵（yaw −90°） | 0.382 m/s | 0.391 m/s | 哨兵被推 **0.241 m**（真接触） |

两个工具的取舍：**要精确落点用 `moving_obstacle.py`（set_pose），要动力学/接触/可测速用
`enemy_drive.py`（velocity control）**。

## 2.13 前视相机 + 装甲板检测 + 让行（S3e，2026-11-26）

**为什么现在才加相机**：前面所有导航验收都只用 Mid-360（点云→/scan）和低位安全雷达，没有图像。
装甲板是比赛要素、必须有视觉；但 llvmpipe 下多一路 RGB 渲染会压 RTF，所以做成**开关**：

```bash
./run_sentry_nav.sh gui:=false camera:=true        # 默认 false：不开就不渲染、不桥接
```

相机规格：`sentry.urdf.xacro` 的 `camera` 参数，480×360 @10 Hz、hfov 1.2 rad，装 base_link
`(0.10, 0, 0.20)`（离地 0.29 m，略低于 Mid-360 的 0.30 m）、下俯 0.08 rad；只在这个开关打开时
桥接 `/camera/image` + `/camera/camera_info`。实测图像 **5–6 Hz**（目标 10 Hz，渲染跟不上）、
`fx=350.8`。

**装甲板**：`spawn_obstacle.py --armor red|blue` 给敌车四面加 RM 小装甲板 135×55×6 mm
（离地 0.15 m、纯视觉无碰撞），车体同时压成深灰以拉开 HSV 对比。

**检测**：`tools/armor_detect.py` —— 纯 HSV 阈值（红两段/蓝一段）+ 轮廓长宽比过滤，输出
`/armor/detections`（JSON）：颜色、像素框、`bearing_rad`、`distance_m = fx*0.135/w_px`。
符号实测：**目标在 +y 时落在图像左侧** ⇒ 世界方位 = −相机方位。
仿真里是纯色块所以 HSV 够用；实车要换成装甲板数字识别，不在本轮范围。

**验收**：`tools/verify_armor.py`（自己起检测节点、按真值摆位、每例采 2 s 取中位）

| 用例 | 结果 |
|---|---|
| 正对 1.0 / 2.0 / 3.0 m | 距离误差 **0.020 / 0.006 / 0.066 m**（容差 max(0.20, 8%)），方位误差 0.000 |
| 横移 ±0.5 m @2.0 m | 方位误差 **0.0322 rad**（1.8°） |
| 蓝色装甲板 @2.0 m | 距离误差 0.006 m、颜色正确 |
| 检出率 | 每个用例 **1.00**（13–15 帧全检出） → 21/21 判据过 |

**让行**（`tools/verify_yield.py`；任务层策略，和 `path_guard` 同因：取消/重发都要发起方的
goal handle）：
①盯 `/scan_safety` 前扇区 ±20° 最小量程 + `path_guard` 的路径净空；
②前方 <1.5 m 有障碍**且**（路径净空 <0.20 m **或** 规划器 3 s 给不出路径）→ 取消目标、退回 1.0 m；
③等路通（本验收用"删模型"模拟敌车离开）→ 2 s 后重发原目标，失败可重试（≤3 次）。

实测（1.2 m 缝在 (0,1.2) 被敌车堵住、goal (0,2.6)）：**触发（前方 1.00 m、无路径）→ 退 0.93 m →
等待 → RESUME → SUCCEEDED 到点 0.06 m**，全程 17.6 s、与敌车最小面距 +0.750 m，**5/5 判据过**。
对比"死路→取消→停住"：让行多一步"等 + 恢复"，不把任务丢在地上。

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
| 低位安全雷达（S3a） | `/scan_safety` **10.0 Hz / 360 束 / 0.05–2.5 m**；空旷处最小有限量程 1.500 m（墙）、**无自身回波**；frame_id `base_footprint`（5/5 项体检过），RTF 0.98–1.00 |
| 盲区对比（S3a，敌车 0.4×0.4×0.25 摆到车头 1.2 m） | `/scan` 最小有限量程 **1.426 m**（只有墙，看不见敌车）vs `/scan_safety` **1.000 m**（= 敌车正面）；两张 costmap 在敌车处 99/100 |
| 贴脸场景 A/B（S3a，goal (0,2.6)、敌车 (0,1.2) 1.5 s 后出现，同一条命令） | 雷达**关**：`SUCCEEDED` 到点 0.184 m、**面距 −0.000 m（撞上）**、90.6 s；雷达**开**：**面距 +0.479 m**、到点失败（原因见 §9） |
| 开阔场地绕行（S3a，雷达开） | (0,0)→(3,−2.5)、敌车 (1.5,−1.25)：**5/5 判据过**，到点 0.168/0.109 m，面距 +1.06/+0.94 m，0.1 s 停滞，9.4/18.6 s |
| 5 Hz 重规划（S3c） | `/plan` 平均 **4.48 Hz**（默认 BT 是 1 Hz）；绕障回归 5/5 过、14.2/15.6 s、峰值 0.96/1.16 m/s |
| 缝窄判死路（S3c） | 敌车卡 1.3 m 走廊、goal 在它后面：**0.8 s 判出死路**（路径最小净空 0.05 m < 0.20），取消后位移 **0.00 m**，面距 **+0.665 m**（不撞、不磨） |
| 死路误判反向回归（S3c） | 开阔 goal (3,−2.5)：不判定，`SUCCEEDED` 0.16 m，14.6 s，全程路径净空最小 0.43 m |
| **5 点巡逻长跑回归（S3a+c 后）** | **5/5 SUCCEEDED**，到点 0.017–0.190 m、最小离障 **1.083 m**、碰撞 0、RTF 0.839、总 126.3 s；`Failed to create plan` **20 次（瞬态，下一周期即恢复）** |
| 5 Hz 归因对照（同一图跑 1 Hz BT） | 5/5 SUCCEEDED、总 116.7 s、leg4 **74.5 s**（5 Hz 时 77.1 s）、`Failed to create plan` **11 次** ⇒ 慢腿与瞬态规划失败是**既有现象**，不是 5 Hz 带来的 |
| 帧对齐校验（S2c） | 注入 (0,0,0)/(+0.30,−0.20,+3°)/(−0.15,+0.25,−5°) → 恢复误差 ≤2.3 mm / ≤0.063°，单次 4.4–4.7 s |
| **走廊通过（S3d，2026-11-25）** | 敌车 (0,1.2)、goal (0,2.6)：`SUCCEEDED` **13.8 / 14.2 / 15.1 s**，路径净空 **0.30–0.32 m**，面距 **+0.40~+0.42 m**；`verify_detour` **5/5**（到点 0.188 m、横向偏移 0.919 m、最长停滞 0.3 s） |
| 缝宽边界三档（S3d） | 0.9 m 净空 0.13 → 判死路；1.0 m 净空 0.18 → 判死路；1.1 m 净空 0.20 → 通过 11.8 s；边界 = **2×(0.31+0.20) = 1.02 m**（解析与实测一致） |
| 开阔不误判（S3d） | (0,0)→(3,−2.5)：`SUCCEEDED` 13.9 s，路径净空 **0.46 m**，guard 不触发 |
| 5 点巡逻（S3d 后） | **5/5 SUCCEEDED**，113.6–121.5 s，到点 ≤0.191 m，最小离障 **1.021–1.095 m**，碰撞 0，RTF 0.85 |
| 巡逻遥测（2026-11-25 新增字段） | leg4 是**最长腿**不是"慢腿"：plan 15.0 m、实走 16.9 m；单位里程耗时与其它腿一致（0.26 vs 0.27–0.44 m/s），零停顿、`Failed to create plan` 0–3 次瞬态 |
| 动力学敌车（S3d） | 指令 0.5/0.3/0.4 m/s → 1 s 窗口测速 **0.468/0.291/0.382 m/s**，逐帧无瞬移尖峰；撞静止哨兵把车**推 0.241 m**（真物理接触） |
| 前视相机（S3e） | `camera:=true` 时 `/camera/image` **5–6 Hz**、480×360、`fx=350.8`；默认关（不渲染、不桥接） |
| 装甲板检测（S3e） | 1/2/3 m 距离误差 **0.020 / 0.006 / 0.066 m**、方位误差 ≤**0.032 rad**、颜色正确、检出率 **1.00**（21/21 判据） |
| 让行（S3e） | 1.2 m 缝被敌车堵住：触发（前方 1.00 m、无路径）→ **退 0.93 m** → RESUME → **SUCCEEDED 0.06 m**，17.6 s，面距 +0.750 m（**5/5**） |

验证报告（本轮新增，全部在 `results/`）：`report_patrol_5hz.json`、`report_patrol_1hz_control.json`、
`report_dead_end_narrow_pass.json`、`report_dead_end_open_pass.json`、`report_detour_narrow_lidar_{on,off}.json`、
`report_detour_regress_{static,spawnafter}_5hz.json`；2026-11-25 新增
`report_dead_end_{180643,180913,181234,181238,181903,182016,182045,182643,191704,191740,191829}.json`
（footprint 前后 / 缝宽三档）、`report_detour_182155.json`、`report_sentry_patrol.json`（含每腿遥测）；
历史报告 `report_map_sdf.json`、`align_case{1,2,3}.json`；证据日志在 WSL `/tmp/sentry_nav/nav.log`。

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
tools/                   # 地图/SDF 复核、巡逻点挑选、巡逻执行（含每腿遥测）、发目标、点地图窗口（map_click.py）、
                         # 敌方机器人障碍（spawn_obstacle.py，可 --armor 加装甲板）、移动/漫游敌车
                         # （moving_obstacle.py / wander_obstacle.py）、动力学敌车（enemy_drive.py）、
                         # 键盘遥控（teleop_gui.py）、绕障验收（verify_detour.py）、判死路验收
                         # （verify_dead_end.py）、装甲板检测（armor_detect.py / verify_armor.py）、
                         # 让行验收（verify_yield.py）
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
18. **gz 传感器的 frame_id 是作用域名**（`<模型>/<链接>/<传感器>`），ros_gz_bridge 原样搬进
    `header.frame_id`；TF 里没有这名字时 costmap 只打一句 `Message Filter dropping message: frame ...`，
    然后整帧丢掉 —— 别去查传感器，先看 frame。修法两种：加 relay 改帧名（本仓库做法），
    或者给这个怪名字补一条 static TF（不推荐，`/` 在帧名里会污染 RViz/其他工具）。
19. **清场规则要覆盖新加的节点**：`run_sentry_nav.sh stop` 按 PATTERN 杀进程，新增
    `safety_scan_relay` 后忘了加进 PATTERN → 每次重启漏一个孤儿进程（实测攒到 13 个、各吃 40% CPU，
    load 13、RTF 1.0→0.48）。症状不是"报错"，而是**车变慢、爬不动、每 15 s 一次 Failed to make
    progress**，非常像调参问题。重启次数多了先 `pgrep -af safety_scan_relay` 数一数。
20. **验收脚本自己也算"被测对象"**：`verify_detour.py` 等 TF 原来写"200 次 spin"，WSL 上消息密集时
    200 次不到 1 s 就转完 → 开局偶发"拿不到 map->base_footprint"；改为按墙钟等 20 s。
    `/navigate_to_pose` 的 `wait_for_server` 也从 15 s 放宽到 30 s（重启后 DDS 发现偶发要十几秒，
    必要时先 `run_sentry_nav.sh stop` 再 `rm -f /dev/shm/fastrtps_*`）。
21. **换 BT XML 要三件齐**：XML 文件进 `install(DIRECTORY ... bt)`、launch 里用
    `default_nav_to_pose_bt_xml` 传真名、然后**用量测确认**（`ros2 topic hz /plan` 应从 1 Hz 变
    5 Hz，实测 4.48 Hz）。少任何一件都不报错，只是安静地还是 1 Hz。
    `RateController` 之外别忘 `ComputePathToPose` 外面还要包 `RecoveryNode`，否则规划失败时没有清图重试。
22. **判死路的阈值要用"发布尺度"**：nav2 发布的 `OccupancyGrid` 里 100=致命、99=内切、
    其余是 `cost*100/255` —— 拿原始 costmap 的 253/254 当阈值会永远判不出来（第一版就是这样，
    全程 `max_cost=100` 而阈值写 200）。本项目改成"对内切区做距离变换取净空(m)"，可读且好标定。
23. **footprint 的 inscribed 半径就是"能不能过"的总闸，判死路阈值必须跟着换口径**：半径 0.6 的圆
    换成 0.62 方形后，同一条 1.2–1.3 m 走廊的路径净空 0.05 → 0.30 m；阈值 0.20 的物理含义也
    从"缝宽 ≥1.2 m"变成"缝宽 ≥1.02 m = 2×(0.31+0.20)"。只改 footprint 不重算阈值 = 两种行为
    同时错（该过的过不去、该拒的放行）。
24. **gz 的 `/world/<w>/remove` 必须给 `type=MODEL`**：只填 `name` 时服务返回 `true` 但**模型没删**
    （2026-11-25 踩：`/scan_safety` 仍能量到 1.000 m、`gz model --list` 还在，代价地图留一块
    "幽灵障碍"，后续用例被污染）。删完必须用 `gz model --list` 回读校验（`spawn_obstacle.py`
    已内置 `verified_gone`）。
25. **动力学敌车**：模型 SDF 要带 `<inertial>` + `gz-sim-velocity-control-system` 插件才吃
    `/model/<name>/cmd_vel`。测速别用"到达时刻"算 dt —— gz 传输会批量到达，dt 可能 ~0，
    逐帧速度会算出 83 m/s 的假尖峰；用 `pose/info` 消息头里的**仿真时间**、并跳过起步加速段（0.6 s）。
26. **相机的 `camera_info` 必须是 VOLATILE/RELIABLE**：用 `TRANSIENT_LOCAL` 订阅 ros_gz_bridge 的
    `/camera/camera_info` 会报 `offering incompatible QoS (DURABILITY)`，然后一条都收不到、`fx` 为
    None，检测节点安静地一条都报不出来（2026-11-25 踩）。
27. **rclpy 的 goal handle `get_result_async()` 只能取一次**：重复调用时，若 action server 的结果
    响应已被消费，后来的 future 永远不 resolve —— 车上早就到点了，客户端 `done()` 一直是 False，
    白等 95 s 才靠收尾兜底拿到结果。做法：发目标时取一次 future 存下来，之后只轮询它。
28. **视觉件不要给碰撞**：装甲板是纯 `<visual>`（135×55×6 mm、离地 0.15 m），车体上加碰撞会让
    规划器把"贴脸"当不可通行；敌车本体碰撞仍是 0.4×0.4×0.25 的 box。

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
- 临时障碍测试：**静态敌车**与**慢速横穿敌车**（0.3–0.4 m/s）已验收；**动力学敌车**已可用
  （`enemy_drive.py`，§2.12），但**追击/迎面拦截**的战术行为没做（敌车不会决策，只会按指令速度走）。
- ~~贴脸场景"不撞了但到不了点"（S3a 遗留）~~ **已解决（S3d，2026-11-25）**：根因不是感知也不是
  inflation，是 footprint 半径 0.6 的圆（inscribed 0.6）把 1.2–1.3 m 走廊的路径净空压到 0.05 m。
  换成 0.62 方形（车体 0.5 + 0.06 安全边）后同场景净空 0.30–0.32 m、13.8–15.1 s 到点、面距 +0.40 m；
  窄缝仍由 `path_guard` 的 0.20 m 阈值判死路（等价缝宽 1.02 m）。详见 §2.11。
- **死路判定在客户端**：`ros2 action send_goal` 直接发的目标没有客户端盯路径 → 不会自动取消。
  要用这个保护就走点地图窗口或 `verify_dead_end.py`。
- 比赛要素：**敌方车（动力学）、装甲板检测（HSV 纯色板）、让行**已做（§2.12/§2.13）；
  **装甲板数字识别 / 真实视觉链路**未做（仿真里是纯色块，实车要换检测器）；
  追击/拦截、与裁判系统交互（装甲板受击判定、补弹/复活）也没做。