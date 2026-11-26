# 项目进度

> 唯一“现在进行时”。每次会话结束更新；历史细节放 `docs/issues/` 与 `docs/decisions/`。
> 最后更新：2026-11-26（footprint 0.62 收口 + 幽灵障碍根因 + 动力学敌车 + 比赛要素：相机/装甲板/让行；
> ①②③④ 四项遗留全部闭环，改后回归全过；**遗留：敌车"发速度但不动"未解**）

## 当前接续（优先于下方 10 月 26 日的历史状态）

> **🆕 2026-11-26（footprint 收口 + 缺陷根因 + 比赛要素）**：详见
> [2026-11-26-sentry-footprint-dynamics-competition.md](docs/handoffs/2026-11-26-sentry-footprint-dynamics-competition.md)。
> ①**footprint 半径 0.6 圆 → 0.62 方形**（车体 0.5 + 每侧 0.06 安全边，inscribed 0.31）：
> 1.2–1.3 m 走廊路径净空 0.05 → **0.30–0.32 m**，13.8–15.1 s 到点、面距 +0.40 m；inflation/MPPI 未动。
> 判死路阈值仍是 0.20 m，但换算成**能过的缝宽 = 2×(0.31+0.20) = 1.02 m**（0.9/1.0 m 拒、1.1 m 过）。
> ②**幽灵障碍根因**：`/world/<w>/remove` 只给 name 不给 `type=MODEL` 时返回 true 却不删模型
> （雷达仍量到 1.000 m、代价地图持续重打标记）；修后敌车区域 >=99 格 814 → 0。
> ③**动力学敌车**：`--dynamic` 加 `<inertial>` + VelocityControl；新 `tools/enemy_drive.py`
> （0.5/0.3 m/s 实测 0.468/0.291 m/s、无瞬移尖峰；撞静止哨兵推开 0.241 m）。
> ④**S3e 比赛要素**：`camera:=true` 前视相机（480×360@5–6 Hz）+ `armor_detect.py`（HSV，
> `/armor/detections`）→ `verify_armor.py` **21/21**（1/2/3 m 距离误差 0.020/0.006/0.066 m）；
> `verify_yield.py` 让行 **5/5**（前方 1.00 m 无路径 → 退 0.93 m → 恢复 → 到点 0.06 m，17.6 s）。
> ⑤**回归全过**：贴脸 A/B（雷达关面距 −0.000 撞上 / 雷达开 +0.433、5/5、9.8 s）、动态敌车横穿 5/5、
> 点地图 selftest 0.146 m、5 点巡逻 **5/5**（116.7 s、RTF 0.943、最小离障 1.043 m）。
> ⑥**遗留（停在这里）**：GUI 栈里敌车"发了 cmd_vel 但位移 0.01 m"——话题存在、订阅者在、900 s 稳定发布；
> 普通动态车同样不动（排除装甲板）；怀疑物理体休眠（SetLinearVelocity 不唤醒）。
> **下次进入点**：`cd ~/sentry_ws && DISPLAY=:0 ./run_sentry_nav.sh gui:=true map_click:=true`
> → `python3 tools/verify_dead_end.py ... --expect-pass` / `verify_armor.py` / `verify_yield.py` / `patrol_s2a.py`。
> **未做**：Nav2 Humble 参数表（等 Humble 环境）、S100 实车（等板子/校园网）、装甲板数字识别、敌车追击/拦截、
> 让行接进 `map_click.py`。

> **🆕 2026-11-24 晚（S3a = a 完成，下一步 c）**：给哨兵仿真补上"低位 2D 安全雷达"，把
> "敌车 0.25 m 高、Mid-360 在 0.30 m 看不见"的盲区堵住。详见
> [2026-11-24-sentry-safety-lidar.md](docs/handoffs/2026-11-24-sentry-safety-lidar.md)。
> ①**改动**：`sentry.urdf.xacro` 给 base_link 加 gpu_lidar（离地 **0.18 m**、360°/360 线/10 Hz/
> 0.05–2.5 m，gz 话题 `/scan_safety_raw`）→ `safety_scan_relay.py` 把 gz 作用域名
> `sentry/base_footprint/safety_lidar` 改成 `base_footprint`（不改的话 costmap 只报
> `Message Filter dropping message` 丢整帧）→ 两张 costmap 的 obstacle_layer 与 collision_monitor
> 都接 `scan_safety`。**为什么不是"离地 0.10 m"**：底盘占 0.03–0.15 m，0.10 m 会被自己壳体挡住，
> 360° 全是自身回波 → NoTouch 永久停。
> ②**盲区实测**：敌车摆到车头 1.2 m，`/scan` 最小有限量程仍是 **1.426 m**（只有墙），
> `/scan_safety` **1.000 m**（= 敌车正面）；两张 costmap 在敌车处 99/100。
> ③**A/B（同一条命令，goal (0,2.6)、敌车 (0,1.2) 1.5 s 后出现）**：雷达**关** → `SUCCEEDED` 0.184 m、
> **面距 −0.000 m（面贴面）**、90.6 s；雷达**开** → **面距 +0.479 m**、但到点失败 2.427 m、202.9 s。
> 开阔场地（敌车 (1.5,−1.25)）**5/5 判据过**（到点 0.109/0.168 m、面距 +0.94/+1.06 m、停滞 0.1 s）。
> ④**踩到的两个事故**：`run_sentry_nav.sh` 清场 PATTERN 漏了新节点 → 重启攒了 13 个
> `safety_scan_relay` 孤儿（各 40% CPU、load 13、RTF 1.0→0.48），症状是"车变慢爬不动"而非报错；
> `safety_lidar:=false` 时 collision_monitor 报 `invalid source` 把车冻 216 s → relay 改为超 0.5 s
> 无数据就发空扫描兜底。⑤**顺带**：`FootprintApproach.time_before_collision` 1.0→0.4 s（1.6 m 前瞻
> 让车全程在 approach/continue 之间切，压到 0.05–0.2 m/s）；`verify_detour.py` 等 TF 改墙钟、
> 新增 `--dump` 轨迹；新增 `tools/check_scan_safety.py`（5 项体检全过）。
> ⑥**未解（下一步 c）**：贴脸场景"不撞了但到不了点"——敌车两侧净空各 1.2–1.3 m，footprint 0.6 的
> inscribed 半径也是 0.6，走廊里非致命格只剩 ~0.1 m，MPPI CostCritic(253/300) 让前向轨迹都接近临界 →
> 小幅转/爬 200 s、10–11 次 Failed to make progress。**无障同目标 9.6 s 到达**、全局 inflation
> 1.5→0.9 实测无效 ⇒ 不是感知/inflation，是 c 的活：缝宽判死路（过不去就快速停）+ 5 Hz 重规划。
> ⑦**c 同日完成**：`bt/sentry_navigate_to_pose.xml` 把 `RateController hz` 1.0→**5.0**（launch 用
> `default_nav_to_pose_bt_xml` 注入），实测 `/plan` **4.48 Hz**；`tools/path_guard.py` 判死路 ——
> 把全局代价地图 `>=99`（99=内切/100=致命）当障碍做距离变换，取 `/plan`（去掉起点 0.6 m、终点
> 附近 0.4 m）最小净空，<**0.20 m** 连续 2 帧即判"过不去"；开阔路径实测 **0.43–0.65 m**、窄缝
> **0.05 m**（3 倍余量）。判定在**客户端**（`map_click.py` / `tools/verify_dead_end.py`）——ROS 2
> 没有"按条件取消别人目标"的公开 API，取消要用发起方的 goal handle；`ros2 action send_goal` 直发的
> 目标不享受该保护。**实测**：窄缝 **0.8 s 判定** → `CANCELED`、取消后位移 **0.00 m**、面距 **+0.665 m**
> （不撞也不磨）；开阔反向回归不误判、`SUCCEEDED` 14.6 s；5 Hz 后绕障回归两例各 **5/5**（14.2/15.6 s、
> 峰值 0.96/1.16 m/s）。坑：nav2 发布的 OccupancyGrid 是 **0–100 尺度**，第一版拿 253/254 当阈值
> 全程只看到 100、永远判不出来 → 改成"对内切区做距离变换取净空(m)"。
> ⑧**5 点巡逻长跑回归（2026-11-25 补跑）**：`patrol_s2a.py` → **5/5 SUCCEEDED**，到点 0.017–0.190 m、
> 最小离障 **1.083 m**、碰撞 0、RTF 0.839、总 126.3 s、0 次 stop。`Failed to create plan` 20 次全为瞬态
> （下一周期恢复），另把 BT 临时改回 1 Hz 对照跑同图：5/5、总 116.7 s、leg4 74.5 s（5 Hz 77.1 s）、
> 失败 11 次 ⇒ 慢腿与瞬态规划失败是**既有现象**，不是 5 Hz 引入的（失败点静态净空 1.45–1.61 m）。
> ⑨**下次进入点**：`cd ~/sentry_ws && ./run_sentry_nav.sh gui:=false`（默认带雷达 + 5 Hz BT）→
> `python3 tools/check_scan_safety.py --frames 8`（5/5）→
> `python3 tools/verify_dead_end.py --goal 0 2.6 --obstacle 0 1.2 --spawn --spawn-after 1.5 --remove`
> （0.8 s 判死路）→ `--expect-no-dead-end` 反向回归 → `python3 tools/patrol_s2a.py`（5/5 长跑）。
> **未做**：巡逻里的瞬态规划失败遥测/退避、巡逻 leg4 慢（~75 s，1 Hz 也慢）的定位、
> 1.2–1.3 m 走廊"真的过去"（要动 footprint，属调参）、敌车动力学化。

> **🆕 2026-11-23 晚（哨兵仿真 S3 收尾，下一步做 a+c）**：把 sentry_sim 做成可交互 + 调导航，并把用户报的
> "敌车突然出现撞上去 / 卡缝里挤过去"根因定位到**感知盲区**。详见
> [2026-11-23-sentry-sim-nav-tuning-and-limits.md](docs/handoffs/2026-11-23-sentry-sim-nav-tuning-and-limits.md)。
> ①**交互**：点地图开哨兵（左键点→自动走，到点误差 0.163–0.193 m）；遥控敌车窗口 `tools/enemy_teleop.py`
> 3 s 把敌车从 (3.0,−2.5) 开到 (4.32,−2.5)；随机漫游敌车 `wander_obstacle.py` 保留。
> ②**卡墙自愈**：车停在净空 0.35 m 处（< robot_radius 0.60）时全局规划器把起点判致命区（实测 42 次
> `Failed to create plan`）→ 落点净空 0.65→**0.85 m**，面板加**自动脱困**（净空梯度倒车后重发目标，
> 实测 clear 0.35→0.80 后到点 0.23 m）。③**不再断链**：裸模式不再 lifecycle deactivate
> `collision_monitor`（那会断 bond、整套导航下线、车全不动），改动态参数 `FootprintApproach.enabled`。
> ④**不钻缝**：`robot_radius 0.40→0.60`（后为半径 0.60 圆形 footprint）+ 全局 inflation **1.5 m**/5 Hz；
> 静态敌车拦路面距 **+0.236 m**、移动敌车横穿 5/5 判据（12.3 s、峰值 1.38 m/s）。⑤**提速 2.7x**：MPPI 总闸是
> `temperature`（0.3→**0.05**）+ `time_steps 56→40`/`batch 2000→1000`，同路线 **22.3 s→8.2 s**、峰值 1.35 m/s。
> ⑥**未解（下一步 a+c）**："敌车半路突然出现在车前"仍会接触 —— 实测最小中心距 **0.450 m（面贴面）**、
> 停滞 5.1 s；根因是**感知盲区**：敌车高 0.25 m、雷达高 0.30 m 且 `lidar_slices:=1` 稀疏单帧，1 m 内扫不到。
> **a** = 加低位 2D 安全雷达（z≈0.10、360°、0.05–2.5 m）桥 `/scan_safety` 喂两张 costmap + collision_monitor；
> **c** = 自定义 BT 重规划提到 5 Hz + 缝宽 <1.2 m 判死路（过不去就停）。
> ⑦**注意**：别在 Gazebo GUI 里插模型（llvmpipe 重绘 → `gz sim gui` 349% CPU、RTF 0.10），用
> `tools/spawn_obstacle.py`（server 侧 static）。
> ⑧**下次进入点**：`cd ~/sentry_ws && DISPLAY=:0 ./run_sentry_nav.sh gui:=true map_click:=true`；
> 复现用例 `python3 tools/verify_detour.py --goal 0 2.6 --obstacle 0 1.2 --spawn --spawn-after 1.5`。
> ⑨**提交**：`8281e49`（卡墙自愈）、`9d2406f`（双人模式）、`b647ffc`（footprint/inflation/NoTouch）。

> **🆕 2026-11-22/23（副线：RM 哨兵仿真，新栈）**：在没有实车、没有地图的条件下，按哨兵真实技术路线
> 搭出可跑可验收的仿真栈，源码落在 [tb3_gazebo_nav2/sentry_sim](tb3_gazebo_nav2/sentry_sim/README.md)，
> 计划/结论见 [2026-11-22-sentry-simulation.md](docs/superpowers/plans/2026-11-22-sentry-simulation.md)。
> ①**架构决定**（用户 11-22 定）：定位主力 **Odin1**（实车官方驱动/仿真接口替身），**不跑 FAST-LIO**（算力留 Nav2）；
> 实车 **RDK S100 + Odin1 + Mid-360**，IMU 用 Mid-360 内置 6 轴 200 Hz；**2D 栅格图用场地 CAD 先验**，不现场建图。
> ②**传感器**：`livox_gz_sim` 提供 Mid-360（Harmonic，rtf 0.91–0.96）；`livox_ros_driver2` 官方包 Jazzy 要 sudo 装 SDK2 →
> 用**消息-only 替身包**；FAST-LIO ROS2 已编译验证（静止 60 s 漂移 11.6 mm / yaw 2.24 mrad）但**不启用**，存档待用。
> ③**车与导航**：4 麦轮哨兵车（0.5×0.5×0.12，规则上限 600×600×600/28 kg；每轮 12 个 45° 被动滚子），
> 全向运动误差 0.38%/4.98%/4.08%；Nav2 **不启 AMCL**、`map→odom` 由 Odin1 替身发、标准 TF 链无多父；
> 单点 (3,−2.5) 0.197 m、(0,3) 0.196 m；**5 点巡逻 5/5 SUCCEEDED**（误差 ≤0.197 m、最小离障 0.371 m、碰撞 0）。
> ④**地图**：复用 rm_field 翻转后 3V3 场地 CAD 图，复核**逐格一致**（0/180224 格不同，边界偏差 0.000 cm）；
> 点云现场建图（`pcd_to_pgm`）实测**漏掉低矮平台**（0.2 m 平台仅 2–15% 占据）——雷达高于平台、光线从上方掠过，
> 2D 投影必然空洞 → **判为不用于导航**，工具留作实验。
> ⑤**帧对齐**（实车上线关键）：`odin1_sim_bridge` 可注入 `map_offset_x/y/yaw`；`map_align_check`（粗搜+精修，4.4–4.7 s）
> 注入 (0,0,0)/(+0.30,−0.20,+3°)/(−0.15,+0.25,−5°) 三组恢复误差 **≤2.3 mm / 0.063°**；实车顺序：建图（起点固定）→重定位→
> 拐角处跑校验→用 `correction` 修正。
> ⑥**Humble/S100 准备（11-23）**：在 `ros:humble-ros-base` 容器（Ubuntu 22.04 / Python 3.10，与 S100 同构）
> 实测 **6/6 包编译通过**；修 3 处双兼容：`pointcloud_to_laserscan` CMake 对 `tf2_sensor_msgs`
> 做 TARGET 守卫（Humble 是纯头文件、无 CMake 目标）+ 反向节点改可选（默认 OFF，**彻底去掉 laser_geometry**）、
> `map_align_check` 的 `backend=auto` 无 scipy 时崩（实车会中招）已修、两个 python 包补 `ament_python`。
> 迁移文档（S100 适配 10 项、上车 6 阶段判据、Odin1 大场地图测试方案）：[2026-11-23-sentry-s100-migration.md](docs/handoffs/2026-11-23-sentry-s100-migration.md)。
> **仍缺**：Nav2 参数表按 Humble 重写（Jazzy 专属 8 字段：`navigators`/`route_server`/`docking_server` 等；Humble 用 `plugin_lib_names`），需能装 nav2 的 Humble 环境实测。
> ⑦**下次进入点**：`cd ~/sentry_ws && ./run_sentry_nav.sh gui:=true`（`stop` 停；`map_offset_*:=` 注入偏移）；
> `ros2 run map_align_check map_align_check --sim-time --frames 5`；source of truth 在仓库 `sentry_sim/`，改完 rsync 进 WSL。
> ⑧**未做**：Nav2 Humble 参数实测、Odin1 大场地图实测（等 S100/实车）、S3 比赛要素（敌方车/装甲板/让行）。
> ⑨**提交**：`7116699`（栈落库）、`06b1164`（Humble 补丁 + 迁移文档）。

> **✅ 2026-11-21（副线 Gazebo）：两笔欠账收口——①清场按 ROS domain 隔离（修跨 domain 误杀 / 误删 FastRTPS 共享内存）；②动态箱 three 档（slow/fast/faster 0.3/0.6/1.0 m/s）+ 0.62 m 超窄门的 2.5 cm 栅格变体。**
> ①**A 清场**：新增 `domain_cleanup.sh`——进程按 `/proc/<pid>/environ` 的 `ROS_DOMAIN_ID` 过滤；共享内存只删没人 mmap/flock/fd 占用的段（`sem.*` 一律保留；端口 `X`/`X_el` 成对保护）。
> 复现旧 bug（旧版 `stop_nav2_field.sh` 误杀 domain 43 的假 bridge）→ 新版存活；真栈(42) + domain 43 真 DDS participant 集成 **8 项全过**；`verify_cleanup_domain_scope.sh` **14 项回归全过**。
> `ROS_DOMAIN_ID` 改 `${ROS_DOMAIN_ID:-42}`；现场抢救 `RM_CLEAN_ALL_DOMAINS=1 bash stop_nav2_field.sh`。提交 `8d0ebae`。
> ②**B1 动态箱档位**：config 改 `default_tier` + `tiers`（世界初始位姿/驱动轨迹/验收判据同源）；`verify_obstacles_tiers.sh` 一次跑三档，
> 实测 **0.293 / 0.587 / 0.979 m/s**、行程 0.59 / 0.97 / 1.27 m，**各 15 项判据全过**（报告 `report_nav2_obstacles{,_fast,_faster}.json`）。
> ③**B2 超窄门**：`make_25mm_variant.sh` 生成 2.5 cm 碰撞栅格变体（**178 块** box vs 基线 48；地图自由区 77.706 m² = 解析值，11 项校核全过）；
> `gate_tight` 门洞 **0.62 m**，`nav2_gap_tight.sh --grid 25mm|5cm` + `verify_narrow_gap.py --scenario gate_tight`：
> 2.5 cm 到点 0.081 / 门线 |x| 0.021 / 最小间距 0.288 m；5 cm 到点 0.068 / 0.022 / 0.284 m；**两套各 6 项全过**
> → 结论：0.62 m 量级 **5 cm 栅格就够**，2.5 cm 变体备而不用（留给 <0.55 m / 厘米级贴边）。
> ④**附带**：`make_box_collisions.py --model-dir/--src/--out-json`、`make_nav_map_up.py --boxes/--tag`、`nav2_field.sh` 新增 `NAV2_MAP=`；`stop_nav2_field.sh` 现在连"地图点选窗口"（`map_click.py`）一起停（之前只停 launch，会留一个窗口实例）。
> ⑤**GUI 实操验收（用户）**：`nav_gui.sh` 三窗口（Gazebo/RViz/地图点选）手点导航正常；用户手插 6 个未建图物体（4 箱 + 2 球，最近距车约 0.5 m）后 **AMCL 发散**：静止车估计在 (-5.02, 2.25) → (1.82, 0.90) 之间跳、状态栏 `定位误差 4.84 m`（真值 (-0.10, 0.61)，按 T 真值重定位后仍复发）——属未建图障碍物密度/距离超出 `likelihood_field_prob` 容忍范围，不是 odom 或地图问题；脚本场景（2 静态 + 1 动态）定位 max 0.30~0.41 m 仍稳。
> ⑥**下次进入点**：`bash verify_obstacles_tiers.sh`（三档约 8 min）；`bash make_25mm_variant.sh && bash nav2_gap_tight.sh`（超窄门约 3 min）；GUI 手玩 `bash nav_gui.sh`（障碍别超过 2~3 个、离车 ≥1.5 m）。
> 详见 `docs/handoffs/2026-11-21-domain-scope-cleanup-and-acceptance-tiers.md`。

> **✅ 2026-11-20 深夜五补（副线 Gazebo）："钻过窄缝后定位飘走"根因查清并修好——不是车/雷达/地图对齐坏，而是两块障碍物只在 Gazebo 世界里、不在导航静态地图里，AMCL 把它们的回波当证据，窄缝里真值位姿的似然被压到贴墙假位姿之下 → 漂 2.9 m 且不自愈。修法：AMCL 改 `likelihood_field_prob` + `do_beamskip: true`（2 行，`make_nav2_params.py` 生成）；新增第 4 个专项场景 `gate_deep`（1.0×1.2 箱、门洞 0.8 m）回归。**
> ①**复现**：`gate_deep` 修复前过门时 in-range 束 70%（峰值 90%）是未建图箱体回波；真值位姿 p≈20，
> "贴平台边/墙角"位姿 p≈50~60；估计 16.5 s 起跳、最终 **2.9 m**、粒子 2115→801→2513、nav2 19 次恢复后
> **ABORTED** 停车；停 80 s 不自愈。场地图 180° 点对称（室内格 99.5%）→ 镜像分支几乎同分，所以飘几米。
> ②**修复后**（两跑）：`gate_deep` SUCCEEDED，到点 **0.293 / 0.231 m**，定位 max **0.413 / 0.318 m**
> （门段 0.156 / 0.149）；标准 `gate` 定位 max **0.087 m**、到点 0.075；绕障 15 项 **全过**、
> 全场定位 max **0.315→0.302 m**（判据从"剔除 ≤2 m 样本"改回**全样本 <0.5 m**）。
> ③**新增/改**：`config/avoid_obstacles.json` 的 `gate_deep` 段、`worlds/…_gap_deep.sdf.xacro`、
> `nav2_gap_deep.sh`、`verify_narrow_gap.py`（`--scenario`，新增第 6 项定位一致判据，报告
> `results/report_narrow_gap_deep.json`）、`make_nav2_params.py`（生成 beam-skip 参数）、
> `make_obstacles_world.py`、`verify_nav2_obstacles.py`、`README.md` §20。
> ④**同时修掉**：`make_obstacles_world.py --only braking` 一直返回 1（`check()` 漏 braking 分支，
> 刹车世界入口 `nav2_braking.sh` 第 0 步就退出）；`map_click.py` 退出 `terminate called without an
> active exception` 与 `goal_pose_bridge.py` SIGINT 的 `RCLError: failed to initialize wait set`
> （改 `signal_handler_options=NO`+信号置位+executor 轮询+统一收尾顺序；实测 TERM 后无 traceback）。
> ⑤**下次进入点**：`bash nav2_gap_deep.sh && python3 verify_narrow_gap.py --scenario gate_deep`
> （约 90 s + 启动）；已提交 `8e37ad3`（收尾 docs 提交见 git log）。详见 `docs/handoffs/2026-11-20-amcl-unmapped-obstacle-drift.md`。

> **✅ 2026-11-20 深夜（副线 Gazebo）：远距离导航定位漂移 + 重定位后路径不更新，两个都修了——AMCL 换成"适配本仿真"的参数（12 m 路线最大误差 0.160 m），重定位后自动重发目标让 nav2 重规划（复现测试：不重发会停在 2.59 m 外，重发后到点 0.24 m）。**
> ①**定位**：官方 amcl 的 `laser_max_range:100`（实际雷达 3.5 m）与 `recovery_alpha_*=0`（恢复关闭）
> 是漂移/跟丢主因；新增 `make_nav2_params.py` → `config/nav2_params_rm.yaml`（3.4 / 0.12 / max_beams 180 /
> 粒子 800-3000 / update 0.10 / recovery 0.001,0.10 / alpha 0.1），`nav2_field.sh` 默认用它。
> 实测 `drive_and_track.py`：(0,0)→(4.6,2.1) 约 12 m，误差起点 0.024 / 中位 0.096 / **最大 0.160** / 终点 0.087 m。
> ②**路径不更新**：`test_relocalize_replan.py` 复现——错误 initialpose → 目标 **ABORTED**；用真值重定位后
> 估计虽对，但目标已死、离目标 2.59 m 不动；**取消+重发目标**后到点 0.24 m。
> ③**修复**：`map_click.py` 重定位后自动"取消并重发上一个目标"（ABORTED 保留、SUCCEEDED/手动取消清空），
> 路径按新位姿重算；配合上一轮的 T 键（仿真真值重定位）与状态栏定位误差显示。
> ④新增工具 `drive_and_track.py`（边开边测定位误差）、`test_relocalize_replan.py`（重定位→重规划回归）、
> `make_nav2_params.py`。⑤仍**未提交**。详见 `rm_field/README.md` §19.8。

> **✅ 2026-11-20 深夜（副线 Gazebo）："地图坐标和实际坐标差 4 m"查清——地图/里程计都没问题，是 AMCL 被误触的"右键重定位"设到了错误位置且不会自愈。已改双击确认 + T 键按真值重定位 + 状态栏显示定位误差；现误差 0.037 m。**
> ①**框架**：导航用 **ROS 2 Nav2 标准框架**（AMCL 定位 / NavFn 规划 / MPPI 控制 / costmap / BT），
> 自写只有仿真世界生成、起栈验收脚本、点选窗口、`/goal_pose` 桥。**AMCL** = 粒子滤波定位，
> 激光+地图估计 map 系位姿并发布 `map→odom` 修正，是估计值、可能跟丢。
> ②**体检**：`check_map_alignment.py` 地图 vs 48 方块 **96.5%** 重合、最优平移 ≈0（地图没错）；
> `check_localization.py` odom 误差 **0.07 m**（里程计没错）；AMCL 估计 (2.15,0.42) vs 真值
> (4.93,2.97) → 差 **3.77 m**，`map→odom`=(0.28,5.81,−103°)。
> ③**根因**：日志 4 条 `initialPoseReceived→Setting pose` 全不在机器人位置（上一版 UI 右键重定位误触），
> 且 **AMCL 不会自愈**（5 分钟后仍差 4.8 m）；按真值重定位后 **0.037 m / yaw 1°**。
> ④**修复**：右键重定位改**双击确认**；新增 **T 键=按 Gazebo 真值重定位**；状态栏显示**定位误差**、
> >0.4 m 提示跟丢；`/initialpose` 连发 3 次；`--self-test` 自检通过。
> ⑤新增 `check_localization.py` / `relocalize_to_truth.py` / `check_map_alignment.py`。
> ⑥仍**未提交**。详见 `rm_field/README.md` §19.7。

> **✅ 2026-11-20 深夜（副线 Gazebo）："绕了一大圈"查清——不是规划器绕远，是两处"口袋"被 0.4 m 墙隔开、最短路本来就是 12.5 m；真毛病是点击落点净空 0.18~0.20 m < 车体半径 0.22（nav2 到不了→反复重规划/转圈）。已加"落点吸附 + Shift 探针"。**
> ①**几何**：`box_027` 是 0.40×1.05 m、高 0.27 m 的墙（x[2.30,2.70] y[2.95,4.00]），把平台北侧
> 1 m 通道从中间堵死，地图上没有可过缺口；离线算（净空 ≥0.24）西口袋→东口袋最短 **12.52 m**、
> 出生点→东口袋约 10.7 m，nav2 实际 `/plan` 就是沿东侧通道北上（530 点 6.65 m 一段），
> **长路线是唯一路线**。
> ②**真问题**：点击 (1.743,3.643) 净空 0.20、(2.986,3.671) 净空 0.18（<0.22）→ 目标在致命区，
> nav2 反复重规划/进恢复；另一次 (2.300,3.500) 点在墙里。
> **已修**：`map_click.py` 落点自动吸附到净空 ≥0.26 m 的最近格（≤1.5 m），障碍内直接拒绝；
> 实测 (2.971,3.671)→发 (2.975,3.575)，`Goal succeeded`。
> ③**新发现**：该口袋附近 AMCL 一次漂移 **0.66 m**（nav2 报到达时真值 (3.32,3.51) vs TF (2.91,2.99)）——
> 属定位偏差，待查/可右键重定位。
> ④**工具**：`analyze_gap_route.py`（离线最短路+净空）、`path_len_once.py`（采 /plan 长度）、
> `list_boxes.py`（按区域列方块）、`kill_map_click.sh`；`Shift+左键`=只报坐标+净空。
> ⑤**栈挂过一次**（退出时 nav2 container `Magick SIGSEGV`，Gazebo/RViz 窗口关掉后 launch 退出）→ 已重启；
> 别随手关 Gazebo/RViz 窗口。⑥仍**未提交**。详见 `rm_field/README.md` §19.6。

> **✅ 2026-11-20 深夜（副线 Gazebo）：用户报"RViz 点了没反应 / Gazebo Move To 没反应"已定位并修——RViz 是"当前工具不是 Nav2 Goal"，Gazebo 的 Move To 本来就不是导航命令；新增【地图点选】窗口，真实鼠标点击实测到点 0.11 m。**
> ①**根因**：RViz 启动后当前工具不是 GoalTool（把 GoalTool 排到 Tools 第一也不改变当前工具）——
> 点地图只动视角。Gazebo Sim 右键 `Move To` 是模型编辑（改位姿），不通知 Nav2，且会破坏 odom/AMCL。
> ②**新交互**：`map_click.py`【地图点选】窗口（`nav_gui.sh` 自动起；单独 `bash start_map_click.sh`）：
> **左键=去那里、右键=重定位(/initialpose)、中键=取消**；显示地图+网格+机器人箭头+目标+规划路径。
> ③**实测**：真实鼠标点 → `CLICK go (2.714,-2.443)` → nav2 `Received a goal` → 到点 TF vs 真值 0.05 m、
> 与点击点 **0.11 m**（中途 MPPI `Failed to make progress`+spin 一次，恢复后照常到点，既有偶发）。
> ④RViz 正确用法：**先点工具栏 "Nav2 Goal" 图标**再点地图（`2D Goal Pose` 同理）。
> ⑤仍按用户要求**先不提交**。详见 `rm_field/README.md` §19.5。

> **✅ 2026-11-20 夜（副线 Gazebo）："在地图上点坐标 → 机器人走过去"交互模式打通，9 项判据全过；GUI 下同一链路实测到点偏差 0.17 m。**
> ①**入口**：`bash nav_gui.sh` 一条命令起 Gazebo GUI + RViz（`config/rm_field_click.rviz`，由
> `make_rviz_config.py` 从 nav2 官方配置生成，**2D Goal Pose + Nav2 Goal + 2D Pose Estimate** 三个工具都在）
> + `/goal_pose` 桥；在 RViz 地图上点一下、拖出朝向即可；再点别处 = 换目标（自动取消旧的）。
> 命令行等价 `bash goto_xy.sh X Y [YAW]`；纯脚本验收 `bash nav2_field.sh && bash start_goal_bridge.sh && python3 verify_point_goal.py`。
> ②**验收**（`results/report_point_goal.json`，链路 = RViz 2D Goal Pose 同款 /goal_pose → 桥 → action）：
> 三次点击到点 **0.164 / 0.082 / 0.129 m**、每点击位移 2.98 / 3.99 / 4.66 m；
> 换目标：先点 D(2.5,-2.0)、3 s 后点 E(1.0,2.0) → 最终停在 **E 0.110 m**、离 D 4.28 m；
> 不出界/不掉台/不压平台全过。
> ③**GUI 实测**：RViz 窗口标题核对到 `config/rm_field_click.rviz`、Gazebo Sim 窗口在、桥单实例；
> 发 /goal_pose (4.0,-2.0) → 桥 `GOAL→ACCEPTED→SUCCEEDED`、TF 偏差 **0.17 m**；GUI 下 RTF **0.62**（llvmpipe）。
> ④**踩坑**：脚本连发 3 次点击被桥当成 3 个目标互相取消 → 改"只发一次 + 桥 1 s 同点去抖"；
> 旧桥实例残留（当时 stop 脚本没杀它）导致点击落到旧桥、日志里看不到但车动了 →
> `stop_nav2_field.sh`/`nav2_field.sh` 清场都加 `goal_pose_bridge[.]py`。
> ⑤**提交状态**：仍按用户要求**先不提交**。详见 `rm_field/README.md` §19。

> **✅ 2026-11-20 夜（副线 Gazebo 补测）：安全层刹车验收 6 项判据全过——collision_monitor 在"突然出现的 2.8 m 宽挡板"前 0.24 s 介入、0.79 s 刹到 0.03 m/s、最小间距 0.335 m；围挡/赛规核对收口（规则图是位图无法读，以官方 CAD 为准）。**
> ①**场景**：`worlds/rmul2027_3v3_up_nav2_braking.sdf.xacro`（同一 JSON 的 popup 段生成）：2.8×0.5×0.6 m
> 宽挡板先藏 (3,-3)，机器人跑 (0,0)→(0,3.0)，离 (0,2.2) 剩 0.85 m 时瞬移拦满 3.2 m 走廊（绕不过去）；
> 入口 `bash nav2_braking.sh` + `python3 verify_braking.py` → `results/report_collision_monitor.json`。
> ②**结果**：pop 前车速 **0.452 m/s**；cm state 事件 **2 次**（STOP + LIMIT，首事件 pop 后 **0.24 s**）；
> 首停 **0.791 s**、最低速 **0.028 m/s**；人-挡板最小间距 **0.335 m**（≥0.22）；z 0.010、不出界；
> 目标必然到不了（挡板拦死）→ 停住后主动取消，不算失败。
> ③**口径**："刹住"= 2.5 s 内出现 <0.05 m/s，不要求停住不动——cm STOP 后恢复行为会倒车让开，更安全；
> 第一版触发 0.7 m 时停车间距只有 0.215 m（差 5 mm），抬到 0.85 m 才是"刹得住"的实测点。
> ④**围挡/赛规**：规则手册场地尺寸是位图（文本层只有图题，正文写"以实际场地为准"），
> 本机无 OCR/视觉引擎（modlens: No vision provider；WSL 无 tesseract/pdftoppm）→ 无法逐项对图；
> 场地几何仍以官方 CAD 为准（12.60×8.60、内侧面 ±6.00/±4.00、墙顶 0.40、墙厚 0.05）。
> ⑤**窄通道（补测）**：`worlds/rmul2027_3v3_up_nav2_gap.sdf.xacro`（同一 JSON 的 gate 段）两块 0.6×0.5 m 箱
> 留出 **0.80 m 门洞**（两侧外缝 0.40 < 车宽 0.44 → 绕不过去），出生 (0,-2.0) → 目标 (0,2.6)：
> **5 项判据全过**——SUCCEEDED、到位 0.124 m、门线上 |x| 最大 **0.02 m**（居中过门）、到两箱间距
> **0.379/0.417 m**、z 0.010。结论：**5 cm 栅格对 0.80 m 门洞够用**，不必重生成 2.5 cm；
> 只有 <0.70 m 门洞/厘米级贴边才需要 2.5 cm 变体。入口 `bash nav2_gap.sh` + `python3 verify_narrow_gap.py`。
> ⑥**仍未执行**：动态箱速度/方向档位（现 0.3 m/s 一档）；<0.70 m 门洞与 2.5 cm 栅格变体。
> ⑦**提交状态**：按用户要求**先不提交**——本轮全部改动留在工作区（15 项绕障 + 6 项刹车 + 5 项窄通道 + 文档）。
> 详见 `rm_field/README.md` §17 / §18。

> **✅ 2026-11-20 晚（副线 Gazebo）：绕障 + 动态避障验收 15 项判据全过；两个动态方案各踩一次坑后改成"触发式让开"。**
> ①**链路**：`capture_route.py` 先采无障基线（3 目标 19.442 m/66.8 s）→
> `config/avoid_obstacles.json`（唯一几何源）→ `make_obstacles_world.py` 生成带障碍物世界
> （静态 2 箱 0.5 m：`(0.85,-1.45)`、`(4.30,-0.60)`；动态 1 箱 kinematic，停在 `(2.50,-1.94)` 挡路，
> 机器人进 1.5 m 内后 0.3 m/s 让开到 `(2.50,-1.35)`）→ `nav2_obstacles.sh` 起栈 →
> `verify_nav2_obstacles.py` 出 `results/report_nav2_obstacles.json`。
> ②**结果**：三目标全 SUCCEEDED（12.646/5.557/6.807 m，到位 0.121/0.088/0.025 m），全程 25.051 m/86.9 s、
> 平均 0.288 m/s、cmd 峰值 0.499、z 全程 0.0100、不压平台墙（−0.562 m）；
> 静态间距 **0.701/0.640 m**；动态箱独立测速 **0.293 m/s**（配置 0.30）、行程 0.590=期望、
> 停位误差 0、人-箱最小间距 **0.892 m**、最近中心距 1.144 m（该段 cmd_vel 曾到 0）；
> 绕行代价相对基线 **+5.57 m**；定位一致（剔除未建图障碍物 ≤2 m 样本）max **0.256 m**。
> ③**两次失败方案（都是机制坑不是参数）**：连续往返横穿 → MPPI 反复 `Failed to make progress`、
> recovery 死循环（第三段 153.9 s），且盲动的箱子会压上机器人（最近 0.077 m）；
> 让开方向朝南 → 让进机器人绕行路径（中心距 0.237 m、穿模 −0.013 m）。
> 改"向北触发式让开+停住"后一次过。
> ④**新发现**：未建图障碍物会把 AMCL 带偏 **0.76~1.05 m**（三次复跑都出现，集中在贴着静态箱/动态箱走的
> 12~32 s 段，30 s 内收敛；无障基线最大只有 0.44 m）——实车临时摆箱/敌车同样会短时漂。
> 报告同时给 all 与剔除后两份定位误差。
> ⑤**踩坑**：`pkill -f 'dynamic_obstacle.py'` 会杀掉调用者自己（静默 exit 1）→ 用 `[.]` 写法并封装
> `start_dynamic.sh`；set_pose 的 0.1 s 步进让相邻帧测速失真 → 测速改用 1 s 窗口位移中位数。
> ⑥**剩余**：`collision_monitor` 本轮 0 次触发（DWB 提前绕开）→ "必撞/突然出现"场景未测；
> 5 cm 栅格贴边/窄通道、围挡 0.40 m 与赛规核对仍未做；动态箱只有 0.3 m/s 单向单程。
> 详见 [HANDOFF 2026-11-20 绕障验收](docs/handoffs/2026-11-20-obstacle-avoidance-acceptance.md)、
> `rm_field/README.md` §16。

> **✅ 2026-11-20（副线 Gazebo）：翻转后场地的 Nav2 地图重建 + 三目标闭环验收全过；顺带修掉潜伏半个月的 PGM 上下翻转 bug。**
> ①**地图**（`make_nav_map_up.py` + `capture_nav_map_up.sh`）：自由区 = 墙面内侧面以内 12.0×8.0 = 96 m²
> − 48 块碰撞 box 投影 18.510 m² = **77.490 m²**；512×352 @0.025 m/px、origin(−6.40,−4.40)；
> 从出生点 (0,0) 泛洪**单一连通**、0 未连通格、3 个验收目标全可达。
> ②**独立校核**（平光正俯视渲染 1 px=0.01 m，`worlds/nav_map_capture_up.sdf`）：渲染地面 bbox **12.00×8.00**
> （四边偏差 ≤0.01）、渲染地面 74.348 vs 地图 74.160、渲染绿色 17.884 vs 地图障碍 17.880（重心差 **0.022 m**）、
> 核心区逐像素一致 free **99.999%** / 障碍 **99.803%**；**11 项断言全过**（`results/report_nav_map_up.json`）。
> ③**Nav2 闭环**（`nav2_field.sh` 已切新世界 `worlds/rmul2027_3v3_up_nav2.sdf.xacro` + 新图）：7 个节点全 active、
> /scan 5.0 Hz、/odom 29.3 Hz、RTF 0.999；(4,2)/(4,−2.5)/(0,0) 三目标 **全 SUCCEEDED**（error_code 0），
> 结束差 **0.290/0.132/0.015 m**，总 19.622 m / 76.66 s、平均 0.256 m/s、cmd_vel 峰值 0.495 ≤ vx_max 0.5；
> z 全程 **0.0100**（不落台不爬台）、TF 离任何平台/墙投影最近 **0.51 m**；**11 项判据全过**
> （`results/report_nav2_field.json`）。定位误差（TF vs Gazebo 真值）均值 0.144 / 最大 0.443 m，最大出现在转弯段
> （AMCL 按 update_min_d=0.25 m 周期修正的滞后；静止/直行段 0.02~0.07 m）。
> ④**踩坑（不是参数问题）**：PGM 写盘漏了上下翻转——map_server 把图像第 0 行当**最上面一行**，而 `write_pgm`
> 直接 `tobytes()` 把整图翻了过来；翻转前自由区是整块矩形所以潜伏了半个月，这轮三目标闭环**第一次全失败**
> （TF 与 Gazebo 真值差 **4.8 m**、goal 1 error 103、车卡在平台上）。修法：落盘写 `img[::-1]` + 新增
> `pgm_orientation` 回读判据（8 个已知点）。另：重启栈时旧 `gz sim` 不响应 SIGTERM，会变成两个仿真器同时发
> `/clock /scan /tf`（症状 `Invalid frame ID "odom"`、话题 0 Hz），`nav2_field.sh` 清场改成 TERM→3 s→KILL −9
> 并清 `/dev/shm/fastrtps*`。另：验收脚本跨 shell 跑会落到 **domain 0**（栈在 `ROS_DOMAIN_ID=42`）→
> 报"等不到 action server"；已在脚本内 `setdefault("ROS_DOMAIN_ID","42")`，`nav2_field.sh` 收尾也会
> 等到 `/navigate_to_pose` 真出现再报 OK。再两条环境坑：`ros2 lifecycle get`/`node list` 走 daemon，
> 本机 WSL 上常看不到 composable 容器里的节点（7 个全报 Node not found，实际全都是 ACTIVE）→
> 启动检查改成直接调 `/<node>/get_state`；`stop_nav2_field.sh` 里过宽的 `pkill -f nav2_` 会把调用者自己
> 杀掉（串联执行时踩到），已删。
> ⑤**剩余**：围挡高度（现为场地自带 0.40 m 外墙）与赛规核对、绕障/动态避障未测、5 cm 栅格贴边场景要再加细。
> ⑥**提交**：`bbf9e57`（地图+验收）→ `bf937fb`（验收默认 domain）→ `1ee79bd`（lifecycle 服务检查 + stop 脚本），
> 已 push `origin/codex/car-local-obstacle-avoidance`（HEAD 与 origin 一致）；知识沉淀见 aiVault
> `wiki/concepts/Nav2_Map_And_Localization_Verification.md`。
> 详见 [HANDOFF 2026-11-20](docs/handoffs/2026-11-20-nav2-map-on-flipped-field.md)、`rm_field/README.md` §15。

> **✅ 2026-11-19（副线 Gazebo）：3V3 场地整体翻转（结构面朝上）+ 重做配色；"轮子转、车不走"的网格碰撞 bug 已用 box 拼块修好并五项验收全过。**
> ①**翻转**：`debug/make_flipped_meshes.py` 把 `RMUL2027_3V3_V1.0.0.stp` 的网格 `z→-z`（绕序反转、再 −0.10 m），
> 使**场地地面落在 z=0**，凸起最高 **+0.403 m**；新模型 `models/rmul2027_3v3_field_up/`。
> ②**配色**：浅灰=可行驶地面（97.3 m²）、绿=内部凸起平台、红=外围墙、黄=墙顶沿、深灰=台体底板/场外地面（z=−0.10）。
> ③**世界/脚本**：`worlds/drive_colored.sdf`（world 名 `drive`）、`drive_colored_preview.sdf`（出图相机 `/hero/image` 等）、
> `gui_colored.sh`、`config/gui_drive.config`。
> ④**验证**：世界模型 `site_ground / rmul2027_3v3_field_up / waffle`；RTF **0.9995**；机器人静置 z=**0.0100**。
> ⑤**✅ bug 已修（18:50）**：根因是 113,080 面三角网格 `structures.stl` 做碰撞把车垫高 6 mm（z=0.0158、2 s 只动 1.4e-5 m）。
> 改法 A：`debug/make_box_collisions.py` 从网格生成 0.05 m 高度图 → 3 层（0.20/0.27/0.40）→ **48 块 box**（31.90 m²，
> 高度误差 rms 1.4 mm），写回 `model.sdf` 的 `BOX_COLLISION_BEGIN/END`。五项验收（`verify_box_collisions.sh` →
> `results/report_box_collisions.json`）全过：静置 z=0.0100；0.15 m/s×2 s 直行 **0.2762 m**；
> 撞东墙停 **5.9316**（期望 5.9315）、撞北墙 **3.9316**、撞 0.20 m 绿平台 **−1.0684**（期望 −1.0685），
> 均不越界、不爬台、z 全程 0.0100、贴住后 2 s 漂移 0。
> ⑥**欠账（下一步）**：2D Nav2 地图与验收数据都基于翻转前模型，**已失效**（新地面不是整块矩形，被平台/墙占掉约 11 m²）：
> 要在新场地上跑 Nav2 需改 `make_nav_map.py` 自由区为"地面多边形减高台/墙"→ 重建 pgm/yaml → 重定起点 → 重跑验收。
> ⑦**手动开车（新增）**：`bash rm_field/drive_teleop.sh` 一条命令起 GUI 世界+小车+键盘遥控（w/s/a/d，松开 0.5 s 自动停；
> 自检 `python3 keyboard_teleop.py --selftest` 实测 1.5 s 前进 0.361 m、左转 1.157 rad、无指令漂移 0），见 README §14。
> 详情与可直接执行的入口：[HANDOFF 翻转+配色](docs/handoffs/2026-11-19-3v3-field-flip-and-colors.md)、
> [HANDOFF 碰撞+键盘遥控](docs/handoffs/2026-11-19-collision-box-and-keyboard-teleop.md)、`rm_field/README.md` §12/§13/§14。
> 提交：`80bf37f`（碰撞）→ `496f905`（键盘遥控），已 push `origin/codex/car-local-obstacle-avoidance`。

> **✅ 2026-11-18（拨弹盘台架）：模式 5 改成"按住转/松开停"并上板通过；测试口径按用户决定降级为比赛可用性四项，T3 最高转速取消。**
> ①**口径变更（用户否决）**：比赛是固定送弹速率、按住才送；3V3 冷却优先每发 +10 热量、冷却 24/s → 持续射速上限 **2.4 发/秒（≈盘轴 8 rpm）**，测"最高空载转速"没有意义，手机 240 fps 复核一项取消（handoff 里只留历史标注）。
> ②**固件**：`CHASSIS_BRINGUP_MODE=5`（默认），C610 ID7 / CAN1 / 1 Mbps；短按 KEY 切档（盘轴 **10/20/30/50 rpm ≈ 3/6/9/15 发/秒**）、按住 ≥0.3 s 开转（软启动 300 ms）、松开纯 P 反向刹车、按住 >20 s 断（防卡键）；保护另有掉线/超速 8000/失控 1.5×/失速 500 ms；调试帧 `0x1F3`。
> ③**踩坑（已修）**：第一版把 `HAL_GPIO_ReadPin` 的 `GPIO_PIN_RESET`(0) 当"按下"返回 → 逻辑全反（上电自己转、按键反而停）。修法：极性只在 `trigger_bench_key_raw_pressed()` 一处翻译；按键状态机抽成纯逻辑（`trigger_bench_logic.c`）+ 主机 gcc 时序/极性回归测试。用户重烧后确认正常。
> ④**证据**：`run_trigger_tests.ps1` PASS（含按键 8 组断言）、既有 263 项回归 PASS、Keil AC5 `0 Error 0 Warning`、hex 17:35；详见 [HANDOFF 2026-11-18](docs/handoffs/2026-11-18-拨弹盘台架与按键修复.md)、测试协议 [experiments/2026-11-18-trigger-bench](docs/experiments/2026-11-18-trigger-bench/README.md)。
> ⑤**下一步**：装罩 → 1 档放 10 发连送 3 轮（零卡滞才算该档可用）→ 数松手后多送几发（每发白烧 10 热量）→ 转着关电池确认失力；结果填 `runs.csv`。**S44/S45 需整机（摩擦轮+17 mm 测速模块）、S114 需电源管理模块，均未验证**；跑底盘前把默认模式改回 3 或 0 重烧。

> **✅ 2026-11-06（副线 Gazebo）：围栏碰撞验收补齐 + RMUL2027 3V3 场地正式接进 Nav2，两块都有可复跑判据。**
> ①**围栏验收**（`rm_field/verify_fence.py` → `results/report_fence.json`）：围挡顶 4 个落体探针全部停在 **z=0.300**（偏差 0）、
> 围挡外侧 2 个探针落到 **−0.503**；TB3 以 **0.7 m/s** 正面顶东/北围挡 16 s，停在 **x=6.1819 / y=4.1820**
> （理论 6.1815/4.1815，误差 **0.4 / 0.5 mm**；离内侧面 6.8 cm = 车头半长 + 围挡厚），`crossed_inner=false`、
> 全程 z 恒定 0.010、末 3 s 漂移 0，连跑两次一致。②**Nav2 地图**（`capture_nav_map.sh` + `make_nav_map.py`）：
> 地图**按 CAD 几何生成**（不是渲染二值化——渲染有阴影/透视/抗锯齿会带 1~3 px 系统偏差），
> `maps/rmul2027_3v3_fenced.pgm/yaml` 512×352 @ **0.025 m/px**、origin(−6.40,−4.40)，自由区 = 围挡内侧面以内
> **12.50×8.50 m = 106.25 m²**（界外全算障碍，规划器不会把车规划出平台）；顶视渲染（20 m 正俯视 1600×1100）做**独立校核**：
> 受光台面 12.500×8.500、围挡中心线 12.549×8.547（图纸 12.550/8.550）、红顶沿环心偏差 4.9 mm，8/8 判据通过。
> ③**Nav2 闭环**（`nav2_field.sh` + `verify_nav2_field.py` → `results/report_nav2_field.json`）：场地 xacro 世界
> （围挡 + Sensors/IMU 系统）+ 自建地图 + nav2 默认参数，目标 (4,2) 与回 (0,0) **均 SUCCEEDED（error_code 0、恢复 0 次）**，
> /scan 4.3–5.0 Hz、/odom 27–29 Hz、**RTF 0.94**，全程 max|x| 3.875 / max|y| 1.877（远在 6.10/4.10 限内）、
> 机器人 z 恒定 **0.010**（没掉下台子）、`cmd_vel` 峰值 **0.496 m/s** ≤ 参数 vx_max 0.5。**两条踩坑都是判据错不是系统错**：
> 用 `/amcl_pose` 算终点（AMCL 只在移动 >0.25 m 才发布，会滞后半米）→ 改采样 TF `map→base_footprint`；
> 凭印象写"TB3 上限 0.26 m/s"→ 查参数实为 `vx_max: 0.5`，改成从参数文件读上限再用 `/cmd_vel` 实测对比。
> **为什么 Nav2 必须用带围挡的世界**：0.503 m 台沿在 lidar 扫描平面之下，1:1 世界对 AMCL 是空场（只有里程计）；
> 0.30 m 围挡横在扫描平面上，±6.25/±4.25 有真实墙面。**接续入口**：`cd tb3_gazebo_nav2/rm_field &&
> bash nav2_field.sh && python3 verify_nav2_field.py && bash stop_nav2_field.sh`；文档 `rm_field/README.md` §11。

> **🏁 2026-11-04 21:16 演示流程（建图→重定位→示教→回放）端到端跑通，排练暴露的三个终点缺陷已全修。** 用户两天后要演示"先建图再自动导航"，今晚在新地图 `venue2-20261104`（`map_*.bin` 143 MB、`path.yaml` 321 点/13.54 m）上排练：**①建图** 2.5 min 开车 → `save_map`/`stop_mapping` 产物齐全；**②重定位** 一次成功；示教首轮**被 Odin1 回调队列溢出中止**（`Queue TF_ODOM/WIWC is full`，里程计停更 0.52 s）→ 板端 CPU 调频改 `performance` 后重示教成功。**③排练连续暴露三个终点缺陷（判据问题，非参数）**：a) `align` 位置判据还看传感器点，155° 原地转正扫 0.52 m → 判"漂离终点 0.22 m"失败 → **改用转心**；b) 示教末段停车录出 3 个重合尾点（间距 0.2 mm），纯追踪拿脚下点当前视目标 → `track_align` 15 s 超时；且纯追踪对**点目标会绕圈**（d 卡 0.33 m、w 恒定 0.57）→ 末段改为**瞄准终点转心 + 先对准再低速直行**收口（必须复用 `near_end`，否则闭合路线一进 `run` 就误判到达）；c) 最终对齐固定 5 s，`w_max=0.35` 时 176° 需 9 s → **超时按初始角度自适应**。**④示教规范新结论**：回放只前进不倒退，示教末段"倒着蹭"进终点会让抵达朝向差 176°、必须原地掉头；**末段应一路向前开过起点 0.3–0.5 m 再停**（重示教后 303 点/14.99 m，急弯停车 0 次）。**⑤验收（两次成功）**：最新一圈 **总 110.2 s**（回起点 26.4 + 跟踪 83.8）、行驶 14.65 m、**横向 max 0.269 / rms 0.111 m**、终点朝向误差 **0.01 rad**、里程计 1184 帧全接受；上一圈（倒车示教）106.9 s / rms 0.133 / 末尾掉头 176°。**⑥`drive_lap.sh` 补齐演示参数**（以前只透传 `v_max`）：`W_MAX`（默认 0.35；内建 1.2 实测"转弯太快"）、`RETURN_W_MAX`、`BASE_OFFSET_X/Y`、`START_MAX_OFFSET_M`、`RETURN_TIMEOUT_S`、`MAX_TIME_S`，数值统一补成浮点（`280` 会被 ROS 2 当 INTEGER，`max_time_s` 是 DOUBLE → 直接崩）。**⑦交付**：演示当天手册 [docs/demo-runbook.md](car_slam_lap/docs/demo-runbook.md)（含时间预算、故障速查、验收基线）；安全测试 **83 项全过**（新增 4 项）；提交 `867a400`。下一步：①演示前再彩排一次完整链路（含从头建图），确认重复性；②7–8 m 处急弯横误差 0.20–0.21 m 可试 `v_max=0.12`/放大 `w_max`；③演示当天按手册操作，路线与地图 `sha256` 绑定，**建图→示教→回放必须同一场次**。

> **🏁 2026-11-04 20:31 首次完整跑完整圈（实车，方案 C 力臂补偿 + 标定 offset）。** 桥接 `status=119` 全绿起步；**自主回起点 54.5 s、补位往返 0 次**（起始状态：距起点 1.64 m、朝向差 113°，靠 `start_max_offset_m=2.0` 放开自动回起点）→ 起始对齐位置误差 0.03 m / 朝向 0.09 rad → **跟踪 15.08/15.51 m、87.7 s、`v_max=0.15 m/s`** → 两个 83°/103° 急弯都自动“停车原地对齐”后继续 → 终点位置到但朝向差 0.73 rad，自动最终对齐到 **0.09 rad** → **结果：成功**（总 142.2 s，横向误差 max=0.233 m / rms=0.072 m，里程计接受 1491 帧 / 拒绝 0 帧，实际轨迹 328 点写 `~/car_ws/logs/lever-arm-20261104b/lap_full2.yaml`）。**力臂标定实测值 `(+0.2626, -0.0471) m，模长 0.267 m`**，与独立几何交叉验证一致（转 180° 传感器点位移实测 0.531 m，理论 2×0.267×sin(95°)≈0.532 m）。**新增实测经验**：①`return_timeout_s` 默认 30 s 只够 ≤0.5 m 小范围补位，**范围放大必须同步放大超时**（1.5 m 实测约 54 s，否则“回起点总超时”失败停车）；②**速度与精度是权衡**：`v_max=0.15` 时 rms 0.072 m / max 0.233 m，`v_max=0.08` 时 rms 0.046 m / max 0.149 m（都是急弯处最大）；③桥接在跑时固件按 `src=serial` 仲裁，**遥控被屏蔽**——要人工开车必须先停 `bridge_node`；④Odin1 是 12 V/2 A 航插独立供电（USB3 只走数据），**电源灯不亮/`lsusb` 无 `2207:0019` = 12 V 那一路电流不够**，断连时刻可从 `~/.ros/log/host_sdk_sample_*.log` 的 `Device disconnected` 反推；⑤上电后若 `status` 缺 `CAN_OK/CAN_ACK`（102/115 类），先整车断电重启清总线状态，别急着改线。下一步：①把整圈结果写进任务卡05验收段并提交；②同型风险仍在**终点后对齐**（已靠力臂补偿覆盖，但 `align` 判据未单独上机复验）；③若要重复跑，注意每次启动前车需在 `start_max_offset_m` 内（超了会拒跑）。

> **2026-11-04 傍晚（上电实测，X5 已于 19:08 关机）：新版 B/C 与标定工具已部署到板端并验证链路，力臂标定未做完——被 Odin1 供电不足挡住。** ①**部署**：板端备份在 `~/car-deployments/lever-arm-20261104-173613/`；`follow_path.py`/`lever_arm.py`/`calibrate_lever_arm.py` 在 `~/car_ws/src` 与 `install` 哈希一致（`6c947593`/`1e324759`/`cc7b0981`），`ros2 pkg executables car_bridge` 有 `calibrate_lever_arm`，板端 **163+21=184 项测试全绿**。②**定位与起点差**：17:33 重定位成功，位姿 `(1.2195,-0.1348,-0.0047)` vs 路线起点 `(1.0242,-0.2788,0.0266)` = **位置差 0.243 m、朝向差 1.8°**（没超 0.5 m，无需人工摆车）。③**CAN 故障与复位（重要）**：首次控制时 status 一直 `102`（缺 `CAN_OK`/`CAN_ACK`），`can_bus.c` 里 `CAN_OK` 语义是 `HAL_CAN_AddTxMessage` 成功入邮箱——总线无 ACK 时同一帧重传占满邮箱即返回 BUSY；**整车断电重启后恢复 `119`（全绿）**，同一状态字在 X5 完全不发帧时实测 `125`（含 estop）。即上电后若再遇到 `102`，先断电重启清总线状态，不要先改线。④**控制链路已验证**：`0.5 s / 0.10 rad/s` 小脉冲后用户确认**车头转动**（X5→串口→CAN→控制器全通）。⑤**力臂标定未完成**：第一次转动尝试被 ③ 的 CAN 故障挡住（脚本自检无位移即发零退出，安全）；CAN 恢复后未再转，因 ⑥。⑥**Odin1 掉线**：`~/.ros/log/host_sdk_sample_11703_*.log` 记 `Device disconnected`，epoch 换算 **18:08:33** 后一直等重连；本次重启后 `lsusb` 从未出现 `2207:0019`，用户确认 **12 V 供电电流不足**（WIRING.md：Odin1 独立 12 V/2 A 航插，USB3 只走数据）。⑦**操作教训**：`pkill -f "…bridge_node"` 的 pattern 会匹配到 ssh 自身的命令行，把自己会话杀掉且可能什么都没杀成——用 `pgrep -f 'lib/car_bridge/bridge_node$' | xargs -r kill` 或按 PID。下一步：①先解决 Odin1 供电并确认 `lsusb` 有 `2207:0019`；②起定位核对起点差；③`ros2 run car_bridge calibrate_lever_arm` + 闭环转 180° 拿 `base_offset_x/y`（脚本已就位 `_probe/deploy-20261104/rotate_for_calibration.py`）；④带补偿跑到"进 `run`"验收起点收敛，再决定整圈。

> **2026-11-04 下午（新会话）：先落盘提交，实现任务05 方案 B，仿真发现 B 在真实力臂下不收敛，再按你的选择实现方案 C（力臂补偿）。** ①**提交**：9 个 commit 已推送 `origin/main`（`99c91e2..553d0f6`，95 文件 +28593 行），工作树干净；`.gitignore` 挡掉厂商 SDK（180 MB）与 Keil DFP pack（39 MB）；此前未跟踪的 `route_bundle`/`tracking_geometry`/`localization_guard` 三模块与 5 个测试文件一并入库（否则提交缺依赖跑不了测试）；把 HEAD 单独检出到临时 worktree 复跑 **158 项 Python + 3 套 C 测试全过**，证明提交自洽。②**B 实现**（`d81a062`）：`align_start` 只判朝向、转动期间不判位置；转正后位置超差回 `return` 补位；往返上限 `START_RETURN_ROUNDTRIP_MAX=3`、共享 `return_timeout_s`、每轮日志、`summary` 记次数。③**关键发现（仿真）**：差速车运动学 + 0.25 m 传感器偏置下，r=0 时 3 轮内收敛；**r=0.25 m + 严格判据进入极限环**（dist 0.099↔0.137 m，第 4 轮安全停车），因为 `align` 要把朝向转回路线方向、`return` 要把车头对准目标点，两者互相抵消；方案 A 参数（0.50 m/34°）同几何可直接进 `run`。④**C 实现（力臂补偿）**：新增 `base_offset_x/y`（默认 0，车体系，`传感器 = 转心 + R(yaw)·offset`）与 `_rotation_center()`，起点范围检查/回起点目标/起始对齐判据全部改用**转心**位姿，原地转不再移动转心；B 的补位往返保留为"未标定或标错 offset"时的安全兜底；README §14.3 写了 180° 标定公式 `offset = -R(-yaw0)·Δs/2`。**同一仿真几何下 C 一次进 `run`、补位往返 0 次**。⑤**标定工具**：新增 `ros2 run car_bridge calibrate_lever_arm`（只读 TF、不发 cmd_vel；静止起点→转够 120°→停稳 1 s→反解并打印 `-p base_offset_x/y:=...`；模长 >0.6 m 或 <0.02 m 告警），纯几何/采集状态机/输出格式在 `test/test_lever_arm.py` 15 项覆盖。本地 **184 项测试全过**（安全测试 79 项）。**未部署、未上车**；X5 仍关机（`ping` 不通，SSH 端口"连通"来自本机策略代理）。下一步：①上电后先跑 `calibrate_lever_arm` 标定 offset（符号+数值，工具免手算）；②再起定位、核对起点位姿差，现场确认后发受限回放（带 `-p base_offset_x/y:=...`）；③终点对齐的同型力臂风险未改，见[任务卡05](car_slam_lap/docs/tasks/2026-11-03-05-start-pose.md)。

> **2026-11-04 本轮会话已收尾，X5 已下发关机。** 完整交接与下次入口见[HANDOFF 2026-11-04](docs/handoffs/2026-11-04-car-slam-lap-calibration-replay.md)。下次三件事：①开机核对进程/串口/证据文件；②起 Odin1 重定位并核对车与路线起点的位姿差（超 0.5 m 先人工摆车）；③在[任务卡05](car_slam_lap/docs/tasks/2026-11-03-05-start-pose.md) 的 A（宽松起点）/ B（先转正再补位）中选一个，再决定是否发运动。**跟踪器主循环（run/track_align）在实车上仍未验证过一秒钟。**

> **2026-11-03 22:04 跟踪器首次实车回放：三次尝试全部安全停车，暴露两个设计缺陷（不是参数问题），两处已修并部署，剩一个需选方案。** 车从角速度标定后的位置（距起点0.44 m、航向差22°）按"自己回起点"跑`follow_path`（`mps_at_max=0.95`、`v_max=0.08`、`w_max=0.25`、`max_time_s=70`）。尝试1：回起点10.2 s到位（0.09 m）成功，但实际轨迹显示车**绕着起点画半径0.14 m的小圈**、到位时朝向差162°，随即"起始对齐"被漂离判据判停。尝试2：同阶段再被判停，实测**原地转77°时位姿点扫过0.316 m**。至此可反推**位姿传感器偏离转心约0.25 m**，于是"位置在起点±0.1 m"与"朝向对齐"两个判据在物理上互斥（转140°就平移0.47 m）——这是设计缺陷。**已修两处并部署**（本地158项测试全过、板端install同哈希`c16ad784…`）：`RETURN_MIN_RADIUS_M=0.40`（回起点行驶中`w<=v/R`封顶，原转弯半径小于到位容差导致绕圈）、`START_ALIGN_DRIFT_SLACK_M=0.15`（给原地转的等效平移留余量）。**但这两处修不掉根因**。下一步二选一，见[任务卡05](car_slam_lap/docs/tasks/2026-11-03-05-start-pose.md)：**A 宽松起点**（0.5 m/34°内直接开跑，让纯追踪收敛，最快验证跟踪器）或 **B 先转正再补位置**（对齐只转不管位置、跑掉就回`return`补位，保留严格起点保证）。附带好消息：标定结论在真实闭环里被验证——指令`w=0.25`实测0.42 rad/s（1.7×），与标定的"长弯稳定段1.5–1.9×"吻合。收尾干净：驱动与桥接已停、`/dev/ttyS1`空闲、无残留进程；车停在地图约`(1.28,-0.53)`、朝向约-61°，距离起点0.36 m、朝向差63°。数据在`_probe/ros-replay-20261103-214827/`与板端同名目录，详见[ROS控制记录](docs/experiments/2026-11-03-car-ros-control/README.md)。

> **2026-11-03 21:43 角速度初标定完成：转向符号正确、`radps_at_max`暂不改（2.0），但发现低速死区、滑转和Odin1离转心0.2 m。** 21:36重定位成功后5发受限角速度脉冲（20°/0.08 m硬上限）：+0.06/30u→窗口内0.021 rad/s、总2.63°；±0.15/75u→12.42°/−12.75°（对称，符号正确，`invert_angular`成立）；+0.25/125u→11.66°；其中+0.30/150u那发第0.284秒`can_ack`瞬时丢失（119→103）被安全门中止、写零停车，约1分钟后自行恢复status119。关键结论：75单位持续1秒窗口内平均0.145 rad/s≈名义0.15，但停发后滑转4.4–4.9°，一次1秒指令总转角约为名义1.45×，稳定段约1.5–1.9×（0.3–0.5秒加速滞后）；约18–30单位以下是死区（ω≲0.07 rad/s转不动）；原地转的漂移/转角比值给出**Odin1距转心0.19–0.22 m**，即"原地转"会带来约2 cm/10°的未指令平移，是回放精度风险。因此`radps_at_max`保持2.0，回放时`w_max`压到≤0.3并避免ω<0.08的死区指令；长弯若持续超转再改约3.0。收尾后驱动已停、串口空闲、无残留进程；五发累计净位移0.069 m、航向+11.14°，回放前需由跟踪器自行对齐。数据在`_probe/ros-angular-cal-20261103-213635/`与板端同名目录，详见[ROS控制与任务04记录](docs/experiments/2026-11-03-car-ros-control/README.md)。

> **2026-11-03 21:31 直线速度初标定完成（三发硬件实测），车实际比配置快约20%，建议`mps_at_max`0.8→0.95。** 21:24起Odin1模式2重定位成功（`map←imu`0.607/−0.237 m、yaw 0.001、odom 10.2 Hz）；零指令自检`status=119`/`src=serial`/CAN·ACK·RC·LINK全绿/CRC0，用户已把S2拨离急停档。随后三发100单位/最长3秒受限脉冲（0.5 m/15°硬上限），三发均正常完成、无故障、未触限：总位移0.28139/0.26680/0.28376 m，yaw +0.442/−0.033/−0.116°，横移≤4 mm。run1与run3一致（稳定段0.09597/0.09705 m/s），run2离群（0.08471，且在t=0.65 s有一次5 mm位置回退）。按最大偏离判重复性11.7%，**未达≤10%采用门槛**，故记为初标定而非冻结值；一致值外推`mps_at_max≈0.95 m/s`（现配置0.8 → 下发0.08 m/s实际约0.096 m/s，快20%）。只测10%占空，高占空线性度未验证。三发累计使车前移0.832 m；驱动已停、串口空闲、板端无残留进程。落地方式为运行时`-p mps_at_max:=0.95`，下次部署再改默认值。下一步：角速度标定（`radps_at_max`2.0仍未标定），再分级回放。数据在`_probe/ros-linear-cal2-20261103-212417/`与板端`~/car_ws/logs/ros-linear-cal2-20261103-212417/`，详见[ROS控制与任务04记录](docs/experiments/2026-11-03-car-ros-control/README.md)。

> **2026-11-03 任务04修复已安装、1秒ROS前进起停实测通过，开始准备两次3秒线性初标定。** 本机与X5各134项全过（板端夹具补齐后无跳过），轨迹10/10、协议11/11；独立模型完成且进度回退33→0，正式ROS缺原始odom拒绝通过。新部署`~/car-deployments/task04-20261103-LUEdPh/`保留回退；所有安装模块哈希一致、配置未变。用户现场确认后1秒100单位短测发49帧非零，里程计前移约7.6厘米，用户确认正向移动后正常停止并急停。接着用户同意两次各3秒、100单位直线标定，0.5米/15°上限、0.3秒原始数据门，重复差异≤10%才采用；当前组`~/car_ws/logs/ros-linear-cal-20261103-P2q2i0/`只刚启动传感器，尚未执行这组运动，结果待采。（补记：该组20:46实际被拒——`readiness_refusal:missing_or_stale_odometry`且底部仍ESTOP，写出的286帧全为零帧、位姿差0.26 mm，无意外运动；21:03零指令复查仍estop。21:24后的三发标定见上方最新块。）此时传感器运行，桥接未运行、串口未占用，用户已中档回中且前方2米空旷。前向派生路线330点/15.5129米，仍未自动回放。详见[控制与修复记录](docs/experiments/2026-11-03-car-ros-control/README.md)。

> **2026-11-03 ROS零指令实测通过，任务04正在委派修复，未做非零短测。** DT7急停确认后真实BridgeNode发251帧、逐帧断言均零，收51条有效ROS状态，CAN/ACK/RC/ESTOP正常、CRC零错；停发后20帧LINK_OK均消失，串口已关闭。完整_tick离线复现掉头缺陷：索引162选目标192、夹角167°仍前进，输出均截获未发给机器人。用户确认末尾倒车，选择修复并派生去掉倒车段；已生成前向独立目录`~/car_ws/routes/venue2-forward-20261103-t0jkaxrb/`，330点/15.5129米，绑定校验通过，原始383点保留。任务04 Harness新会话session-c556f46f-6f5f-4680-b934-a959cff8b44b已接收任务，修复与独立验收未完成；基线108项重跑通过。暂无非零动作，用户已通知继续保持急停；后续准备好短测后需重新确认现场。详见[ROS控制与任务04记录](docs/experiments/2026-11-03-car-ros-control/README.md)。

> **2026-11-03 移动电源供电后真实map示教完成，尚未自动回放。** 新包正式部署已保留；用户纯遥控完成路线并停车急停，严格录制正常退出0、生成正式bundle，独立校验退出0。正式新目录`~/car_ws/routes/venue2-map-teach-20261103-93xnjX/`，map/imu、383点、15.816米、176.958秒（含静止等待），记录首末相差11.734厘米/1.673°，不是自动跟踪误差。路线含两端掉头和相近往返路段，防跳段/急弯处理尚待验收。首轮9MFKUq因遥控不动中止且无manifest；只收显示src=rc而CAN/ACK缺失，用户急停复位C8T6后CAN/ACK恢复，并确认纯遥控正常；根因仍未确定。两轮路线/清单/日志/只收状态/统计/预览已取回本机并校验摘要。当前驱动/录制/桥接/跟踪均停止、串口无人占用，X5保持开机，用户最后已停车急停。下一步先做正式ROS串口零指令与速度/转向标定，审查往返近邻段/掉头跟踪，再分级验证返回起点与完整回放。详见[真实map示教记录](docs/experiments/2026-11-03-car-map-teach/README.md)。

> **2026-11-03 首次示教因遥控不动中止，C8T6复位后纯遥控恢复，已重新开始严格示教。** 首轮新目录9MFKUq只有静止噪声（158点/167秒/0.03米），已异常终止、无manifest，不可用于回放。只收状态显示src=rc/RC正常而CAN_OK/ACK缺失；用户确认中档且供电/接线未动、控制器常亮。回急停并复位C8T6后只收状态125，CAN/ACK恢复、急停有效；用户确认纯遥控低速起停正常，已回急停，恢复原因尚未确定。当前新目录`~/car_ws/routes/venue2-map-teach-20261103-93xnjX/`，模式2重定位及严格录制实际就绪后已再次通知用户DT7示教。驱动与录制正运行，未开串口桥接；等待用户停车急停并回复完成，先结束真实录制节点6390，再停传感器，不要误操作上一轮PID。详情见[示教记录](docs/experiments/2026-11-03-car-map-teach/README.md)。

> **2026-11-03 已换移动电源重启，严格map示教录制已就绪，等待用户完成路线。** USB识别和新包正式版本正常，模式2venue2旧图重定位成功；新独立目录`~/car_ws/routes/venue2-map-teach-20261103-9MFKUq/`，严格录制日志确认实际定位就绪后，已通知用户用DT7低速示教，结束停车并急停再回复。没有串口桥接/跟踪/自检，串口无人占用。传感器与录制正在运行，尚未正常收尾或生成可验收新路线；不能直接停掉驱动，应先正常结束本次录制并检查manifest。运行身份及接续见[示教记录](docs/experiments/2026-11-03-car-map-teach/README.md)。

> **2026-11-03 新包正式部署通过，用户改用移动电源前已关机，尚未示教。** 任务03冻结代码+ROS打包元数据已安装，108项单测、轨迹10/10、协议11/11、bash语法、colcon构建、正式安装导入哈希、真实ROS缺定位拒绝及桥接dry-run通过；全程未开串口或电驱运动。正式部署目录`~/car-deployments/route03-20261103-P93p63/`，旧源码/构建/安装/配置/脚本备份及rollback.sh已保留并取回本机。首次检查脚本误将预期退出码触发回滚，已恢复旧版、修正后第二次部署成功，首次记录保留。速度比例仍为待实测确认默认值0.8m/s、2.0rad/s，尚未做正式ROS串口/标定或新map示教。用户要求关机换移动电源：关机前无项目进程、串口无人占用，sync+shutdown后SSH断开，两次Ping及SSH端口均超时，X5已离线。下一步换供电开机后核对IP/安装版本/USB与定位，准备独立新路线输出并确认录制就绪后，再由用户DT7示教。见[部署与关机接续](docs/experiments/2026-11-03-car-package-deploy/README.md)。

> **2026-11-03 准确50厘米直线复测通过，下一步准备新包可回退部署。** 用户在当前车位正前方设置准确50厘米标记；新采A2基准后，用户确认同一车身参考点前移到位，无横移/角度偏转，再采B2。两次独立启动均重定位成功，均242条原始odom/242条地图TF，时间推进、数值有效。定位位移50.9078厘米，距离误差0.9078厘米、yaw差1.689°，满足已约定5厘米/5°标准；结合首轮返回偏差2.21厘米/0.049°，本轮局部已知位移与返回重复性有通过证据。前轮边界结果保留，根因未定，不外推为全图精度/全路线验收。两轮原始bag/日志/比较JSON已取回，归档摘要经板端反查一致。驱动与录包均停止、串口无人占用，X5仍开机，小车停在前方50厘米新标记处并保持急停；未部署/烧写/调参/电驱运动。下一步盘点正式包和任务03冻结包，备份并准备可回退部署，核对ROS桥接及速度/转向标定，再做严格map示教与分级回放。详见[直线复测记录](docs/experiments/2026-11-03-car-relocal-static/marked-displacement.md)。

> **2026-11-03 地面标记与人工位移首轮完成：返回重复性通过，已知位移距离项待精确复测。** 用户同意0.50米位移、距离误差≤5厘米及朝向变化≤5°，并保持急停。A标记处→B实际后移50厘米且左移3～5厘米→C放回标记，三次独立启动均重定位成功。定位测得AB位移45.03厘米；按名义50厘米误差4.97厘米，计入报告横移后误差5.06～5.22厘米，距离项不接受为通过，朝向差1.10°满足标准。C相对A返回偏差2.21厘米、朝向差0.049°，本轮返回重复性通过。未电驱运动、未开串口、未调参或部署。三轮驱动/录包已结束，串口无人占用；X5保持开机、车已放回标记。数据和日志已取回、归档SHA256一致；下一步同一车身参考点与准确相距50厘米的两个地面标记做直线复测，不先进入回放。见[人工位移记录](docs/experiments/2026-11-03-car-relocal-static/marked-displacement.md)。

> **2026-11-03 再次开机接续：venue2旧图两次重定位成功，独立空间精度待验。** 用户保持DT7急停；先架空、后落地，两次有界启动均明确出现relocalization success。架空145条odom/146条地图TF，落地272/272，时间均推进、原始位姿有效；落地26.4秒静止位置各轴极差0.65～0.92毫米、yaw极差0.022°，仅代表输出波动，不是绝对精度。现场无旧起点标记，尚未验证独立空间精度，也未确定上轮失败原因。未改配置/地图/固件、未正式部署、未启用串口或底盘运动。两轮驱动已停止，串口无人占用，X5保持开机；两轮原始bag和日志已取回、归档SHA256一致。下一步建立当前落地位置/朝向标记，做独立位移、朝向与重启重复性核验，再准备部署与新map示教。详情见[静止重定位记录](docs/experiments/2026-11-03-car-relocal-static/README.md)。下方“已关机”为此前停止点。

> **2026-11-03 会话已结束，X5已执行关机并核验离线，本机Harness服务已停止。** 关机前无机器人应用与串口占用；关机后SSH断开、两次Ping超时及TCP22不可达。任务03软件独立108项、轨迹10/10、协议11/11、bash语法、真实ROS缺定位拒绝均通过；X5直接串口+100/1000持续1秒、归零与交回遥控已实测，用户确认起停正常。未正式部署/烧写/提交推送。下一步先收口旧图重定位（本轮190条odom、odom→map为0），再准备新包部署、新路线示教与分级回放。准确接续见[本轮最终交接](docs/handoffs/2026-11-03-car-slam-lap-closeout.md)，下方记录为过程中历史。

> **2026-11-03 最新：X5直接串口驱动车轮已实测通过，无需据初检判定X5不能控制。** 用户确认纯遥控正常；只收状态发现CAN_OK/CAN_ACK已恢复。用户确认架空、摇杆回中、S2中档，授权+100/1000持续1秒；零接管→短时驱动→零停止全程status119/src=serial/CAN正常，停发后LINK_OK掉落并回rc、CRC零错。用户确认车轮转动后正常停止。已归零并关闭串口。此前CAN状态缺失的恢复原因未定，不再作为当前阻塞；旧图重定位尚未就绪仍待处理。未改固件，未正式部署。下方初检为历史，证据见[本轮记录](docs/experiments/2026-11-03-car-route-resume/README.md)。

> **2026-11-03 接续：任务03地图路线绑定已完成软件独立验收；当前按用户要求交回纯遥控。** X5地址确认10.77.156.239。Windows/X5隔离副本108项unittest、轨迹10/10、协议11/11、bash语法通过；真实ROS无定位30秒后退出1且无manifest。未正式部署或烧写。架空零指令链路及watchdog通过，S2拨到底后estop已确认，但CAN/ACK缺失，C8T6复位未恢复；Odin1长窗口190条原始odom正常，odom→map为0，旧图定位未就绪。用户要求取消X5↔C8T6通信后，已确认无控制/自检进程、串口无人占用；等待其DT7纯遥控反馈，不重新启动串口。详情与证据见[本轮记录](docs/experiments/2026-11-03-car-route-resume/README.md)。下方11月2日停止点为历史。

> **2026-11-02 晚间 `car_slam_lap` 实施停止点（用户要求明天继续）：安全基础与回起点完成软件验收，任务03地图路线绑定刚开始即取消。** SSH/静止传感器盘点完成；任务01独立47项测试，任务02累计62项，轨迹10/10、协议11/11；X5独立ROS域隔离回放测得原始odom停更后0.50653秒转零。未部署正式包、未烧写、未做串口零指令自检或实车运动。当前有未完成的`test_route_bundle.py`，对应模块尚未实现，全量测试不能宣称通过。已确认板端无驱动/桥接/跟踪/回放进程，SSH/SFTP退出，Harness任务已取消。明天优先读[实施交接](docs/handoffs/2026-11-02-car-slam-lap-execution.md)，从任务03继续；证据见[本轮实验](docs/experiments/2026-11-02-car-route-readiness/README.md)。下条为本轮开始前的历史状态。

> **2026-11-02 `car_slam_lap` 本轮对话已收尾，等待新对话接续。** 结论、已交付文档、未实施项和接续提示见[路线复现讨论交接](docs/handoffs/2026-11-02-car-slam-lap-route-discussion.md)。下一步先解答新手说明中的疑问，再核实地图与轨迹坐标关系；本轮没有开展导航改造或实车运动。

> **2026-11-02 小车练手说明：已确认本次使用 `car_slam_lap` 实体小车，而非 Gazebo。** 新增[路线二与现有方案的区别（ROS 新手版）](car_slam_lap/路线二与现有方案的区别_ROS新手版.md)，区分时间动作重放、现有位置闭环跟踪、地图定位后先回起点的目标流程。优先复用 Odin1 建图/重定位；跨会话坐标一致性、定位检查和返回起点均为待实施/验证项，远距离绕障返回另定范围。本轮仅编写说明，未改控制代码、未部署或启动实车；下一步先核实地图与轨迹坐标关系。

> **2026-11-02 副线（视觉输入/工具）：新增 Windows 相机帧率测试脚本 `tools/cam_fps/cam_fps.py`。** 对 ZSKJ-Camera 实测（MJPG/MSMF）：640×480 与 640×360 = 30 fps、800×600 = 20 fps、1024×768 与 1280×720 = 10 fps、1280×1024 = 5 fps，**1080p 及以上不支持**（驱动回退）；YUYV 除 640×480 外均 6.7～8.3 fps。排查中确认两个 Windows 采坑：DSHOW 后端会把 MJPG 请求换成 YUY2，且驱动拒绝分辨率后会输出伪帧率（1080p 报 113 fps、320×240 报 187 fps），扫描必须用 `--exact` 过滤。数据、交叉验证与复现命令见[本轮实验](docs/experiments/2026-11-02-zskj-camera-fps/README.md)，用法见 `tools/cam_fps/README.md`。底盘/仿真主线未改动。

> **最新接续（2026-11-01，副线 Gazebo 仿真）：RMUL2027 3V3 场地已 1:1 进 Gazebo，比例/碰撞/台面高度/观感四项都量化验收完毕。**
> 场地 12.600 × 8.600 m、台面 z=0、台下 0.503 m；台面高度不再靠“看着像”，而是像素级实测：正立面亮带 1734×69 px → 墙高 **501.3 mm**（焦距换算）/ **494.1 mm**（12.6 m 宽度做标尺），墙面近景墙高 **451 px = 画面 62.6%**，低机位斜视侧壁带 74 px（预测 75 px），`bash capture_height.sh` 11 条断言全过。用户两轮反馈“看不出高台/没看到围栏”后定位到四级原因（俯角太小、地面≈背景、**投影≈天空**、单一材质无竖直参照），改法全部是颜色/光照/相机，几何未动；另加 **0.30 m 围挡**做可选变体世界（`worlds/rmul2027_3v3_field_fenced.sdf`，GUI 默认加载它；1:1 图纸世界 `rmul2027_3v3_field.sdf` 仍是验收基线）。同时确认 `<gui><camera><track>` 会覆盖 `--gui-config` 的 camera_pose、WSLg 抓图不可靠，均已记入 issue。遗留：围栏碰撞未实测（0.7 m/s 撞边不越界）、0.30 m 高度待赛规核对、Nav2 地图未开始。记录见[本轮实验](docs/experiments/2026-11-01-rm-field-gazebo/README.md)、[围挡决策](docs/decisions/2026-11-01-rm-field-fenced-gui-variant.md)、[GUI 观感 issue](docs/issues/2026-11-01-wslg-gui-visibility.md)；细节在 `tb3_gazebo_nav2/rm_field/README.md` §4/§9/§10。底盘主线状态见下条，未被本轮改动。

> **最新接续（2026-10-30）：按用户要求恢复模式3单轮闭环，已编译，等待用户重新烧写。** KEY按1～4次选择对应CAN ID，停手3秒后执行+400rpm 8秒→零速1秒→−400rpm 8秒→零速1秒，结束释放；Kp=2/Ki=0.004/限流±1500保持不变，运行中KEY取消、掉线和绝对转速>1500rpm取消。仅修改默认模式入口，复用原有选轮与闭环代码，不增加电流；模式4源码及历史数据保留。速度主机测试及映射/按键263项全部通过；Keil AC5全量构建0错误0警告，Code=64924，链接表确认chassis_task调用speed_loop并移除open_loop_loop。切回前快照及本次待烧写快照分别位于`_probe/restore-speed400-20261030-192152/`和其`ready-to-flash/`子目录。未代烧写、未启动实物运动，4号不转原因尚未确定。下方开环停止点为切回前记录。

> **最新停止点（2026-10-30）：4号已润滑并拆减速箱，模式4开环复测已完成。** 原始`_probe/wheel4-no-gearbox-20261030-run1/capture.csv`共815177帧，三次输出均超过99%时间接近静止，第三段550保持约8秒、编码器净转约0.07转，未持续旋转，不能判断周期顿挫是否消失。其余三路指令零、诊断无故障码、末帧归零；用户确认运行中按过KEY。未改PID/固件/限流。下一步断电检查裸轴阻力和拆装干涉，再决定受限复测，暂不继续加电流。Harness负责分析、Codex已独立复算；检测脚本的连续转动顿挫分支仍待修正验收，详见[本轮记录](docs/experiments/2026-10-30-wheel4-no-gearbox/README.md)。下方10月29日状态为历史。

> **当前最终停止点：400rpm第二轮已完成，400/600单变量对比已出（2026-10-29 19:47）。** 第二轮180.022秒、827033帧、6923条诊断；预检模式3/四轮在线/零指令，无需RESET，完整1→2→4各一次18秒，无限流/故障诊断、非选中轮零指令、退出释放正常；两轮400rpm原始与摘要已独立复算、ZIP逐成员SHA-256校验。400/600对比：2/4降速簇约每15转子转一次，簇Δt比1.40～1.51、簇Δrev比0.94～1.00（实测转速比1.49～1.52）→ 支持转动位置相关而非时间相关；1号两轮无稳定段降速簇、7阶纹波47→70.5Hz。400rpm未改善2/4相对波动（标准差为目标的9%～13.8%），不替代600rpm验收。下一项单变量建议：1号与2号完整轮组互换（不改固件、不烧写）；保持模式3/Kp2/Ki0.004/限流1500，不启动四轮联动。详见[400rpm定位实验](docs/experiments/2026-10-29-speed400-comparison/README.md)。

> （历史）2026-10-29 早先的400rpm首轮、烧写准备与600rpm两轮验收状态见[400实验](docs/experiments/2026-10-29-speed400-comparison/README.md)与[定量验收报告](docs/experiments/stage1-acceptance/README.md)；本轮之前的状态不再作为接续依据。

> **第二阶段任务0实测结论：未通过。** 2026-10-29已完成60秒静止基线与两轮1→2→3→4，每轮两次完整序列。1号样本内达标；2/4号波动、3号正向波动及2～4号调节时间未达标。平均误差≤1.458%、超调≤17.834%，零速与释放检查通过；未见故障诊断或1500限幅。保持模式3/Ki=0.004，不进入四轮联动；下一步断电检查机械接触/阻力与供电，再定单变量实验。数据2481501帧及三份SHA-256校验备份已保留，见[定量验收报告](docs/experiments/stage1-acceptance/README.md)。下方“待采集”为本轮早先状态。

> **第二阶段接续已启动：当前执行任务0。** 用户已在采集前确认采用计划验收阈值，记录于[第一阶段定量验收](docs/experiments/stage1-acceptance/README.md)。本地模式3及Ki=0.004参数已核对；台架、板上固件与USB-CAN状态待确认，尚未新增实测数据或启动四轮联动。下方“会话已收尾”为上一轮历史状态。

> **2026-10-29会话已收尾。第一阶段：单轮基础功能完成，定量验收未完成；第二阶段：仅完成计划，尚未实施。**
> 汇总与数据清单：[单轮会话收尾](docs/experiments/2026-10-29-single-wheel-session/README.md)。
> 下次从[第二阶段任务计划](docs/superpowers/plans/2026-10-29-cboard-stage2.md)的任务0开始，先确认验收阈值并收口单轮残项。

- 比赛方向：高校联盟赛 3V3，先做重装机器人；当前主线是 C 板四轮全向底盘。
- 电调 ID 已人工确认：右前1、左前2、左后3、右后4，1/2 之间为车头。
  四轮正转从车外侧看轮心均为顺时针；方向表尚不据此修改，整车解算前须结合安装角度核对。
- C 板已固定在右侧1/4之间，使用板载 IMU，安装轴向待确认。
- 新增单轮速度模式3：KEY选轮，±600转子rpm正反阶跃和零速段，限流±1500，
  按键取消、掉线取消、超速取消，测试结束释放。详见 `cboard_chassis/README.md` 顶部。
- 主机速度测试通过，原映射/按键263项检查通过，Keil AC5实编译0错误0警告。
  用户已上板验证四轮正反转、KEY取消；2026-10-29直接CAN采集90秒、410146帧，四轮各一次完整测试。
  **功能验证通过，第一阶段定量验收未完成**：1/2/3号末段平均欠速，4号正转曾受外力。
  详细误差、曲线和3431条遥测见 `docs/experiments/2026-10-29-single-wheel/README.md`。
- 同日第二轮架空复测90秒、413604帧、3462条遥测；完整2→3→4→1，剔除开头不完整1号片段。
  末段平均正转欠速6.9%～13.0%、反转10.0%～16.3%；4号未重现上轮受扰近停，零速末段≤3rpm。
  本次未修改参数。对比与曲线：`docs/experiments/2026-10-29-single-wheel-run2/README.md`。
- 2026-10-29 已将模式3正反恒速段从各2秒改为各8秒，零速段仍各1秒，单轮共18秒；转速、PI、限流及取消保护不变。
  主机速度时序测试及原263项回归通过；Keil AC5实编译0错误0警告，用户已确认烧写。
- 长时复测已完成：299.969秒、1378646帧、11539条遥测，完整1→2→3→4→1；用户确认架空无接触。
  后2秒平均误差0.35%～3.06%，1号重复较好；仍有速度纹波及2/4号间歇下降，3号反向欠速最大。
  未获得持续±5%内保持的调节时间，未采样到限流/故障取消；报告见 `docs/experiments/2026-10-29-single-wheel-long8s/README.md`。
- 原始反馈/指令关联分析已完成：CAN指令最长间隔2.432ms，指令峰值1202未限幅；降速时有电流补偿。
  1号纹波与转子第7阶相关（约70Hz），2/4号另有低频降速；反转继承正向积分，约1.1～1.2秒才换为反向。
  详见 `docs/experiments/2026-10-29-single-wheel-long8s/DYNAMICS.md`，物理根因尚未唯一确定。
- Ki单变量对照0.002→0.004/2ms已由用户烧写，首轮采集1366467帧、11507条诊断。
  正常速度段后2秒平均误差0.10%～0.69%，反转积分换向约0.42～0.55秒；纹波及2/4号慢降速仍在。
  用户报告2号接触不良并更换CAN通信线：异常尝试有1500限流和约10.4秒反馈空档，出现反馈不可用取消。
  最后选入1/2/4号整轮完整且无饱和；3号速度段完整，但最后零速仅约0.72秒，不能称整轮验收完成。
  报告：`docs/experiments/2026-10-29-single-wheel-ki004/README.md`。2号换线影响不与Ki效果混为一谈。
- 3号完整补测已完成：119.958秒采集、551270帧、4614条诊断；活动18.013秒，末尾零速与退出后零电流均完整。
  正反后2秒均值593.82/−598.83rpm，误差1.03%/0.19%，无故障诊断或限流；纹波仍在。
  结合前轮1/2/4号记录，四轮均有完整单轮功能序列，尚非同一窗口的连续四轮定量验收。
  详见 `docs/experiments/2026-10-29-single-wheel-ki004-motor3/README.md`。
- 数据已完整保留并额外备份：五轮合计4120133帧、34553条诊断；5个原始记录ZIP及当前固件快照在本地
  `_probe/archives/2026-10-29-single-wheel/`，逐成员SHA-256读回校验通过。清单在`docs/experiments/2026-10-29-single-wheel-session/manifest.json`。
- 下次接续：保持模式3与Ki=0.004，先确认单轮平均误差、波动、超调及调节时间判据，固定可靠接线后重复验收。
  第二阶段规划为受限四轮联动→解算方向核对→低速基础运动；第一阶段进入条件未通过前不启动四轮运动。
  官方正常模式的速度单位、PID和16000电流上限尚未迁移验证，不能直接切模式0；拟议模式4尚未实现。
  之后基础运动→IMU姿态/解耦→功率保护→融合里程计。本次采集未修改固件参数。
- 已确认后续实际文件改动经检查后按明确路径提交推送，避免混入原有未跟踪资料。
- 会话收尾与下次接续入口：`docs/handoffs/2026-10-29-cboard-speed-loop.md`。

## 一句话状态

RDK X5 单类 person YOLOv8n 定稿：**576 + 三段流水线 + C++ 后处理 = 104.1 FPS**，
person AP 0.7178（640 为 0.7244）。切尾巴、量化、板端测速全部跑通。
2026-10-20 原 SD 卡损坏 → 重灌镜像 + 板端 30 min 重建，Odin1 链路已恢复（10 Hz 数据、网页预览经端口转发在 PC 上验证）。
另起 `car_slam_lap/`：把这套感知装到 C8T6 双驱小车上，跑「建图 → 重定位 → 走一圈」。
X5 侧、链路协议、固件改造都已完成，**Keil AC5 实编译通过（0 error / 0 warning，Code 10.9 KB）**。
2026-10-21 按实物原理图修正：CAN 是 SN65HVD230 裸收发器（不是串口转CAN模块），
电机链路已改回片内 CAN1 + 250K（实测）；**X5 键盘控制 + CAN 已通**。
2026-10-22 遥控器链路已打通并实车验证：本款接收机是 **18 字节通道数据、帧首没有 `0xA9`**
（原来死等帧头，字节全被丢）。固件改成按帧间空闲定界，三档映射实测定稿
（GPS=全速 / 中间=1/3 速 / 下面=急停），`rc_ok` 稳定、遥控能开车。下一步用遥控器给全屋建图。

2026-10-26 **`car_slam_lap` 端到端跑通**：遥控示教建图（venue2，330 点 / 15.66 m）→ 重定位
（`map←imu` 与录制末点差 **3 cm / 0.6°**）→ 纯追踪自动回放（249 点 / 14.19 m，横向误差
**rms 0.103 m**、无碰撞）。同日修掉三个“伪装成硬件问题”的软件 bug：Odin1 驱动**不读**
`-p config_file`（重定位配置从未生效）、`save_map.sh` 找错存图目录且不等传输完成、
`follow_path` 启动竞态 / 名称遮蔽 / SIGINT 处理。详见
`docs/issues/2026-10-26-odin-config-and-follow-path.md`。

**2026-10-23 起换 C 板做底盘**：`cboard_chassis/`（官方 `17.chassis_task` 原样 + 调车层）。
本赛季底盘是 **4 轮全向（45° X 型），不是麦轮**——解算符号与麦轮相同、只差 1/√2，
官方公式可直接用。C620 的 CAN ID 仍是 1/2/3/4，但**哪个 ID 装在哪个角、正反如何，尚未标定**。
2026-10-26 **固件已烧进 C 板**（Keil F8，`Verify OK / Application running`），
当时进入阶段0验ID；截至2026-10-29，ID已确认并切换为模式3单轮速度闭环。
Ki=0.004四轮均有完整单轮序列，3号补测末尾零速与释放已确认；2号换过CAN线，纹波和低频降速尚待定量验收，详见顶部接续和会话交接。

## 里程碑

| # | 里程碑 | 状态 | 说明 |
| :--- | :--- | :--- | :--- |
| 1 | 仓库与协作规则初始化 | ✅ 完成 | 2026-10-17，首次推送 |
| 2 | 通读三份 RM2027 手册，产出视觉相关约束清单 | ⬜ 待开始 | 文本已提取到 `_extract/` |
| 3 | 确定赛季视觉目标与量化指标 | ⬜ 待开始 | 依赖里程碑 2 |
| 4 | RDK X5 环境跑通（相机采集 + BPU 推理） | ⬜ 待开始 | 镜像已备 |
| 5 | 单类 person YOLOv8n 切尾巴 + X5 .bin | ✅ 完成 | 2026-10-17，AP 0.78 不变、量化 cosine ≥0.997 |
| 6 | X5 板端 512 输入达 100+ FPS | ✅ 完成 | 2026-10-18，C++ 后处理 135.3 FPS |
| 7 | 576 输入定稿（精度/速度折中） | ✅ 完成 | 2026-10-18，104.1 FPS、AP 0.7178 |
| 8 | 坏卡重灌 + Odin1 数据链路恢复 | ✅ 完成 | 2026-10-20，镜像校验一致、30 min 重建、端口转发验证 |
| 9 | 小车底盘串口化 + 建图走圈（练习） | ✅ 完成 | 2026-10-26 端到端跑通：遥控示教建图（venue2：330 点 / 15.66 m）→ 重定位（`map←imu` 与录制末点差 3 cm / 0.6°）→ 纯追踪自动回放（249 点 / 14.19 m，横向误差 rms 0.103 m、无碰撞）。修掉 Odin1 `config_file` 不生效 / `save_map.sh` 找错目录且不等传输 / `follow_path` 三处 bug，详见 `docs/issues/2026-10-26-odin-config-and-follow-path.md` |
| 10 | C板四轮全向底盘标定 | 🔶 进行中 | ID/轮位与四轮单轮功能序列已验证，五轮CAN数据已备份；Ki=0.004定量整定未验收。第二阶段四轮联动/基础运动计划已形成，尚未实施。 |
| 11 | RMUL2027 3V3 场地 1:1 入 Gazebo + 台面高度验收 | ✅ 完成 | 2026-11-01：12.600×8.600 m、落差 0.503 m 的 CAD 场地进 Jazzy/Harmonic，比例与碰撞量化验收（`report_field_test.json`）；台面高度像素级验收 501.3/494.1 mm（`report_height.json`），观感改造后 GUI 实拍六级亮度阶梯成立。详见 `docs/experiments/2026-11-01-rm-field-gazebo/README.md`。 |

## 正在做

- **C板底盘（当前主线）**：`cboard_chassis/`，已确认四轮ID，模式3单轮闭环已运行。
  正反恒速段各8秒、零速各1秒，共18秒，已烧写并完成架空采集；400rpm两轮重复与400/600对比已完成，
  2/4降速簇约每15转子转一次、随转速缩放（Δt比1.40～1.51、Δrev比0.94～1.00），1号无稳定段降速簇。
  保持Ki=0.004、模式3、限流1500，不同时增加滤波或放宽限流；下一步做1/2号轮组互换单变量实验（不烧写）。
  接续入口：`docs/handoffs/2026-10-29-stage1-to-stage2-next.md`。
  问题收口前不进入四轮联动；收口后回600rpm按原标准重新验收。
- 576 定稿；接入 USB 摄像头做端到端实测（720p60 / 480p120），之后换正式相机。
- Odin1 链路 2026-10-20 已恢复：板端起驱动 + 网页预览，PC 经 `plink -L 8080` 端口转发查看
  （`x5_odin/`，启动/打包/校验脚本齐全）。
- `car_slam_lap/`：三条链路都写完了 —— 协议（C+Python 同源，70+12 项断言）、`car_bridge`
  ROS2 包（桥接/键盘遥控/录轨迹/纯追踪/点云导出/速度标定）、固件（USART1=X5、
  USART2=遥控器、**CAN1=SN65HVD230 250K（实测）**、10 ms 仲裁、看门狗）、板端脚本与 Odin1 两份配置。
  自检五组全绿，其中第 5 组直接调 `UV4.exe` 真编译（Code 11152 / RAM 1408）。
  **2026-10-26 已端到端跑通**（建图 venue2 → 重定位 → 自动回放，rms 0.103 m）。接下来可选：
  - 掉头切角调参：max 误差 0.423 m 出现在远端 180° 掉头（纯追踪切角），降 `VMAX` / `lookahead_min`；
  - 用这套流程给**正式场地**建图：走完整环线、回到起点（<0.5 m）、**最后再** `save_map`；
  - 长期解：给 `bridge_node` 加 `passive:=true`（只收 status、不发帧），建图时遥控和状态可共存。
  X5 侧已部署，`/dev/car_link -> ttyS1` 已确认；键盘 `teleop_keys` 也能开车。
- **Gazebo 仿真（副线）**：`tb3_gazebo_nav2/`，RMUL2027 3V3 场地 **已翻转成结构面朝上**（2026-11-19），
  碰撞用 48 块 box 拼块（五项读数验收全过）；旧的 1:1 平整台面世界与围挡变体保留作历史。
  可行驶地面 ≈ 12.0×8.0 减两块平台，墙内侧面 x=±6.00 / y=±4.00。
  入口：`bash rm_field/gui_colored.sh`（GUI）、`bash rm_field/verify_box_collisions.sh`（碰撞验收）；
  文档见 `rm_field/README.md` §12/§13，历史 §9–§11 是翻转前结论。

## 下一步（按当前主线优先级）

1. 保持Ki=0.004和可靠接线，先做1号/2号完整轮组互换单变量实验（不烧写），用已冻结口径判断降速簇跟随轮组还是电机-减速箱；问题收口前不启动四轮运动，也不放宽600rpm验收门槛。
2. 按第二阶段计划推进几何/方向表核对、拟议受限模式4、四轮架空联动与低速六方向运动；每步单独验收，不直接切官方模式0。
3. 核对C板IMU轴向/偏移，完成航向保持与坐标解耦。
4. 功率保护及轮速/IMU融合里程计，逐阶段验收，不直接上导航。
5. 视觉赛规约束、正式相机链路和知识沉淀保留为后续任务，待底盘主线明确后接续。
6. （副线）Gazebo 场地：~~补围栏碰撞验收 → 顶视渲染出 `map.pgm/yaml` 接 Nav2~~ ✅ 2026-11-06 已完成
   （围挡 0.7 m/s 撞墙停位误差 0.4 mm 不越界；地图 12.50×8.50 m 自由区、两点往返 SUCCEEDED）。
   **2026-11-19 场地已翻转+改配色，旧地图失效**；box 碰撞已修（48 块，五项验收全过）。
   剩：①按新地面重建 Nav2 地图/起点/验收（自由区=地面减高台与墙）；②围挡高度 0.30 m 按赛规核对后再定稿；
   ③绕障/动态避障未测。
## 阻塞 / 风险

- **烧写只能用 Keil 界面按 F8**：`UV4 -f` 命令行在这台破解版 Keil 上必崩（`0xC0000409`），
  VS Code 只负责编译（已封成任务）。换电脑/重装 Keil 后需重新确认。
- **STM32CubeProgrammer 禁用**（2026-10-26 决定）：它与这颗 ST-Link 建立不了 SWD，
  但会报出 SN/FW/电压，看起来像"芯片没应答"，已把排查带偏一整轮。不要再用它烧录，
  也不要用它的输出判断硬件好坏。详见 `docs/issues/2026-10-26-cboard-flash-toolchain.md`。
- Keil 的 `Debug` 选项卡必须是 **ST-Link Debugger**（不是 ULINK2/ME），否则 F8 报
  `No ULINK2/ME Device found`；配置存在 `MDK-ARM/chassis_task.uvoptx`（用户级，换机要重设）。
- C 板底盘：**ID与轮位已确认**，但整车解算方向符号仍待联动核对，方向表全+1尚未经验证。原默认表
  不应仅凭“从外侧看顺时针”直接定案；完成单轮整定后再低速架空验证。
- 按键验 ID 只覆盖 1~4 号（`0x200` 一帧）；若实际编号是 5~8，需要再补 `0x1FF` 那条发送路径。
- 全向轮是否为 **45° X 型**未验证（手推侧面能平移才是；若是平行装法，解算要另写）。
- 底盘 `vx/vy/wz` 目前不是真实物理量（全向轮比麦轮快 1.41 倍），接 X5 导航前必须统一单位。
- RDK X5 镜像约 9.68 GiB 未入库：换机需按 `docs/environment.md` 重新获取并校验 SHA256。
- 规则为 RM2027 V1.0.0（20261015），后续版本可能调整，需定期核对。
- 相机型号、SDK/工具链版本尚未记录，补齐前实验不可复现。

## 最近变更

- 2026-11-01：**RMUL2027 3V3 场地在 Gazebo 里“看得见、量得出”收尾（副线）**。两轮：① 三台取证相机 + `verify_height.py`
  把 0.503 m 做成像素验收（正立面 1734×69 px → 501.3 mm / 宽度标尺 494.1 mm；近景 451 px = 画面 62.6%；
  斜视侧壁带 74 vs 75 px），11 条断言全过；② 用户反馈“还是平面 + 小车”后，把 GUI 相机搬进离屏渲染逐行读剖面，
  定位四级原因（俯角 12° 下 0.503 m 仅 20~40 px、地面≈背景、**投影≈天空**、单一材质无竖直参照），
  改光照/配色/相机（只改颜色，几何未动）+ 新增 0.30 m 围挡变体世界，改造后亮度阶梯
  天空 42/56、投影地面 88、亮地面 95/90、侧壁 117/123、围栏 137、台面 214，三条几何线预测↔实测差 ≤4.9 px。
  两个关键坑写进 issue：`<gui><camera><track>` 会覆盖 `--gui-config` 的 camera_pose；WSLg 抓 GUI 图不可靠
  （X 侧冻结帧、Windows 侧副屏裁剪），画面判断改用离屏相机。详见
  [实验记录](docs/experiments/2026-11-01-rm-field-gazebo/README.md)、[围挡决策](docs/decisions/2026-11-01-rm-field-fenced-gui-variant.md)、
  [GUI 观感 issue](docs/issues/2026-11-01-wslg-gui-visibility.md)。

- 2026-10-29：**400rpm第二轮采集 + 400/600单变量对比完成**。第二轮180.022秒、827033帧、6923条诊断，无需RESET，完整1→2→4各一次；无限流/故障、非选中轮零指令、释放正常；两轮摘要已独立复算并以ZIP逐成员SHA-256校验。对比结论：2/4降速簇约每15转子转一次，簇Δt比1.40～1.51、Δrev比0.94～1.00（转速比1.49～1.52），支持转动位置相关而非时间固定；1号无稳定段降速簇，7阶纹波47→70.5Hz；400rpm未改善2/4的相对波动。起转粘滞扫描为探索性观察（2/4在400段开始有0.08～0.86秒冻结）。建议下一项单变量为1/2号轮组互换（不改固件）。详见[400rpm定位实验](docs/experiments/2026-10-29-speed400-comparison/README.md)。
- 2026-10-26：**`car_slam_lap` 端到端跑通（建图 → 重定位 → 自动回放）**。venue2 实测：
  路径 330 点 / 15.66 m；重定位 `map←imu` 与录制末点差 **3 cm / 0.6°**；回放 249 点 /
  14.19 m / 167.6 s，横向误差 **rms 0.103 m**、无碰撞。修掉三处软件 bug：
  ① Odin1 驱动**不读** `-p config_file`（源码写死读
  `~/odin_ws/src/odin_ros_driver/config/control_command.yaml`），此前重定位从未真正进入
  mode 2；② `save_map.sh` 只看 `~/odin_ws` 且等 8 s 就拷（实际图存到 `~/car_ws/src/...`，
  传输是异步的）；③ `follow_path` 启动竞态、`save_actual` 名称遮蔽、SIGINT 抛
  `RuntimeError`。修复与验证见 `docs/issues/2026-10-26-odin-config-and-follow-path.md`。
  待改进：远端 180° 掉头切角（max 0.423 m），终点位置差 0.48 m 略超 A5 判据。
- 2026-10-26：**验 ID 的轮速调慢**（实测 +2500 空载约 3 A，轮子转太快、看不清是哪个角）。
  `CHASSIS_KEY_SPIN_CURRENT` 2500 → **900**（约 1.1 A），时长仍 1.5 s；
  阶段 1 的 `CHASSIS_SWEEP_CURRENT` 3000 → **1000**。两处都只改电流值，时序、状态机、
  263 项断言不变。转速基本正比于电流，**低于 ~500 会顶不过静摩擦**（只嗡嗡响不转，
  不是电机坏了）；调法写进 `cboard_chassis/README.md` §2「转得太快 / 想再慢一点」。
- 2026-10-26：**C 板固件烧进去了**（Keil F8，`Erase Done / Programming Done / Verify OK / Application running`）。
  烧写这条路上踩了三个独立的坑，全部记进 `docs/issues/2026-10-26-cboard-flash-toolchain.md`：
  ① 缺 `Keil.STM32F4xx_DFP` → `UV4 -f` 静默无动作；
  ② `Debug` 选项卡选成了 `ULINK2/ME` → F8 报 `No ULINK2/ME Device found`（Utilities 显示
  `-- Use Debug Driver --` 时烧写跟随 Debug 页）；
  ③ **STM32CubeProgrammer 在这台机器上连不上 ST-Link**，却照样报出 SN/FW/电压，
  看起来像芯片没应答，把排查方向带偏了一整轮 —— **以后禁用**。
  另外破解版 Keil 的 `UV4 -f` 批处理必崩（`0xC0000409`），所以最终定下：
  **VS Code 编译（`Ctrl+Shift+B`）+ Keil 界面 F8 烧写**，任务已封装好。
  下一步：上电按 KEY 核对四个电调的编号。
- 2026-10-25：**新增阶段 0「按键验 ID」**（当前默认模式）。底盘已架起、四个电调重新编号，
  需要核对"电调里设的编号"和"实际是哪一路"。做法：按 C 板 KEY（PA0，内部上拉、按下为低）
  N 次，**最后一次之后静默 3 s** 就驱动 N 号电调 1.5 s（+2500）；按超 4 次清零不驱动；
  转动中按键不打断但会记账。每按一次蜂鸣器"嘀"一声（调车模式下 `test_task` 已静音，
  否则它会对永远掉线的 yaw/pitch/trigger 一直报警），`0x1F1[7]` 同时显示计数。
  3 s 判定写成纯状态机，`tests/` 里精确卡到毫秒边界（含连按重置、转动中按键、超界），
  263 项断言通过；Keil 三种模式均 0 Error / 0 Warning。详见 `cboard_chassis/README.md` §2 阶段 0。
- 2026-10-23：**C 板底盘调车层完成（待上电）**。基线 `17.chassis_task` 原样编译 0/0
  （Code 63192），说明框架本身没问题；问题在于第一次上电时**看不见哪一步没通**。
  新增 `cboard_chassis/`：官方文件只插 5 处 `[BRINGUP]`，另加 3 个文件——
  单电机扫描（不依赖遥控器/陀螺仪，按裸 ID 1~4 正反转）、轮位↔CAN ID↔方向两张标定表、
  `0x1F0`/`0x1F1` 调试帧（USB-CAN 分析仪直接可读）、启动死等改 3 s 超时。
  底盘改**全向轮**后仍可用官方解算（符号与麦轮相同，只差 1/√2，给定 1 m/s 实际约 1.41 m/s）。
  纯逻辑自测 171 断言（恒等/打乱/重复 ID/越界四套表）通过；Keil 实编译 0 Error / 0 Warning，
  Code 64424。详见 `cboard_chassis/README.md`、`docs/decisions/2026-10-23-cboard-chassis-bringup.md`。
- 2026-10-22：**遥控器 DBUS 定位并修好（待烧写验证）**。现象是 `/car/status`
  `status=51`（`can_ok|link_ok|can_ack|uart2_rx`）、无 `rc_ok`/`dbus_frame`。
  抓包 `_extract/ttl.bin`（1026 B / 100000 8E1）周期严格 18 字节、`A9` 出现 **0 次**，
  按位布局解出来是 `ch=0,0,0,0, s1=1, s2=3`（摇杆居中、s2 急停档）——数据本身没问题，
  是**这一款接收机的帧首没有 `0xA9`**，而固件死等帧头，每个字节都被丢。
  修复：`DBUS.c` 改成按帧间空闲（>2 ms）定界，兼容 18 字节无帧头 / 19 字节带帧头；
  加保留字节校验防噪声假帧；新增 `tests/dbus_frame_test.c`（46 项），
  `run_tests.ps1` 五组全绿，Keil 实编译 0 Error / 0 Warning（Code 11152）。
  补记：第一版还要求 s1/s2 必须为 1/2/3，但这台遥控器 OFF 档编码就是 0，
  切到 OFF 后合法帧被误杀（status 127→115、`rc_ok` 消失）；已允许 s1/s2=0。
  另把 `uart2_rx` 从锁存位改成 **500 ms 实时位**，接收机中途停发时不再被旧帧掩盖。
  S2 映射按用户要求改为 **GPS(1)=全速 / 中间(3)=三分之一速 / 下面(2)=急停**
  （`DBUS_ESTOP_S2=2`，`gain` 判 3）。
  详见 `docs/issues/2026-10-22-rc-dbus-not-decoded.md`。
- 2026-10-21：**CAN 也通了（实测 250K）**：`/car/status` → `status=19`、
  `flags=["can_ok","link_ok","can_ack"]`、`src="serial"`、`crc_errors=0`；复位后轮子能转。
  踩坑：① PA9/PA10 插错 + 烧完没断电复位 → `/dev/ttyS1` 0 字节；
  ② 500K 下 `can_ok=0`，改 `can.c` Prescaler=8（250K）后 `can_ok`/`can_ack` 都出现。
  下一步：离地验方向、`calibrate_speed` 标定、建图走圈。
- 2026-10-21：**X5 侧已部署到 172.17.90.239**（`deploy.ps1 -Install`，colcon build 通过，
  `can_ack` 代码已在板端，`/dev/car_link -> ttyS1`）。C8T6 还需重烧
  `MDK-ARM/C8T6_CAN_car/C8T6_CAN_car.hex`，否则板上仍跑旧的 USART3/slcan 固件。
- 2026-10-21：**接线拓扑再修正一次：CAN 是 SN65HVD230 裸收发器，不是串口转CAN模块。**
  用户补了 `CAN收发器原理图.pdf` / `CAN收发器说明书.pdf`：板子只有 SN65HVD230 + 120Ω
  + 10K，H1 = GND/3V3/R/D，H2 = CANH/CANL，没有 MCU、没有 UART。原来的
  「USART3 → slcan → 模块」物理上不可能工作。已改回 **片内 CAN1（PA11/PA12）**：
  `can_frame.c` 纯帧编码 + `can_bus.c` 非阻塞发送 + ACK 解析（status b4）；
  `can.c`/`.ioc` 设 250K（实测；PDF 写 500K）；`slcan.c/h` 与测试删除；Keil 实编译 0 error / 0 warning。
  接线、上电顺序、终端电阻、控制器 PowerKey/Break 见 `car_slam_lap/WIRING.md`。
  决策记录：`docs/decisions/2026-10-21-can-direct-sn65hvd230.md`。
- 2026-10-20：**小车串口链路通了（验收 A1 通过）**。RDK(172.17.90.239) 上部署 `car_bridge` 并
  跑通 `link_test`：300 帧发 / 70 帧收、**10.00 Hz**、最大间隔 101 ms、`crc_err=0`、
  仲裁 `src=serial`、停发后 `LINK_OK` 正确掉落（300 ms 看门狗真实有效）。
  排障过程：① `ros2` 找不到 → 每个 shell 要 source；② `rm@local` 邮箱非法导致 colcon 解析
  manifest 失败、包装上但 `ros2 pkg` 找不到；③ 一帧上行都没有 → **串口 ISP 烧写后 BOOT0 忘了
  拨回 0**，芯片跑的是系统 bootloader，固件根本没执行。另外确认 RDK 40pin 串口确实是
  `/dev/ttyS1`（`pinctrl-handles` 里 `34070000.serial` 绑定 `uart1grp`），而 `ttyS5` 是蓝牙
  （`hciattach` 占着，写它会阻塞）。这几条已写进 `car_slam_lap/README.md` §11。

- 2026-10-20：新增 `car_slam_lap/`（原 `practice/`）——把 Odin1 + X5 装到 C8T6 双驱小车上。
  控制源从 DT7 遥控器改为 **X5 串口 → C8T6 → CAN**，遥控器保留作手动驾驶与硬急停。
  协议：9 字节下行 / 7 字节上行、CRC-8/ATM、50 Hz/10 Hz，C 与 Python 两侧断言同一组黄金向量
  （`tests/run_tests.ps1` 全绿）。建图/走圈走 Odin1 原生 SLAM（mode 1）+ 重定位（mode 2）+
  录路点纯追踪回放，不引 FAST-LIO2/nav2。顺带确认 **CAN 实际是 250 kbit/s**（文档写 500K 有误）、
  驱动 TF 链是 `odom→imu→lidar`（无 `odin1_base_link`）。决策见
  `docs/decisions/2026-10-20-car-slam-lap-route.md`。
  固件改动清单已列。
- 2026-10-20：**修正 `car_slam_lap/` 的链路拓扑**（第一版按代码反推，与实物不符）。
  实际是：USART1(PA9/PA10, 115200 8N1) ← RDK X5 40pin 第 8/10 脚（节点 `/dev/ttyS1`，
  官方文档确认）；USART2(PA2/PA3, 100000 8E1) ← DT7 接收机；USART3(PB10/PB11) → **外挂
  「串口转CAN 模块」**（slcan 文本，`T000013148…\r`，启动发 `S5`+`O`）→ 250K CAN。
  固件随之完成：`uart_rx.c` 统一接收中断、`x5_link.c`、`slcan.c`、10 ms 仲裁 + 双看门狗，
  并修掉原工程的 `rc_tick` 恒假超时、CAN 阻塞狂发、`HAL_UART_RxCpltCallback` 重复定义三处缺陷。
  离线检查四组全绿：C 断言 70+21、6 个固件模块语法检查、Python 19、Keil 工程一致性。

- 2026-10-20：原 SD 卡损坏 → 重灌 `3.5.0` 镜像（SHA256 复核一致，首启自动扩容到 29 GB）；
  板端 30 min 重建 odin_ros_driver + ROS2 依赖；Odin1 恢复 **10.11 Hz 点云 / 10.31 Hz 图像 / 355 Hz IMU**；
  网页预览经 `plink -L 8080:127.0.0.1:8080` 在 PC 端 `http://127.0.0.1:8080/` 验证通过。
  过程、脚本与坑见 `x5_odin/README.md` §11。

- 2026-10-17：初始化仓库；加入协作规则、环境清单、issue/decision 模板；首次推送。
- 2026-10-17：完成单类 person 切尾巴（`x5_yolo/`）；494 张 COCO person AP@0.5 0.7800 不变；
  opset 11 + hb_mapper 1.24.3 本地编出 3.7 MB `.bin`；决策见 `docs/decisions/2026-10-17-yolo-person-head-x5.md`。
- 2026-10-18：板端实测 640 73.9 FPS、512 115.2 FPS；512 `.bin`（3.4 MB）与快速包已交付。
- 2026-10-18：C++ 后处理（ctypes）上线；512 135.3 FPS、640 87.2 FPS。
- 2026-10-18：576 定稿，104.1 FPS、AP 0.7178；三档对比见
  `docs/decisions/2026-10-18-yolo-input-576.md`。
- 2026-10-18：USB2 MJPG 摄像头实测（软解瓶颈 24/48/11 FPS），待换 CSI/USB3；
  aiVault 草稿见 `docs/aiVault/2026-10-18-rdk-x5-yolo-int8-tailcut.md`。

