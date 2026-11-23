# 2026-11-23 晚 哨兵仿真：交互化 + 导航调参 + "贴脸未识别"定位

> 一句话：把 sentry_sim 从"能跑巡逻"推到**可交互**（点地图开哨兵 / 遥控敌车）、
> **绕障不钻缝**（footprint + 大 inflation）、**提速 2.7x**（MPPI temperature），
> 并把用户报的"敌车突然出现撞上去 / 卡缝里挤过去"定位到**感知盲区**（未修，见 §6）。

## 1. 环境与分支

| 项 | 值 |
|---|---|
| 机器 | Windows 本机 + WSL Ubuntu 24.04，ROS 2 Jazzy，Gazebo Harmonic 8.15 |
| 工作区 | `~/sentry_ws`（source of truth 在仓库 `tb3_gazebo_nav2/sentry_sim/`） |
| 仓库 | `E:\RM` → `github.com/ethan123-zhang/RM_Vision`，分支 `codex/car-local-obstacle-avoidance` |
| 本轮提交 | `8281e49` 卡墙自愈 · `9d2406f` 双人模式 · `b647ffc` footprint/inflation/NoTouch · 收尾提交见 §8 |

## 2. 问题表（现象 → 根因 → 修法 → 验证）

| # | 现象（数字/日志） | 根因 | 修法 | 验证 |
|---|---|---|---|---|
| 1 | 车停在墙边后"点哪都不动"；`Failed to create plan from (-3.90, 0.33)` 共 **42 次** | 车所在点净空 **0.350 m** < `robot_radius` 0.60 → `global_costmap` 把**规划起点**判成致命区，NavFn 一律失败 | 落点净空门槛 0.65→**0.85 m**（goal 容差 0.2 → 停位净空 ≥0.65）；面板加**自动脱困**：净空 <0.72 时沿净空梯度倒车（0.25 m/s，走 `cmd_vel_nav` 安全链）再重发目标 | `RESCUE (-3.90,0.33) clear=0.35 → (-4.34,0.33) clear=0.80` → `Goal succeeded`，到点 0.23 m |
| 2 | 手动关掉碰撞监控后**车完全不动**（遥控"打不开"） | `collision_monitor` 被 **lifecycle deactivate** → 断 bond → `lifecycle_manager` 判定节点失联，把 `bt_navigator` 等整套导航节点停掉（日志 `CRITICAL FAILURE: SERVER collision_monitor IS DOWN`） | 裸模式改走**动态参数** `FootprintApproach.enabled=false`（节点保持 active、bond 不断）；遥控窗口启动/退出都强制安全模式 | `SET_GUARD enabled=True -> True`，随后 `/cmd_vel_nav 0.50 → /cmd_vel_smoothed 0.50 → /cmd_vel 0.00`（前方 0.68 m 有墙被拦） |
| 3 | 在 Gazebo GUI 里插模型会"死机"，车被推去撞墙 | llvmpipe 软渲染 + 动态模型持续重绘：`gz sim gui` **349% CPU**、RTF **0.10**；落下的球/箱子把车推到墙边 | 用 `tools/spawn_obstacle.py`（server 侧 **static** 模型）替代 GUI 插入 | 插入后 RTF 0.7–0.8，无卡死 |
| 4 | 遇到障碍不绕、从 0.8~1.1 m 的缝里挤过去 | `robot_radius 0.40` 时 NavFn 把窄缝当正常通路 | `robot_radius → 0.60`（后改半径 0.60 的**圆形 footprint**）；连通性离线校验：(3,−2.5)/(0,3)/(−4.1,−3.4) 仍可达 | 静态敌车拦路：绕障面距 **+0.236 m**、横向偏移 0.708 m、静态净空 0.861 m（8.4 s，峰值 0.98 m/s） |
| 5 | 点地图把目标点在敌车上，车会顶上去 | 落点吸附只认**静态 CAD 图**，看不见临时障碍 | 面板订阅 `/global_costmap/costmap`：目标落在代价 ≥85 的格就挪到 1 m 内最近空地，挪不开就拒绝；**代价图超过 2 s 未更新则跳过该检查**（避免旧帧把空地误判 99） | 点敌车正中心（代价 99）→ 自动挪 0.6 m 外 |
| 6 | 车速太慢 | MPPI 的**总闸是 `temperature`**（0.3 时实际只跑 ~0.2 m/s），不是 `vx_max`；另 `time_steps 56/batch 2000` 在 RTF≈0.7 时控制器只到 0.9 Hz | `temperature 0.05`、`vx_std 0.4`、`vx_max 2.0`、smoother 上限 1.2、`time_steps 40 / batch_size 1000` | 同路线 **22.3 s → 8.2 s**，峰值 **1.35 m/s**（控制环回到 15 Hz） |
| 7 | **"敌车突然跑到车前面，识别不到、不躲避；敌车卡在墙缝里时从边缘挤过去（面贴面）"** | **感知盲区**：敌车高 0.25 m、雷达装在 0.30 m，且仿真雷达 `lidar_slices:=1`（单帧稀疏）→ 1 m 内扫不到；贴脸时 `/scan` 和两张代价地图都收不到点 | **未修**（见 §6 a+c） | 复现数据：半路出现敌车 → 最小中心距 **0.450 m（= 面接触）**、停滞 5.1 s、路径 11.4 m（直线 2.6 m）；同一时刻敌车若已被扫到，代价为 99/100 |

## 3. 本轮验收数据（实测）

| 项 | 数值 |
|---|---|
| 点地图导航到点误差 | 0.163 / 0.107 / 0.189 / 0.193 m（4 次，`Goal succeeded`） |
| 静态敌车绕障（5/5 判据） | 面距 +0.236 m、横向 0.708 m、静态净空 0.861 m、8.4 s、峰值 0.98 m/s |
| 移动敌车横穿（5/5 判据） | 到点 0.169 m、与"当时敌车位置"最小中心距 1.082 m（面距 +0.632 m）、12.3 s、峰值 1.38 m/s |
| 双人模式 | 敌车遥控 3 s：`(3.00,−2.50) → (4.32,−2.50)`；点地图 (0,3)/(3,−2.5) 坐标精确（`0.000/3.000`、`3.000/−2.500`） |
| 半路出现敌车 | **FAIL**：中心距 0.450 m（面接触）、最长停滞 5.1 s、11.4 m 路径 |

## 4. 当前状态

- 本机 WSL：`cd ~/sentry_ws && DISPLAY=:0 ./run_sentry_nav.sh gui:=true map_click:=true` 可起；21 个 nav2 节点 active，RTF 0.78。
- 交互：点地图开哨兵（已验收）、遥控敌车窗口 `enemy_teleop.py`、自动漫游敌车 `wander_obstacle.py`、脱困按钮/R 键。
- 未解决：**贴脸识别的感知盲区**、缝宽判定与"过不去就停"、重规划频率。

## 5. 未完成 / 卡点

| 项 | 卡在哪 |
|---|---|
| **a. 低位 2D 安全雷达** | 需改 `sentry.urdf.xacro`（z≈0.10、360°、0.05–2.5 m 的 ray 传感器）+ `sentry_sim.launch.py`（桥 `/scan_safety`）+ 两张 costmap 的 obstacle_layer 源 + collision_monitor sources；**未动** |
| **c. 5 Hz 重规划 + 缝宽判死路** | 需自定义 BT XML（`RateController hz=5`）与规划前缝宽检查（<2×0.6 m 判死路）；**未动** |
| b. `lidar_slices` 1→3 | 未试；CPU/RTF 风险（当前 RTF 0.78） |
| 敌车物理推力 | 现为 static 模型（`set_pose` 推），不参与物理；dynamic 版未验收 |
| NoTouch 硬停回归 | r=0.5 m 多边形可能在窄通道偶发停车，未做长跑回归 |

## 6. 下次进入点（下个对话直接从这开始：a + c）

```bash
# 1) 起栈（带点地图窗口）
cd ~/sentry_ws && DISPLAY=:0 ./run_sentry_nav.sh gui:=true map_click:=true
# 2) 复现"半路突然出现"（当前 FAIL 的用例）
python3 tools/verify_detour.py --goal 0 2.6 --obstacle 0 1.2 --spawn --spawn-after 1.5
# 3) 复现"卡在缝里"（把敌车放在墙与箱子之间，量面距/是否停）
python3 tools/spawn_obstacle.py spawn 2.0 1.0 ; python3 tools/verify_detour.py --goal 3.5 -1.5 --obstacle 2.0 1.0
```

a 的落点：`sentry_description/urdf/sentry.urdf.xacro` 加传感器 → `sentry_sim.launch.py` 桥 `/scan_safety`
→ `sentry_nav2.yaml` 的 `local_costmap.obstacle_layer.observation_sources` 加 `scan_safety`、
`collision_monitor.observation_sources` 加 `scan_safety`；验收判据沿用 `verify_detour.py`
（面距 ≥0.05 m 且到点成功）＋ 缝宽 <1.2 m 必须 ABORT/停车。

## 7. 相关文件

- 工具：`tools/{map_click, teleop_gui, enemy_teleop, wander_obstacle, spawn_obstacle, moving_obstacle, verify_detour}.py`
- 配置：`src/sentry_bringup/config/sentry_nav2.yaml`（footprint 0.6 圆、global inflation 1.5/5 Hz、NoTouch 硬停、MPPI temperature 0.05）
- 文档：`sentry_sim/README.md` §2.5（点地图+脱困）、§2.6（敌车障碍）、§2.7（随机漫游+遥控）、§2.8（双人模式）、§7 坑 11–17
- 报告：`results/report_detour_enemy_on_path.json`、`report_detour_enemy_moving.json`、`report_detour_213949.json`（半路出现的 FAIL 证据）

## 8. 提交

- `8281e49` fix(sentry): 点地图卡墙自愈（落点 0.85 + 自动脱困）
- `9d2406f` feat(sentry): 双人模式（点地图开哨兵 + 遥控敌车）+ 代价图新鲜度保护
- `b647ffc` tune(sentry): footprint 感知 CostCritic、全局 inflation 1.5 m/5 Hz、NoTouch 硬停多边形
- 收尾提交：见 `git log`（本文件 + PROGRESS/MEMORY）

## 9. 遗留风险

- **感知盲区未解**：比赛前必须完成 a，否则"敌车突然出现"仍会接触（实测面接触）。
- 大 inflation（全局 1.5 m）会让规划路径偏长；窄通道场景需回归。
- 敌车为 static 干扰模型，真实对抗（动态、能顶车）未验证。