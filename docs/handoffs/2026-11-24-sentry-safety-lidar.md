# 2026-11-24 哨兵仿真：a = 低位 2D 安全雷达（不撞了，但窄处到不了点）

> 一句话：把"敌车 0.25 m 高、Mid-360 在 0.30 m 看不见"的盲区补上了 —— 新增离地 0.18 m 的
> 360° 安全雷达 `/scan_safety`，进两张 costmap + `collision_monitor`。**同一场景从"面贴面
> (−0.000 m)"变成"面距 +0.479 m"**。贴脸场景"过不去"的部分由 c 收口：**5 Hz 重规划 +
> 缝窄判死路（0.8 s 判定 → 取消 → 停住）**，不再原地磨 200 s。

## 1. 环境与分支

| 项 | 值 |
|---|---|
| 机器 | Windows 本机 + WSL Ubuntu 24.04，ROS 2 Jazzy，Gazebo Harmonic |
| 工作区 | `~/sentry_ws`（source of truth = 仓库 `tb3_gazebo_nav2/sentry_sim/`） |
| 仓库 | `E:\RM` → `github.com/ethan123-zhang/RM_Vision`，分支 `codex/car-local-obstacle-avoidance` |
| 跑法 | `cd ~/sentry_ws && ./run_sentry_nav.sh gui:=false`（默认带安全雷达） |

## 2. 改动清单（4 处代码 + 2 个工具）

| 文件 | 改动 |
|---|---|
| `src/sentry_description/urdf/sentry.urdf.xacro` | `base_link` 加 `gpu_lidar`：`<pose>0 0 0.09 0 0 0</pose>`（离地 **0.18 m**）、360°/360 线/10 Hz/0.05–2.5 m、gz 话题 `/scan_safety_raw`；新增 xacro 开关 `safety_lidar` |
| `src/sentry_bringup/launch/sentry_sim.launch.py` | 桥 `/scan_safety_raw`；起 `safety_scan_relay.py`；透传 `safety_lidar` 开关 |
| `src/sentry_bringup/launch/sentry_nav2.launch.py` | 透传 `safety_lidar` 开关 |
| `src/sentry_bringup/scripts/safety_scan_relay.py`（新） | 改 `header.frame_id` 为 `base_footprint`；raw 超 0.5 s 无数据则发空扫描（防 collision_monitor "invalid source" 冻车） |
| `src/sentry_bringup/config/sentry_nav2.yaml` | 局部/全局 `obstacle_layer` 加 `scan_safety` 源；`collision_monitor.observation_sources` 加 `scan_safety`（`min_height 0`）；`FootprintApproach.time_before_collision` 1.0 → **0.4** |
| `run_sentry_nav.sh` | 清场 PATTERN 加 `safety_scan_relay`（原漏杀，见 §4 事故） |
| `tools/check_scan_safety.py`（新） | `/scan_safety` 体检 5 项：频率/束数/量程/无自身回波/TF |
| `tools/verify_detour.py` | 等 TF 改墙钟 20 s（原"200 次 spin"在消息密集时不到 1 s 就转完）；`wait_for_server` 15→30 s；新增 `--dump` 轨迹 CSV |

## 3. 问题表（现象 → 根因 → 修法 → 验证）

| # | 现象 | 根因 | 修法 | 验证 |
|---|---|---|---|---|
| 1 | 敌车 0.25 m 高、雷达 0.30 m：贴到 1.2 m 以内就"看不见" | Mid-360 单层扫描平面在 0.30 m，高于敌车顶面 0.25 m | 加离地 0.18 m 的 2D 安全雷达 | 敌车摆到 (0,1.2)：`/scan` 最小 **1.426 m**（只有墙）vs `/scan_safety` **1.000 m**（敌车正面）；costmap 99/100 |
| 2 | 安全雷达数据到了，costmap 却一句 `Message Filter dropping message: frame 'sentry/base_footprint/safety_lidar'` 就整帧丢掉 | gz 传感器 frame 是作用域名，TF 无此名 | relay 改 `frame_id=base_footprint`（传感器 x/y 偏移 0，精确） | 日志 0 条 drop；体检 frame_id PASS |
| 3 | `safety_lidar:=false` 时整车 **216 s 纹丝不动** | costmap/碰撞监控里列着 `scan_safety`，但无发布者 → `Robot to stop due to invalid source` | relay 兜底发空扫描 | 关雷达跑同一用例：`invalid_source=0`，正常导航（面距 −0.000 m，即旧行为） |
| 4 | 一次 200 s 的跑法 `Robot to approach` 切了 **1099 次**、车压到 0.05–0.2 m/s | `FootprintApproach.time_before_collision 1.0 s` ≈ 前方 1.6 m 内就减速 → 来回切 | 1.0 → 0.4 s（1.2 m/s 上限、3 m/s² 减速，停车距离 0.24 m；硬停仍由 NoTouch 0.5 m 兜底） | 同类跑法 approach 事件降到 10–17 次 |
| 5 | 重启多次后车越来越慢（RTF 1.0 → 0.48、load 13） | `run_sentry_nav.sh` 清场 PATTERN 没含新节点 → 每次重启漏一个 `safety_scan_relay` 孤儿（各 40% CPU），攒到 13 个 | PATTERN 补 `safety_scan_relay` | 清场后 `pgrep -c` = 0、RTF 回到 0.98–1.00 |
| 6 | `verify_detour` 偶发"拿不到 map->base_footprint" / "action 15 s 没就绪" | 等 TF 按迭代次数（不是墙钟）；重启后 DDS 发现慢 | 改墙钟 20 s；action 等 30 s | 连跑不再假失败 |

## 4. 验收数据（干净环境，RTF 0.95–1.00）

| 项 | 读数 |
|---|---|
| 传感器体检（`check_scan_safety.py`） | 10.0 Hz / 360 束 / 0.05–2.5 m / 最小有限 1.500 m（墙，无自身回波）/ frame_id `base_footprint` → **5/5** |
| 盲区对比 | `/scan` 1.426 m vs `/scan_safety` 1.000 m（敌车车头 1.2 m 处） |
| costmap 落地 | 敌车中心代价 **99**、前表面 **100**（全局图，0.025 m 栅格） |
| **贴脸场景 A/B**（goal (0,2.6)、敌车 (0,1.2) 1.5 s 后出现） | **雷达关**：`SUCCEEDED` 0.184 m、**面距 −0.000 m（面贴面）**、90.6 s、9.46 m<br>**雷达开**：面距 **+0.479 m**、到点失败（2.427 m）、202.9 s、19.95 m |
| 开阔场地（雷达开，5/5 判据） | 敌车 1.5 s 后出现：到点 0.109 m、面距 +0.943 m、18.6 s、停滞 0.1 s<br>敌车先摆好：到点 0.168 m、面距 +1.064 m、9.4 s |
| 无障对照（同目标 (0,2.6)） | `SUCCEEDED` 0.192 m、**9.6 s**、2.49 m 直线 → 说明"到不了点"不是目标点或 CPU 问题 |

报告：`results/report_detour_191713.json`（关）、`report_detour_191452.json`（开）、
`report_detour_185403.json` / `_185444.json`（开阔场地）、`report_detour_192500.json`（inflation 0.9 反证）。

## 5. c（同日完成）：5 Hz 重规划 + 缝窄判死路

| 项 | 做法 | 实测 |
|---|---|---|
| 5 Hz 重规划 | `src/sentry_bringup/bt/sentry_navigate_to_pose.xml`（复制 Nav2 默认 BT，只把 `RateController hz` 1.0 → 5.0），launch 用 `default_nav_to_pose_bt_xml` 注入 | `/plan` 平均 **4.48 Hz**（改前 1 Hz） |
| 缝窄判死路 | `tools/path_guard.py`：把全局代价地图 `>=99`（99=内切/100=致命）当障碍做距离变换，取 `/plan`（跳过起点 0.6 m、终点附近 0.4 m）的**最小净空**；< **0.20 m** 连续 2 帧 → 判死路 | 开阔路径净空 **0.43–0.65 m**；窄缝 **0.05 m** → 阈值 0.20 有 3 倍余量 |
| 判出来之后 | 客户端取消目标（ROS 2 没有"按条件取消别人目标"的公开 API，取消要发起方的 goal handle）：`map_click.py` 的 `RosBackend.dead_end()` + `tools/verify_dead_end.py` | 起点后 **0.8 s** 判定 → `CANCELED`，取消后位移 **0.00 m**，面距 **+0.665 m** |
| 反向回归 | 开阔场地不许误判 | 不判定、`SUCCEEDED` 0.16 m、14.6 s、路径净空最小 0.43 m |
| 绕障回归 | 5 Hz 后重跑 S3 两个用例 | 各 **5/5 判据过**：静态敌车 14.2 s、面距 +0.978 m；半路出现 15.6 s、面距 +0.927 m |

**踩到的坑**：①第一版判据用"代价阈值 200"，但 nav2 发布的 OccupancyGrid 是 0–100 尺度
（100=致命/99=内切），永远判不出来（全程 `max_cost=100`）→ 改成"对内切区做距离变换取净空(m)"。
②算距离变换要按"新的一帧 /plan"节流（5 Hz），否则 GUI 10 Hz tick 里每帧都做一次 180k 格 EDT。
③换 BT XML 三件事必须齐：装到 `share/.../bt`、launch 传真名、**用量测确认**（`ros2 topic hz /plan`），
少一件不报错、只是安静地还是 1 Hz。

## 6. 未完成 / 卡点

| 项 | 卡在哪 |
|---|---|
| 1.2–1.3 m 走廊"真的过去" | 现在按死路处理（取消+停）。要真过只能调 footprint/底盘参数（footprint 半径 0.6 的 inscribed 已是 0.6），属调参，未做 |
| 死路判定只在客户端 | `ros2 action send_goal` 直接发的目标没人盯路径 → 不自动取消；要保护就走点地图窗口/`verify_dead_end.py` |
| 5 点巡逻长跑回归 | 5 Hz 后只回归了绕障两例，`patrol_s2a` 未重跑 |
| 敌车物理推力 | 仍是 static 模型（`set_pose` 推），真实对抗未验证 |

## 7. 下次进入点

```bash
cd ~/sentry_ws && ./run_sentry_nav.sh gui:=false          # 默认带安全雷达 + 5 Hz BT
python3 tools/check_scan_safety.py --frames 8             # 5/5
# 贴脸场景：不撞 + 判死路 → 0.8 s 内取消并停住（c 的验收）
python3 tools/verify_dead_end.py --goal 0 2.6 --obstacle 0 1.2 --spawn --spawn-after 1.5 --remove
# 开阔场地：不误判、5/5 绕障（a+c 的回归）
python3 tools/verify_dead_end.py --goal 3 -2.5 --obstacle 6 6 --expect-no-dead-end
python3 tools/verify_detour.py --goal 3 -2.5 --obstacle 1.5 -1.25 --spawn --spawn-after 1.5 --remove
# 还没跑的长跑回归
python3 tools/patrol_s2a.py        # 5 点巡逻（改 5 Hz 后未重跑）
```

## 8. 相关文件

- 工具：`tools/check_scan_safety.py`（新）、`tools/path_guard.py`（新，死路判定）、
  `tools/verify_dead_end.py`（新，c 验收）、`tools/verify_detour.py`（`--dump`）、`tools/map_click.py`
- 配置：`src/sentry_bringup/config/sentry_nav2.yaml`、`src/sentry_bringup/bt/sentry_navigate_to_pose.xml`（新）、
  `src/sentry_description/urdf/sentry.urdf.xacro`
- 网关/兜底：`src/sentry_bringup/scripts/safety_scan_relay.py`

## 9. 提交

见 `git log`（本轮：`feat(sentry): S3a 低位 2D 安全雷达…`、`feat(sentry): S3c 5 Hz 重规划 + 缝窄判死路…`）。

## 10. 风险 / 注意

- 安全雷达进 `collision_monitor` 后，NoTouch 0.5 m 硬停变"真的有效"（旧 `/scan` 源因 `min_height 0.15`
  一直空转）。窄通道里可能偶发停车 —— 建议下一次做 5 点巡逻长跑回归。
- 全局 inflation 仍 1.5 m（b647ffc 的值），本轮实测降到 0.9 对窄缝无效也未回退验证其他场景 → 未改动。
- 死路判定在客户端；`ros2 action send_goal` 直发的目标不享受该保护。
- 敌车是 static 模型（`set_pose` 推），不参与物理；真实对抗仍未验证。