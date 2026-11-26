# 2026-11-26 哨兵仿真：footprint 0.62 收口 + 幽灵障碍根因 + 动力学敌车 + 比赛要素（相机/装甲板/让行）

> 承接 [2026-11-24-sentry-safety-lidar.md](2026-11-24-sentry-safety-lidar.md) 的遗留项
> （① 1.2–1.3 m 走廊"真的过去" ② 巡逻规划失败遥测/leg4 慢 ③ 敌车动力学化 ④ S3 比赛要素）。
> 全部在 WSL `~/sentry_ws`（Ubuntu 24.04 / ROS 2 Jazzy / gz Harmonic 8.15），仿真侧，
> 未上实车。分支 `codex/car-local-obstacle-avoidance`。

## 0. 提交与范围

| 提交 | 内容 |
| :-- | :-- |
| `edbb689` | footprint 半径 0.60 圆 → 0.62 方形（1.2–1.3 m 走廊可过、窄缝仍判死路） |
| `14d4fa5` | `spawn_obstacle remove` 补 `type=MODEL`（幽灵障碍根因）+ 死路边界三档实测 |
| `0a2c6cd` | 巡逻每腿遥测 + 动力学敌车 `enemy_drive.py`（leg4 = 最长腿，不是慢腿） |
| `7603e47` | S3e 比赛要素：前视相机 + 装甲板检测（21/21）+ 让行（5/5） |
| `a0f1b18` | footprint 改动后的回归（贴脸 A/B、动态敌车、点地图、5 点巡逻） |

## 1. footprint：1.2–1.3 m 走廊"真的过去"

**现象**：敌车 (0,1.2) 挡在车头、goal (0,2.6)，两侧各 1.2–1.3 m；车不撞也到不了点
（S3c 之前是原地磨 200 s，S3c 之后是 0.2–0.8 s 判死路取消）。

**根因**：footprint 是**半径 0.60 的圆**（inscribed 0.60）⇒ 缝要 ≥1.2 m 才勉强够，
1.2–1.3 m 走廊里路径净空只剩 **0.05 m** < `path_guard` 阈值 0.20。

**改法**：局部/全局 costmap 的 footprint → **0.62 方形**（真实车体 0.5×0.5 + 每侧 0.06 m
安全边，inscribed 0.31 / circumscribed 0.438）。inflation(1.5)、CostCritic、MPPI 全部未动。

**证据**（同一命令，`verify_dead_end` / `verify_detour`）：

| 用例 | 结果 |
| :-- | :-- |
| 走廊（敌车 (0,1.2)、goal (0,2.6)） | `SUCCEEDED` **13.8 / 14.2 / 15.1 s**，路径净空 **0.30–0.32 m**，面距 **+0.40~+0.42 m** |
| 同用例 `verify_detour` | **5/5**：到点 0.188 m、横向偏移 0.919 m（确实绕）、最长停滞 0.3 s、14.2 s |
| 开阔 (0,0)→(3,−2.5) | `SUCCEEDED` 13.9 s、净空 **0.46 m**、guard 不误判 |

**边界换算**（重要）：判死路阈值仍是 0.20 m，但它量的是"到内切区边界"的距离，
**能过的缝宽 = 2×(inscribed + 阈值) = 1.02 m**。三档实测（两块 wall 夹出精确缝宽，每档重启栈）：

| 缝宽 | 路径最小净空 | 结果 |
| :-- | :-- | :-- |
| 0.9 m | 0.13 m | 0.2 s 判死路、取消后位移 0.00 m |
| 1.0 m | 0.18 m | 0.4 s 判死路 |
| 1.1 m | 0.20 m（压线） | 不判、`SUCCEEDED` 11.8 s、面距 +0.293 m |

**否决的方案**：纯 0.5 方形（inscribed 0.25）也能过走廊（净空 0.35 m），但会**放行 0.8 m 缝**，
且开阔路线擦墙角时净空 0.18 m 被误判 → 才加 0.06 m 安全边。

## 2. 幽灵障碍（`remove` 没删掉模型）

**现象**：敌车删掉后 `/scan_safety` 仍量到 1.000 m、代价地图那块障碍一直在，后续用例被污染。

**根因**：`/world/<w>/remove` 的 `Entity` 请求只填 `name`、不填 `type` 时**返回 true 但不删模型**
（服务把空实体当成功）。`/scan_safety` 持续看到它 ⇒ 代价地图持续重新打标记，看起来像"清除失败"。

**修法**：`req.type = Entity.MODEL`；删完 `sleep 0.5` 再用 `gz model --list` 回读校验
（`spawn_obstacle.py` 打印 `verified_gone`，仍在就返回 1）。
**验证**：敌车位置区域 `>=99` 格 **814 → 0**（max 100 → 12）；`/scan_safety` 最小量程 1.000 → 1.500 m。

## 3. 巡逻遥测 + leg4 慢的结论

`patrol_s2a.py` 每腿新增：实走距离、均速、首帧 `/plan` 长度、`/cmd_vel_smoothed` 均值/峰值/零速占比、
最长停滞、以及从 nav2 日志窗口数出来的 `Failed to create plan` / `Failed to make progress`
（日志解析用合成日志单测 **4/4**）。`SENTRY_PATROL_DUMP_PLANS=<dir>` 可导出每腿折线。

**leg4（66–75 s）的结论：不是慢腿，是最长腿。**
- plan **15.05 m** / 实走 16.91 m（直线 9.19 m；离线最短合法路线 **13.31 m**，nav2 多绕 13%）
- 单位里程耗时与其它腿一致：**0.26 m/s** vs 其它腿 0.27–0.44 m/s；零停顿（最长 0.1 s）
- `planner_fail` 0–3 次，全瞬态（下一周期恢复），不是卡住

## 4. 动力学敌车（`tools/enemy_drive.py`）

`spawn_obstacle --dynamic` 现在给模型加 `<inertial>` + `gz-sim-velocity-control-system`，
接受 `/model/<name>/cmd_vel`（`gz.msgs.Twist`），由物理积分、会与哨兵真实碰撞。
测速用 `/world/<w>/pose/info` 的**仿真时间**按 1 s 窗口算（用到达时刻算会因批量到达出现 83 m/s 假尖峰）。

| 指令 | 1 s 窗口测速（中位） | 逐帧最大 | 位移 |
| :-- | :-- | :-- | :-- |
| 0.5 m/s × 2.5 s | 0.468 m/s | 0.491 m/s | 1.12 m |
| 0.3 m/s × 3.0 s | 0.291 m/s | 0.291 m/s | 0.81 m |
| 0.4 m/s 撞静止哨兵 | 0.382 m/s | 0.391 m/s | 哨兵被推 **0.241 m**（真物理接触） |

分工：**要精确落点用 `moving_obstacle.py`（set_pose），要动力学/接触/可测速用 `enemy_drive.py`**。

## 5. 比赛要素（S3e）

**相机**：`sentry.urdf.xacro` 加 `camera` 参数（默认 false，不开就不渲染、不桥接）；
480×360 @10 Hz、hfov 1.2 rad、base_link `(0.10,0,0.20)`（离地 0.29 m）下俯 0.08 rad。
实测图像 **5–6 Hz**（llvmpipe 渲染跟不上）、`fx=350.8`。

**装甲板**：`spawn_obstacle.py --armor red|blue` 四面加 135×55×6 mm 板（离地 0.15 m、纯 visual 无碰撞），
车体压深灰拉对比。`tools/armor_detect.py` HSV 阈值 + 长宽比过滤，输出 `/armor/detections` JSON
（颜色/像素框/`bearing_rad`/`distance_m = fx*0.135/w_px`）。符号实测：**目标在 +y 时落在图像左侧**
⇒ 世界方位 = −相机方位。

**装甲板验收** `tools/verify_armor.py`：**21/21** —— 正对 1/2/3 m 距离误差 **0.020 / 0.006 / 0.066 m**、
横移 ±0.5 m 方位误差 **0.0322 rad**、蓝板颜色正确、每例检出率 **1.00**。

**让行** `tools/verify_yield.py`（任务层策略：取消/重发都要发起方 goal handle）：
前方 <1.5 m 有障碍**且**（路径净空 <0.20 m **或** 规划器 3 s 给不出路径）→ 取消 → 退 1.0 m →
等路通 → 2 s 后重发原目标（≤3 次重试）。
实测（1.2 m 缝被敌车堵住、goal (0,2.6)）：**触发（前方 1.00 m、无路径）→ 退 0.93 m → RESUME →
`SUCCEEDED` 到点 0.06 m**，17.6 s，面距 +0.750 m，**5/5**。

## 6. footprint 改动后的回归（全部重启栈跑）

| 回归 | 结果 |
| :-- | :-- |
| 贴脸 A/B（goal (0,2.6)、敌车 (0,1.2) 1.5 s 后出现） | 雷达**关**：`SUCCEEDED` 但**面距 −0.000 m（撞上）**、81.2 s、路径 9.23 m；雷达**开**：面距 **+0.433 m**、**5/5**、9.8 s、路径 3.55 m |
| 动态敌车横穿（1.5,0.65)↔(1.5,−0.15) @0.3 m/s） | **5/5**：到点 0.182 m、最小中心距 1.479 m（面距 +1.029 m）、横向 1.325 m、14.9 s |
| 点地图链路自检 | `map_click.py --click 3 -2.5` → `CLICK go` → `SELFTEST SUCCEEDED`、dist_to_goal **0.146 m** |
| 最终 5 点巡逻 | **5/5**、116.74 s、RTF **0.943**、到点 ≤0.182 m、最小离障 1.043 m、碰撞 0、plfail 0 |

## 7. 未完成 / 已知问题

1. **敌车在本栈"点了速度但不动"（未解，停在这里）**：插件话题 `/model/enemy_robot/cmd_vel`
   存在且订阅者在线，`enemy_drive` 稳定发布（900 s / 17872 帧），但模型位移 0.01 m（只有 set_pose
   的推动）。普通动态车（无装甲板）同样不动，排除装甲板 SDF 问题；怀疑**物理体休眠**（DART 自动
   sleep，VelocityControl 的 SetLinearVelocity 不唤醒），或本 GUI 栈下插件没生效。
   2026-11-25 头less 栈下"生成后 ~1 s 内驱动"实测是能动的（0.468 m/s），差异点未定位。
   下一步：试 `/world/<w>/wrench` 唤醒、或在 SDF 里查休眠开关、或对比 headless 栈复跑。
2. **四（等硬件）**：Nav2 **Humble 参数表重写**（Jazzy 专属 8 字段，需要能装 nav2 的 Humble 环境）；
   **S100 实车 6 阶段**（需要板子可达 / 校园网）。
3. **装甲板数字识别**未做（仿真里是纯色块）；敌车**追击/拦截**决策未做。
4. 让行策略目前只在 `verify_yield.py` 里，**没有接进点地图窗口**（`map_click.py` 只有判死路）。

## 8. 下次进入点

```bash
# 起栈（交互）
cd ~/sentry_ws && DISPLAY=:0 ./run_sentry_nav.sh gui:=true map_click:=true
# 相机/装甲板（需要时加 camera:=true，然后）
python3 tools/armor_detect.py --print
# 核心验收（每条都重启栈）
python3 tools/verify_dead_end.py --goal 0 2.6 --obstacle 0 1.2 --spawn --spawn-after 1.5 --remove --expect-pass
python3 tools/verify_armor.py
python3 tools/verify_yield.py
python3 tools/patrol_s2a.py
# 停
./run_sentry_nav.sh stop
```

## 9. 报告文件（`tb3_gazebo_nav2/sentry_sim/results/`）

`report_dead_end_{180643,180913,181234,181238,181903,182016,182045,182643,191704,191740,191829}.json`
（footprint 前后 + 缝宽三档）、`report_detour_{182155,151915,151951,152115}.json`、
`report_armor_215649.json`、`report_yield_150910.json`、`report_sentry_patrol.json`（含每腿遥测）。