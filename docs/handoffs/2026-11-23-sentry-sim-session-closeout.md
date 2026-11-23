# 2026-11-23 收尾：哨兵仿真闭环 + S100 迁移准备

> 本轮把"RM 哨兵仿真"从零搭到可验收，并把上车路线（RDK S100 + Odin1 + Mid-360）的
> 适配清单、上车判据、最大未知项的测试方案全部写好。源码在
> [tb3_gazebo_nav2/sentry_sim](../../tb3_gazebo_nav2/sentry_sim/README.md)，
> 计划与决策在 [2026-11-22-sentry-simulation.md](../superpowers/plans/2026-11-22-sentry-simulation.md)，
> 实车迁移在 [2026-11-23-sentry-s100-migration.md](2026-11-23-sentry-s100-migration.md)。

## 1. 做完的（都有读数）

| 阶段 | 内容 | 结果 |
|---|---|---|
| S0 | Mid-360 的 Harmonic 插件（`livox_gz_sim`）+ `livox_ros_driver2` 消息替身包 | 点云 20000 点/帧 9.66 Hz、IMU 200 Hz、RTF 0.91–0.96 |
| S1a | 4 麦轮哨兵车（0.5×0.5×0.12）+ Odin1 接口替身 + `/scan` | 全向误差 0.38%/4.98%/4.08%；`/odin1/odometry` 10 Hz |
| S1b | Nav2 **不启 AMCL**，`map→odom` 由 Odin1 发 | 标准 TF 链无多父；(3,−2.5) 0.197 m、(0,3) 0.196 m |
| S2a | 地图/SDF 复核 + 5 点巡逻 | 地图**逐格一致**（0/180224）；巡逻 **5/5 SUCCEEDED**，最小离障 0.371 m |
| S2b | 点云 → pgm 工具 | 工具可用但**判为不用于导航**：0.2 m 平台只检出 2–15%（2D 投影盲区） |
| S2c | Odin1 map 帧 ↔ 场地帧对齐 | 三组注入恢复误差 **≤2.3 mm / 0.063°**，单次 4.4–4.7 s |
| — | Humble/Jazzy 双兼容 | `ros:humble-ros-base` 容器 **6/6 包编译通过**，补 3 处兼容 |

## 2. 关键决定（用户拍板）

1. 定位主力是 **Odin1**，仿真用接口替身；**不跑 FAST-LIO**（把算力留给 Nav2）。
2. 实车 **RDK S100 + Odin1 + Mid-360**，IMU 用 Mid-360 内置 6 轴 200 Hz。
3. **2D 栅格图用场地 CAD 先验**（已逐格核对），不现场建图 —— 因为低矮平台在 2D 投影里必然有空洞。
4. 实车定位若用 LIO，需要外挂算力；当前选 Odin1 定位，FAST-LIO 只存档。

## 3. 本轮最值得记住的坑

- 麦轮不能用圆柱轮（四轮力抵消，横移/自转为 0）；每轮 12 个 45° 被动滚子，手性 FL+BR / FR+BL。
- `<max_jerk>` 会让车在停车瞬间翻过去（SpeedLimiter 穿零反向）。
- gz `MecanumDrive` 默认就占 `/cmd_vel`、`/odom`，必须显式改名。
- `pointcloud_to_laserscan` 的高度带不设好，地面回波会在 3–4 m 处成"假墙"。
- TF 不能双父：必须 `map→odom→base_footprint`。
- 3D 雷达建 2D 图漏低矮障碍（详见计划 §0.3）。
- Humble 的 `tf2_sensor_msgs` 是纯头文件、没有 CMake 目标；Jazzy 有 —— 双兼容要 `if(TARGET ...)`。
- `map_align_check` 的地代价要用"到 CAD 边界面"，用"到占据格"会恒定偏 ~0.57° yaw。

## 4. 未完成 / 阻塞

| 项 | 卡在 |
|---|---|
| Nav2 参数表改 Humble 版（8 个 Jazzy 专属字段） | 需要能装 nav2 的 Humble 环境（容器 apt 被网络拦） |
| Odin1 大场地图（12.6×8.6 m）表现 | 等 S100/实车 |
| S3 比赛要素（敌方车、装甲板、遭遇让行） | 未开始 |
| 推 GitHub | 本机到 GitHub 时通时断（11-23 出现 TLS 失败） |

## 5. 下次进入点（测试用）

> **现场状态**：2026-11-23 收尾时已在 WSL 里用
> `cd ~/sentry_ws && DISPLAY=:0 ./run_sentry_nav.sh gui:=true` 起好哨兵仿真 + Nav2（domain 45），
> 下次对话先 `./run_sentry_nav.sh status` 确认，再决定是否重启（重启前先 `stop`）。

```bash
# 状态 / 停止 / 重启（GUI）
cd ~/sentry_ws && ./run_sentry_nav.sh status
./run_sentry_nav.sh stop
./run_sentry_nav.sh gui:=true

# 发目标（单点验收）
python3 ~/sentry_ws/tools/send_nav_goal.py 3 -2.5
python3 ~/sentry_ws/tools/send_nav_goal.py 0 3

# 5 点巡逻（脚本在仓库 tools/ 与 ~/sentry_ws/tools/）
python3 ~/sentry_ws/tools/patrol_s2a.py

# 帧对齐校验（静止在出生点即可；看 confidence.gap_m）
ros2 run map_align_check map_align_check --sim-time --frames 5 --max-points 3000 \
    --out /tmp/align.json --fix-yaml /tmp/align_fix.yaml

# 注：当前地图与巡逻点
#   map: /mnt/e/RM/tb3_gazebo_nav2/rm_field/maps/rmul2027_3v3_up.yaml
#   patrol: src/sentry_bringup/config/patrol_points.yaml
# 提醒：(3,0) 在平台里，不可达；可达示例 (3,-2.5)、(0,3)、5 点巡逻那套
```

## 6. 提交

- `7116699` 哨兵仿真栈落库（75 文件：6 包 + 工具 + 验收 JSON + README）
- `06b1164` Humble/Jazzy 双兼容补丁 + 实车迁移文档
- 本次收尾提交见 git log（PROGRESS + 本交接 + 知识沉淀指针）