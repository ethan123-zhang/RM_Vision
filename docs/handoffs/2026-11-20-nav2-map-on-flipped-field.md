# 2026-11-20 翻转后场地的 Nav2 地图重建 + 三目标闭环（副线 Gazebo）

**目标**：补掉 2026-11-19 的欠账——2D Nav2 地图和验收数据都基于翻转前场地，已失效。
按"自由区 = 墙面内侧面以内 − 平台/墙投影"重建地图，在翻转后场地上把 Nav2 闭环重新跑通。

---

## 1. 环境与分支

| 项 | 值 |
|---|---|
| 机器 | Windows + WSL2 Ubuntu-24.04（ROS 2 Jazzy / Gazebo Harmonic 8.15.0） |
| 仓库 | `E:\RM` → `github.com/ethan123-zhang/RM_Vision` |
| 分支 | `codex/car-local-obstacle-avoidance` |
| 起点提交 | `aa2d0ce`（碰撞 box + 键盘遥控的交接） |
| 本轮提交 | 一个提交，包含本节所有文件（见 §6） |

## 2. 问题表（现象 → 根因 → 修法 → 验证）

| # | 现象（带数字） | 根因（机制） | 修法 | 验证（前后对比） |
|---|---|---|---|---|
| M1 | 旧图自由区 = 整块 12.5×8.5 矩形；翻转后地面被平台/墙占掉 18.5 m² | 旧 `make_nav_map.py` 把自由区写成"围挡内侧面以内"，对应翻转前平整台面 | 新 `make_nav_map_up.py`：自由区 = 墙面内侧面以内（96 m²）− 48 块碰撞 box 投影（18.510 m²）| 自由区 **77.490 m²**（解析值 77.490，偏差 0.000）|
| M2 | 只靠几何生成，没有独立证据 | — | 平光正俯视渲染（1 px=0.01 m，关阴影）+ 颜色分类独立校核 | 地面 bbox 12.00×8.00（四边偏差 ≤0.01）、绿色 17.884 vs 障碍 17.880（重心差 0.022 m）|
| B1 | 三目标闭环第一次全失败：goal 1 `error_code 103`、车卡在 (2.5,−1.6)；TF 报 (2.29,−1.43)、Gazebo 真值 (1.73,3.42)，**差 4.8 m** | **PGM 上下翻转**：`build_map()` 第 0 行是 y 最小的一行，map_server 把图像第 0 行当**最上面一行**；`write_pgm()` 直接 `tobytes()` → 整图翻了过来。翻转前自由区是整块矩形，翻转前后一样 ⇒ 潜伏半个月 | `write_pgm()` 落盘写 `img[::-1]`（预览同步）；新增 `read_pgm()`/`occupancy_at()` 按 map_server 约定回读 + **`pgm_orientation` 判据**核对 8 个已知点 | 回读：`(−2.5,−2.3)` 0→**100**（障碍）、`(−2.5,2.3)` 100→**0**（自由）；闭环三目标全 SUCCEEDED |
| B2 | 重启栈后所有生命周期节点 `Node not found`、话题 0 Hz、`local_costmap` 一直 `Invalid frame ID "odom"` | 旧 `gz sim` 不响应 SIGTERM，清场后残留 → **两个仿真器同时发 `/clock /scan /tf`**；另有 FastRTPS 共享内存端口残留 | `nav2_field.sh` 清场改 **TERM → 3 s → KILL −9**，并 `rm /dev/shm/fastrtps*` | 清场后 `残留: 0`，重起 7 节点全 active |
| B3 | 换 shell 跑 `python3 verify_nav2_field.py` 报 `!! 等不到导航 action server`，但栈里 `ROS_DOMAIN_ID=42 ros2 action list` 能看到 `/navigate_to_pose` | 栈在 domain 42（`nav2_field.sh` 自己 export），**脚本的 export 不传给调用者 shell** → 验收进程落在 domain 0 | 脚本内 `os.environ.setdefault("ROS_DOMAIN_ID", "42")`（显式优先）+ 启动时打印实际 domain；`nav2_field.sh` 收尾等到 `/navigate_to_pose` 真出现在 `ros2 action list`（≤60 s）再报 OK | 在**未导出** domain 的 shell 里直接跑：11 项全过（0.173/0.088/0.094 m，19.762 m / 80.08 s）|
| B4 | 起栈输出里 7 个节点全是 `Node not found`，但 5b 显示 action server 就绪 | `ros2 lifecycle get`/`ros2 node list` 走 ros2 daemon 的节点列表，本机 WSL 上 daemon 看不到 composable 容器里的节点；同一时刻 `ros2 service call /amcl/get_state` = `id=3`（ACTIVE） | 第 5 段改用 `/<node>/get_state` 服务并把 id 翻成状态；文档记 `--no-daemon` 排查法 | 起栈输出 7×**ACTIVE**，5b 就绪 |
| B5 | 串联执行 `stop_nav2_field.sh && nav2_field.sh` 时脚本跑一半静默退出（exit 1）| `stop_nav2_field.sh` 里有 `pkill -f 'nav2_'`，调用者命令行含 `nav2_field.sh` → 命中自己 | 删掉这条过宽 pattern（nav2 节点都在 `component_container` 里，其余 pattern 已覆盖）| 重新串联执行正常走完 |

## 3. 地图校核（`bash capture_nav_map_up.sh` → `results/report_nav_map_up.json`）

11 项断言全过：

| 项 | 判据 | 实测 |
|---|---|---|
| 自由区面积 | = 96.0 − 18.510 ±0.10 | **77.490**（dev 0.000）|
| 自由区 bbox | 12.00 × 8.00（±1 像素）| **12.00 × 8.00**（dev 0）|
| 地图尺寸/origin | 12.80×8.80 m，origin −6.40/−4.40 | 512×352 @0.025 ✓ |
| 出生点 | (0,0) 必须是自由格 | 格 (176,256) 自由 ✓ |
| 连通性 | 泛洪覆盖全部自由格 + 3 目标可达 | 未连通格 **0**，3/3 可达 ✓ |
| PGM 方向回读 | 8 个已知点自由/障碍必须对 | 8/8 ✓ |
| 渲染地面 bbox | 12.00×8.00 ±0.05 | 12.00×8.00，四边偏差 **≤0.01** |
| 渲染地面面积 ↔ 地图 | 同区域差 ≤0.60（平台顶 2% 透视放大）| 74.348 vs 74.160（差 0.188）|
| 渲染绿色 ↔ 地图障碍 | 面积差 ≤0.60、重心差 ≤0.10 m | 17.884 vs 17.880（差 0.004）、重心 **0.022 m** |
| 逐像素一致率 | 离分界 ≥0.10 m 处 ≥99.5% | free **99.999%**、障碍 **99.803%** |

几何来源：`debug/box_collisions.json` 的 48 块 box（5 cm 栅格，与 `model.sdf` 的
`BOX_COLLISION_BEGIN/END` 同源）。地图与**物理**一致优先于与网格一致：网格碰撞把车垫高 6 mm
那次事故的教训是"视觉网格 ≠ 碰撞几何"。

## 4. 闭环验收（`bash nav2_field.sh` + `python3 verify_nav2_field.py` → `results/report_nav2_field.json`）

11 项判据全过：

| 项 | 判据 | 实测 |
|---|---|---|
| 生命周期 | 7 节点全 active | 全 active [3] |
| 话题频率 | scan/odom > 1 Hz | scan **5.0**、odom **29.3** Hz |
| 三目标 | 全 SUCCEEDED（error_code 0）| (4,2) 25.9 s、(4,−2.5) 15.5 s、(0,0) 27.8 s |
| 到位 | 结束距目标 < 0.30 m | **0.290 / 0.132 / 0.015 m** |
| 不出界 | max&#124;x&#124; ≤5.85、max&#124;y&#124; ≤3.85 | 4.433 / 2.521 |
| 不落台/不爬台 | z ∈ (−0.05, 0.06) | 全程 **0.0100** |
| 不压平台/墙 | TF 不得侵入任何 box ≤0.05 m | 最近 **−0.5105 m**（离投影还有半米）|
| 真在跑 | 峰值 ≤ vx_max×1.05、平均 >0.05 | 0.495 ≤ 0.5；平均 0.256 m/s |
| 定位一致 | TF vs Gazebo 真值 < 0.50 m | 均值 0.144 / **最大 0.443 m** |

整圈：19.622 m / 76.66 s（RTF 0.999）。此后又复跑两次，读数一致：
B3 修复后（**未导出 `ROS_DOMAIN_ID`** 的新 shell）：到位 0.173 / 0.088 / 0.094 m、19.762 m / 80.08 s、
峰值 0.492；B4/B5 修复后清场重起（最后一次）：到位 **0.268 / 0.081 / 0.138 m**、
**19.626 m / 75.96 s**、平均 0.258 m/s、cmd_vel 峰值 **0.489**、定位误差 max 0.406 / mean 0.131，
11 项仍全过。

**定位误差怎么读**：`whole_run.loc_err_m.series_2s` 每 2 s 一个点。转弯段瞬时到 0.44 m
（AMCL 按 `update_min_d=0.25 m` 周期修正 + 电机里程计偏航误差在里程计原点上放大），
静止/直行段 0.02~0.07 m。判据定 0.50 m 是为了抓"地图世界错位"这种 4.8 m 量级的错误。

## 5. 当前状态

- **能做什么**：`bash nav2_field.sh` 起栈后可在翻转后的 3V3 场地上点目标导航；三个验收目标全通；
  `bash capture_nav_map_up.sh` 可一条命令重建地图并自校核。
- **卡在哪**：绕障/动态避障没测；`collision_monitor` 用 nav2 默认参数。
- 仿真进程已全部停掉（`gz sim` 计数 0，无残留 `nav2_container`）。

## 6. 文件清单与提交

新增：

- `worlds/rmul2027_3v3_up_nav2.sdf.xacro` — Nav2 用世界（翻转场地 + 场外底板 z=−0.10 + 传感器/IMU）
- `worlds/nav_map_capture_up.sdf` — 出图用正俯视取证世界（平光无阴影，1 px=0.01 m）
- `make_nav_map_up.py` — 几何生成 + 渲染校核 + 连通性 + PGM 方向回读（11 项断言）
- `capture_nav_map_up.sh` — 渲染 → 生成 → 校核
- `maps/rmul2027_3v3_up.pgm` / `.yaml` / `_preview.png`
- `results/report_nav_map_up.json`
- `shots/nav_topdown_up.png`、`shots/nav_map_up_overlay.png`（`.gitignore` 已加白名单）

修改：

- `nav2_field.sh` — 指向新世界/新图；清场改 TERM→KILL 并清 FastRTPS 共享内存；生命周期查询加重试；
  收尾先等 `/navigate_to_pose` 出现在 `ros2 action list`（≤60 s）再报 OK，并给出带 domain 的验收命令
- `verify_nav2_field.py` — 判据换成 ±6.00/±4.00；新增"不压平台/墙""不爬台""定位一致"；
  三个目标；`loc_err_m` 含 2 s 采样趋势；`os.environ.setdefault("ROS_DOMAIN_ID", "42")`（跨 shell 直接跑）
- `results/report_nav2_field.json`、`README.md`（§13.3 欠账划掉、新增 §15）
- `PROGRESS.md`、本 handoff

## 7. 下次进入点

```bash
cd /mnt/e/RM/tb3_gazebo_nav2/rm_field
bash capture_nav_map_up.sh          # 重建地图 + 11 项校核（约 1 min）
bash nav2_field.sh                  # 起栈（headless；--gui 带 RViz）
python3 verify_nav2_field.py        # 三目标闭环（约 80 s）
bash stop_nav2_field.sh
```

## 8. 遗留风险

- 碰撞/地图都是 5 cm 栅格：台沿/墙角 ≤5 cm 量化误差；贴边或窄通道场景要 `--cell 0.025` 重新生成
  **碰撞与地图（两者必须同源，一起提交）**。
- 围挡高度（翻转后是场地自带 0.40 m 外墙）与赛规一致性仍未核对。
- `verify_nav2_field.py` 的"不压平台/墙"用的是碰撞 box 投影 + 5 cm 容差：能抓定位跳变/真撞，
  但抓不到 1~2 cm 的擦碰。
- 旧地图 `maps/rmul2027_3v3_fenced.*`、旧世界 `worlds/rmul2027_3v3_nav2.sdf.xacro`、
  旧脚本 `make_nav_map.py` / `capture_nav_map.sh` 保留作历史，**已不代表当前场地**。

## 9. 提交记录（本轮收尾）

| 提交 | 内容 |
|---|---|
| `bbf9e57` | 翻转后场地重建 Nav2 地图 + 三目标闭环全过；修 PGM 上下翻转潜伏 bug |
| `bf937fb` | 验收脚本默认 `ROS_DOMAIN_ID=42`；起栈脚本等到 action server 再报 OK |
| `1ee79bd` | 起栈生命周期检查改用 `/<node>/get_state` 服务；删掉会自杀的 `pkill -f nav2_` |

已推送 `origin/codex/car-local-obstacle-avoidance`，回读 HEAD 与 origin 一致（`1ee79bd`）。
知识沉淀：aiVault `wiki/concepts/Nav2_Map_And_Localization_Verification.md`（连带 INDEX/LOG）。
流程沉淀：技能 `simulating-ros2-nav2-in-wsl`（自建 pgm 方向约定、TF vs 真值核对、`lifecycle get` 误报）。