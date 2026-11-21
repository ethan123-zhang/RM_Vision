# 2026-11-21 交接：清场按 ROS domain 隔离 + 动态箱三档 + 0.62 m 超窄门（2.5 cm 栅格变体）

> 范围：`tb3_gazebo_nav2/rm_field`（副线 Gazebo 仿真），两笔 §17.6 欠账 + §20 末尾记的"发现但未改"。
> 用户要求顺序：先 A（清场域隔离）→ 再 B（验收档位）→ 用户亲自验收测试。

## 1. A：清场只在本 ROS_DOMAIN_ID 内生效

### 1.1 原始问题（2026-11-20 实测踩到）

`nav2_field.sh` / `stop_nav2_field.sh` 用裸 `pkill -f` 清场、再 `rm -f /dev/shm/fastrtps_*`：

- `pkill` 不认 ROS domain → 并行调试另一个 domain 时把它那边的 `goal_pose_bridge.py`、组件容器一起
  SIGKILL（上轮误杀过一次 bridge，导致点击落到旧桥、日志里看不到但车动了）；
- `rm -f /dev/shm/fastrtps_*` 删掉**别的 domain 正在用**的 DDS 共享内存/端口文件 → 症状是
  "action server 看着 ready，但 `/clock` 收不到、bridge 永远不 READY"。

### 1.2 修法

新增 `domain_cleanup.sh`（两个入口脚本 source 后调用）：

| 对象 | 归属判定 | 依据（2026-11-21 实测） |
|---|---|---|
| 进程 | `/proc/<pid>/environ` 的 `ROS_DOMAIN_ID`（未设置 = ROS 默认 0）；environ 读不到一律不动 | 栈内 gz/bridge/容器都继承 launch 的环境 |
| 数据段/端口段 | 被 mmap（`/proc/*/maps`）、被 fd 打开（`/proc/*/fd`）或 flock 被占 → 保留 | 活 participant 的段都被 mmap；FastRTPS 把 flock 打在 `_el` 文件上 |
| `sem.fastrtps_*_mutex` | **一律保留** | POSIX 信号量没有文件锁/fd/mapping 可查；删活栈的信号量比留孤儿文件危险 |
| 端口/数据段成对 | `X` 与 `X_el` 任一在用 → 两个都保留 | 只锁 `_el` 时 plain 无人映射，单独判会删掉 → 下次 init 报 `open_and_lock_file failed` |
| `(deleted)` 映射 | 不算"在用文件"（磁盘上无文件可删），也不保护同名后来者 | FastRTPS/glibc 先 unlink 再继续用映射 |

其它：`ROS_DOMAIN_ID` 从写死 42 改成 `${ROS_DOMAIN_ID:-42}`（清场跟着同一 domain）；
`RM_CLEAN_ALL_DOMAINS=1` 是现场抢救回退（全 domain 清 + 清全部 shm）。

### 1.3 证据

- `verify_cleanup_domain_scope.sh`：**14 项 PASS / 0 FAIL**，不需要 Gazebo
  （假进程用 `exec -a` 造同 cmdline，mmap/flock/fd/sem 用 python/flock 假持有）。
- 旧/新对比：旧版（HEAD）`stop_nav2_field.sh` 在 domain 42 执行 → domain 43 假 bridge **被误杀**；
  新版同场景 → **存活**。
- 真栈集成（`_probe/cleanup/integration_probe.sh`，8 项全过）：真栈在 42 + domain 43 真 DDS
  participant（`ros2 topic pub`）+ 4 个同 cmdline 假进程：
  - 42 运行中清共享内存：**30 个"在用"文件一个没删**，清完 `/navigate_to_pose` 仍在；
  - 42 停栈：残留 0；domain 43 的 5 个进程全存活。
- 端口成对修复后：完整起栈日志从 4 条 `RTPS_TRANSPORT_SHM ... open_and_lock_file failed` → **0 条**。

提交 `8d0ebae`（含 README §21）。

## 2. B1：动态箱三档（slow / fast / faster）

### 2.1 配置与代码

`config/avoid_obstacles.json → dynamic` 改成档位制：

```json
"default_tier": "slow",
"tiers": {
  "slow":   {"speed": 0.3, "start": [2.5, -1.94], "end": [2.5, -1.35]},
  "fast":   {"speed": 0.6, "start": [2.5, -1.94], "end": [2.73, -1.07]},
  "faster": {"speed": 1.0, "start": [2.5, -1.94], "end": [3.35, -1.09]}
}
```

- 三档同一起点（起点在无障基线走道上，`obstacles_on_route` 判据不变）；
- `fast`/`faster` 让开方向偏东北：离线查地图，x=2.5 往北只有约 0.9 m 自由段（再远压到凸起结构）；
  `faster` 用 45° 让 1.20 m，保证运动段 ≥1.2 s（验收用 1 s 窗口独立测速）；
- `dynamic_obstacle.py --tier` / `make_obstacles_world.py --tier` / `verify_nav2_obstacles.py --tier`
  读同一档位 → 世界初始位姿、驱动轨迹、验收判据同源；
- 一行跑三档：`bash verify_obstacles_tiers.sh`（每档独立起栈→验收→停栈）。

### 2.2 结果（三档各 15 项判据全过）

| 档位 | 配置速度 | 实测速度（1 s 窗口中位） | 行程 | 人-箱最近中心距 / 间距 | 报告 |
|---|---|---|---|---|---|
| slow | 0.3 | **0.293** | 0.59 m | 1.09 / 0.84 m | `report_nav2_obstacles.json` |
| fast | 0.6 | **0.587** | 0.97 m | 1.33 / 1.06 m | `report_nav2_obstacles_fast.json` |
| faster | 1.0 | **0.979** | 1.27 m | 1.10 / 0.75 m | `report_nav2_obstacles_faster.json` |

判据集合沿用 §16：生命周期/话题/三目标成功与到位/不出界/不掉台/不压平台/定位一致/speed_sane/
障碍物挡路（起点侵入基线）/静态间距/动态速度与行程/动态间距与遭遇，在 = 15 项。

## 3. B2：0.62 m 超窄门 + 2.5 cm 碰撞栅格变体

### 3.1 变体生成（`bash make_25mm_variant.sh`，不碰 5 cm 基线）

- `models/rmul2027_3v3_field_up_25mm/`：`make_box_collisions.py --cell 0.025` →
  **178 块** box（基线 5 cm 是 48 块），高度误差 max 41 mm / rms 1 mm；
  视觉网格仍引用 `model://rmul2027_3v3_field_up/meshes/...`（不复制 12 MB 网格）；
- `debug/box_collisions_25mm.json`（地图几何源）；
- `worlds/rmul2027_3v3_up_nav2_25mm.sdf.xacro`（world 名保持 `rmul2027_3v3`，服务名不变）；
- `maps/rmul2027_3v3_up_25mm.pgm/yaml`：自由区 **77.706 m² = 解析值 77.706**（96.0 − 18.294），
  `results/report_nav_map_up_25mm.json` **11 项校核全过**（PGM 方向回读/连通性/正俯视渲染比对等）。

### 3.2 场景与入口

- `config/avoid_obstacles.json → gate_tight`：两块 1.0×0.5×0.6 m、内侧面 ±0.31 → 门洞 **0.62 m**
  （车宽 0.44，两侧外缝 0.09 m，绕不过去）；
- `bash nav2_gap_tight.sh`（默认 25mm）/ `--grid 5cm`（基线对照）；
- `python3 verify_narrow_gap.py --scenario gate_tight --grid 25mm`
  （5 cm 对照 `--grid 5cm --tag 5cm`）；穿门判据从写死 0.42 改成"门洞半宽 + 2 cm"（=0.33）。

### 3.3 结果（两套栅格各 6 项判据全过）

| 栅格 | 到点 | 门线 \|x\| max | 到门箱最小间距 | 定位 max / 门段 max | 报告 |
|---|---|---|---|---|---|
| 2.5 cm | 0.081 m | 0.021 m | 0.288 / 0.327 m | 0.099 / 0.089 m | `report_narrow_gap_tight.json` |
| 5 cm | 0.068 m | 0.022 m | 0.284 / 0.329 m | 0.095 / 0.090 m | `report_narrow_gap_tight_5cm.json` |

**结论**：0.62 m 门洞 5 cm 栅格也过得去（读数差 1~2 mm 级）→ 此量级不必启用 2.5 cm 变体，
变体备而不用；本次没有找到"5 cm 失败、2.5 cm 成功"的门洞边界，不臆造结论。

## 4. 当前状态

- 仿真进程已全停（`stop_nav2_field.sh`，回到 42 残留 0）；`/dev/shm` 无陈旧 fastrtps 大段（sem 保留）。
- 自动验收全部可复跑，耗时：清场回归 <1 min；动态箱三档约 8 min；超窄门一档约 3 min。
- 工作树包含本轮全部改动，`git status` 干净前不要跑 `git clean`。

## 5. 风险 / 注意

- `verify_obstacles_tiers.sh` 会**真的**在该 domain 起停栈：跑之前别在同一 domain 上留想保留的栈。
- `<0.70 m` 结论只覆盖 0.62 m；门洞越窄，costmap inflation（robot_radius 0.22 / inflation 0.55）
  越可能先拒，未测 0.55~0.62 之间的边界。
- `make_25mm_variant.sh` 会把 `models/..._25mm/model.sdf` 的碰撞块重写（幂等），但**不会**碰基线模型。
- 2.5 cm 变体只换了碰撞盒；AMCL 地图仍是同一张 0.025 m/px 图（分辨率没变，变的是栅格化来源精度）。

## 6. 关键文件

新增：`domain_cleanup.sh`、`verify_cleanup_domain_scope.sh`、`verify_obstacles_tiers.sh`、
`nav2_gap_tight.sh`、`make_25mm_variant.sh`、`models/rmul2027_3v3_field_up_25mm/`、
`debug/box_collisions_25mm.json`、`worlds/rmul2027_3v3_up_nav2_25mm.sdf.xacro`、
`worlds/rmul2027_3v3_up_nav2_gate_tight*.sdf.xacro`、`maps/rmul2027_3v3_up_25mm.*`、
`results/report_nav2_obstacles{,_fast,_faster}.json`、`results/report_narrow_gap_tight*.json`、
`results/report_nav_map_up_25mm.json`。

修改：`nav2_field.sh`、`stop_nav2_field.sh`、`nav2_obstacles.sh`、`dynamic_obstacle.py`、
`make_obstacles_world.py`、`verify_nav2_obstacles.py`、`verify_narrow_gap.py`、
`make_nav_map_up.py`、`debug/make_box_collisions.py`、`config/avoid_obstacles.json`、`README.md`（§21/§22）。

## 7. 下次进入点

```bash
cd /mnt/e/RM/tb3_gazebo_nav2/rm_field

# A 回归（不需要 Gazebo）
bash verify_cleanup_domain_scope.sh

# B1 三档动态箱（约 8 min，会起停仿真栈）
bash verify_obstacles_tiers.sh

# B2 超窄门（2.5 cm 变体已生成；重生成也只要几秒）
bash make_25mm_variant.sh
bash nav2_gap_tight.sh && python3 verify_narrow_gap.py --scenario gate_tight --grid 25mm
bash stop_nav2_field.sh
bash nav2_gap_tight.sh --grid 5cm && python3 verify_narrow_gap.py --scenario gate_tight --grid 5cm --tag 5cm
bash stop_nav2_field.sh
```