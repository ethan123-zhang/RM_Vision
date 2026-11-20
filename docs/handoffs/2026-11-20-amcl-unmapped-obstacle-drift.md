# 2026-11-20 深夜五补：未建图障碍物把 AMCL 带偏（"钻过窄缝后定位飘走"）——根因、修复、回归

**目标**：用户报"在场地中间摆两块障碍物，让车从 0.8 m 缝隙钻过去，钻过去后定位漂到别处"。
要求：查清根因、按方案 1（AMCL beam skip）修复并测试，同时把交接文档里其它待修 BUG 一起解决。

---

## 1. 环境

| 项 | 值 |
|---|---|
| 机器 | Windows + WSL2 Ubuntu-24.04（ROS 2 Jazzy / Gazebo 8.15.0，llvmpipe）|
| 仓库 | `E:\RM` → `github.com/ethan123-zhang/RM_Vision` |
| 起点提交 | `353743d`（AMCL 参数修复 + 点选导航）|
| 本轮提交 | `8e37ad3`（代码 + 本交接初稿）；收尾补哈希见随后 docs 提交 |

## 2. 根因（证据链）

1. **两块障碍物只在 Gazebo 世界里，不在导航静态地图里**：`worlds/…_gap_deep.sdf.xacro` 的两块
   1.0×1.2×1.0 m 箱子由 `make_obstacles_world.py` 从 `config/avoid_obstacles.json` 生成；静态图
   `maps/rmul2027_3v3_up.pgm` 只含 48 块场地碰撞 box。离线核对：两块箱投影在 PGM 里 100% 是自由格。
2. **门里扫描被未建图回波占满**：过门时 in-range 束 **平均 70%、峰值 90%** 是箱体回波
   （`_probe/` 探针 + 真值位姿射线投射；标准 0.6×0.5 小门这套是 43%/60%）。
3. **AMCL 似然不再指向真值**：nav2 的 likelihood field 是加性打分
   `p = 1 + Σ pz³`，`pz = z_hit·exp(−d²/2σ²) + z_rand/range_max`，`range ≥ laser_max_range` 跳过
   （`nav2_amcl/src/sensors/laser/likelihood_field_model.cpp`）。未建图回波端点落在自由区，
   每束只值 0.0032（命中地图 0.271）。实测：**真值位姿 p≈20，贴平台边/墙角的错误位姿 p≈50~60**。
4. **滤波器跟过去且不自愈**：粒子数 2115→801（min 下限）→2513（恢复注入），粒子展布 std 峰值
   0.58 m；估计 16.5 s 起跳 → 17.1 s 1.54 m → **最终 2.9 m**；nav2 19 次恢复后 **ABORTED**，
   车停在门后 1.3 m 处；停 80 s 误差不变。
5. **为什么是"另一个地方"**：场地图 **180° 点对称**（室内格一致率 99.5%），镜像位姿对同一帧扫描
   几乎同分；真值证据被削弱后，估计可落到镜像/贴墙分支，误差是米级而不是厘米级。
6. **用户日志对得上**：2026-11-20 21:48:46 按 T 前，AMCL 认为 (2.09,1.48)、真值 (-0.322,0.953)
   → 差 **2.45 m**，按 T 立刻恢复。同日志 21:38:54/59 的 `/initialpose` 是上一轮回归脚本
   `test_relocalize_replan.py` 的"错误位姿（真值+2.5 m）→ 真值"序列，不是本次漂移原因。

## 3. 修复（方案 1）

`make_nav2_params.py` 的 AMCL 覆盖新增两项并重新生成 `config/nav2_params_rm.yaml`（diff 仅 2 行）：

| 参数 | 修复前 | 修复后 |
|---|---|---|
| `laser_model_type` | `likelihood_field` | **`likelihood_field_prob`** |
| `do_beamskip` | `false` | **`true`**（`beam_skip_distance 0.5 / beam_skip_threshold 0.3 / beam_skip_error_threshold 0.9` 用官方默认）|

原理：beam skip 把"所有粒子都解释不了"的束当离群束丢掉——正好是地图里没有的那 70~90%。

## 4. 回归（新增第 4 个专项场景）

`config/avoid_obstacles.json` 新增 `gate_deep`（两块 1.0×1.2×1.0 m，门洞仍 0.80 m，窄段深 1.2 m）
→ `make_obstacles_world.py --only gate_deep` → `worlds/rmul2027_3v3_up_nav2_gap_deep.sdf.xacro`
→ `bash nav2_gap_deep.sh` → `python3 verify_narrow_gap.py --scenario gate_deep`。

| 场景 | 修复前 | 修复后（两跑）|
|---|---|---|
| gate_deep | 漂 **2.9 m**、ABORTED、停车 | SUCCEEDED；到点 **0.293 / 0.231 m**；定位 max **0.413 / 0.318 m**（门段 0.156 / 0.149）|
| gate（0.6×0.5 小门）| 定位 max 0.383 m | **0.087 m**，到点 0.075 m |
| 绕障 15 项 | 全场定位 max 0.764 m（判据只能剔除 ≤2 m 样本）| **15/15 全过**，全场定位 max **0.315→0.302 m**，判据改回全样本 <0.5 m |

`verify_narrow_gap.py` 新增第 6 项判据 `localization_consistent`：TF(map->base_footprint) vs Gazebo
真值 全局 max <0.50 m，并单列"门线 ±0.9 m 穿门段"的 max（原来 5 项只测几何，漂了也能全过）。

## 5. 同日修掉的另外两个 BUG

| # | 现象 | 根因 | 修法 |
|---|---|---|---|
| B1 | `make_obstacles_world.py --only braking` 恒返回 1（"生成的障碍物模型数 1 != 配置 0"）→ `nav2_braking.sh` 第 0 步就退出，刹车验收入口是坏的 | `check()` 只写了 obstacles/gap 分支，漏了 braking 分支（HEAD 就有）| 补 braking 校核：模型存在、kinematic、隐藏点/弹出点在场内、宽度 ≥2.6 m、弹出点在中线 |
| B2 | `map_click.py` 退出 `terminate called without an active exception`；`goal_pose_bridge.py` SIGINT 时 `RCLError: failed to initialize wait set` | `rclpy.spin()` 在后台线程里只能靠 `rcl_shutdown` 停；默认信号 handler 先关 context，spin 还在 wait set → RCLError/terminate | 改 `rclpy.init(signal_handler_options=NO)`；信号只置位；用 `SingleThreadedExecutor.spin_once` 轮询；统一收尾顺序（停 spin → executor.shutdown → destroy_node → rclpy.shutdown）；Tk 关窗走同一路径 |

实测（有栈时）：两脚本 `kill -TERM` 后日志只有"收到退出信号…"，无 traceback/terminate；
`map_click.py --self-test` 仍能正常点选自检（SELFTEST start → 右键确认 → T 真值重定位 → done，exit=0）。
关闭窗口（WM_DELETE_WINDOW）、启动瞬间被杀（0.3/0.6 s × TERM/INT）、连续两次 TERM、
`wait_for_server` 阻塞期 TERM、90 s 无 server 超时返回 2 等路径都在 `ROS_DOMAIN_ID=43` 隔离环境
（假 action server）逐条验过：旧版 `terminate`(exit 134)/`RCLError`(exit 1) 先复现，新版全部 exit 0。

**发现但本轮未改（并行调试才会踩到）**：`nav2_field.sh` / `stop_nav2_field.sh` 的清场用
`pkill -f '<名字>'`，**跨 ROS domain 生效**；本次并行测试时就把它杀的 bridge SIGKILL 过。
`nav2_field.sh` 还会 `rm -f /dev/shm/fastrtps_*`，会切断其它 domain 存活进程的 DDS 共享内存通路
（症状：action server 看着 ready，但 /clock 收不到、bridge 永远不 READY）。
建议后续改成按 `ROS_DOMAIN_ID`（或 pid 文件/进程组）过滤，并且别删别人正在用的 shm 文件；
本工程单 domain（42）跑主仿真时不受影响。

## 6. 当前状态

- **能做什么**：`bash nav2_gap_deep.sh` + `python3 verify_narrow_gap.py --scenario gate_deep`
  6 项判据全过；标准窄门、绕障 15 项、点选窗口自检、两个脚本干净退出都已实测。
- **风险**：`gate_deep` 的"到位 <0.30 m"余量偏小（修复后两跑 0.293 / 0.231 m），
  因为 beam skip 在未建图长窄段里只能用约 16% 的束，定位残差 ~0.3 m；如果后续再跑挂了这条，
  先看定位 max 是否仍 <0.5（判据主体），不要先怀疑规划。
- 仿真进程已全停（`stop_nav2_field.sh`，残留 0）。

## 7. 关键文件

新增：`worlds/rmul2027_3v3_up_nav2_gap_deep.sdf.xacro`、`nav2_gap_deep.sh`、
`results/report_narrow_gap_deep.json`、本文件。
修改：`config/avoid_obstacles.json`、`config/nav2_params_rm.yaml`、`make_nav2_params.py`、
`make_obstacles_world.py`、`verify_narrow_gap.py`、`verify_nav2_obstacles.py`、
`map_click.py`、`goal_pose_bridge.py`、`README.md` §20、`PROGRESS.md`、`results/report_narrow_gap.json`、
`results/report_nav2_obstacles.json`。

## 8. 下次进入点

```bash
cd /mnt/e/RM/tb3_gazebo_nav2/rm_field
bash nav2_gap_deep.sh                                  # 放大门 + 栈（约 60~90 s）
python3 verify_narrow_gap.py --scenario gate_deep      # 6 项判据（约 40 s）
bash stop_nav2_field.sh
# 其它回归（各自重启栈）：
bash nav2_gap.sh         && python3 verify_narrow_gap.py
bash nav2_obstacles.sh   && python3 verify_nav2_obstacles.py
```

**提交状态**：已提交 `8e37ad3`（代码 + 交接初稿）；收尾文档（本哈希补记）为随后的 docs 提交。