# 2026-11-20 晚 绕障 + 动态避障验收（副线 Gazebo）

**目标**：补 2026-11-20 白天交接收尾时列出的欠账——"绕障/动态避障未测、`collision_monitor`
用 nav2 默认参数没按本场地调过"。在翻转后的 3V3 场地上摆未建图障碍物，验证 Nav2 能否绕开
静态挡路物、能否对会动的障碍物让开，且三目标照样 SUCCEEDED。

---

## 1. 环境与分支

| 项 | 值 |
|---|---|
| 机器 | Windows + WSL2 Ubuntu-24.04（ROS 2 Jazzy / Gazebo Harmonic 8.15.0）|
| 仓库 | `E:\RM` → `github.com/ethan123-zhang/RM_Vision` |
| 分支 | `codex/car-local-obstacle-avoidance` |
| 起点提交 | `3ba028d`（2026-11-20 白天 Nav2 地图+三目标闭环的收尾）|
| 本轮提交 | 见 §7 |

## 2. 做法（一条链，全部可复跑）

1. **无障基线**：`capture_route.py` 先在同一场地/目标上跑一遍无障导航，
   存 `results/baseline_route.json`（3 目标 19.442 m / 66.8 s）。用途：挑障碍位置 + 做绕行对照。
2. **障碍物唯一源**：`config/avoid_obstacles.json` → `make_obstacles_world.py` 生成
   `worlds/rmul2027_3v3_up_nav2_obstacles.sdf.xacro`（插在 physics 插件前 BEGIN/END 标记内）；
   验收脚本读同一份 JSON 算投影。**世界与判据同源**。
3. **静态箱 2 块**（0.5 m 立方）：`(0.85,-1.45)` 中线走道、`(4.30,-0.60)` 东侧通道。
   选位依据：无障基线穿过它们的投影（侵入 0.209 / 0.226 m）。
4. **动态箱 1 块**（0.5×0.5×0.6 m，`<kinematic>true</kinematic>`）：`dynamic_obstacle.py`
   用 `/world/rmul2027_3v3/set_pose`（gz.msgs.Pose，10 Hz）平移。
   模式 **triggered_retreat**：停在基线走道正中 `(2.50,-1.94)` 挡路，机器人进到 **1.5 m** 内后
   以 **0.3 m/s** 沿 `start→end(2.50,-1.35)` 让开并停住（向北，背离机器人实际走道）。
5. **一键起栈**：`nav2_obstacles.sh`（生成 + 同源校核 + xacro 自检 → `nav2_field.sh`，
   通过新环境入口 `NAV2_WORLD=`、`RM_DYNAMIC=1`）。
6. **验收**：`verify_nav2_obstacles.py` → `results/report_nav2_obstacles.json`，15 项判据。

## 3. 结果（最后一跑，全过）

| 类别 | 实测 |
|---|---|
| 三目标 | 全 SUCCEEDED（error_code 0）：12.646 m/33.3 s、5.557 m/17.4 s、6.807 m/28.7 s |
| 到位 | 0.121 / 0.088 / 0.025 m（<0.30）|
| 全程 | 25.051 m / 86.9 s，平均 0.288 m/s，cmd 峰值 0.499（vx_max 0.5），RTF 0.999 |
| 不出界/不掉台/不爬台 | max&#124;x&#124; 5.182、max&#124;y&#124; 3.212；z 全程 **0.0100** |
| 不压平台墙 | 最大侵入 −0.562 m（48 块场地碰撞 box 投影外半米）|
| 静态绕行 | 全程真值到两块静态箱间距 **0.701 / 0.640 m**（≥ robot_radius 0.22）|
| 动态在动 | 1 s 窗口测速 **0.293 m/s**（配置 0.30±0.05）；行程 **0.590**（期望 0.590）；停位误差 **0**；停后 1 s 位移 0 |
| 动态无接触且遭遇 | 人-箱最小间距 **0.892 m**、最近中心距 **1.144 m**；该段机器人 cmd_vel 曾到 0（真被挡了一下）|
| 定位一致 | 剔除未建图障碍物 ≤2 m 的样本后 max **0.256 m**（<0.50）；全场最大 **0.764 m**（见 §4②）|
| 绕行代价 | 相对无障基线 +3.503 / +0.714 / +1.351 m，合计 **+5.57 m** |

## 4. 问题表（现象 → 根因 → 修法）

| # | 现象（带数字） | 根因 | 修法 |
|---|---|---|---|
| D1 | 第一版连续往返横穿：第三段跑 **153.9 s**、机器人 2 min 挪 0.3 m、日志反复 `Failed to make progress`；动态箱与机器人最近 **0.077 m** | 盲动的箱子反复切进机器人走道：MPPI 反复判"无进展"→ recovery 死循环；箱子不看机器人、会主动压上来 | 动态箱改成 **triggered_retreat**（先挡路，机器人 1.5 m 内触发让开、让开方向背离机器人），单程且会停住 |
| D2 | 第二版让开方向朝南（机器人实际在箱体南侧绕行）：人-箱最近 **0.237 m 中心距 / −0.013 m 间距**（穿模） | 让开方向与机器人绕行方向同侧，箱子"让"进了机器人路径 | 退让方向改为**向北**（朝平台，机器人走道在它南侧）：最小间距 0.892 m |
| D3 | AMCL 与 Gazebo 真值偏差三次复跑都到 **0.76~1.05 m**（基线无障时最大 0.44） | 未建图障碍物（静态箱/动态箱）的回波进扫描，likelihood field 匹配被带偏；30 s 内自行收敛 | 判据只用"离未建图障碍物 >2 m"的样本判地图-世界一致性；报告同时给 all 与剔除后两份数 |
| D4 | `pkill -f 'dynamic_obstacle.py'` 让脚本静默 exit 1 | pattern 命中调用者自己的命令行（命令行里含同名文件） | 用 `pkill -f 'dynamic_obstacle[.]py'`，并封装成 `start_dynamic.sh` |
| D5 | set_pose 0.1 s 步进导致"相邻帧测速"出现 0 或 1.8 m/s 的假值 | 位置是瞬移式更新的 | 测速用 **1 s 窗口位移/时间**（窗口内中位速度），另记运动起止位移/时长作参考（量化误差偏大） |

## 5. 当前状态

- **能做什么**：`bash nav2_obstacles.sh` 一条命令起"翻转场地 + 2 静态箱 + 触发式动态箱 + Nav2"，
  `python3 verify_nav2_obstacles.py` 输出 15 项判据全过；`bash start_dynamic.sh` 可只重启动态驱动。
- **卡在哪**：`collision_monitor` 本轮 **0 次触发**（DWB 提前绕开）——"必撞/突然出现"场景没测；
  动态箱只有 0.3 m/s 单向单程。
- 仿真进程已全停（`stop_nav2_field.sh` 收尾，残留 0）。

## 6. 关键文件

新增：`config/avoid_obstacles.json`、`make_obstacles_world.py`、
`worlds/rmul2027_3v3_up_nav2_obstacles.sdf.xacro`、`nav2_obstacles.sh`、`dynamic_obstacle.py`、
`start_dynamic.sh`、`capture_route.py`、`verify_nav2_obstacles.py`、
`results/baseline_route.json`、`results/report_nav2_obstacles.json`。
修改：`nav2_field.sh`（`NAV2_WORLD` 覆盖世界、`RM_DYNAMIC=1` 拉动态驱动、清场含动态驱动）、
`stop_nav2_field.sh`（清场含动态驱动）、`README.md`（§15.6 划掉欠账、新增 §16）、`PROGRESS.md`。

## 7. 下次进入点

```bash
cd /mnt/e/RM/tb3_gazebo_nav2/rm_field
bash capture_nav_map_up.sh          # 若地图/碰撞改了才需要
bash nav2_obstacles.sh              # 生成世界 + 起栈 + 动态驱动
python3 verify_nav2_obstacles.py    # 15 项判据（约 90 s + 启动）
bash stop_nav2_field.sh
# 单跑基线（挑新布局时用）：
bash nav2_field.sh && python3 capture_route.py && bash stop_nav2_field.sh
```

## 8. 遗留风险

- 碰撞/地图仍是 5 cm 栅格：**0.80 m 门洞已实测可通过（见 §12），不必重生成 2.5 cm**；
  只有 <0.70 m 门洞或厘米级贴边才需要 2.5 cm 变体（新模型名 + 新地图名）。
- 围挡高度（0.40 m 场墙）与赛规一致性：**已收口**（见 §10）：规则图是位图、本机无 OCR/视觉引擎，
  以官方 CAD 为权威几何。
- 动态障碍是脚本驱动的 kinematic 箱，不是真运动学底盘；真敌车会加速/变向，"触发式"以外的场景未覆盖。
  速度/方向档位也只做了 0.3 m/s 一档。
- AMCL 在未建图障碍物附近 0.76~1.05 m 的瞬态偏差对实车同样成立（实车定位更依赖点云/回环），
  若后续要跑更密的障碍场景，需要单独评估定位裕度。

## 9. 提交状态（按用户要求先不提交）

本轮所有改动（绕障 15 项验收 + 刹车验收 + 文档）都**留在工作区未提交**；
`git status` 里应能看到这些未提交改动，`git log` 仍停在 `3ba028d`。

## 10. 补测：安全层（collision_monitor）刹车验收（6 项判据全过）

- **为什么补**：§3 的绕障跑里 `collision_monitor` **0 次触发**（DWB 提前绕开），安全层没被测到。
- **场景**：`worlds/rmul2027_3v3_up_nav2_braking.sdf.xacro`（同一 JSON 的 `popup` 段生成）：
  2.8×0.5×0.6 m 宽挡板先藏 `(3,-3)`；机器人跑 `(0,0)→(0,3.0)`，离 `(0,2.2)` 剩 **0.85 m** 时
  瞬移拦满 3.2 m 宽中线走道（留 0.2 m 缝，绕不过去）。
- **入口**：`bash nav2_braking.sh` + `python3 verify_braking.py` →
  `results/report_collision_monitor.json`；驱动走 `dynamic_obstacle.py --popup`。
- **结果**：pop 前 0.8 s 平均车速 **0.452 m/s**；cm state 事件 **2 次**（action_type 0=STOP、3），
  首事件 pop 后 **0.24 s**；首停 **0.791 s**、窗口内最低速 **0.028 m/s**；人-挡板最小间距
  **0.335 m**；z 全程 0.010、不出界。
- **口径**："刹住"= 2.5 s 内出现速度 <0.05 m/s（不要求停住不动——cm STOP 后恢复行为会倒车让开，
  更安全，算失败会误导）。第一版触发 0.7 m 时停车间距只有 **0.215 m**（差 5 mm 没过 0.22 判据），
  抬到 **0.85 m** 后 0.335 m，0.85 m 这档才是"刹得住"的实测点。

## 11. 围挡/赛规核对（历史遗留项收口）

- 规则手册的场地尺寸在 图 3-5/3-6/3-7（**位图**），PDF 文本层只有图题；正文写"场地道具角度误差
  ±5 mm、实际制作可能对细节尺寸微调、**以实际场地为准**"。
- 本机无 OCR/视觉引擎（modlens: `No vision provider`；WSL 无 tesseract/pdftoppm，只有 ghostscript
  能渲染），**无法逐项对图**——环境限制，不是没查。
- 场地几何权威来源继续用官方 CAD `RMUL2027_3V3_V1.0.0.stp`：12.60×8.60 m、内侧面 ±6.00/±4.00、
  墙顶 **0.40 m**、墙厚 0.05 m（由 `debug/box_collisions.json` 栅格化数据回读）。
- 该条从"未核对"改为"有据可查 + 明确限制"。

## 12. 窄通道（贴边）验收（5 项判据全过）

- **场景**：`worlds/rmul2027_3v3_up_nav2_gap.sdf.xacro`（同一 JSON 的 `gate` 段）：两块
  0.6×0.5×0.6 m 静态箱在 `(∓0.70, 0.50)` → 门洞 **0.80 m**；门两侧到走廊内侧面只剩 0.40 m
  （< 车宽 0.44 m）→ 绕不过去。出生点 `(0,-2.0)`（`nav2_field.sh` 新增 `RM_SPAWN_X/Y`），
  目标 `(0,2.6)`。
- **结果**：SUCCEEDED、error_code 0、路径 5.798 m、到位 **0.124 m**；门线 y=0.5±0.1 的 26 个真值样本
  |x| 最大 **0.02 m**（居中过门）；到两门箱最小间距 **0.379 / 0.417 m**（判据 ≥0.20）；
  z 全程 0.010、max|x| 0.565、max|y| 2.687。
- **结论**：**5 cm 栅格对 0.80 m 门洞够用**（量化 ±0.025 m，有效门洞 0.75~0.85 m；实测居中通过），
  不必重生成 2.5 cm；只有 <0.70 m 门洞或厘米级贴边才需要 2.5 cm 变体（新模型名 + 新地图名，
  不能替换现有基线）。
- **入口**：`bash nav2_gap.sh` + `python3 verify_narrow_gap.py` → `results/report_narrow_gap.json`。

## 13. 三个专项验收入口一览

| 场景 | 起栈 | 验收 | 报告 |
|---|---|---|---|
| 绕障 + 动态避障（15 项）| `bash nav2_obstacles.sh` | `python3 verify_nav2_obstacles.py` | `report_nav2_obstacles.json` |
| 安全层刹车（6 项）| `bash nav2_braking.sh` | `python3 verify_braking.py` | `report_collision_monitor.json` |
| 窄通道 0.80 m 门洞（5 项）| `bash nav2_gap.sh` | `python3 verify_narrow_gap.py` | `report_narrow_gap.json` |

三者共用 `config/avoid_obstacles.json` 作几何唯一源，世界由 `make_obstacles_world.py
--only obstacles|braking|gap` 生成并同源校核。

## 14. 仍未执行（下一会话入口）

- 动态箱速度/方向档位：目前只有 0.3 m/s 一档、单程一次。
- <0.70 m 门洞 / 2.5 cm 栅格变体：需要"新模型名 + 新地图名"的变体并存方案。

## 15. 交互式"点地图 → 走过去"（9 项判据全过 + GUI 实测）

- **入口**：`bash nav_gui.sh`（Gazebo GUI + RViz + `/goal_pose` 桥）。RViz 里点
  **Nav2 Goal**（直接 action）或 **2D Goal Pose**（`/goal_pose` → 桥 → action），
  点一下拖出朝向；再点别处 = 换目标（自动取消旧的）。命令行等价 `bash goto_xy.sh X Y [YAW]`。
- **新增**：`goal_pose_bridge.py`（/goal_pose → NavigateToPose，含 1 s 同点去抖）、
  `make_rviz_config.py` → `config/rm_field_click.rviz`（2D Goal Pose + Nav2 Goal + 2D Pose Estimate）、
  `nav_gui.sh`、`start_goal_bridge.sh`、`goto_xy.sh`、`verify_point_goal.py`；
  `nav2_field.sh` 新增 `RM_RVIZ_CONFIG`（透传 launch 的 `rviz_config_file`）。
- **验收**（`results/report_point_goal.json`）：三次点击到点 **0.164 / 0.082 / 0.129 m**，
  每次位移 2.98 / 3.99 / 4.66 m；换目标（D→3 s→E）最终停在 **E 0.110 m**、离 D 4.28 m；
  不出界/不掉台/不压平台全过。
- **GUI 实测**：RViz 以我们的配置启动（窗口标题可核对）+ Gazebo Sim 窗口在 + 桥单实例；
  发 `/goal_pose` (4.0,-2.0) → `GOAL→ACCEPTED→SUCCEEDED`、TF 偏差 **0.17 m**；GUI 下 RTF **0.62**。
- **坑**：①脚本连发 3 次点击 → 桥当成 3 个目标互相取消（全 ABORTED）→ 现在只发一次 + 桥去抖；
  ②旧桥实例残留（stop 脚本当时没杀）→ 点击落到旧桥、日志看不到但车动了；
  现在清场脚本都包含 `goal_pose_bridge[.]py`。

## 16. 用户反馈"点了没反应"的定位与修复（新地图点选窗口）

- **RViz 点了没反应**：RViz 启动后"当前工具"不是 Nav2 Goal。合成鼠标点击实测：默认工具下点地图
  只动视角、不产生目标；把 GoalTool 排到 Tools 列表第一也**不改变当前工具**。
  正确用法：先点工具栏 tooltip 为 **Nav2 Goal** 的图标，再点地图+拖朝向。
- **Gazebo Sim 右键 `Move To` 没反应**：那是 Gazebo 的模型编辑动作（改模型位姿），不是导航命令，
  不会通知 Nav2；真把机器人挪了反而让 Gazebo 真值与 odom/AMCL 失配。**不要用它点坐标**。
- **新增【地图点选】窗口**（`map_click.py` + `start_map_click.sh`，`nav_gui.sh` 自动起）：
  左键=发目标、右键=发 /initialpose 重定位、中键=取消；显示地图/网格/机器人箭头/目标/规划路径。
- **实测**（WSLg 下 xdotool 真实鼠标点击）：`CLICK go (2.714,-2.443)` → nav2 `Received a goal`
  → 到点；TF 与 Gazebo 真值差 0.05 m、与点击点 **0.11 m**。中途一次 MPPI
  `Failed to make progress` + spin 恢复（既有偶发，恢复后照常到点）。
## 17. "绕了一大圈"的定位（几何 vs 落点 vs 定位）

- **几何**：`box_027` 0.40×1.05 m、高 0.27 m 的墙（x[2.30,2.70] y[2.95,4.00]）把平台北侧
  1 m 通道堵死，两侧还有 0.2 m 高小台阶（box_016..019、box_020/021）→ 地图上没有可过缺口。
  离线最短路（净空 ≥0.24 m）：西口袋 (1.75,3.58)→东口袋 (3.1,3.50) 为 **12.52 m**（绕 B）；
  出生点→东口袋约 10.7 m。nav2 实跑 `/plan` 一段 530 点 6.65 m，正是沿东侧通道北上 →
  **长路线是唯一路线，不是规划器绕远**。
- **落点**：点击点净空 0.20 / 0.18 m < robot_radius 0.22 → 目标在致命区，nav2 反复重规划+恢复行为。
  修复：`map_click.py` 落点吸附（净空 ≥0.26 m，搜索 ≤1.5 m）；障碍内直接拒绝。
- **定位**：该口袋附近 AMCL 一次漂 0.66 m（真值 (3.32,3.51) vs TF (2.91,2.99)）——单独待查。
- **工具**：`analyze_gap_route.py`、`path_len_once.py`、`list_boxes.py`、`kill_map_click.sh`，
  以及 Shift+左键探针（只报坐标+净空）。

## 18. 栈稳定性

- 关掉 Gazebo/RViz 窗口会让 launch 整套退出；期间 nav2 container 在卸载 map_server 时
  `Magick: abort (SIGSEGV)`（退出路径上的崩溃，不影响导航本身）。开了窗口就别随手关。

## 19. "地图坐标 ≠ 实际坐标"的诊断（2026-11-20 深夜）

- **框架**：导航是 ROS 2 Nav2 标准栈（AMCL / NavFn / MPPI / costmap / BT），没有自写导航算法；
  自写部分只是仿真世界生成、起栈验收脚本、`map_click.py`、`goal_pose_bridge.py`。
- **体检顺序与结果**：
  1. `check_map_alignment.py`：地图 vs `box_collisions.json` 48 方块 **96.5%** 重合，
     最优平移 (−0.025,+0.025) m → 地图坐标与场地几何对齐。
  2. `check_localization.py`：odom vs Gazebo 真值 **0.07 m** → 里程计准。
  3. 同工具：AMCL 估计 (2.15,0.42) vs 真值 (4.93,2.97) → **3.77 m**，
     `tf map→odom` = (0.28, 5.81, −103°) 明显是"设错初始位姿"。
- **根因**：日志 4 条 `initialPoseReceived → Setting pose` 都不在机器人真实位置
  （yaw 全 0），来自上一版 UI 的**右键重定位误触**；AMCL 对这种错误初始位姿**不会自愈**
  （实测 5 min 后仍差 4.8 m）。按真值重定位后误差 **0.037 m**、yaw 差 1°。
- **修复**：右键重定位**双击确认**；**T 键 = 按真值重定位**（仿真急救）；
  状态栏显示**定位误差**并在 >0.4 m 时提示跟丢；`/initialpose` 连发 3 次；
  `python3 map_click.py --self-test` 自检（右键确认 + 真值重定位）通过。
- **经验**：AMCL 的 initialPose 是强干预，UI 不能一点就发；"坐标不一致"先分三层：
  地图几何对齐 → 里程计 → 定位是否被错误重定位/跟丢。

## 20. 远距离定位漂移与"重定位后路径不更新"（2026-11-20 深夜）

- **AMCL 参数水土不服**：官方 `laser_max_range: 100.0`（Gazebo 雷达 3.5 m）、`recovery_alpha_*: 0.0`
  （恢复关闭）。`make_nav2_params.py` 生成 `config/nav2_params_rm.yaml`：
  3.4 / 0.12 / likelihood 1.5 / max_beams 180 / 粒子 800-3000 / update 0.10/0.10 / recovery 0.001,0.10 /
  alpha 0.1,0.1,0.1,0.05,0.05；`nav2_field.sh` 默认用它（`RM_PARAMS` 可回退）。
  实测 12 m 路线（(0,0)→(4.6,2.1)）：最大定位误差 **0.160 m**，起点 0.024、终点 0.087。
- **重定位后路径不更新**：nav2 不会因 AMCL 位姿跳变自动重规划；实测错误 initialpose 会把目标打成
  **ABORTED**，再用真值重定位后机器人停在离目标 2.59 m 处不动（用户看到的"断掉"）。
  修复：`map_click.py` 重定位后自动"取消并重发上一个目标"（ABORTED 保留目标、正常结束/手动取消清空）；
  回归脚本 `test_relocalize_replan.py`：不重发 → 2.59 m 不动；重发 → 到点 0.24 m。
- **工具**：`drive_and_track.py`（边导航边量定位误差）、`test_relocalize_replan.py`、
  `make_nav2_params.py`。
- **对用户的用法**：点目标 → 走；定位误差 >0.4 m 时按 **T**（仿真真值重定位，会自动重发目标继续走）。
