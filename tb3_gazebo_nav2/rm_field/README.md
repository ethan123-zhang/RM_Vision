# RMUL 2027 3V3 场地 → Gazebo（Harmonic / gz sim 8）加载与验收

把 `3D_CAD_Model/RMUL2027_3V3_V1.0.0.stp`（Creo 图纸）转成 Gazebo 可用的场地模型，
并做了**碰撞**与**比例**两方面的量化验收。全部脚本在本目录，可直接复跑。

---

## 0. 结论速览

| 项目 | 结论 | 依据 |
|---|---|---|
| 场地尺寸 | **12.600 × 8.600 m**，高 **0.503 m**（台面 z=0，地面 z=-0.503） | STEP bbox → STL bbox；正俯视相机像素反推 12.614 × 8.594 m（误差 0.11% / 0.06%） |
| 台面平整度 | 13 个落体探针全部落在 **z = 0.0000**，最大偏差 **0.0 mm** | `results/report_field_test.json` → `evaluation.flat` |
| 边缘位置 | X 边界实测 **6.295 m**、Y 边界 **4.295 m**（图纸 6.300 / 4.300，探针间距 1 cm） | `evaluation.edge_x / edge_y` |
| 边缘落差 | **0.503 m**（探针掉到地面 z=-0.503） | 同上 `drop_height_measured` |
| 机器人静置 | TB3 稳定停在 **z = 0.01001**（轮半径），1 s 漂移 **0.00 mm**，无下陷/抖动 | `robot.B1_rest` |
| 机器人直行 | 3 s @0.3 m/s 走 **0.8646 m**（理论 0.9，含加速段），全程 z 恒定 **0.01001** | `robot.B2_drive` |
| 冲出边缘 | 中心过 6.30 m 后翻落，最终 **z = -0.493**（= 地面 -0.503 + 轮高 0.010），无穿模 | `robot.B3_edge` |
| 机器人比例 | 正俯视图中机器人 13×14 px → **0.278 × 0.299 m**（TB3 底盘 0.281 × 0.306 m） | `analyze_topdown.py` |
| 碰撞体方案 | box 与 CAD 网格两种碰撞体结果一致；box 省内存（148 vs 215 MB），**默认用 box** | `results/report_field_test_mesh.json` |
| 台面高度（可测可见） | 正立面亮带 **1734 × 69 px** → 墙高 **501.3 mm**（焦距换算）/ **494.1 mm**（12.6 m 宽度做标尺），宽高比 25.50 vs 图纸 25.05；墙面近景墙高占画面 **62.6%** | `results/report_height.json`、§9 |
| 围栏验收 | 围挡顶落体探针 **4/4 落在 z=0.300**（偏差 0）、围挡外侧 2 个探针落到 **-0.503**；TB3 以 **0.7 m/s** 撞东/北围挡，停在 **x=6.1819 / y=4.1820**（离内侧面 6.8 cm = 车头半长 + 围挡厚），全程 **z 恒定 0.010**、`crossed_inner=false`、末 3 s 漂移 **0** | `results/report_fence.json`、§11.1 |
| Nav2 地图 | 顶视渲染 ↔ 几何生成地图互相校核：渲染受光台面 **12.500 × 8.500 m**、围挡中心线 **12.549 × 8.547**（图纸 12.550/8.550）、红顶沿环心偏差 **4.9 mm**；地图自由区 **106.25 m² = 12.50 × 8.50**，8/8 判据通过 | `results/report_nav_map.json`、§11.2 |
| Nav2 导航 | 场地世界 + 自建地图闭环：到 (4,2) 与回 (0,0) 两个目标均 **SUCCEEDED**（`error_code 0`，恢复 0 次），/scan **4.3–5.0 Hz**、/odom **27–29 Hz**、RTF **0.94**；全程 `cmd_vel` 峰值 **0.496 m/s**（参数 vx_max 0.5）、机器人 z 恒定 **0.010**、8/8 判据通过 | `results/report_nav2_field.json`、§11.3 |

> **注意**：这张 STEP 图纸里**只有场地地板/平台本身**（台面 + 0.503 m 高的下部结构），
> **没有围挡、护栏、坡道、立柱等高于台面的物体**。所以"撞墙/绕障"这类测试在图纸范围内
> 无从谈起，本目录的碰撞验收 = 台面支撑、无穿透、行驶稳定、边缘真实跌落。

---

## 1. 环境

- WSL2 Ubuntu-24.04 + ROS 2 Jazzy + Gazebo Harmonic（`gz sim 8.15.0`）+ dartsim
- 无 `/dev/dri`，渲染走 llvmpipe 软件 GL（GUI 能跑，帧率低；离屏相机出图很快）
- 工具：`gmsh 4.12.1`（含 OCCT，`apt install gmsh python3-gmsh`）、python3 + numpy + Pillow

## 2. 转换链路（为什么这么转）

```
RMUL2027_3V3_V1.0.0.stp   (单位 mm，SI_UNIT(.MILLI.,.METRE.))
   │  gmsh + OCCT：读 STEP → 曲面网格 → STL        step2mesh.py
   ▼
build/field.stl            (154,520 三角面；模型系 X=12.6m 长边, Z=8.6m 短边, Y=上)
   │  纯 numpy 变换 (x,y,z) → (x, -z, y-0.100)：米制 + Y-up→Z-up，台面压到 z=0
   ▼
models/rmul2027_3v3_field/meshes/field_zup.stl   (12.6 × 8.6 × 0.503 m，台面 z∈[-0.503, 0])
```

校验（`results/report_geometry.json`）：0 退化三角形、0 开放边、0 非流形边；
闭合体积 **15.8801 m³** 与 STEP 体积 15,880,106,384 mm³ 完全一致；表面积 349.331 m²；
台面（z=0）39,622 面片，台面板底（z=-0.100）29,728 面片。

## 3. 文件清单

> **网格不入库**（`.gitignore` 忽略 `tb3_gazebo_nav2/**/*.stl`，场地网格 7.5 MB）。
> 克隆后先跑一次重建（需要 WSL 里装 gmsh）：
> `python3 step2mesh.py && FIELD_ROOT=$PWD python3 make_field_assets.py`，
> 产出 `build/field.stl` 与 `models/rmul2027_3v3_field/meshes/field_zup.stl`。
> 只保留证据图入库：`shots/height_*.png`、`shots/gui_fenced_view*.png`、
> `shots/nav_topdown.png`、`shots/nav_map_overlay.png`。

| 文件 | 作用 |
|---|---|
| `step2mesh.py` | STEP → STL（gmsh），输出 bbox/体积报告 + 三视图 |
| `analyze_mesh.py` | 按 z 分层统计水平面，确认台面/结构层 |
| `make_field_assets.py` | 米制 + Y-up→Z-up 变换、网格质量检查、写最终 STL |
| `models/rmul2027_3v3_field/` | 场地模型（visual=CAD 网格，collision=12.6×8.6×0.503 的 box） |
| `models/rmul2027_3v3_field_mesh/` | 对照组：collision 直接用 CAD 网格 |
| `worlds/rmul2027_3v3_field.sdf` | 主世界（dartsim 1 ms、地面平面、场地、GUI 相机） |
| `worlds/rmul2027_3v3_field_meshcollision.sdf` | 对照组世界 |
| `worlds/visual_capture.sdf` | 出图专用：3 台相机（斜俯视/正俯视/近景）+ Sensors 系统 |
| `start_server.sh` | 起无 GUI 的服务器（跑测试用） |
| `run_field_gui.sh` / `gui_final.sh` / `gui_restart.sh` | 起 GUI（场地 + 机器人） |
| `stop_field.sh` | 停止 |
| `verify_field.py` | **主验收脚本**：31 个落体探针 + TB3 静置/直行/冲边 |
| `capture_camera.py` / `capture_final.sh` / `capture_overview.sh` | 离屏相机出图 |
| `capture_height.sh` | **出"台面高度"三张取证图**（正立面全景 / 墙面近景 / 低机位斜视）并跑像素验收 |
| `verify_height.py` | **台面高度验收**：几何预测（针孔投影）↔ 渲染实测（亮度剖面/亮面 bbox） |
| `annotate_height.py` | 在渲染图上标出墙顶/墙底、0.503 m 与 12.6 m 标尺，出 `shots/height_annotated_*.png` |
| `gui_view_report.py` | 统计 GUI 截图 3D 视口里台面/侧壁/背景占比（判"看着平不平"） |
| `onscreen_gui.sh` / `grab_x11.sh` / `grab_liveness.sh` | 把 WSLg 窗口挪进屏幕 / X 侧抓图 / 判断抓到的是不是冻结帧 |
| `analyze_topdown.py` | 用正俯视图像素反推场地/机器人尺寸（比例验收） |
| `measure_view.py` / `crop_ascii.py` / `seam_check.py` | 无视觉引擎时用 ASCII/统计"看"渲染结果 |
| `compare_perf.sh` | box vs 网格碰撞的启动时间/RTF/CPU/内存对比 |
| `gui_cam.py` / `make_gui_config.sh` | GUI 相机位姿（`/gui/camera/pose` 回读、`gui.config` 覆盖） |
| `show_gazebo.ps1` / `list_windows.ps1` / `grab.ps1` | Windows 侧抓 WSLg 窗口（焦点会还原） |
| `verify_fence.py` / `run_fence_test.sh` | **围栏验收**：围挡顶落体探针 + TB3 0.7 m/s 撞围挡不越界（§11.1） |
| `worlds/nav_map_capture.sdf` | 顶视取证世界：围挡场地 + 20 m 正俯视相机（1600×1100） |
| `capture_nav_map.sh` / `make_nav_map.py` | 出顶视图 → 按几何生成 Nav2 地图 → 渲染↔地图互相校核（§11.2） |
| `maps/rmul2027_3v3_fenced.{pgm,yaml}` | **Nav2 地图**：512×352 @ 0.025 m/px，自由区 = 围挡内侧面以内 12.50×8.50 m |
| `worlds/rmul2027_3v3_nav2.sdf.xacro` | Nav2 用世界（围挡场地 + Sensors/IMU 系统，接受 `headless` 参数） |
| `nav2_field.sh` / `stop_nav2_field.sh` | 起/停 Nav2 + 场地；`--gui` 带 Gazebo GUI + RViz |
| `verify_nav2_field.py` / `run_nav2_verify.sh` | **Nav2 导航验收**：两个目标闭环 + 不出界/不掉落/速度合规（§11.3） |
| `debug/make_box_collisions.py` | **翻转场地的碰撞**：structures.stl → 0.05 m 高度图 → 48 块 box，写回 model.sdf（§13）|
| `verify_box_collisions.py` / `verify_box_collisions.sh` | **box 碰撞验收**：静置 z / 直行 / 撞东墙 / 撞北墙 / 撞绿平台（§13.2）|
| `debug/box_collisions.json` | box 列表 + 墙内侧面 + 候选平台（验收脚本从这里读几何，不写死） |

## 4. 怎么跑

```bash
cd /mnt/e/RM/tb3_gazebo_nav2/rm_field

# A. 起 GUI 看场地（带一台 TB3 放在 (1.5,1.0)）
bash gui_final.sh

# B. 只跑服务器 + 完整验收（约 40 s，结果写 results/report_field_test.json）
bash start_server.sh
FIELD_ROOT=$PWD python3 verify_field.py

# C. 对照组：碰撞体换成 CAD 网格
bash start_server.sh $PWD/worlds/rmul2027_3v3_field_meshcollision.sdf mesh
FIELD_ROOT=$PWD REPORT_TAG=_mesh python3 verify_field.py

# D. 出图（无需 GUI；headless rendering + llvmpipe，约 30 s）
bash capture_final.sh            # -> shots/final_overview.png / final_topdown.png / final_closeup.png

# E. 从图纸重做一遍（需要 gmsh）
python3 step2mesh.py && FIELD_ROOT=$PWD python3 make_field_assets.py

# F. 停止
bash stop_field.sh
```

**手动开车（2026-11-19 新增）**：`bash drive_teleop.sh` 一条命令起 GUI 世界 + 小车 + 键盘遥控，
之后在终端里按 w/s/a/d 开车（详见 §14）。

Windows 侧抓图（WSLg 窗口在 X 侧 grab 会是旧帧，必须抓 Windows 合成后的画面）：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File E:\RM\tb3_gazebo_nav2\rm_field\show_gazebo.ps1
```

## 5. 验收细节

### 5.1 碰撞（`verify_field.py`）

- **A. 落体探针**：31 个 3 cm 立方体（mass 0.01）从 z=0.30 自由落体，4 s 后读位姿。
  - 13 个落在台面内部（含中心 3×3 m 区域）：全部 z = **0.0150**（= 台面 0 + 半高 0.015），即台面精确在 z=0；
  - X 向边缘扫 6.24→6.36 m（9 个，y 各不相同）：6.29 及以内停在台面，6.30 及以外掉到地面 z=-0.488；
  - Y 向边缘扫 4.24→4.36 m（9 个）：4.29 停留、4.30 掉落 → 边界 **4.295 m**。
  - 判据：`drop_height_measured = 0.503` 与模型高度完全一致，说明没有穿透、没有悬空。
- **B1 机器人静置**：TB3 waffle 放在台面，稳定 z=0.01001（轮半径 0.010），1 s 内 XY 漂移 0.000 m，无抖动/下陷。
- **B2 直行**：`/cmd_vel` 0.3 m/s × 3 s → 位移 0.8646 m（理论 0.9 m，差值来自 DiffDrive 的 1 m/s² 加速段 + 发布延迟），z 始终 0.01001 → 台面平整、无台阶。
- **B3 冲出边缘**：0.7 m/s 直行，t=8.139 s 中心越过 6.30 m，t=8.39 s 开始下坠，最终停在 z=-0.493（地面 -0.503 + 0.010），min z=-0.493，**没有穿过地面**。

### 5.2 比例

1. **几何**：STL bbox 12.6 × 8.6 × 0.503 m；闭合体积与 STEP 一致（15.8801 m³）→ 尺度没被拉伸。
2. **渲染像素**：正俯视相机（(0,0,20)，hfov 1.2 rad，1280×720，21.379 mm/px）里场地亮面 bbox 590×402 px
   → **12.614 × 8.594 m**，相对误差 0.109% / 0.064%；长宽比 1.4677（图纸 1.4651）。
3. **机器人比例**：同一张图里 TB3 底盘 13×14 px → 0.278 × 0.299 m（实际 0.281 × 0.306 m），
   说明"机器人相对场地"的尺度也是对的。

### 5.3 性能对比（`compare_perf.sh`）

| 碰撞体 | 世界就绪 | 稳态 RTF | 物理 CPU | 内存 |
|---|---|---|---|---|
| box（12.6×8.6×0.503） | 2117 ms | 1.0007 | 28.0% | 148 MB |
| CAD 网格（154,520 面） | 2086 ms | 0.9999 | 25.6% | 215 MB |

两者验收结果完全相同，box 省 67 MB 内存、接触计算更可预测，所以默认用 box；
网格碰撞保留为对照组，方便以后换场地时复验。

## 6. 踩过的坑（换环境时最容易再踩）

1. **STEP 是 Y 轴向上、单位 mm**：直接丢进 Gazebo 会躺倒且放大 1000 倍。必须做 `(x,y,z)→(x,-z,y-0.100)` + 0.001 缩放。
2. **`gz.msgs10` Python 绑定的 `EntityFactory.sdf` 字符串字段不序列化**（赋值成功、`SerializeToString` 丢掉）→ 生成模型必须用 `sdf_filename` 指文件，不能传字符串。
3. **`/world/<w>/create/blocking` 返回 false 不代表失败**：世界刚装载时首调会返回 false 但实体已生成；以 `pose/info` 出现为准（脚本里已做重试 + 校验）。
4. **TB3 的 DiffDrive 订阅的是全局 `/cmd_vel`**，不是 `/model/<name>/cmd_vel`。
5. **Gazebo GUI 的初始视角不在 world 的 `<gui><camera>` 里**，而在 `~/.gz/sim/8/gui.config` 的 `<camera_pose>`；本目录用 `config/gui_field.config` + `--gui-config` 覆盖。`/gui/camera/pose` 是 GUI 的**输出**（10 Hz 回读当前位姿），往里发消息不会改变视角。
6. **WSLg 窗口最小化后会跑到 (-32000,-32000)**，且 `import -window`（X 侧）抓到的是旧帧；要抓实时画面必须用 Windows 侧截图并把窗口 `SW_RESTORE`（见 `show_gazebo.ps1`，抓完会把焦点还给原窗口）。
7. **PowerShell 里 `wsl -d Ubuntu-24.04 -e bash -c "..."` 会被双引号/`$(...)` 破坏**：复杂命令一律写成 `.sh` 再 `bash /tmp/x.sh`。
8. 无视觉模型时，用 `crop_ascii.py` / `measure_view.py` 把渲染图压成 ASCII 亮度图 + 统计，同样能做像素级判定（本目录的比例结论就是这么来的）。

## 7. 后续可以接的

- 把场地接进 Nav2：正俯视渲染图 → 二值化 → `map.pgm/map.yaml`（场地 12.6×8.6 m，保守内缩 5 cm 做边界）。
- 图纸补了围挡/障碍后，同一套 `verify_field.py` 直接扩测试项即可。
- 若要做真实 RMUL 赛规尺寸校核，建议再人工核对一次图纸版本（V1.0.0）与官方 2027 规则。

---

## 8. 下一阶段待办（下个对话从这里接）

### 8.1 背景：关于"凸起的平台"

`map_surface.py`（把网格顶面按 5 cm 栅格化成高度图）的结论：
**图纸里的可行驶顶面是完全平的**——覆盖率 99.03%、面积 109.42 m²、
高度范围 **+0.0000 .. +0.0000 m**，没有任何台阶 / 凸台 / 洼坑 / 坡道。

所以：

- **"凸起的平台"就是整块 12.6×8.6 m、高 0.503 m 的场地本体**，它已经在模型里（渲染图里能看到
  0.503 m 高的四壁，地面在它下面 z=-0.503）；不是漏做。
- 图纸里**没有**额外的高台、围栏、挡板、坡道——凡图纸里没有的几何，我都没有自行添加；
  这也是机器人能直接冲出边缘的原因（有意为之：先按图纸 1:1 还原）。
- 用户"没看到平台"更可能是**GUI 视角问题**：默认 GUI 相机把场地拍成一个平铺的灰色大平面
  （**2026-11-01 已量化收口：整场视野里 0.503 m 只有约 16 px，加上配色对比不足；见 §9**）
  （实测视口内场地只占 14%~22%，其余是地面），加上 WSLg 窗口容易被最小化到 -32000，
  所以看不出高度。这一条是我没做好的部分。

### 8.2 TODO 1：场地边缘加围栏（最高优先级）—— ✅ 2026-11-01 已实现（作为可选变体）

已按下面的参数做成**独立模型 + 独立世界**，1:1 图纸世界保持不动：

| 参数 | 实做值 |
|---|---|
| 围栏高度 | 0.30 m（台面以上） |
| 厚度 | 0.05 m，四条边贴在 x=±6.275 / y=±4.275 |
| 碰撞体 | 4 个 `box`（0.30 m 高，TB3 爬不上去） |
| 外观 | 浅灰 0.80 + 红色顶沿（顶沿是让人一眼看出"墙顶在哪"） |
| 文件 | `models/rmul2027_3v3_fence/{model.sdf,model.config}` |
| 世界 | `worlds/rmul2027_3v3_field_fenced.sdf`（GUI 默认加载这个） |
| 1:1 验收 | 仍用 `worlds/rmul2027_3v3_field.sdf`（无围栏），`verify_field.py` 结论不变 |

```bash
bash gui_final.sh                                             # 默认：带围挡世界
WORLD=$PWD/worlds/rmul2027_3v3_field.sdf bash gui_final.sh     # 1:1 无围挡世界
```

**验收**（2026-11-06 已补，见 §11.1）：`verify_fence.py` —— 围挡顶 4 个落体探针全部停在
z=0.300，围挡外侧 2 个探针落到 −0.503；TB3 以 0.7 m/s 连续顶东/北围挡 16 s，停在
x=6.1819 / y=4.1820（离内侧面 6.8 cm，正好是车头半长 + 围挡厚），不越界、不掉落、末段无漂移。

### 8.3 TODO 2：把"平台高度"讲清楚（GUI 视角 + 取证图）—— ✅ 2026-11-01 已完成

做法、数字与结论见下面 **§9**。要点：GUI 相机改成低机位斜视（`config/gui_field.config`）、
补了正立面/墙面近景/低机位斜视三台取证相机，并把"墙高 = 0.503 m"做成可复跑的像素验收
（`verify_height.py`，已全部通过）。

### 8.4 TODO 3（可选）：接 Nav2 —— ✅ 2026-11-06 已完成

做法与数字见 **§11.2 / §11.3**。要点（有一处和原计划不同）：

- **地图不是"渲染图二值化"出来的**，而是**按 CAD 几何直接生成**（`make_nav_map.py`）：
  自由区 = 围挡内侧面以内（|x|<6.25、|y|<4.25），界外一律 0（障碍）→ 规划器永远不会把车
  规划到台子外。顶视渲染图只用来做**独立校核**（渲染实测受光台面 12.500×8.500 m，
  与地图自由区一致到 0.1%）。
- 为什么不用 1:1（无围挡）场地跑 Nav2：0.503 m 的台沿在 lidar 平面（z≈0.11）**之下**，
  AMCL 扫不到任何特征，只能纯里程计；加了 0.30 m 围挡后，AMCL 在 ±6.25/±4.25 有真实墙面。

```bash
bash nav2_field.sh              # 起 Nav2（headless），自动发初始位姿 (0,0)
python3 verify_nav2_field.py    # 两个导航目标闭环验收 → results/report_nav2_field.json
bash stop_nav2_field.sh
bash nav2_field.sh --gui        # 想亲眼看：Gazebo GUI + RViz
```

### 8.5 复跑命令

```bash
bash gui_final.sh                                   # 看场地（改围栏后换成 fenced 世界）
bash gui_final.sh $PWD/config/gui_wall.config       # 看侧壁特写（更容易看出 0.503 m）
bash start_server.sh && FIELD_ROOT=$PWD python3 verify_field.py
FIELD_ROOT=$PWD python3 map_surface.py models/rmul2027_3v3_field/meshes/field_zup.stl 0.05
bash capture_height.sh                              # 台面高度三张取证图 + 像素验收
python3 annotate_height.py                          # 生成带标注的证据图
bash run_fence_test.sh                              # 围栏验收（探针 + 0.7 m/s 撞墙）
bash capture_nav_map.sh                             # 顶视图 → 地图 pgm/yaml → 互相校核
bash nav2_field.sh && python3 verify_nav2_field.py && bash stop_nav2_field.sh   # Nav2 闭环
```

---

## 9. 台面高度：为什么"看着像一块平板"，以及怎么把它量出来（2026-11-01）

### 9.1 结论（先看这段）

**0.503 m 的台面高度是真的，模型没有漏做，也没有被压平。** 三条独立证据：

| 证据 | 实测 | 判据 |
|---|---|---|
| 正立面全景（1920×1080） | 亮带 **1734 × 69 px**；墙高 **501.3 mm**（用相机焦距算）、**494.1 mm**（用 12.6 m 宽度当标尺） | 对 503.0 mm 误差 **0.3% / 1.8%** |
| 墙面近景（1280×720） | 墙高 **451 px / 720 px = 62.6% 画面**；上/下边缘与预测差 **0.9 / 3.1 px** | "半米"肉眼可见 |
| 低机位斜视（1600×900） | 亮面（台面顶）bbox 与"零厚度平板"预测差 **≤ 3.5 px**；台面下缘再往下 **74 px** 侧壁带（预测 75.0 px，差 1 px） | 渲染出来的是 0.503 m 实体，不是一张贴图平面 |

**"看不到"是两个原因叠加，不是几何错：**

1. **旧 GUI 相机太高太远**：`(-9.5, -11.5, 9.0)`、俯角 26°，整场视野里 12.6 m 宽 → 0.503 m 只占
   `0.503/12.6 × 视口宽 ≈ 16 px`。这是几何决定的：场地长宽比 25:1，整场视野里"半米"永远是十几个像素。
2. **配色对比不足**：地面 `0.16 灰`、背景 `0.12`，场地 `0.55 灰`；渲染经过 sRGB 编码后，
   背景亮度 ≈ 103.7、地面 ≈ 106、墙面 ≈ 124.7 —— 三者在 100~125 这条窄带里，肉眼几乎分不开。

所以这轮只做两件**不改几何**的事：**相机改低机位斜视** + **地面/背景压暗**，
再补三台取证相机把高度做成**可复跑的像素验收**。

### 9.2 三台取证相机（`worlds/visual_capture.sdf`）

| 相机 | 位姿 (x y z roll pitch yaw) | hfov | 分辨率 | 用途 |
|---|---|---|---|---|
| `elevation_wide` | `0 -25.0 -0.2515 0 0 1.5708` | 0.65 | 1920×1080 | 整条前立面：12.6 m × 0.503 m 的亮带，宽高比 25.05 |
| `elevation_wall` | `2.0 -9.0 -0.2515 0 0 1.5708` | 0.30 | 1280×720 | 墙面近景：距墙 4.7 m，墙高占画面 62.6% |
| `oblique_low` | `-10.5 -9.0 2.2 0 0.1650 0.7086` | 1.20 | 1600×900 | 低机位 3/4 斜视：台面 + 侧壁 + 地面同框（"这是高台"主图） |

两台正立面相机都放在**墙体半高** `z = -0.2515`：这样上/下对称，近处立面把远处立面完全遮住，
画面就是一整条干净矩形，量高度不受视差干扰（放高一点，远处墙顶会露在近墙之上，边缘就乱了）。

### 9.3 验收方法：几何预测 ↔ 渲染实测（不是"看着像"）

```
SDF 里读到相机位姿/hfov/分辨率
   │  针孔模型：f=(W/2)/tan(hfov/2)
   │  u = W/2 - f·y_c/x_c,  v = H/2 - f·z_c/x_c
   ▼
预测：墙顶/墙底落在第几行、亮带该多宽、墙带该多厚
   ▲
   │  渲染实测：列中值亮度剖面找最大梯度 / 亮面 bbox / 边缘行
   │
真实渲染图（headless + llvmpipe）
```

两条链对得上，才说明"渲染出来的墙 = 0.503 m 且比例没被拉伸"。
`verify_height.py` 里 11 条断言全部通过，报告在 `results/report_height.json`。

低机位那张还带一个**反事实检验**：同时预测"有 0.503 m 厚度"和"零厚度平板"两种 bbox，
实测亮面贴合前者、并比后者向下多出 74 px 的侧壁带 —— 直接排除"这只是一张平面图"。

### 9.4 怎么跑 / 产出什么

```bash
cd /mnt/e/RM/tb3_gazebo_nav2/rm_field
bash capture_height.sh          # 起无 GUI 渲染服（约 40 s）→ 三张图 → 自动跑验收
python3 annotate_height.py      # 生成带红线的标注图（人眼直接看）
```

| 文件 | 内容 |
|---|---|
| `shots/height_elevation_wide.png` | 正立面全景（12.6 m × 0.503 m 亮带） |
| `shots/height_elevation_wall.png` | 墙面近景（墙高 451 px） |
| `shots/height_oblique_low.png` | 低机位斜视（台面+侧壁+地面） |
| `shots/height_annotated_wide.png` | 上面两张 + 墙顶/墙底线、0.503 m 双箭头、12.6 m 标尺 |
| `shots/height_annotated_wall.png` | 同上（近景版） |
| `shots/height_annotated_oblique.png` | 斜视图 + "visible side wall band = N px (= 0.503 m)" |
| `results/report_height.json` | 每台相机的预测值/实测值/断言结果 |

### 9.5 GUI 里怎么看

- 默认 `config/gui_field.config` 的 `<camera_pose>` = `-8.5 7.5 2.6 0 0.226 -0.723`
  （低机位 3/4 斜视，+y 侧：台面 + 近处围栏 + 侧壁 + -0.503 m 地面 + 台子在地面上的投影都入画）。
  生效与否用 `/gui/camera/pose` 回读确认（这是 GUI 的**输出**，不是设置入口）：
  ```bash
  bash gui_pose_check.sh      # 打印 GUI 当前相机位姿，应与 config 里一致
  # 实测回读 position = (-8.5, 7.5, 2.6)，四元数对应 pitch 0.226 / yaw -0.723 ✓
  ```
- 默认加载带围挡世界（`rmul2027_3v3_field_fenced.sdf`）；要看 1:1 无围挡：
  `WORLD=$PWD/worlds/rmul2027_3v3_field.sdf bash gui_final.sh`。
- 想看清楚"半米"的墙：`bash gui_final.sh $PWD/config/gui_wall.config`（相机在墙前 2.7 m）。
- 心里要有数：**整场视野里 0.503 m 只有 20~40 px**，要"一眼看见半米"只能靠近景或正立面图。

### 9.6 这轮踩的坑

1. **XML 注释里的 `--` 会炸严格解析器**：世界文件注释里有 `--headless-rendering`，
   `xml.etree` 报 `not well-formed (invalid token)`（gz 自己的解析器容忍，所以仿真照跑）。
   `verify_height.py` 里解析前先把 `<!--...-->` 整段剥掉。
2. **渲染图是 sRGB 编码，不是线性亮度**：背景 `0.12 0.14 0.17` 线性值渲染出来是 **103.7**，
   不是 35。按线性值设阈值（65/110）会把整张图判成"全亮"，必须按实测定阈值。
3. **正立面相机的高度取"墙体半高"最干净**（见 §9.2 最后一段）。
4. **GUI 抓图在 WSLg 下不可靠**：X 侧 `import -window` 有时拿到冻结帧（隔 3 s 抓两张差异 0.02%），
   Windows 侧 Gazebo 窗口跑在副显示器 `(-2527, 16)`，`CopyFromScreen` 会抓不全。
   → 画面验收一律用离屏相机（同一世界、同一套相机参数），GUI 只做位姿回读 + 一次实拍交叉验证
   （2026-11-01 实拍剖面 天空 56 / 围栏 137 / 台面 214 / 侧壁 117 / 地面 90，与离屏一致）。
5. **世界里的 `<gui><camera><track>` 会把相机拽走**：`<track>rmul2027_3v3_field</track>`
   是"跟随模型"模式，会覆盖 `--gui-config` 的 `camera_pose`（实测回读变成 6.3 m 外的环绕位姿）。
   已从两个场地世界里删掉 `<track>`，位姿才稳定生效。

---

## 10. 第二轮：从"平面上面有个小车"到"高台 + 围栏"（2026-11-01 晚）

### 10.1 现象

用户看完 GUI 的反馈原话：**"我并没有看到有围栏和凸起的平台，我的视角还是平面上面有个小车。"**

### 10.2 先复现，别猜

不看 GUI 截图（WSLg 抓图不可靠，见 §9.6 第 4 条），而是把 **GUI 的位姿/视野原样搬到离屏相机**：
`worlds/gui_capture_fenced.sdf` 的 `/gui_view/image` = 相机 `-8.5 7.5 2.6 / pitch 0.226 / yaw -0.723`、
hfov 1.15、1000×845。然后**逐行看亮度剖面**，一行一行读出画面里有什么。

第一版（旧配色）剖面的数字：

```
台面 185.6 | 侧壁 124.7（约 70 px） | 地面 48.1 | 背景 39.6
```

侧壁**确实画出来了**，但地面和背景几乎同黑（48 vs 40），台子周围一片全黑。

### 10.3 四级原因（按贡献排序）

| # | 原因 | 证据 | 结果 |
|---|---|---|---|
| 1 | 相机俯角只有 12°、距离 12 m | 0.503 m 在画面里 20~40 px | 整场读成"薄片" |
| 2 | 地面亮度 ≈ 背景亮度 | 48 vs 40（差 8） | 台子没有"落在地面上"的参照 |
| 3 | **台子的投影亮度 ≈ 天空亮度** | 都 ≈ 56 | 台子根部一圈黑，读成**浮在黑洞上的卡片** ← 主因 |
| 4 | 整块场地单一材质 + 无竖直参照物 | 只有一个灰度 | 除侧壁外没有任何高度线索 |

第 3 条最容易漏：把地面平面从 60×60 m 放大到 400×400 m 之后那条黑带还在，才确认它不是
"看不到地面"，而是**阴影色和天空色撞在一起**。改法不是"把影子调亮"，而是让天空更暗，
使 `天空 < 投影中的地面 < 亮地面 < 侧壁 < 围栏 < 台面` 六级都分开。

### 10.4 改了什么（只动颜色/光照/相机 + 新增围栏，场地几何未动）

| 项目 | 之前 | 现在 | 为什么 |
|---|---|---|---|
| 天空 `background` | `0.02 0.02 0.025` | `0.020 0.023 0.032` | 压到 42，和投影/地面拉开 |
| 环境光 `ambient` | 0.55 | 0.35 | 环境光太高会把投影冲没 |
| 主光 sun | 0.95 / `-0.35 0.25 -0.9` | 1.0 / `-0.25 0.30 -0.92` | 偏顶光：台面亮、投影短而清楚 |
| 补光 fill | 无 | 0.60 / `0.60 -0.40 -0.70` | 背光的 -x/+y 侧壁不再黑成背景 |
| 地面材质 | 0.03 灰 | `0.09 0.095 0.11` | 地面变成"能看见的一层" |
| 地面平面尺寸 | 60×60 m | 400×400 m | 原来的掠射光线打到平面外就露出天空色 |
| 围栏 | 无 | 0.30 m 高 / 5 cm 厚 / 带碰撞 / 红色顶沿 | 竖直参照物，机器人也掉不下去 |
| GUI 相机 | `-9 -7.5 2.6` | `-8.5 +7.5 2.6`（+y 侧） | 这一侧能看到台子投在地面上的影子 |
| `<gui><camera><track>` | 有 | 已删 | 它会覆盖 `--gui-config` 的 camera_pose |

### 10.5 现在的亮度阶梯（实测，同一条中心列）

| 层 | 离屏渲染 | GUI 实拍（WSLg 窗口） |
|---|---|---|
| 天空 | 42 | 56 |
| 投影中的地面 | 88 | — |
| 亮地面 | 95 | 90 |
| 平台侧壁 | 117~123 | 117 |
| 围栏 | 137 | 137 |
| 台面 | 214 | 214 |

六档分开、GUI 与离屏一致；台子在 GUI 里现在读作"一块落在某层地面上的高台，四周有围栏"。

### 10.6 验收命令

```bash
bash capture_probe.sh gui_fenced_view.png \
     worlds/gui_capture_fenced.sdf /gui_view/image    # 出 GUI 视角的等价渲染
python3 probe_report.py shots/gui_fenced_view.png     # 逐行亮度剖面（看六级阶梯）
python3 annotate_gui_view.py                          # 三条线：围栏顶 / 平台边 / 地面 + 标注图
bash capture_height.sh                                # 0.503 m 的既有验收仍然全部通过
```

`annotate_gui_view.py` 输出的预测↔实测（同一条列，x=0.6 处）：

| 边界 | 预测行 | 实测行 | 偏差 |
|---|---|---|---|
| 围栏顶（z=+0.30） | 442.4 | 437.5 | −4.9 px |
| 平台边（z=0） | 466.5 | 465.5 | −1.0 px |
| 地面（z=−0.503） | 506.2 | 502.5 | −3.7 px |

只改颜色/光照，所以 §4 的比例与碰撞验收（`results/report_field_test.json`）**不受影响**；
§9 的台面高度验收重跑后数字不变（501.3 mm / 494.1 mm / 451 px 等），结论仍是"全部通过"。

### 10.7 新增文件

| 文件 | 作用 |
|---|---|
| `models/rmul2027_3v3_fence/{model.sdf,model.config}` | 0.30 m 围挡（visual + collision + 红顶沿） |
| `worlds/rmul2027_3v3_field_fenced.sdf` | GUI 默认世界（1:1 场地 + 围挡 + 新配色） |
| `worlds/gui_capture_fenced.sdf` | 带围挡世界 + 与 GUI 同位的相机（出"你应该看到什么"） |
| `worlds/gui_probe.sdf` ~ `gui_probe_v4.sdf` | 配色/光照实验留档（四版对照） |
| `capture_probe.sh` | 起离屏服务器 → 生成 TB3 → 抓任意相机话题 |
| `probe_report.py` | 逐行亮度剖面 + 台面/侧壁/地面分层判据 |
| `annotate_gui_view.py` | GUI 视角标注图（围栏 0.30 m / 平台 0.503 m）+ 预测↔实测 |
| `grab_liveness.sh` | 判断 WSLg 抓图是不是冻结帧 |

### 10.8 还没验证的

1. ~~**围栏碰撞没测**~~ → ✅ 2026-11-06 已补测，见 §11.1（探针落在 z=0.300、0.7 m/s 撞墙不越界）。
2. 围栏高度 0.30 m 是"TB3 爬不上去"的工程估计，赛规若有明确数值需按赛规改。
3. 1:1 图纸世界（无围栏）仍是验收基线；带围挡世界只用于交互与观感。

---

## 11. 第三轮：围挡验收 → Nav2 地图 → Nav2 闭环（2026-11-06）

这一轮把 §8.2 欠的两条验收补齐，并把场地正式接进 Nav2。三块各自有独立的验收脚本，
判据全是读数（探针 z、坐标、动作状态、话题频率），没有"看着像"。

### 11.1 围挡挡不挡得住（`verify_fence.py` → `results/report_fence.json`）

| 测试 | 做法 | 实测 | 判据 |
|---|---|---|---|
| A 围挡顶是不是实体 | 4 个探针从 z=0.70 落在东/西/南/北围挡**中心线**上 | 4/4 停在 **z = 0.300**，最大偏差 **0.0** | 落体停在围挡顶面 (= 0.30) |
| B 围挡外面是不是空的 | 2 个探针放在 (6.40,0)/(0,4.40)，即围挡外侧 | 2/2 落到 **z = −0.503** | 直接落到地面（没有隐形平台） |
| C-x 撞东围挡 | `/cmd_vel` 给 0.7 m/s 连续 16 s，正面顶墙 | 停在 **x = 6.1819**（理论 6.1815，误差 **0.4 mm**），`crossed_inner = false`，**z 恒定 0.010**，末 3 s 漂移 **0.0** | 不超过内侧面 6.25、不掉落、停住后无抖动 |
| C-y 撞北围挡 | 同上，朝 +y | 停在 **y = 4.1820**（理论 4.1815，误差 **0.5 mm**），其余同上 | 同上 |

**为什么停在 6.1815 而不是 6.25**：TB3 碰撞盒 0.265×0.265 m，模型里盒心比 base 原点
后移 0.064 m，所以车头前表面在中心前 **0.0685 m**；围挡内侧面 6.250，减去 0.0685 = **6.1815**。
实测 6.1819 → 误差 0.4 mm，说明"车头贴住围挡内侧面"这件事是能量出来的，不是目测。

**踩过的坑**：第一版把"最后 3 s 位移"当抖动判据，得到 0.63 m（≡ 12.6 m/s，物理不可能）——
是 Gazebo pose 采样的一次性瞬跳，不是车在抖。改成先按步长 >0.15 m 剔除跳变再算漂移，
重跑 `jumps = 0`、漂移 0.0，连续两次一致。

```bash
bash run_fence_test.sh      # 起服务器 → 探针 + 撞墙 → 停服务器，~70 s
```

### 11.2 顶视渲染 → Nav2 地图（`capture_nav_map.sh` + `make_nav_map.py`）

**关键决定：地图按几何生成，渲染图只做独立校核。**

- 地图（`maps/rmul2027_3v3_fenced.pgm/yaml`）：512×352 px @ **0.025 m/px**，origin **(−6.40, −4.40)**，
  覆盖 12.80×8.80 m；自由区 = **|x| < 6.25 且 |y| < 4.25**（围挡内侧面以内），其余全 0。
  0.025 m/px 是特意选的：12.5/0.025 = 500、8.5/0.025 = 340 正好整数像素，自由区面积可精确断言。
- 为什么不做"渲染图二值化"：渲染有阴影（东/南侧台面边缘一条 0.15 m 暗带）、透视（围挡红顶沿
  在 z=+0.33，投影比台面边界外扩 ~0.15 m）、抗锯齿，二值化后边界会带 1~3 px 系统偏差；
  而 CAD 几何给出的内侧面位置是定义值，不该被渲染噪声污染。
- 渲染图的作用变成**独立交叉校验**（`nav_map_capture.sdf`，20 m 正俯视 1600×1100，f=2000 px）：

| 校核项 | 实测 | 图纸/几何 | 偏差 |
|---|---|---|---|
| 受光台面 bbox（= 围挡内侧面以内） | **12.500 × 8.500 m**（1250×850 px） | 12.500 × 8.500 | **0.0** |
| 围挡红顶沿中心线间距（z=+0.33 透视改正） | **12.5492 × 8.5465 m** | 12.550 / 8.550 | 0.8 / 3.5 mm |
| 红顶沿环中心 | (−0.0049, +0.0049) m | (0, 0) | 4.9 mm |
| 红顶沿宽度 | 0.0787 / 0.0688 m | 0.070 | 8.7 / 1.2 mm |
| 地图自由区面积 | **106.25 m²** | 12.50 × 8.50 = 106.25 | **0.0** |
| 地图自由区 bbox | **12.500 × 8.500 m** | 12.500 × 8.500 | **0.0** |

8 条判据全过（`results/report_nav_map.json` → `summary.ok = true`）。叠加图
`shots/nav_map_overlay.png`：绿框 = 地图自由区边界，青框 = 图纸 12.6×8.6 场地边界。

**为什么 Nav2 必须用带围挡的场地，不能用 1:1 基线世界**：0.503 m 的台沿在 lidar 扫描平面
（z≈0.11）**以下**，1:1 世界对 AMCL 来说是一片空场，只有里程计；0.30 m 围挡正好横在扫描平面
上，±6.25/±4.25 有真实墙面可用于定位。

### 11.3 Nav2 闭环（`nav2_field.sh` + `verify_nav2_field.py` → `results/report_nav2_field.json`）

链路：`nav2_bringup/tb3_simulation_launch.py`（`world:=` 我们的 xacro、`map:=` 我们的地图、
`params_file:=` nav2 默认参数、`headless:=True`）+ `nav2_minimal_tb3_sim` 的机器人/桥接。
起点 (0,0)，AMCL 初始位姿发三次。

| 判据 | 实测 |
|---|---|
| 仿真链路 | /scan **4.3–5.0 Hz**、/odom **27–29 Hz**、仿真时钟与 TF `map→base_footprint` 可用、RTF **0.94** |
| 目标 1 (4.0, 2.0) | **SUCCEEDED**，`error_code 0`，恢复 **0** 次，11.6 s 仿真时长，路程 4.335 m，平均 0.374 m/s，终点距目标 **0.179 m** |
| 目标 2 (0.0, 0.0) | **SUCCEEDED**，`error_code 0`，恢复 **0** 次，27.0 s，路程 4.824 m，平均 0.179 m/s，终点距目标 **0.037 m** |
| 不出界 | 全程 max\|x\| = **3.875 m**、max\|y\| = **1.877 m**（围挡内侧面 6.25/4.25，留 0.15 m 车身余量后限值 6.10/4.10） |
| 不掉落 | 全程机器人 z = **0.010 ~ 0.010**（Gazebo 实测，未掉下 0.503 m 台子） |
| 速度合规 | `cmd_vel` 峰值 **0.496 m/s** ≤ 参数 `vx_max 0.5`；角速度峰值 0.791 rad/s |

**踩过的坑（两条，都是判据本身错了，不是系统错）**：

1. 一开始用 `/amcl_pose` 算终点和时长，得到"11.4 s 走完 4.47 m"→ 平均 0.39 m/s。原因是 AMCL
   只在"移动 >0.25 m 或转 >0.2 rad"时才发布，用它算终点会滞后半米、算时长会丢时间。
   改成直接采样 TF `map→base_footprint`（导航栈真正在用的位姿），并区分 wall/sim 时间。
2. 判速度时凭印象写了"TB3 上限 0.26 m/s"，实测 0.38 m/s 直接判失败。查 `nav2_params.yaml`
   发现 `vx_max: 0.5`（TB3 底盘本身不限速，快慢完全由这份参数决定）。改成**从参数文件读
   `vx_max`** 再用 `/cmd_vel` 实测峰值对比 → 0.496 vs 0.5，合规。

```bash
bash nav2_field.sh                  # ~40 s 拉起，打印 7 个节点生命周期状态
python3 verify_nav2_field.py        # ~60 s，两个目标闭环，落 report_nav2_field.json
bash stop_nav2_field.sh
bash nav2_field.sh --gui            # Gazebo GUI + RViz（窗口已确认可开：Gazebo Sim / RViz）
```

**还没验证的**：

1. 只测了无障碍的"两点往返"；障碍物绕行、动态避障、多目标巡逻没测（场地现在是空矩形）。
2. 参数用 nav2 默认值，没为 12.6×8.6 场地调过（inflation 0.55 m 会吃掉一部分通道宽度）。
3. 围挡高度 0.30 m 与赛规一致性未核对（见 §10.8 第 2 条）。
---

## 12. 2026-11-19：按用户判断整体翻转场地 + 重做配色（含一个已知 bug）

**结论**：场地模型整体上下翻转，变成"结构面朝上"的 3V3 场地；配色重做为
**浅灰=可行驶地面（z=0，97.3 m²）/ 绿=内部凸起平台 / 红=外围墙 / 黄=墙顶沿 / 深灰=台体底板与场外地面（z=-0.10）**。

- 翻转实现：`debug/make_flipped_meshes.py`（z→-z、反转三角形绕序、再整体 −0.10 m，使场地地面落在 z=0），
  新模型 `models/rmul2027_3v3_field_up/`（网格不入库，克隆后必须先跑这个脚本）。
- 驾驶世界：`worlds/drive_colored.sdf`（world 名 `drive`），启动 `bash gui_colored.sh`；
  出图世界 `worlds/drive_colored_preview.sdf`（相机话题 `/hero/image`、`/view_top/image`、`/view_under/image`）。
- 为什么翻：用户判断 —— 2027 赛季 3V3 场地**没有斜坡**，且官方图 3-4/3-5 显示场地含围挡、两块高台、隧道、工作台。
  因此本文档 §9/§10/§11 的"0.503 m 平整台面"判定**作废**（保留作历史）。
- **已修复（2026-11-19 晚）**：把 `structures.stl`（113,080 面三角网格）当碰撞体时，机器人被垫高到 z=0.0158、
  **轮子空转、车不走**（2 s 内 x 只动 1.4e-5 m）；改成 48 块 box 拼块后恢复（z=0.0100，2 s 前进 0.276 m），
  撞墙/撞台停车位置误差 0.1 mm。做法与验收见 §13。
- §11 的 2D 导航地图（`maps/rmul2027_3v3_fenced.pgm`）与 Nav2 验收数据基于翻转前模型，**对当前场地已失效**，
  要在新场地上跑 Nav2 需重建地图、重定起点、重跑验收。

---

## 13. 2026-11-19 晚：box 拼块碰撞（修掉"轮子转车不走"）

**结论**：三角网格做碰撞会让 dartsim 把机器人垫高 6 mm、轮子失去摩擦；
用**高度图生成的 48 块 box** 替代后，静置/直行/撞墙/撞台四项读数全部回到正常值，误差 0.1 mm。

### 13.1 做法（`debug/make_box_collisions.py`）

1. 把 `meshes/structures.stl` 光栅化成 **0.05 m 高度图**：近水平面按平面求 z、只填格心落在三角形内的格子；
   近竖直面用包围盒覆盖取最大 z（保守，薄墙不会漏）。
2. 高度直方图聚类成 3 层：**0.20 / 0.27 / 0.40 m**（实测就是平台顶、台沿、墙顶三档）。
3. 每层做"行内游程 + 同游程纵向合并"压矩形，每个矩形是一个 `<collision>` box，
   从 z=−0.10 顶到该层高度；地面仍是原来的 12.6×8.6 box（顶面 z=0）。
4. 写回 `models/rmul2027_3v3_field_up/model.sdf` 的
   `BOX_COLLISION_BEGIN/END` 之间（**勿手改**，改了就重跑脚本），并输出
   `debug/box_collisions.json`（box 列表 + 墙内侧面 + 候选平台，验收脚本从这里读几何，不写死数字）。

```bash
cd /mnt/e/RM/tb3_gazebo_nav2/rm_field
python3 debug/make_flipped_meshes.py      # 克隆后先重建网格（.stl 不入库）
python3 debug/make_box_collisions.py      # 48 块 box，写回 model.sdf
python3 debug/make_box_collisions.py --dry-run --ascii   # 只看统计 + 粗俯视高度图
```

**覆盖度**：占据 31.90 m²（=平台+墙在 0.05 m 栅格上的投影），高度误差 **max 0.035 / rms 0.0014 m**
（最大误差出现在台沿的 1 格过渡上）。

### 13.2 验收（`bash verify_box_collisions.sh` → `results/report_box_collisions.json`）

| 项 | 判据 | 实测 |
|---|---|---|
| A 静置 (0,0) | z = 0.0100 ± 0.003，漂移 < 0.01 m | **z=0.0100**，漂移 0.0001（网格碰撞时是 0.0158）|
| B 直行 0.15 m/s × 2 s | 位移 ≥ 0.24 m（旧 bug 只有 1.4e-5 m）| **0.2762 m**，z 偏移 0 |
| C 撞东墙 x=+6.00 | 停 5.9315 ± 0.02，不越界，z 恒定 | **5.9316**（误差 0.1 mm），z=[0.0100,0.0100]，贴住 2 s 漂移 0 |
| D 撞北墙 y=+4.00 | 停 3.9315 ± 0.02 | **3.9316**（误差 0.1 mm）|
| E 撞绿平台 0.20 m | 停 −1.0685 ± 0.02；**不许爬上平台**（z 仍 0.010）| **−1.0684**，z=[0.0100,0.0100]，未爬台 |

墙/台沿坐标不是手写死的：从 `debug/box_collisions.json` 读（东/北墙内侧面 6.00/4.00、
平台南侧面 −1.00），重新生成碰撞后验收自动跟着更新。停车位置 = 墙面 − 0.0685 m
（TB3 waffle 碰撞盒 0.265 m 见方、盒心后移 0.064 m → 车头外缘前伸 0.0685 m）。

### 13.3 剩余

- ~~2D Nav2 地图仍是翻转前的，**已失效**~~ → **2026-11-20 已重做**：见 §15
  （自由区 = 墙面内侧面以内 − 平台/墙投影 = 77.49 m²，三目标闭环验收全过）。
- 碰撞是 5 cm 栅格拼块：台沿/墙角有 ≤5 cm 的量化误差，若以后要贴边行驶再提高分辨率（`--cell 0.025`）。

---

## 14. 键盘遥控：手动开小车（2026-11-19）

**一条命令**（起 GUI 世界 + 在 (0,0) 放一台 waffle + 把终端交给键盘遥控）：

```bash
cd /mnt/e/RM/tb3_gazebo_nav2/rm_field
bash drive_teleop.sh
```

- Gazebo 窗口在 WSLg 里（Alt+Tab 找 **Gazebo Sim**）；**键盘焦点留在终端窗口**，Gazebo 只用来"看"，
  不要去点它（点了焦点就跑过去了）。
- Ctrl+C 退出（退出前一定发零）；Gazebo 世界不会跟着关，要关：`pkill -f "gz sim"`。

| 键 | 作用 |
|---|---|
| `w` / `↑` | 前进 |
| `s` / `↓` | 后退 |
| `a` / `←` | 原地左转 |
| `d` / `→` | 原地右转 |
| `q` / `e` / `z` / `c` | 前进左/右转、后退左/右转 |
| 空格 | 立即停车 |
| `+` / `-` | 前进速度档 0.10 → 0.50 m/s |
| `[` / `]` | 转向速度档 0.3 → 1.5 rad/s |

**为什么松开键能自动停**（终端收不到 keyup）：每个按键给指令 **0.5 s TTL**，按住时系统的按键重复
（约 30 次/秒）不断续期，松手 0.5 s 后自动发零。所以"按一下"是点动，"按住"是持续走。

**自检**（世界 + waffle 在跑时）：

```bash
python3 keyboard_teleop.py --selftest
```

实测（2026-11-19，headless 世界）：按住 w 1.5 s 位移 **+0.361 m**；按住 a 1.5 s 航向 **+1.157 rad**；
无指令 1 s 漂移 **0.0000 m**；方向键 ↑ 同样有效（1.2 s 走 0.285 m）。
另在**真实 pty 终端**里敲键验证：`w` → x 0 → **0.109 m**；`d` → yaw **−0.369 rad**；松手 2 s 位姿完全不变。

**限制 / 已知**：

- 终端必须是 tty（Windows Terminal 里跑 WSL 就是）。管道输入走"一行一键"的降级路径，只适合脚本。
- 速度用一阶平滑（α=0.45 @20 Hz）滤掉按键抖，起步/停手各有约 0.1 s 过渡。
- 还没有"按角度转 90°"那类精细指令；需要时再加。

---

## 15. 2026-11-20：翻转后场地的 Nav2 地图 + 三目标闭环验收（补 §13.3 的欠账）

**一句话**：自由区从"整块矩形"改成"墙面内侧面以内 − 48 块碰撞 box 的投影"= **77.49 m²**。
地图由**碰撞体几何**生成，正俯视渲染做独立校核（11 项全过），Nav2 三个目标全部 SUCCEEDED。

### 15.1 为什么重做

翻转前地图是 `maps/rmul2027_3v3_fenced.*`：自由区 = 围挡内侧面以内的整块 12.5×8.5 矩形。
翻转后地面上多了 0.19~0.40 m 的凸起结构（平台/筋/外墙），自由区变成"矩形 − 凸起投影"；
继续用旧图会**把车规划到平台上去**。旧图/旧世界（围挡变体）留在仓库里只作历史。

### 15.2 生成地图（`capture_nav_map_up.sh` + `make_nav_map_up.py`）

```bash
cd /mnt/e/RM/tb3_gazebo_nav2/rm_field
bash capture_nav_map_up.sh          # 起取证世界 → 抓 1600x1100 正俯视 → make_nav_map_up.py（约 1 min）
```

- **权威几何 = 碰撞体**：`debug/box_collisions.json` 的 48 块 box（5 cm 栅格，与 `model.sdf` 的
  `BOX_COLLISION_BEGIN/END` 同源）。机器人实际就是被这 48 块 box 挡住的，地图必须和物理一致；
  5 cm 量化误差交给 Nav2 inflation（robot_radius 0.22 / inflation_radius 0.55）吸收。
- **独立校核 = 平光正俯视渲染**（`worlds/nav_map_capture_up.sdf`，1 px = 0.01 m，关阴影避免误判颜色）。

| 校核项 | 判据 | 实测 |
|---|---|---|
| 自由区面积 | = 96.0 − 18.510（解析值）±0.10 | **77.490**（偏差 0.000）|
| 自由区 bbox | 12.00 × 8.00 m（= 墙面内侧面 ±6.00/±4.00）| **12.00 × 8.00** |
| 连通性 | 从 (0,0) 泛洪覆盖全部自由格，且能到每个目标 | 未连通格 **0**，3 目标全可达 |
| 渲染地面 bbox | 12.00 × 8.00 ±0.05 | **12.00 × 8.00**，四边偏差 ≤0.01 |
| 渲染地面面积 ↔ 地图 | 同区域 ±0.60（平台顶面 2% 透视放大）| 74.348 vs 74.160（差 0.188）|
| 渲染绿色 ↔ 地图障碍 | 面积 ±0.60、重心 ≤0.10 m | 17.884 vs 17.880（差 0.004）、重心差 **0.022 m** |
| 逐像素一致率（离分界 ≥0.10 m）| ≥99.5% | free **99.999%**、障碍 **99.803%** |
| PGM 方向回读 | 8 个已知点的自由/障碍必须对 | 全对（见 §15.4）|

产物：`maps/rmul2027_3v3_up.pgm/.yaml`（512×352 @0.025 m/px，origin −6.40/−4.40）、
`results/report_nav_map_up.json`、`shots/nav_topdown_up.png`、`shots/nav_map_up_overlay.png`。

### 15.3 闭环验收（`nav2_field.sh` + `verify_nav2_field.py`，均已切到新世界/新图）

```bash
bash nav2_field.sh                  # 起栈（默认 headless；--gui 带 RViz）
python3 verify_nav2_field.py        # 三目标闭环（约 80 s；脚本内部默认 ROS_DOMAIN_ID=42，跨 shell 直接跑）
bash stop_nav2_field.sh
```

世界换成 `worlds/rmul2027_3v3_up_nav2.sdf.xacro`：翻转场地 + 场外底板 z=−0.10 + 传感器/IMU 系统。
**不再需要围挡变体**——翻转后场地自带的 0.40 m 外墙就在 lidar 扫描面（z≈0.17）之上。

| 判据 | 实测 |
|---|---|
| 生命周期 | 7 个节点全 **active**；/scan 5.0 Hz、/odom 29.3 Hz，RTF 0.999 |
| 三目标闭环 | (4,2) / (4,−2.5) / (0,0) 全 **SUCCEEDED**（error_code 0），25.9 / 15.5 / 27.8 s |
| 到位 | 结束差 **0.290 / 0.132 / 0.015 m**（判据 <0.30）|
| 不出界 | max&#124;x&#124; 4.433、max&#124;y&#124; 2.521，限 5.85 / 3.85 |
| 不落台/不爬台 | Gazebo z 全程 **0.0100**（窗口 −0.05~0.06）|
| 不压平台/墙 | TF 点离任何 box 投影最近 **0.51 m**（判据：不得侵入 >0.05）|
| 真在跑 | 19.622 m / 76.66 s，平均 **0.256 m/s**，cmd_vel 峰值 **0.495 ≤ vx_max 0.5** |
| 定位一致 | TF 与 Gazebo 真值偏差 均值 **0.144** / 最大 **0.443 m**（转弯段，见 §15.4）|

复跑一次（在**没有导出 `ROS_DOMAIN_ID`** 的新 shell 里，验证坑 3 的修法）：到位
0.173 / 0.088 / 0.094 m，19.762 m / 80.08 s、平均 0.247 m/s、cmd_vel 峰值 0.492，
z 仍恒 0.0100，**11 项仍全过**。

### 15.4 这轮踩的坑（两个都不是参数问题）

**坑 1：PGM 上下翻转——旧图一直有，只是矩形看不出来。**
第一次闭环全失败（goal 1 error_code 103，车卡在 (2.5,−1.6)，TF 报 (2.29,−1.43) 而 Gazebo 真值
(1.73,3.42)，**偏差 4.8 m**）。根因：`build_map()` 的第 0 行是 y 最小的一行，而 map_server
把**图像第 0 行当最上面一行**；`write_pgm()` 直接 `tobytes()` 就把整图上下翻了过来。
翻转前自由区是一整块矩形，翻转前后一模一样 → 潜伏了半个月。
修法：落盘写 `img[::-1]`，预览同步翻转；新增 `read_pgm()`/`occupancy_at()` 按 map_server 约定回读，
**`pgm_orientation` 判据**核对 8 个已知点（`(−2.5,−2.3)` 必须 100=障碍、`(−2.5,2.3)` 必须 0=自由）。

**坑 2：重启栈时旧 `gz sim` 不响应 SIGTERM。**
会出现两个仿真器同时发 `/clock /scan /tf`，症状是 `local_costmap` 一直
`Invalid frame ID "odom"`、话题频率 0、生命周期查询 `Node not found`。
修法：`nav2_field.sh` 清场改成 **TERM → 3 s → KILL −9**，并清 `/dev/shm/fastrtps*` 残留端口文件。

**坑 3：验收脚本跑在另一个 ROS domain 上（"等不到 action server"）。**
栈在 `ROS_DOMAIN_ID=42` 里（`nav2_field.sh` 自己 export），但脚本的 export 不会传给调用者的
shell。换个终端直接 `python3 verify_nav2_field.py` 时它会落到 **domain 0**，于是报
`!! 等不到导航 action server`，而栈其实一切正常（`ROS_DOMAIN_ID=42 ros2 action list` 能看到
`/navigate_to_pose`）。修法：脚本里 `os.environ.setdefault("ROS_DOMAIN_ID", "42")`（显式环境变量优先），
启动报告里也打印实际 domain；`nav2_field.sh` 收尾会直接给出带 domain 的验收命令，
并在退出前**等到 `/navigate_to_pose` 真的出现在 `ros2 action list`**（最多 60 s），
不再出现"脚本说 OK、验收找不到服务器"。

**定位误差怎么读**：电机里程计在转弯段有偏航误差，AMCL 按 `update_min_d=0.25 m` 周期修正，
TF 瞬时能差到 0.44 m；静止/直行段只有 **0.02~0.07 m**（`report_nav2_field.json` 的
`whole_run.loc_err_m.series_2s` 有每 2 s 的趋势）。这是 AMCL 的正常滞后，不是地图错位
（PGM 翻转那次是 4.8 m，量级完全不同），所以判据定在 0.50 m。

**坑 4：`ros2 lifecycle get` / `ros2 node list` 在这台机器的 WSL 上不可信（"Node not found"）。**
它们走 `ros2 daemon` 的**节点列表**，而 daemon 经常看不到 composable 容器里的节点（`ros2 node list`
为空），于是 7 个节点全报 `Node not found` —— 但同一时刻 `ros2 service call /amcl/get_state`
返回 `id=3`（ACTIVE）、`ros2 action list` 也能看到 `/navigate_to_pose`，栈其实是好的。
不要拿 `lifecycle get` 当"栈没起来"的证据。修法：`nav2_field.sh` 第 5 段改成直接调
`/<node>/get_state` 服务并把 id 翻译成状态（1 unconfigured / 2 inactive / **3 ACTIVE** / 4 finalized）；
临时排查可以 `ros2 node list --no-daemon`。

**坑 5：`stop_nav2_field.sh` 里曾有 `pkill -f 'nav2_'`，会把自己杀掉。**
调用者的命令行里只要含 `nav2_field.sh`（例如串联命令 `bash stop_nav2_field.sh && bash nav2_field.sh`），
这条过宽的 pattern 就命中调用者自己，表现为脚本跑一半静默退出。已删掉这条（其余 pattern 足够，
nav2 节点都在 `component_container` 里）。

### 15.5 文件与入口

| 文件 | 作用 |
|---|---|
| `worlds/rmul2027_3v3_up_nav2.sdf.xacro` | Nav2 用世界（翻转场地 + 传感器/IMU）|
| `worlds/nav_map_capture_up.sdf` | 出图用的正俯视取证世界（平光无阴影）|
| `make_nav_map_up.py` | 几何生成地图 + 渲染校核 + 连通性 + PGM 方向回读（11 项断言）|
| `capture_nav_map_up.sh` | 一条命令：渲染 → 生成 → 校核 |
| `nav2_field.sh` / `verify_nav2_field.py` | 起栈 / 三目标闭环验收（均指向新世界与新图）|

### 15.6 剩余

- 围挡高度（现为场地自带 0.40 m 外墙）与赛规的一致性仍未核对（历史遗留项）。
- 绕障/动态避障未测 → **2026-11-20 晚已完成，见 §16**（静态 2 箱 + 触发式动态箱，15 项判据全过）。
- 碰撞/地图都是 5 cm 栅格：贴边行驶或窄通道场景再加细（`--cell 0.025` 重新生成碰撞与地图）。

---

## 16. 2026-11-20 晚：绕障 + 动态避障验收（15 项判据全过）

补 §15.6 的第二条欠账：在翻转后的 3V3 场地上摆未建图障碍物，验证 Nav2 会不会**绕开挡路的东西**、
会不会**对会动的东西让开**，全程不碰、不压台、不出界、三目标照样 SUCCEEDED。

### 16.1 为什么先采"无障基线"

`capture_route.py` 在**同一场地、同一目标**上先跑一遍无障导航，把 TF 轨迹存成
`results/baseline_route.json`（3 目标 19.442 m / 66.8 s）。基线有两个用处：
①挑障碍位置——箱子必须摆在基线真的会走的窄道上，否则"绕过去了"什么都证明不了；
②当对照——验收里用"基线轨迹侵入箱体投影的深度"证明箱子确实挡路（`avoid_box_1` **0.209 m**、
`avoid_box_2` **0.226 m**、动态箱起点 **0.243 m**）。

### 16.2 障碍物怎么定义（唯一源 → 世界 + 验收同一份）

- `config/avoid_obstacles.json` 是几何**唯一源**；`make_obstacles_world.py` 从它生成
  `worlds/rmul2027_3v3_up_nav2_obstacles.sdf.xacro`（插在 physics 插件前的 BEGIN/END 标记里，可重复生成），
  `verify_nav2_obstacles.py` 直接读同一份 JSON 算投影——避免"视觉几何 vs 碰撞几何"那类错位。
- 静态箱 2 块（0.5×0.5×0.5 m）：`(0.85,-1.45)` 的中线走道、`(4.30,-0.60)` 的东侧通道。
- 动态箱 1 块（0.5×0.5×0.6 m）：`<kinematic>true</kinematic>`，由 `dynamic_obstacle.py` 用
  `/world/rmul2027_3v3/set_pose`（gz.msgs.Pose，10 Hz）平移。**为什么不用连续往返横穿**：第一版
  这么做过——盲动的箱子反复切进机器人走道，nav2 默认 MPPI 反复报 "Failed to make progress"、
  recovery 死循环（第三段跑了 154 s），而且箱子不认机器人、会直接压上去（实测最近 0.077 m）。
  现在改成 **triggered_retreat**：箱子先停在基线走道正中挡路，机器人进到 1.5 m 内后，
  箱子以 0.3 m/s 沿 `start(2.50,-1.94) -> end(2.50,-1.35)` **让开并停住**（向北朝平台方向，
  机器人实际走道在它南侧，让开方向永远背离机器人）。动作由 gz pose 真值触发，不依赖本车话题。
- 静态/kinematic 两种模型都用 set_pose 探针实测过：**pose/info 都会真实变化**；选 kinematic 是因为
  物理里会逐步跟随 pose（机器人推不飞它，撞上去就是撞墙），语义更接近"敌车"。

### 16.3 怎么跑

```bash
cd /mnt/e/RM/tb3_gazebo_nav2/rm_field
bash nav2_obstacles.sh          # 重新生成世界+同源校核+xacro 自检 → 起栈 → 拉起动态驱动
python3 verify_nav2_obstacles.py   # 15 项判据 → results/report_nav2_obstacles.json
bash stop_nav2_field.sh
# 只调动态布局、不想重起整栈时：
bash start_dynamic.sh           # 重读 config，把箱子摆回 start 等触发
```

`nav2_obstacles.sh --static-only` 只测静态绕障；`--gui` 带 Gazebo/RViz。
`nav2_field.sh` 本身新增两个环境入口：`NAV2_WORLD=`（换世界）、`RM_DYNAMIC=1`（拉动态驱动），
默认行为不变。

### 16.4 验收结果（`results/report_nav2_obstacles.json`，2026-11-20 19:35 最后一跑）

**15 项判据全过**（`summary.ok=true`）：

| 类别 | 判据 | 实测 |
|---|---|---|
| 链路 | sim_time / TF / scan 5.0 Hz / odom 29.3 Hz | ✓ |
| 导航 | 三目标全 SUCCEEDED、error_code 0 | (4,2) 33.3 s / 12.646 m，(4,-2.5) 17.4 s / 5.557 m，(0,0) 28.7 s / 6.807 m |
| 到位 | 结束距目标 < 0.30 m | **0.121 / 0.088 / 0.025 m** |
| 不出界/不掉台/不爬台 | max&#124;x&#124;≤5.85、max&#124;y&#124;≤3.85、z∈(-0.05,0.06) | 5.182 / 3.212、z 全程 **0.0100** |
| 不压平台墙 | TF 不侵入 48 块场地碰撞 box 投影 >5 cm | 最大 **−0.562 m**（离投影还有半米）|
| 速度 | 峰值 ≤ vx_max(0.5)×1.05、平均 >0.05 | **0.499** / 平均 **0.288 m/s** |
| 定位一致 | 剔除未建图障碍物 ≤2 m 样本后 max < 0.50 m | **0.256 m**（全场最大 0.764，见 16.5②）|
| 箱子挡路（有效性）| 无障基线侵入静态箱投影 ≥0.10 m、动态箱起点 ≥0.10 m、退让终点离基线 ≥0.30 m | 0.209 / 0.226 / 0.243、终点 **0.319 m** |
| 静态绕行 | 全程真值到每块箱 AABB 间距 ≥ robot_radius 0.22 | **0.701 / 0.640 m** |
| 动态在动 | 1 s 窗口测速 = 0.30±0.05、行程 = \|end-start\|±20%、停在终点、停后 1 s 位移 ≤0.05 | **0.293 m/s**、0.590/0.590 m、停位误差 **0**、停后 **0** |
| 动态无接触且真遭遇 | 最小间距 ≥0.22、最近中心距 <2.0 | **0.892 m** / **1.144 m**（本车在动态箱触发段 cmd_vel 曾到 **0**，是真被挡了一下）|

绕行代价（相对无障基线）：第一段 **+3.503 m**、第二段 +0.714 m、第三段 +1.351 m，
合计 +5.57 m / 25.051 m —— 多出来的路就是"绕开"的量化代价。

### 16.5 踩的坑（这轮全是"判据/机制"坑，不是调参）

① **盲动的动态障碍物会把 nav2 逼进 livelock**：连续往返横穿时 MPPI 反复 failed to make progress，
vehicle 在走廊里来回抖 154 s；而且盲动的箱子会主动压上停着的机器人（实测 0.077~0 m 级间距）。
动障验收要么用"让开"式（本仓库 current），要么就得给 nav2 换动态感知的控制器——这是结论，不是参数问题。

② **未建图障碍物会把 AMCL 带偏约 0.8~1.05 m**：三次复跑都出现，且都集中在 12~32 s
（贴着 avoid_box_1/动态箱走的那段），随后 30 s 内收敛回 0.05~0.2 m；机器人在同一段照样到达目标。
所以"地图-世界一致性"判据改成只用在离未建图障碍物 >2 m 的样本上（报告里同时给 all 与
excluding_unmapped 两份数，不藏）。这条对实车有直接含义：场地里临时摆箱/敌车会让 AMCL 短时漂。

③ **`pkill -f 'dynamic_obstacle.py'` 会杀掉调用者自己**（命令行里含同一串），表现为脚本静默 exit 1。
用 `pkill -f 'dynamic_obstacle[.]py'` 或 `start_dynamic.sh`（脚本内已用括号写法）。

④ set_pose 的 0.1 s 步进会让"瞬时速度"失真：测速要用 ≥1 s 窗口位移/时间（窗口内中位速度），
不要用相邻帧差分。

### 16.6 文件与入口

| 文件 | 作用 |
|---|---|
| `config/avoid_obstacles.json` | 障碍物布局唯一源（静态箱 + 动态箱 start/end/speed/trigger_dist）|
| `make_obstacles_world.py` | JSON → 带障碍物世界（同源回读校核；`--check` 只校核）|
| `worlds/rmul2027_3v3_up_nav2_obstacles.sdf.xacro` | 生成物（勿手改，BEGIN/END 标记内）|
| `nav2_obstacles.sh` | 生成世界 + xacro 自检 → `nav2_field.sh`（`NAV2_WORLD`/`RM_DYNAMIC`）|
| `dynamic_obstacle.py` | 动态箱驱动：triggered_retreat / patrol 两种模式，set_pose @10 Hz |
| `start_dynamic.sh` | 只重启动态驱动（改布局后不用重起整栈）|
| `capture_route.py` | 采无障基线轨迹 `results/baseline_route.json` |
| `verify_nav2_obstacles.py` | 绕障验收 15 项判据 → `results/report_nav2_obstacles.json` |

### 16.7 剩余

- 碰撞/地图仍是 5 cm 栅格：贴边、窄通道场景要 `--cell 0.025` 重生成碰撞与地图（两者同源）→ 未执行，见 §17.6。
- 围挡高度（0.40 m 场墙）与赛规一致性：**已收口**，见 §17.5（规则图是位图无法读；以官方 CAD 为准）。
- `collision_monitor` 本轮**一次都没触发**（DWB 提前绕开了）→ **已补测**：§17 用"突然出现的 2.8 m 宽挡板"
  实测它在 pop 后 0.24 s 介入、0.79 s 刹到 0.03 m/s、最小间距 0.335 m。
- 动态箱只有 0.3 m/s 一档、单向单程：多速度档、多方向、两车相遇场景未测（见 §17.6）。

---

## 17. 2026-11-20 晚补：安全层（collision_monitor）刹车验收

### 17.1 为什么单独做

§16 的绕障验收里 `collision_monitor` **0 次触发**——DWB 提前绕开了，安全层根本没轮上。
要测它必须构造"绕不过去、又突然出现"的场景，所以另开一个世界和验收脚本。

### 17.2 场景与入口

- 世界 `worlds/rmul2027_3v3_up_nav2_braking.sdf.xacro`（仍由 `config/avoid_obstacles.json` 的
  `popup` 段生成，同源）：一块 **2.8×0.5×0.6 m** 宽挡板，先藏在 `(3,-3)`；
  机器人跑中线走道 `(0,0) → (0,3.0)`，离出现点 `(0,2.2)` 只剩 **0.85 m** 时挡板瞬移到正前方。
  中线走道宽 3.2 m，挡板 2.8 m 只留 0.2 m 缝——机器人**绕不过去**，只能刹。
- 入口：`bash nav2_braking.sh`（生成世界 + xacro 自检 + 起栈 + 挡板驱动），
  `python3 verify_braking.py` → `results/report_collision_monitor.json`。

### 17.3 结果（6 项判据全过）

| 判据 | 实测 |
|---|---|
| 挡板真的瞬移出现 | 出现前在 hidden ±0.05、出现后到 appear ±0.05 ✓ |
| 出现前车真在跑 | pop 前 0.8 s 平均 **0.452 m/s**（下限 0.20）|
| collision_monitor 真的介入 | pop 后 **2 次** state 事件（action_type 0 = STOP、3），首次在 pop 后 **0.24 s** |
| 刹住 | 首停 **0.791 s**（窗口 2.5 s），窗口内最低速 **0.028 m/s** |
| 不接触 | pop 后人-挡板 AABB 最小间距 **0.335 m**（判据 ≥ robot_radius 0.22）|
| 不掉台/不爬台/不出界 | z 全程 0.010；max&#124;x&#124; 2.195、max&#124;y&#124; 2.274（限 5.85/3.85）|

目标点必然到不了（挡板拦死），脚本在机器人停住后主动取消目标；`--` 这不是失败。

### 17.4 两个口径（为什么这么定）

- **"刹住" = 2.5 s 内出现速度 <0.05 m/s**，不要求"停住不动"：实测 collision_monitor 先 STOP，
  随后恢复行为会**倒车让开**（更安全）；把"之后又动了"算失败会误导。
- **挡板宽 2.8 m**：中线走道 3.2 m，留 0.2 m 缝，机器人物理上过不去 → 排除"绕过去"的假通过。
  第一版用 0.7 m 触发距离时，停车间距只有 0.215 m（差 5 mm 没过 0.22 判据），
  把触发距离放到 0.85 m 后间距 0.335 m——**0.85 m 这档才是"刹得住"的实测点**。

### 17.5 围挡/赛规核对（历史遗留项收口）

- 规则手册的场地尺寸在 **图 3-5 / 3-6 / 3-7（位图）**，PDF 文本层只有图题；正文写的是
  "场地道具角度误差 ±5 mm、实际制作可能对细节尺寸微调、**以实际场地为准**"。
- 本机没有 OCR/视觉引擎（modlens 报 `No vision provider`；WSL 里没有 tesseract/pdftoppm，
  只有 ghostscript 能渲染），所以**无法逐项对图**——这是环境限制，不是没查。
- 场地几何的权威来源继续用**官方 CAD** `RMUL2027_3V3_V1.0.0.stp`：12.60×8.60 m、
  内侧面 ±6.00/±4.00、墙顶 **0.40 m**、墙厚 0.05 m（从 `debug/box_collisions.json`
  栅格化数据回读），与"以实际场地为准"的口径一致。这条从"未核对"改成"有据可查 + 明确限制"。

### 17.6 仍未执行

- **2.5 cm 栅格 / 贴边 / 窄通道**：0.80 m 门洞已用现有 5 cm 栅格实测通过（见 §18）：居中过门、
  两侧中心距 0.38 m，**不必**重生成。只有 <0.70 m 门洞或要求厘米级贴边时才需要 2.5 cm 变体
  （新模型名 + 新地图名，不能替换现有基线）。
- **动态箱速度/方向档位**：目前只有 0.3 m/s 一档、单程一次。

---

## 18. 2026-11-20 夜补：窄通道（贴边）验收——0.80 m 门洞 5 项判据全过

### 18.1 场景

- 世界 `worlds/rmul2027_3v3_up_nav2_gap.sdf.xacro`（同一 JSON 的 `gate` 段生成）：
  两块 0.6×0.5×0.6 m 静态箱放在 `(∓0.70, 0.50)`，内侧面 ±0.40 → **门洞 0.80 m**；
  门两侧到走廊内侧面只剩 **0.40 m < 车宽 0.44 m** → 机器人**绕不过去**，只能从门洞过。
- 出生点 `(0,-2.0)`（`nav2_field.sh` 新增 `RM_SPAWN_X/Y` 覆盖），目标 `(0,2.6)`：直线穿门。
- 入口：`bash nav2_gap.sh` + `python3 verify_narrow_gap.py` → `results/report_narrow_gap.json`。

### 18.2 结果（5 项判据全过）

| 判据 | 实测 |
|---|---|
| 导航 | SUCCEEDED、error_code 0、路径 5.798 m、到位 **0.124 m** |
| 真的穿门 | 门线 y=0.5±0.1 的 26 个真值样本里 |x| 最大 **0.02 m**（门洞半宽 0.40）|
| 贴边安全 | 到两块门箱最小间距 **0.379 / 0.417 m**（判据 ≥0.20；车体半宽约 0.19）|
| 不出界/不掉台 | z 全程 0.010；max&#124;x&#124; 0.565、max&#124;y&#124; 2.687 |

### 18.3 结论：5 cm 栅格对 0.80 m 门洞**够用**

- 门洞名义 0.80 m，5 cm 碰撞栅格的量化误差 ±0.025 m → 有效门洞 0.75~0.85 m；
  实测机器人以 **|x| ≤ 0.02 m** 居中通过、两侧各留约 **0.38 m** 中心距。
- 所以"5 cm 栅格导致窄通道不可用"这一担心，在 **0.80 m 门洞**这个量级上不成立；
  真正需要 2.5 cm 的场景是更窄的门洞（<0.70 m）或要求厘米级贴边，而不是常规 3V3 走道。
- 这也回答了 §16.7/§17.6 里"贴边/窄通道要重生成 2.5 cm 碰撞与地图"的欠账：
  **在 0.80 m 量级不必重生成**；更窄场景再单独开变体（新模型名 + 新地图名，不动现有基线）。

### 18.4 入口一览（三个专项验收）

| 场景 | 起栈 | 验收 | 报告 |
|---|---|---|---|
| 绕障 + 动态避障 | `bash nav2_obstacles.sh` | `python3 verify_nav2_obstacles.py` | `report_nav2_obstacles.json` |
| 安全层刹车（突然出现的宽挡板）| `bash nav2_braking.sh` | `python3 verify_braking.py` | `report_collision_monitor.json` |
| 窄通道（0.80 m 门洞）| `bash nav2_gap.sh` | `python3 verify_narrow_gap.py` | `report_narrow_gap.json` |

三个场景共用 `config/avoid_obstacles.json` 作几何唯一源，世界都由 `make_obstacles_world.py`
按 `--only obstacles|braking|gap` 生成并同源校核。

---

## 19. 2026-11-20 夜：在地图上点坐标 → 机器人走过去（交互模式）

### 19.1 怎么用（用户操作）

```bash
cd /mnt/e/RM/tb3_gazebo_nav2/rm_field
bash nav_gui.sh          # 起 Gazebo GUI + RViz + /goal_pose 桥；窗口出现在 Windows 桌面（WSLg）
```

1. 等 **Gazebo Sim** 与 **RViz** 两个窗口起来（本机是 llvmpipe 软渲染，RTF≈0.62，慢一点正常）。
2. 在 RViz 里点地图：
   - **Nav2 Goal**（左侧面板，推荐）：直接发 Nav2 action；
   - **2D Goal Pose**（工具栏）：发 `/goal_pose`，经桥转成 action（可脚本化复现）。
   点一下、按住拖出朝向再松手。
3. 机器人自动规划并走过去；**再点别处 = 换目标**（桥会取消上一个目标，不会排队）。
4. 定位偏了：工具栏 **2D Pose Estimate** 重新给初始位姿（发 `/initialpose`）。
5. 停止：`bash stop_nav2_field.sh`。

命令行等价：`bash goto_xy.sh X Y [YAW]`（直接发 NavigateToPose，不依赖桥）。
纯脚本验收：`bash nav2_field.sh && bash start_goal_bridge.sh && python3 verify_point_goal.py`。

### 19.2 实现（三个文件 + 一处参数）

| 文件 | 作用 |
|---|---|
| `goal_pose_bridge.py` | 订阅 `/goal_pose`（DViz 2D Goal Pose 发的就是这条）→ 发 `NavigateToPose`；新点击先取消旧目标；1 s 内同点重复点击去抖 |
| `config/rm_field_click.rviz` | 由 `make_rviz_config.py` 从 nav2 官方配置生成：**2D Goal Pose** + **Nav2 Goal** + **2D Pose Estimate** 三个工具都在，Fixed Frame=map |
| `nav_gui.sh` | 生成 RViz 配置 → `nav2_field.sh --gui` → `start_goal_bridge.sh` → 打印操作说明 |
| `start_goal_bridge.sh` / `goto_xy.sh` | 单独重启桥 / 命令行点坐标 |

`nav2_field.sh` 新增 `RM_RVIZ_CONFIG`（透传给 launch 的 `rviz_config_file`），GUI 模式才用得到。

### 19.3 验收结果（`results/report_point_goal.json`，9 项判据全过）

链路：`/goal_pose` → 桥 → `/navigate_to_pose`（与 RViz 2D Goal Pose 点击完全同一条消息）。

| 判据 | 实测 |
|---|---|
| 三次点击都到点 | (2.5,−2.0) **0.164 m**、(4.5,1.5) **0.082 m**、(0,0) **0.129 m**（判据 <0.30）|
| 点击真的驱动了机器人 | 每次点击后位移 2.979 / 3.990 / 4.663 m |
| 换目标取最新 | 先点 D(2.5,−2.0)、3 s 后点 E(1.0,2.0)：最终停在 E（**0.110 m**），离 D **4.282 m** |
| 不出界/不掉台/不压平台 | max&#124;x&#124; 4.763、max&#124;y&#124; 1.992；z 全程 0.010；场地碰撞投影侵入 −0.466 m |

**GUI 模式实测**（同一套栈，headless=False）：RViz 以 `config/rm_field_click.rviz` 启动
（窗口标题可核对）、Gazebo Sim 窗口在、桥单实例；往 `/goal_pose` 发 (4.0,−2.0) →
桥日志 `GOAL → ACCEPTED → SUCCEEDED`、TF 到位偏差 **0.17 m**；GUI 下 RTF **0.62**。

### 19.4 踩的坑

- **一次点击被发成多个 action**：脚本里为防丢消息连发 3 次 → 桥当成 3 次点击、互相取消，
  结果全 ABORTED。现在：桥对"1 s 内同点重复"去抖，脚本只发一次。
- **旧桥实例残留**：改代码前起的桥不随栈清场（当时 `stop_nav2_field.sh` 还没杀它），
  两个桥同时订阅 `/goal_pose`，点击会落到旧的那个（日志里看不到、机器人却动了）。
  现在 `stop_nav2_field.sh` / `nav2_field.sh` 清场都包含 `goal_pose_bridge[.]py`，
  `start_goal_bridge.sh` 也先杀旧实例并回读实例数（12×0.5 s 等 READY）。
- GUI 模式 RTF 0.62（llvmpipe 软渲染 + Gazebo GUI + RViz），到点判定与 headless 一致，
  但交互会慢半拍；验收/回归仍用 headless。

### 19.5 2026-11-20 深夜补：为什么"点了没反应"，以及新增【地图点选】窗口

**用户实测**：在 RViz 地图上点了没反应；在 Gazebo Sim 里右键 `Move To` 也没反应。两条都不是 bug：

- **RViz**：启动后"当前工具"不是 **Nav2 Goal**。实测（合成鼠标点击）默认工具下点地图→只动视角，
  不产生任何目标；即便把 GoalTool 排到 Tools 列表第一（配置里已排）也不改变"当前工具"。
  正确用法：**先点工具栏里 tooltip 为 "Nav2 Goal" 的图标**，再到地图上点一下、按住拖出朝向松手。
  同理 `2D Goal Pose` 也要先选它。
- **Gazebo Sim 的右键 `Move To`**：那是 Gazebo 自己的模型编辑动作（把模型位姿改一下），
  **不是导航命令**，不会通知 Nav2；而且真把机器人模型挪了会让 Gazebo 真值与 odom/AMCL 失配。
  别用；点坐标请用 RViz 或下面的地图点选窗口。

**新增【地图点选】窗口**（`map_click.py`，`nav_gui.sh` 会自动起；单独起：`bash start_map_click.sh`）：

| 操作 | 效果 |
|---|---|
| **左键点地图** | 发导航目标（直发 NavigateToPose；再点=换目标）|
| **右键点地图** | 发 `/initialpose`（定位偏了用它重定位）|
| **中键** | 取消当前目标 |

窗口内容：2D 地图（灰度分障碍/自由/未知）、1 m 网格、绿色箭头=机器人当前位姿（TF）、
蓝叉=最近目标、灰线=当前全局路径（/plan）、底部状态栏显示 ROS 状态与坐标。

**实测**（真实鼠标事件，xdotool 在 WSLg 里点窗口）：
`CLICK go (2.714,-2.443)` → nav2 日志 `Received a goal` → 机器人到点，TF 与真值差 0.05 m、
与点击点距离 **0.11 m**。中途出现过一次 MPPI `Failed to make progress` + spin 恢复，属既有偶发行为
（headless 验收里也见过），恢复后照常到点。

文件：`map_click.py`（Tkinter + rclpy，地图渲染按 yaml 的 resolution/origin）、`start_map_click.sh`；
`nav_gui.sh` 第 3 步会拉起它并打印用法。

### 19.6 2026-11-20 深夜再补："绕了一大圈"的实测结论 + 落点吸附

**用户反馈**：可以让机器人"从洞过去"，但它绕了一大圈。逐项查了：

1. **那两个点分属两个被墙隔开的"口袋"**（`debug/box_collisions.json`）：
   `box_027` 是一堵 **0.40×1.05 m、高 0.27 m 的墙**（x[2.30,2.70]、y[2.95,4.00]），
   把平台北侧那条 1 m 宽的通道从中间堵死；两侧还各有一小段 0.2 m 高的小台阶
   （box_016/017/018/019 与 box_020/021）。地图上**没有**能过的缺口。
2. **离线算最短可行驶路线**（净空 ≥0.24 m）：
   西侧口袋 (1.75,3.58) → 东侧口袋 (3.1,3.50) 最短 **12.52 m**（绕 B 一圈）；
   从出生点 (0,0) 去东侧口袋约 10.7 m。nav2 实际规划/执行的就是这条路
   （实测 `/plan` 一段 530 点、6.65 m，沿 x≈4.2 的东侧通道北上再左转）。
   → **"绕一大圈"是最短路线的形状，不是规划器绕远**。
3. **真正的毛病在落点**：用户点的 (1.743,3.643) 净空 **0.20 m**、(2.986,3.671) 净空 **0.18 m**，
   都小于车体半径 0.22 m → 目标点在致命区，nav2 到不了、会反复重规划/进恢复行为（看起来像转圈）；
   另一处 (2.300,3.500) 直接点在墙里（pgm=0）。
   **已修**：`map_click.py` 点击后自动吸附到最近"净空 ≥0.26 m"的格子（搜索半径 1.5 m），
   点在障碍里/附近没位置会拒绝并提示。实测：点 (2.971,3.671)（净空 0.18）→
   发目标 (2.975,3.575)（自动南移 0.10 m），nav2 正常规划、`Goal succeeded`。
4. **顺带发现：该口袋附近 AMCL 漂了 0.66 m**——nav2 报 `Reached the goal` 时，
   TF 是真值 (3.317,3.51)，而 map→base_footprint 是 (2.906,2.99)。这不是点击问题，
   是定位在该区域的一次偏差；处理办法：右键（地图点选窗口）或 RViz 2D Pose Estimate 重定位，
   或下一步单独查这块区域的 AMCL 参数/地图特征。

**新增工具**：
- `map_click.py`：Shift+左键 = 只报坐标+净空（不导航），用来指认结构（实测
  `PROBE (2.457,3.100) clearance=0.00` → 那里就是墙）。
- `analyze_gap_route.py`：离线算"忽略 inflation / 限定净空"的最短路，判断"洞能不能过"。
- `path_len_once.py`：采样一次 `/plan`，打印当前全局路径长度与途经点。
- `list_boxes.py`：按矩形范围列场地方块（核对几何用）。
- `kill_map_click.sh`：安全地清掉所有地图点选窗口实例（避免 pkill 命中调用者自身）。
- 另：`start_map_click.sh`/`start_goal_bridge.sh` 的 pkill/pgrep 模式改成
  `python3 map_click[.]py` / `python3 goal_pose_bridge[.]py`——之前的短模式会匹配到
  `wsl -lc '... python3 -m py_compile map_click.py ...'` 这类调用者命令行，导致"命令静默退出 1"。

**另外**：期间整套栈挂过一次（nav2 container 在退出时 `Magick: abort (SIGSEGV)`，
Gazebo/RViz 窗口被关后 launch 随之退出），所以有一阵点击完全没反应——已重启；
现在 RViz/Gazebo 窗口**不要随手关**，关了整套导航就没了。

### 19.7 2026-11-20 深夜三补："地图坐标和实际坐标不一致"的定位结论

**用户问**：为什么地图坐标和实际坐标不一致？用的是 Nav2 还是自写算法？AMCL 是什么？

**用的框架**：**ROS 2 Nav2 标准框架**，导航部分没有自写算法——
定位 `nav2_amcl`（AMCL 粒子滤波）、全局规划 `nav2_navfn_planner`（NavFn/A* 型）、
局部控制 `nav2_mppi_controller`、代价地图 `nav2_costmap_2d`（static + obstacle + inflation）、
行为树 `nav2_bt_navigator` + `nav2_behaviors`（spin/backup）。
本项目自写的只有：仿真世界/场景生成（`make_*.py`）、起栈与验收脚本、
`map_click.py` 点选窗口、`goal_pose_bridge.py` 话题桥——都是"外壳"，不参与导航算法。

**AMCL**（Adaptive Monte Carlo Localization）：粒子滤波定位。用激光扫描 + 已知地图，
估计机器人在 `map` 系里的位姿（2000 个左右的位姿粒子 → 权重 → 重采样），
再把估计结果以 `map→odom` 变换发布出去（这样 `odom` 系的里程计漂移被地图修正）。
它输出的是**估计值**，不是真值：需要合理初始位姿，可能跟丢/漂移，跟丢后不一定自己恢复。

**逐项体检结果**：

| 检查 | 工具 | 结果 |
|---|---|---|
| 地图 vs 场地几何是否对齐 | `check_map_alignment.py` | 48 个方块 39 个 ≥90% 覆盖，整体 **96.5%**，最优平移 (−0.025,+0.025) 米 → **对齐，没有系统性偏移/旋转** |
| 底盘里程计准不准 | `check_localization.py` | odom 与 Gazebo 真值差 **0.07 m** → 准 |
| AMCL 位姿 | 同上 | 当时 map 估计 (2.15,0.42)，真值 (4.93,2.97) → **差 3.77 m**，`map→odom` 变成 (0.28, 5.81, −103°) |

**根因**：错误的重定位。日志里 3 条 `initialPoseReceived` → `Setting pose`：
`(-0.657,-1.571)`、`(0.629,-1.629)`、`(3.843,-1.843)`、`(2.200,-1.929)`（yaw 全 0）——
都不在机器人真实位置。这些是**我上一版 UI 的右键"重定位"**被误触发出的：
点错一下 AMCL 就被强行设到错误位置，而且**实测不会自愈**（5 分钟后仍差 4.8 m）。
用真值重定位后误差回到 **0.037 m**（yaw 差 1.0°）→ 定位能力本身没问题。

**修复（已实现并自检）**：
- 右键重定位改为**双击确认**（1.5 s 内同一处 ±0.3 m 再点一次），误触只提示不生效；
- 新增 **T 键 = 按 Gazebo 真值一键重定位**（仿真专用急救；`RELOCALIZE truth (...)`）；
- 状态栏显示 **定位误差 x.xx m**（与仿真真值比，>0.4 m 提示"跟丢了，按 T 重定位"）；
- `/initialpose` 连发 3 次（新 publisher 的发现延迟会吞掉单次发布）；
- 自检：`python3 map_click.py --self-test` → `CLICK relocalize confirmed` → `RELOCALIZE truth` → TF 回到真值 0.04 m。

**新增工具**：`check_localization.py`（估计 vs 真值）、`relocalize_to_truth.py`（按真值重定位并验证）、
`check_map_alignment.py`（地图与方块几何对齐核对）。

**教训**：AMCL 的 `initialPose` 是"强干预"，UI 上不能一点就发；地图/里程计都正常时，
"坐标不一致"优先查定位是否被错误重定位或跟丢。

### 19.8 2026-11-20 深夜四补：远距离导航定位漂移 + 重定位后路径不更新（两个都修了）

**用户报**：①远距离自动导航容易重定位错；②重新定位后位置准了，但路线没更新、"中间断掉"。

**① 定位漂移：AMCL 参数在本仿真里水土不服**
官方 `nav2_params.yaml` 的 amcl 段两处明显不对：
- `laser_max_range: 100.0`，而 Gazebo TB3 雷达量程只有 **3.5 m** → 打到量程上限的光束
  被当成"3.5 m 处真有障碍"参与打分，开阔区和地图预测冲突 → 累积偏置；
- `recovery_alpha_slow/fast: 0.0` → **恢复机制完全关闭**，跟丢/被设错位姿就永远回不来。

新增 `make_nav2_params.py` 生成 `config/nav2_params_rm.yaml`（官方参数 + 16 个 amcl 覆盖），
`nav2_field.sh` 默认使用它（`RM_PARAMS=<路径>` 可临时回退官方）：

| 参数 | 官方 | 本项目 |
|---|---|---|
| laser_max_range | 100.0 | **3.4**（略小于雷达 3.5，量程外的光束按无效处理）|
| laser_min_range | −1.0 | 0.12 |
| laser_likelihood_max_dist | 2.0 | 1.5 |
| max_beams | 60 | **180** |
| min/max_particles | 500/2000 | **800/3000** |
| update_min_d / update_min_a | 0.25 / 0.2 | **0.10 / 0.10** |
| recovery_alpha_slow / fast | 0.0 / 0.0 | **0.001 / 0.10** |
| alpha1..alpha5 | 0.2 | 0.10/0.10/0.10/0.05/0.05 |

**效果**（`drive_and_track.py`，出生点 (0,0) → (4.6,2.1)，约 12 m 路线）：
定位误差 起点 **0.024 m**、中位数 0.096 m、**最大 0.160 m**、终点 0.087 m，导航 SUCCEEDED。
对比之前：绕障验收最大 0.764 m，口袋里直接跟丢到 0.66~4.3 m。

**② 重定位后路线不更新：nav2 不会因为 AMCL 跳变自动重规划**
`test_relocalize_replan.py` 复现（目标 (4.6,−2.0)）：
1. 发目标走起来，6 s 后估计与真值差 0.15 m；
2. 发一个错误 initialpose（真值 +2.5 m）→ 5 s 后估计 (7.34,0.71)，**目标 ABORTED（status 6）**；
3. 再用真值重定位（不重发目标）→ 估计回到 (4.70,0.59)，但目标已死、离目标 **2.59 m**，
   **机器人不会自己接着走** ← 这就是用户看到的"断掉"；
4. 取消并重发同一目标 → 机器人按新位姿重规划、到点（距目标 **0.24 m**）。

**修复**（`map_click.py`）：重定位后若还有"上一个目标"，自动 `cancel + 1 s 后重发`，
强制 nav2 按新位姿重新规划。细节：目标 **ABORTED 时保留 `last_goal`**（等重定位后重发），
正常 SUCCEEDED / 被中键取消时清掉，避免莫名其妙又跑起来。

**操作总结**：点地图 → 走；状态栏"定位误差"变大（>0.4 m 会提示）→ 按 **T**（仿真真值重定位），
窗口会自动把上一个目标重发出去继续走；平移到没有真值的真实场景时，双击右键给真实位置同理。

## 20. 2026-11-20 深夜五补：未建图障碍物把 AMCL 带偏（"钻过缝隙后定位飘走"）——根因 + 修复 + 回归

**用户报**：在场地中间摆两块障碍物，让车从 0.8 m 缝隙钻过去，钻过去后定位漂到别处。

**复现**：`config/avoid_obstacles.json` 新增 `gate_deep`（两块 1.0×1.2×1.0 m，门洞仍 0.80 m，
窄段沿行进方向 1.2 m ≈ Gazebo GUI 默认箱量级），入口 `bash nav2_gap_deep.sh`。
修复前（`likelihood_field` + `do_beamskip:false`）实测：

- 过门时 in-range 束里 **平均 70%、峰值 90%** 是地图上没有的箱体回波；
- AMCL 自己的打分（nav2 实现是加性 `p += pz³`，`pz = z_hit·exp(−d²/2σ²) + z_rand/range_max`，
  `range ≥ laser_max_range` 的束跳过）：真值位姿 **p≈20**，"贴着平台边/墙角"的错误位姿 **p≈50~60**；
- 估计在 16.5 s 开始跳、17.1 s 已偏 1.54 m、**最终 2.9 m**；粒子数 2115→801（下限）→2513、
  粒子展布 std 峰值 0.58 m；nav2 19 次恢复后 **ABORTED**，车停在门后 1.3 m 处不动（不自愈）。

**机制**：地图里没有的回波落在自由区 → 每束只值 0.0032（命中地图 0.271）；门内这类束占多数后，
真值位姿不再是最大似然，估计被拉到"所有束都能解释成障碍"的贴墙/角落位姿；AMCL 跟丢后不会自己回来
（实测停 80 s 不变）。场地图还 **180° 点对称**（室内格 99.5% 一致），镜像分支也几乎同分 →
所以是"飘到另一个地方"，不是偏几十厘米。

**修复**（只改 2 行，由 `make_nav2_params.py` 生成，勿手改 yaml）：
`laser_model_type: "likelihood_field_prob"` + `do_beamskip: true`——把"所有粒子都解释不了"的束
当离群束丢掉。`config/nav2_params_rm.yaml` 已重新生成。

**回归结果**（`bash nav2_gap_deep.sh` + `python3 verify_narrow_gap.py --scenario gate_deep`）：

| 场景 | 修复前 | 修复后（两跑）|
|---|---|---|
| gate_deep（1.0×1.2 箱，深 1.2 m）| 漂 2.9 m、ABORTED | SUCCEEDED；到点 **0.293 / 0.231 m**；定位 **max 0.413 / 0.318 m**（门段 0.156 / 0.149 m）|
| gate（0.6×0.5 箱）| 0.383 m 最大定位误差 | **0.087 m**，到点 0.075 m |
| 绕障 15 项 | 全场定位 max 0.764 m（判据只能剔除 ≤2 m 样本）| **15/15 全过**，全场定位 max **0.315→0.302 m**，判据改回全样本 |

**窄通道验收新增第 6 项判据** `localization_consistent`：TF(map->base_footprint) vs Gazebo 真值
全局 max <0.50 m，并单独给"门线 ±0.9 m 穿门段"的 max——原来 5 项只测几何，漂了也照样全过。

**同日修掉的另外两个 bug**

1. `make_obstacles_world.py --only braking` 一直返回 1：`check()` 漏了 braking 分支
   （"生成的障碍物模型数 1 != 配置 0"），`nav2_braking.sh` 第 0 步就退出、刹车验收入口是坏的。
   已补 braking 校核（模型存在/kinematic/隐藏点与弹出点/宽度 ≥2.6 m），4 个变体现在都能生成+校核。
2. 退出路径崩溃：`map_click.py` 收盘 `terminate called without an active exception`；
   `goal_pose_bridge.py` SIGINT 时 `RCLError: failed to initialize wait set`。根因是
   `rclpy.spin()`/默认信号处理会先 `rcl_shutdown`，后台循环还在 wait set 里。改成
   `signal_handler_options=NO` + 信号只置位 + executor 轮询 + 统一收尾顺序
   （停 spin → shutdown executor → destroy node → rclpy.shutdown），Tk 关窗也走同一路径。
   实测：两个脚本 `kill -TERM` 后日志只有"收到退出信号…"，无 traceback/terminate。

---

## 21. 2026-11-21：清场按 ROS domain 隔离（修跨 domain 误杀 / 误删共享内存）

§20 末尾记的"发现但本轮未改"已修。原来 `nav2_field.sh` / `stop_nav2_field.sh` 用裸
`pkill -f` 清场、再 `rm -f /dev/shm/fastrtps_*`：

- `pkill` 不认 ROS domain —— 并行调试另一个 domain 时会把那边的 `goal_pose_bridge.py`、
  组件容器一起 SIGKILL（2026-11-20 实测误杀过一次 bridge）；
- `rm -f /dev/shm/fastrtps_*` 会把**别的 domain 正在用的** FastRTPS 共享内存/端口文件删掉
  （症状：action server 看着 ready，但 `/clock` 收不到、bridge 永远不 READY）。

修法：新增 `domain_cleanup.sh`，两个入口脚本 source 后调用 `kill_in_domain` /
`clean_stale_fastrtps_shm`：

- **进程**：读 `/proc/<pid>/environ` 的 `ROS_DOMAIN_ID`（未设置 = ROS 默认 0），只杀本 domain；
  environ 读不到（权限）的一律不动。`ROS_DOMAIN_ID` 也从写死 42 改成 `${ROS_DOMAIN_ID:-42}`，
  清场跟着同一个 domain 走。
- **共享内存**：活 participant 的痕迹实测有三种 —— 数据段/端口段被 **mmap**
  （`/proc/*/maps` 扫得到）、`*_el` 文件被 **flock** 锁住且 fd 打开、以及
  `sem.fastrtps_port*_mutex`（POSIX 信号量，没有文件锁/fd/mapping 可查）。
  前两种只要有一项在用就保留；**sem 一律保留**（删掉活栈的信号量比留个孤儿文件危险）。
  `(deleted)` 的旧映射不算"在用文件"（磁盘上本来就没文件可删）。
- 现场抢救回退：`RM_CLEAN_ALL_DOMAINS=1 bash stop_nav2_field.sh`（全 domain 清，含 shm）。

验收（不需要 Gazebo 的回归 + 真栈集成）：

```bash
bash verify_cleanup_domain_scope.sh     # 13 项：domain 过滤 / stop 入口 / flock/mmap/fd 保护 / 回退路径
```

| 证据 | 结果 |
|---|---|
| 旧版 `stop_nav2_field.sh`（HEAD）在 domain 42 执行，domain 43 的假 bridge | **被误杀**（复现 bug）|
| 新版同场景 | **存活** |
| 真栈（domain 42）+ domain 43 真 DDS participant（`ros2 topic pub`）+ 4 个同 cmdline 假进程 | 清场后 42 栈 30 个"在用"文件一个没删、`/navigate_to_pose` 仍在；`stop_nav2_field.sh` 后 42 残留 0，43 的 5 个进程全存活 |
| 栈停掉后再清 | 删掉 16 个崩溃/强杀留下的孤儿文件（原始目的仍是有效的）|