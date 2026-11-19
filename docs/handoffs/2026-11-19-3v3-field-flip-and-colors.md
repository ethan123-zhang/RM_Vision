# 2026-11-19 3V3 场地翻转 + 配色重做（含：轮子空转不走的 bug 定位）

**一句话**：按用户要求把 `RMUL2027_3V3_V1.0.0.stp` 导出的场地**上下翻转**（结构面朝上、平整面作台体底板），
重新分配了配色（灰地面 / 绿凸起 / 红外墙 / 黄顶沿），跑通 Gazebo 世界并实测机器人能开；
同时**定位到"发前进指令轮子转、车不走"的根因**：三角网格碰撞把机器人垫高了 6 mm，轮子失去摩擦。
**临时处置**：网格碰撞已停用（车能开，但凸起结构暂时没有碰撞、能穿过去）。修碰撞是下个对话的第一件事。

---

## 1. 本轮做了什么（按发生顺序）

### 1.1 取证：官方 3V3 场地到底长什么样

- 无 PDF 渲染器（WSL 无 poppler、Windows 无 PyMuPDF、PyPI 被代理挡），改用**纯 Python 硬刮 PDF 内嵌 JPEG**：
  `_probe/carve_pdf_jpegs.py`（`_probe/` 已在 .gitignore，产物不入库）。
- 抠出的图 3-4 / 3-5 显示：场地有**黑色外围墙、两块带蓝色控制区的高台、隧道、工作台/补给区**。
- 用户明确：**2027 赛季 3V3 场地没有斜坡**。所以我此前"没有斜面 ⇒ 不可能是场地"的推断作废。

### 1.2 翻转模型（用户决定：以"结构面朝上"为准）

`debug/make_flipped_meshes.py`：

- `z -> -z` 翻转，三角形绕序同时反转（法向保持朝外），再整体 `-0.10 m`，**使场地地面落在 z = 0**；
- 按面朝向/位置分类导出到 `models/rmul2027_3v3_field_up/meshes/`：

| 网格 | 面数 | 面积 | 用途/颜色 |
|---|---|---|---|
| `floor.stl` | 41,440 | 97.3 m² | 场地地面（z≈0）→ **浅灰，机器人开这里** |
| `inner.stl` | 24,524 | 50.5 m² | 内部凸起平台/筋 → **绿色** |
| `wall.stl` | 30,881 | 64.9 m² | 外围墙墙体 → **红色** |
| `wall_top.stl` | 18,053 | 28.2 m² | 外围墙顶沿 → **黄色** |
| `base.stl` | 39,622 | 108.4 m² | 台体底板（朝下，z=−0.10）→ **深灰** |
| `structures.stl` | 113,080 | — | 除地面外的全部面（**原计划做碰撞，见 §3 bug**） |

翻转后 bbox：x ±6.3、y ±4.3、z **−0.10 … +0.403**（地面 0、高台 +0.19/+0.37、墙顶 +0.40）。

### 1.3 新模型 / 新世界 / 启动方式

| 文件 | 作用 |
|---|---|
| `models/rmul2027_3v3_field_up/model.{config,sdf}` | 翻转+分色模型；地面用 box 碰撞（0.1 m 厚、顶面 z=0）；`structures_collision` **当前被注释停用**（原因见 §3） |
| `worlds/drive_colored.sdf` | 驾驶世界（world 名 `drive`）；`site_ground` 放在 **z = −0.10**（台体底板同高） |
| `worlds/drive_colored_preview.sdf` | 出图世界（多三台相机 `/hero/image`、`/view_top/image`、`/view_under/image`） |
| `config/gui_drive.config` | GUI 相机位姿 `-9.0 8.0 4.0 0 0.32 -0.727`，窗口 1400×900，Entity Tree 浮动 |
| `gui_colored.sh` | 一键起世界 + 在 (0,0) 生成 TB3 + 打印开车命令 |
| `debug/render_drive_preview.sh` / `render_under.sh` / `render_top_bottom.sh` | 出图（headless + llvmpipe） |

### 1.4 验证（读数）

| 项 | 结果 |
|---|---|
| 世界加载 | `/world/drive` 下模型：`site_ground`、`rmul2027_3v3_field_up`、`waffle` |
| RTF | **0.9995**（网格碰撞在时；物理没被拖慢） |
| 机器人静置 z | **0.0100**（去掉网格碰撞后）/ **0.0158**（有网格碰撞，被垫高 6 mm） |
| 前进 0.15 m/s × 2 s 位移 | **0.42 m**（去掉网格碰撞后，x: 0 → 0.4209）；有网格碰撞时 **≈0**（x 只变 1.4e-5 m） |

---

## 2. 上一轮（同一天早些时候）的颜色标注产物：保留但已被取代

`debug/split_field_mesh.py` + `debug/render_colored.sh` + `debug/annotate_colored.py`
产出的 `debug_colored_{hero,wall,elevation}(_annotated).png` 是**翻转前**的取证图（用来证明"0.503 m 台面在地面之上"）。
翻转后这套结论不再适用于当前世界，文件保留作历史，不要拿它当现在的场地证据。

---

## 3. ❗未完成：#B1 发前进指令"轮子转、车不走"

### 3.1 现象（用户实测 + 我复现）

- `gz topic -t /cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.15}'` 发 2 s，画面里**轮子在转，车原地不动**；
- 实测位姿：x `0.005291` → `0.005305`（2 s 内只动 0.014 mm），z = **0.0158**；
- `/cmd_vel` 有订阅者（diff-drive 插件正常收指令）。

### 3.2 已定位的根因（本轮已做对照实验）

把 `models/rmul2027_3v3_field_up/model.sdf` 里的 **`structures_collision` 注释掉**（只留地面 box）后，
同一套指令下机器人 **2 s 走了 0.42 m、z = 0.0100** 恢复正常。
⇒ 问题出在**用 113,080 面的三角网格 `structures.stl` 当碰撞体**：机器人被垫高约 6 mm，轮子空转失去摩擦。

判别脚本：`debug/probe_drive_issue.sh`（headless 起世界 → spawn → 前进 2 s → 打印位姿）。

### 3.3 当前临时状态

- `structures_collision` 已停用 ⇒ **绿色凸起平台、红色外围墙暂时没有碰撞，机器人能开穿过去**；
- 这只适合"先看配色 / 先能开"，**不能当验收状态**。

### 3.4 修复候选（下次从这里挑一个）

| 方案 | 做法 | 代价/风险 |
|---|---|---|
| **A（推荐）** | 用 box 近似做碰撞：从高度图（0.2–0.3 m 栅格）里找高度 >5 cm 的区域，合成若干 box；或直接手写"两块高台 + 四面墙 + 隧道墙"的 box 列表 | 碰撞稳定、便宜；box 数量 10–50 个，要写生成脚本并验证撞墙位置 |
| B | 把 `structures.stl` 降采样（2 cm 体素）后再当碰撞网格 | 改动小；dartsim 三角网格接触本身就不稳，可能仍有 1–3 mm 抬升 |
| C | 保持无碰撞 | 只能纯驾驶练习，不能做导航/碰撞验收 |

建议先做 A，然后用**围挡/高台的撞墙读数**验收（撞东墙停在 x≈6.0 附近、撞高台边缘停住、`z` 全程恒定 0.010）。

---

## 4. 另一条欠账：2D 导航地图已失效

`maps/rmul2027_3v3_fenced.pgm`、`verify_nav2_field.py`、`results/report_nav2_field.json` 都是**按翻转前的平整台面**生成的。
翻转后场地地面变成 97.3 m²（比原来的 106.25 m² 小，被高台/墙占掉一部分），**地图与坐标都对不上**。
要在翻转后的场地上跑 Nav2，需要：按新地面重建 pgm/yaml → 重定起点（地面 z=0，AMCL 初始位姿）→ 重跑验收。

---

## 5. 下次进入点（可直接复制执行）

```bash
cd /mnt/e/RM/tb3_gazebo_nav2/rm_field

# 0) 仓库里 .stl 不入库：克隆/清过工作区后先重建翻转网格
python3 debug/make_flipped_meshes.py

# 1) 起彩色世界（world 名 drive）
bash gui_colored.sh

# 2) 开车（轮子转但不走时先看这句的结果）
gz topic -t /cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.15}'
gz topic -t /cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.0}, angular: {z: 0.0}'
gz topic -e -t /world/drive/pose/info -n 1 | grep -A6 'name: "waffle"'

# 3) 复现/验证 bug 的对照实验（会自动起 headless 世界并跑一次前进）
bash debug/probe_drive_issue.sh

# 4) 出图（配色核对）
bash debug/render_drive_preview.sh
```

要改碰撞看这里：`models/rmul2027_3v3_field_up/model.sdf`（`structures_collision` 现在是注释状态，旁边有 2026-11-19 的说明）。

---

## 6. 坑与注意事项（本轮踩到的）

1. **WSLg 窗口位置**：GUI 窗口被放到 X 坐标 2572/2733（`screen 0`），`xdotool windowmove` 被 Weston 忽略、挪不动；
   用 Windows 侧 `Alt+Tab` 找 **Gazebo Sim**；`onscreen_gui.sh` 只在窗口刚起来时偶尔生效。
2. **出图世界与驾驶世界的相机话题不同**：预览世界用 `/hero/image`，旧脚本曾写 `/drive_view/image`，
   会导致"抓图失败但脚本照常打印 done"、留下旧图 → 改脚本时**必须同步 topic 名**。
3. 每次 `gui_colored.sh` / `render_*.sh` 都会 `pkill -f "gz sim"`，起之前先确认没有别的仿真在跑。
4. 本次对话**未提交任何 commit 之前**的状态：所有新文件都是 untracked；`.stl` 与 `shots/*.png` 被 .gitignore 挡住，
   所以克隆后必须靠 `debug/*.py` 重新生成网格。

---

## 7. 本轮新增/改动的文件清单

**模型/世界**
- `models/rmul2027_3v3_field_up/{model.config,model.sdf}`（新，翻转+分色）
- `worlds/drive_colored.sdf`（重写：翻转世界）
- `worlds/drive_colored_preview.sdf`（重写：出图相机）
- `worlds/debug_height_colored.sdf`、`worlds/preview_back_side.sdf`、`worlds/probe_flipped_field.sdf`（诊断用，保留）
- `models/{debug_field_colored,debug_fence_blue,drive_field_colored,drive_fence_red,probe_field_flipped}/`（前两轮/诊断用，保留）

**脚本**
- `debug/make_flipped_meshes.py`（翻转+分色+碰撞网格，**必跑**）
- `debug/probe_drive_issue.sh`（bug 判别）
- `debug/render_drive_preview.sh`、`debug/render_under.sh`、`debug/render_top_bottom.sh`、`debug/render_back_side.sh`
- `debug/flip_field_mesh.py`、`debug/make_solid_meshes.py`、`debug/split_field_mesh.py`、`debug/annotate_colored.py`、`debug/analyze_levels.py`、`debug/relief_map.py`（诊断/历史）
- `gui_colored.sh`、`config/gui_drive.config`

**未入库**：`models/**/*.stl`、`shots/*.png`、`_probe/`（含从 PDF 抠出的官方示意图）