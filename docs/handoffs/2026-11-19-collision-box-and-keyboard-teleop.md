# 2026-11-19 碰撞改 box 拼块 + 键盘遥控手动开车（副线 Gazebo）

**目标**：把翻转后 3V3 场地的"轮子转、车不走"碰撞 bug 修掉，并给出一条能手动开车验证场地的操作路径。

---

## 1. 环境与分支

| 项 | 值 |
|---|---|
| 机器 | Windows + WSL2 Ubuntu-24.04（ROS 2 Jazzy / Gazebo Harmonic 8.15.0） |
| 仓库 | `E:\RM` → `github.com/ethan123-zhang/RM_Vision` |
| 分支 | `codex/car-local-obstacle-avoidance` |
| 起点提交 | `40a9bd0`（场地翻转 + 配色） |
| 本轮提交 | `80bf37f`（碰撞修复）、`496f905`（键盘遥控），均已 push |

## 2. 问题表（现象 → 根因 → 修法 → 验证）

| # | 现象（带数字） | 根因（机制） | 修法 | 验证（前后对比） |
|---|---|---|---|---|
| B1 | 发 `linear.x=0.15` 2 s，**轮子在转、车不动**：x 只动 1.4e-5 m，z=**0.0158**（正常 0.0100） | `structures.stl`（113,080 面三角网格）做 collision：dartsim 三角网格接触把车体垫高约 6 mm，**轮子离地失去摩擦** | `debug/make_box_collisions.py`：网格 → 0.05 m 高度图（水平面重心坐标判格心、竖直面包围盒覆盖）→ 高度聚成 0.20/0.27/0.40 三层 → 行游程合并成 **48 块 box**，写回 `model.sdf` 的 `BOX_COLLISION_BEGIN/END` | 修复前 z=0.0158、2 s 位移 ≈0；修复后 **z=0.0100、2 s 走 0.2762 m** |
| B1a | （修复过程中的次级 bug）高度图里 0.20 m 层只覆盖 9 m²、墙内侧面算成 5.25 m（应为 6.00） | 三角形"格心在内部"的边函数符号写反 ⇒ **所有水平面一格都没填**，只剩竖直面包围盒投影 | 三条边函数统一乘 `s = sign(area)`；竖直面仍用包围盒覆盖（投影退化成线，不能按面积判） | 48 块 box、占据 31.90 m²、高度误差 max 0.035 / rms 0.0014 m |
| B2 | 无 | — | 五项读数验收 `verify_box_collisions.{py,sh}` | 见 §3 表 |
| C1 | 需要手动开车验证场地/碰撞，但不希望依赖鼠标或 RViz | Gazebo 没有鼠标开车 GUI；终端收不到 keyup | `keyboard_teleop.py`：cbreak 读单字节 + 方向键转义序列，KEY→(lx,az)，**0.5 s TTL** 实现"按住走/松手停"，一阶平滑 α=0.45@20 Hz；`drive_teleop.sh` 一条命令起 GUI 世界+小车+遥控 | 自检 4 项全过；真实 pty 敲键实测见 §4 |

## 3. 碰撞验收（`bash verify_box_collisions.sh` → `results/report_box_collisions.json`）

| 项 | 判据 | 实测 |
|---|---|---|
| A 静置 (0,0) | z=0.0100±0.003，漂移<0.01 m | z=**0.0100**，漂移 0.0001 |
| B 直行 0.15 m/s×2 s | ≥0.24 m（旧 bug 1.4e-5 m） | **0.2762 m**，z 偏移 0 |
| C 撞东墙 x=+6.00 | 停 5.9315±0.02、不越界、z 恒定 | **5.9316**（误差 0.1 mm），z=[0.0100,0.0100]，贴住 2 s 漂移 0 |
| D 撞北墙 y=+4.00 | 停 3.9315±0.02 | **3.9316**（误差 0.1 mm） |
| E 撞绿平台 0.20 m | 停 −1.0685±0.02、**不许爬上台** | **−1.0684**，z=[0.0100,0.0100]，未爬台 |

几何（实测自高度图，写进 `debug/box_collisions.json`）：墙内侧面 **x=±6.00 / y=±4.00**，平台顶 0.20 m、
台沿 0.27、墙顶 0.40；可行驶地面约 12.0×8.0 减两块平台。停车位置 = 墙面 − 0.0685 m（TB3 车头外缘）。

## 4. 键盘遥控（`bash drive_teleop.sh`，README §14）

- `--selftest`（世界在跑时）：按住 w 1.5 s → **+0.361 m**；按住 a 1.5 s → 航向 **+1.157 rad**；
  无指令 1 s 漂移 **0.0000 m**；`↑` 1.2 s → 0.285 m。**全过**。
- 真实 pty 终端里敲键：`w` → x 0→**0.109 m**；`d` → yaw **−0.369 rad**；松手 2 s 位姿完全不变。
- 键位：w/s/a/d、q/e/z/c 斜向、空格停、+/- 速度档、[/] 转向档；键盘焦点留在终端，Gazebo 窗口只用于看。

## 5. 当前状态

- **能做什么**：GUI 里手动开车；撞墙/撞台会被挡住且不越界、不爬台；`verify_box_collisions.sh` 可重复验收。
- **卡在哪**：没有 2D Nav2 地图（旧图基于翻转前场地，已失效），所以还不能在 RViz 里点目标点导航。
- 仿真进程已全部停掉，WSL 里无残留 `gz sim`。

## 6. 已完成 / 未完成 / 下次进入点

**已完成（带证据）**：碰撞修复（48 块 box，五项读数全过，`report_box_collisions.json`）；键盘遥控（自检+pty 实测）；
README §13/§14、PROGRESS 同步；两次提交已 push。

**未完成**：
1. **D：按新地面重建 Nav2 地图**（自由区 = 地面多边形减高台/墙）→ 重定 AMCL 起点 → 重跑 `verify_nav2_field.py`。
2. 围挡高度 0.30 m 与赛规一致性未核对（历史遗留，且翻转后围挡是场地自带墙，不是补丁）。
3. 绕障/动态避障未测。

**下次进入点**：

```bash
cd /mnt/e/RM/tb3_gazebo_nav2/rm_field
bash drive_teleop.sh                 # 手动开车（GUI + 键盘）
python3 debug/make_flipped_meshes.py && python3 debug/make_box_collisions.py   # 网格丢了/改碰撞时重建
bash verify_box_collisions.sh        # 碰撞回归（约 70 s）
# 下一步 D：
python3 -m py_compile make_nav_map.py   # 先改自由区生成逻辑，再 capture_nav_map.sh / nav2_field.sh / verify_nav2_field.py
```

## 7. 遗留风险与文件

- **碰撞是 5 cm 栅格拼块**：台沿/墙角有 ≤5 cm 量化误差；贴边行驶场景再加细（`make_box_collisions.py --cell 0.025`）。
- **`debug/box_collisions.json` 与 `model.sdf` 必须同源**：重新生成碰撞后要一起提交，否则验收读旧几何。
- **手动开车不防撞**：`drive_teleop.sh` 没有避障，速度档最高 0.5 m/s；场地外沿墙能挡住，但 PIT 区/台沿仍可能被撞上去。
- 文件：`debug/make_box_collisions.py`、`debug/box_collisions.json`、`verify_box_collisions.{py,sh}`、
  `results/report_box_collisions.json`、`keyboard_teleop.py`、`drive_teleop.sh`、`README.md` §12–§14。