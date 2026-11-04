# 演示当天操作手册：建图 → 重定位 → 示教 → 回放

> 适用：RDK X5 + Odin1 + C8T6 练习车，固定场地「先建图、再自动跑固定路线」的演示。
> 全部命令都在 X5 上跑（`ssh sunrise@10.77.156.239`），本手册按**现场顺序**排列。
> 2026-11-04 晚在新地图 `venue2-20261104` 上端到端验证过两遍（一轮成功、一轮重示教后成功）。

---

## 0. 前置（演示前 10 分钟做完）

| 项 | 检查 |
| :--- | :--- |
| 整车电源 | 电池上电，底盘控制器按 PowerKey 开机 |
| **Odin1 12 V** | 航插**独立 12 V/2 A** 供电（USB3 只走数据）；灯不亮 = 电流不够，别当成软件问题 |
| 控制器 | DT7 开机、S2 拨到**中间**、油门回中；e-stop 在手边 |
| 场地 | 地面无反光/无强逆光，起点附近 2 m 空旷；人清场 |
| X5 | `ssh sunrise@10.77.156.239` 能进；`lsusb \| grep 2207` 应有 `2207:0019` |

自检（可选但推荐）：

```bash
source /opt/ros/humble/setup.bash; source ~/car_ws/install/setup.bash
ros2 run car_bridge bridge_node &        # 起桥接
sleep 3; ros2 topic echo /car/status --once
```

期望 `status=119`：`can_ok link_ok rc_ok can_ack uart2_rx dbus_frame` 六项全绿。
少 `can_ack`（103）常见于桥接刚起的 1–2 s，等一会儿再看；持续缺 `can_ok/can_ack`
（102 类）**先整车断电重启**，不要改线。

> 桥接在跑时固件按 `src=serial` 仲裁，**遥控会被屏蔽**。要人工开车必须先停桥接：
> `pgrep -f "lib/car_bridge/bridge_node$" | xargs -r kill`

---

## 1. 建图（约 6–12 分钟，含开车时间）

```bash
bash ~/car_slam_lap/x5/scripts/start_mapping.sh demo_YYYYmmdd
```

起来后用**遥控器**把场地走一圈（5–10 分钟）：

- 速度匀速、转弯走大弧，**不要长时间原地打转**（点云质量会差）
- 尽量把演示路线覆盖到，走廊/门框处慢一点、多走一点
- 灯别关、别站到雷达正前方一直挡着

走完：

```bash
bash ~/car_slam_lap/x5/scripts/save_map.sh     # 触发存图，等 "Map transfer completed"
bash ~/car_slam_lap/x5/scripts/stop_mapping.sh # 停驱动、恢复厂商配置
```

产物自检（`~/car_ws/maps/demo_YYYYmmdd/`）：`map.pcd`、`map_bev.png`、`map_info.txt`、
`odin_map/map_*.bin`、`path.yaml`。

> 2026-11-04 实测：开车 ~2.5 min → `map_*.bin` 143 MB、`map.pcd` 8.5 MB、`path.yaml` 321 点/13.54 m。

---

## 2. 重定位（40–90 秒）

```bash
bash ~/car_slam_lap/x5/scripts/start_relocal.sh ~/car_ws/maps/demo_YYYYmmdd
```

成功标志：`✓ 重定位成功：map -> odom 已发布`。

失败时：①`lsusb` 确认 `2207:0019` 还在；②把车挪到场地中间、换个朝向再跑一次
（重定位需要特征，贴着白墙或全场都动过会失败）。

---

## 3. 示教（约 3–4 分钟，含开车时间）

**先停桥接**（示教必须遥控）：

```bash
pgrep -f "lib/car_bridge/bridge_node$" | xargs -r kill
bash ~/car_slam_lap/x5/scripts/teach_map_route.sh ~/car_ws/maps/demo_YYYYmmdd
```

看到“现在用 DT7 开车示教”后，把车摆到**演示时的起点**（车头朝演示出发方向），然后：

1. 速度 **0.2–0.3 m/s**，转弯走**大弧**（转弯半径 ≥ 0.5 m；本车最小半径约 0.43 m，
   太急只能靠“停车原地对齐”硬掰，横向误差会到 0.2–0.3 m）
2. 按演示路线开一圈，尽量走中间、别贴边
3. **末段一路向前开过起点 0.3–0.5 m 再停**（不要倒车、不要原地搓）
   —— 回放只走前进；上次倒着蹭进终点，导致最后必须原地掉头 176°
4. 停稳后按 **Ctrl-C** 收尾（正常结束才会写 `route_map.yaml.manifest.json`）

成功标志：`✓ 严格示教完成` + 生成 `route_map.yaml` 与 `route_map.yaml.manifest.json`。

> 示教中止“原始里程计停更 0.52 s”= Odin1 回调队列溢出（偶发）。板端已把 CPU
> 调频设为 `performance` 降低概率；重开示教即可，别改安全阈值。

---

## 4. 回放（约 2 分钟）

```bash
# 1) 起桥接（遥控会被屏蔽，这是正常的）
ros2 run car_bridge bridge_node &

# 2) 跑演示路线
VMAX=0.15 W_MAX=0.35 RETURN_W_MAX=0.30 \
BASE_OFFSET_X=0.2626 BASE_OFFSET_Y=-0.0471 \
START_MAX_OFFSET_M=2.0 RETURN_TIMEOUT_S=150 MAX_TIME_S=320 \
bash ~/car_slam_lap/x5/scripts/drive_lap.sh ~/car_ws/maps/demo_YYYYmmdd
```

参数说明（都可省，省了用脚本默认）：

| 变量 | 演示用值 | 含义 |
| :--- | :--- | :--- |
| `VMAX` | 0.15 | 目标速度上限（m/s）；精度优先用 0.12–0.15 |
| `W_MAX` | 0.35 | 跟踪段角速度上限（rad/s）；**不传就是 1.2，转弯会很猛** |
| `RETURN_W_MAX` | 0.30 | 回起点/起始对齐的角速度上限 |
| `BASE_OFFSET_X/Y` | +0.2626 / −0.0471 | 力臂补偿（本车已标定，换雷达安装必须重标） |
| `START_MAX_OFFSET_M` | 2.0 | 允许车离起点多远仍自动回起点 |
| `RETURN_TIMEOUT_S` | 150 | 回起点+起始对齐总预算（**范围放大必须同步放大**：1.5 m 约需 54 s） |
| `MAX_TIME_S` | 320 | 总时限，0=不限 |

脚本自带 5 s 预启动；`Ctrl-C` 随时停（会写 `lap_actual.yaml` 并停零）。
数值写 `0.15` 这种带小数点的；写 `280` 会被 ROS 2 推断成整数而崩溃（脚本已自动补 `.0`）。

**成功标志**：`结果: 成功` + `结束原因: 到达终点（位置与朝向均达标…）`。

> 2026-11-04 实测基线（`v_max=0.15`、新路线 303 点/14.99 m）：
> 总 110.2 s（回起点 26.4 s + 跟踪 83.8 s）、行驶 14.65 m、横向误差 max 0.269 / rms 0.111 m、
> 终点朝向误差 0.01 rad、里程计 1184 帧全接受、急弯停车 0 次。

---

## 5. 时间预算

| 阶段 | 耗时 | 备注 |
| :--- | :--- | :--- |
| 上电 + 自检 | 3–5 min | 含 `status=119` 确认 |
| 建图 | 6–12 min | 开车 5–10 min + 存图 ~30 s |
| 重定位 | 1–2 min | 通常 40–90 s |
| 示教 | 4–5 min | 开车约 3.5 min |
| 回放 | ~2 min | 110 s |
| 合计 | **~16–25 min** | 建议留 40 min 余量 |

---

## 6. 故障处置速查

| 症状 | 处理 |
| :--- | :--- |
| `lsusb` 无 `2207:0019` / Odin1 灯不亮 | 12 V 供电电流不足：查航插、换电源；不是驱动问题 |
| `status` 缺 `can_ok`/`can_ack`（102 类） | **整车断电重启**清总线状态，别先改线 |
| `status=103`（只缺 `can_ack`） | 多为桥接刚起瞬态，等 1–2 s；持续则同上 |
| 示教中止“原始里程计停更” | Odin1 队列溢出，重开示教；板端已设 performance 调频 |
| 重定位 90 s 未成功 | 车挪到特征多的地方/换朝向重跑；确认雷达在 |
| 回放报“起点偏差 > start_max_offset_m” | 人工把车摆回起点，或放大 `START_MAX_OFFSET_M` 并同步放大 `RETURN_TIMEOUT_S` |
| 转弯太猛 | 确认传了 `W_MAX`（默认 1.2 rad/s） |
| 报“漂离终点”/“对齐超时” | 2026-11-04 已修（判据改转心、超时按角度自适应）；若重现先看 `lap_actual.yaml` 末段 |
| 桥接在跑时遥控失灵 | 正常仲裁：停 `bridge_node` 才能用遥控 |
| 想重跑但路线报 bundle 校验失败 | 路线与地图 `sha256` 绑定；重定位的老地图/新地图不能混用 |

---

## 7. 收尾

```bash
pgrep -f "lib/car_bridge/bridge_node$" | xargs -r kill   # 停桥接（停发指令 → 固件 0.3 s 超时归零）
# 若还在跑定位：Ctrl-C start_relocal 的前台/停 host_sdk_sample
sudo shutdown -h now                                     # 演示结束关机（断电前先关 X5）
```

演示结束后把关键文件拷回本机留档：`route_map.yaml`、`route_map.yaml.manifest.json`、
`lap_actual.yaml`、`map.pcd`、`odin_map/map_*.bin`。