# car_slam_lap —— 小车「建图 + 走一圈」练习场

> **这是干嘛的**：拿现成的 C8T6 双驱底盘 + RDK X5 + Odin1，跑通
> **房间建图 → 重定位 → 小车自己走一圈** 这条完整链路，为之后调比赛机器人打基础。
>
> **当前状态**（2026-10-20）：
> - ✅ 链路协议（C + Python 双端，黄金向量测试通过）
> - ✅ X5 侧全部 ROS2 节点、配置、脚本、部署
> - ⬜ 固件 HAL 部分（USART3 接入、仲裁、看门狗）—— 等你确认接线后做，清单见 [§9](#9-固件还差什么)
>
> 原来的底盘工程只支持 DT7 遥控器；本项目的目标是把控制源换成
> **X5 串口 → C8T6 → CAN → 电机控制器**，同时保留遥控器做手动驾驶和急停。

---

## 1. 目录

```
car_slam_lap/
├─ firmware/C8T6_DT7_car/         原 Keil 工程（遥控器版）+ Hardware/LINK/protocol.c
├─ tests/                         协议测试（gcc 编译，不需要硬件）
│   ├─ protocol_test.c            70 项断言：CRC/布局/重同步/坏帧/黄金向量
│   └─ run_tests.ps1              一键跑 C + Python 全部离线测试
├─ x5/
│   ├─ car_bridge/                ROS2 包（python）
│   │   ├─ car_bridge/protocol.py         与固件同源的帧实现
│   │   ├─ car_bridge/path_io.py          轨迹文件读写 + 几何（不依赖 ROS）
│   │   ├─ car_bridge/bridge_node.py      /cmd_vel → 串口，回读 /car/status
│   │   ├─ car_bridge/link_test.py        链路自检（含看门狗验证）
│   │   ├─ car_bridge/teleop_keys.py      键盘遥控
│   │   ├─ car_bridge/record_path.py      建图时录轨迹
│   │   ├─ car_bridge/follow_path.py      纯追踪回放走一圈
│   │   ├─ car_bridge/map_from_cloud.py   点云累积成 PCD + 俯视图
│   │   ├─ car_bridge/calibrate_speed.py  实测 m/s 与指令的换算
│   │   ├─ test/test_protocol.py          同一组黄金向量
│   │   └─ test/test_path_io.py           轨迹文件往返读写
│   ├─ config/                    Odin1 的建图 / 重定位两份配置
│   ├─ scripts/                   板端一键脚本（安装/建图/存图/重定位/走圈）
│   └─ pc/deploy.ps1              打包 + pscp + 远程安装
└─ README.md                      本文件
```

数据流：

```
PC 键盘 ─┐
         ├→ /cmd_vel ─→[car_bridge 50Hz]─串口 115200→[C8T6 USART3]
路点回放 ─┘                                              │ 10ms 仲裁
                                                         ↓ CAN 250k, extID 0x1314
                                                   [XHSMCUW50 双驱] → 左右轮
Odin1 ─USB3→ /odin1/{odometry, cloud_slam, imu} ─→ 建图(mode 1) / 重定位(mode 2)
C8T6 ── status 10Hz ──→ /car/status ──→ 日志 + follow_path 的安全联锁
```

---

## 2. 接线

| 连接 | 细节 | 注意 |
| :--- | :--- | :--- |
| X5 USB ↔ USB-TTL ↔ C8T6 | X5 任意 USB 口 → CH340/CP2102 → **PB10/PB11（USART3）** | **TX 接 RX、RX 接 TX**；115200 8N1 |
| 共地 | USB-TTL GND ↔ C8T6 GND | 少了这根线，收到的一定是乱码 |
| USB-TTL 供电脚 | **不接**（3V3/VCC 悬空） | C8T6 由车上 5V 供电，接上会打架 |
| DT7 接收机 ↔ C8T6 | PA9/PA10（USART1），100000 8E1 | 原样保留，s2 打到底 = 硬急停 |
| 电机控制器 ↔ C8T6 | CAN_H/CAN_L，PA11/PA12，**250 kbit/s**，扩展帧 `0x1314` | 总线两端各 120 Ω 终端电阻 |
| Odin1 ↔ X5 | Type-C → **蓝色 USB3 口** | 必须 USB3，SLAM 模式传地图文件要靠它 |
| Odin1 供电 | 航插 12V / 2A（≥15W） | 跟 Type-C 分开，Type-C 不供电 |

> ⚠️ 为什么 X5 接 USART3 而不是 USART1：遥控器接收机占着 USART1，而且它是
> 100000 8E1 的专用时序；USART2 留给 printf 调试。USART3(PB10/PB11) 是空的。

---

## 3. 串口协议

二进制帧，115200 8N1，小端。

**下行 X5 → C8T6，50 Hz，9 字节**

```
+------+------+-----+-----+-----------+-----------+------+
| 0xA5 | 0x5A | LEN | CMD | speed i16 | angular   | CRC8 |
|      |      | 0x05| 0x01| LE        | i16 LE    |      |
+------+------+-----+-----+-----------+-----------+------+
                          [-1000,1000] [-1000,1000]
```

**上行 C8T6 → X5，10 Hz，7 字节**

```
+------+------+-----+-----+--------+-----+------+
| 0x5A | 0xA5 | LEN | CMD | status | src | CRC8 |
|      |      | 0x03| 0x81|  位    | 0-2 |      |
+------+------+-----+-----+--------+-----+------+
```

- `speed > 0` 前进，`angular > 0` 右转
- `status`：b0 CAN 正常 / b1 串口在线 / b2 遥控在线 / b3 急停
- `src`：0 无 / 1 串口 / 2 遥控
- **CRC8 = poly 0x07 / init 0x00 / 不反转**（CRC-8/ATM），覆盖 `LEN` 字节及其后 `LEN` 个字节
- 外部校验：`crc8("123456789") == 0xF4`

黄金向量（`tests/protocol_test.c` 与 `x5/car_bridge/test/test_protocol.py` 断言同一份）：

| 内容 | 字节 |
| :--- | :--- |
| `set_vel(0, 0)` | `A5 5A 05 01 00 00 00 00 EF` |
| `set_vel(400, -200)` | `A5 5A 05 01 90 01 38 FF 70` |
| `set_vel(-1000, 1000)` | `A5 5A 05 01 18 FC E8 03 4C` |
| `status(0x03, serial)` | `5A A5 03 81 03 01 62` |

**为什么是二进制帧而不是 `V,400,-200\n` 这种文本**：文本协议上线快，但 CRC 和长度
字段是后面接更多外设（云台、发射机构）时的必需品；这套帧格式和 RoboMaster 常见的
裁判系统/串口协议同构，练手价值更高。调试照样方便——`bridge_node` 的 `dry_run`
参数会把每帧 hex 打出来。

---

## 4. 部署（PC → 板端）

```powershell
# PC：打包 + 上传 + 远程安装（一条命令）
pwsh -File car_slam_lap\x5\pc\deploy.ps1 -Install
```

分步做也行：

```powershell
pwsh -File car_slam_lap\x5\pc\deploy.ps1          # 只上传
```

```bash
# 板端
cd ~ && rm -rf car_slam_lap && mkdir -p car_slam_lap
unzip -o car_slam_lap_deploy.zip -d car_slam_lap
bash ~/car_slam_lap/x5/scripts/install_car.sh
```

`install_car.sh` 会做：apt 依赖（`python3-serial`/`python3-yaml`）→ 建
`~/car_ws/{src,config,maps,logs}` → 拷源码和配置 → 装 udev 规则（USB-TTL 固定成
`/dev/car_link`）→ `colcon build`。

前置：Odin1 驱动已按 [x5_odin/README.md](../x5_odin/README.md) 部署在 `~/odin_ws`，本套脚本不碰它。

---

## 5. 第一次跑：按这个顺序

```bash
# ① 链路自检（车轮不会动；验证收发 + 看门狗）
ros2 run car_bridge link_test --port /dev/car_link
#   期望：status ~10 Hz、crc_err 0、停发后 LINK_OK 掉落 -> PASS

# ② 底盘桥接（另开一个终端，保持运行）
ros2 run car_bridge bridge_node

# ③ 键盘遥控（第三个终端）
ros2 run car_bridge teleop_keys
#   w/s/a/d 走一下，确认方向和停止都正常

# ④ 标定速度（让 /cmd_vel 的 m/s 是真单位）
ros2 run car_bridge calibrate_speed
ros2 run car_bridge calibrate_speed --ros-args -p axis:=angular
#   把打印的建议值填进 scripts/drive_lap.sh 的 v_max 依据里（见 §7）
```

---

## 6. 建图

```bash
bash ~/car_slam_lap/x5/scripts/start_mapping.sh lab1
#  → 起 Odin1（建图模式）+ 点云导出 + 轨迹录制 + 底盘桥接
#  → 你开车把房间走一圈（遥控器，或 ros2 run car_bridge teleop_keys）
bash ~/car_slam_lap/x5/scripts/save_map.sh        # 存设备地图 .bin
bash ~/car_slam_lap/x5/scripts/stop_mapping.sh    # 收尾，落盘
```

产物都在 `~/car_ws/maps/lab1/`：

| 文件 | 是什么 |
| :--- | :--- |
| `odin_map/*.bin` | 设备专有地图，**重定位要用它** |
| `path.yaml` | 地图坐标系下的轨迹，走圈时照着跑 |
| `map.pcd` | 累积点云，拖进 CloudCompare 看房间 |
| `map_bev.png` | 俯视图 + 尺寸标注，一眼核对"建的是不是这个屋子" |
| `map_info.txt` | 点数、包围盒尺寸、点云覆盖面积 |
| `*.log` | 驱动/节点日志 |

> 起点要**贴个标记**：建图从哪儿开始，走圈就得把车放回哪儿、朝哪儿。
> Odin1 的地图原点 = 建图启动时里程计的原点，重定位要求 1 m / ±10° 内。

---

## 7. 走圈

```bash
bash ~/car_slam_lap/x5/scripts/start_relocal.sh lab1   # 加载地图、等 map→odom
bash ~/car_slam_lap/x5/scripts/drive_lap.sh lab1       # 纯追踪跑一圈
```

`start_relocal.sh` 判成功的依据是 TF `map ← imu` 通了（重定位成功前 `map` 帧不在
TF 树里）。成功后：

- `follow_path` 每 50 ms 查一次位姿，算前视点、曲率、角速度，发 `/cmd_vel`
- 到终点会做一次原地朝向修正，然后停
- 结束时打印横向误差 max/rms，并把实际轨迹写到 `lap_actual.yaml`

**第一次一定要先用 `VMAX=0.15` 慢速试**：`VMAX=0.15 bash drive_lap.sh lab1`。
`v_max` 是 m/s，别超过标定出来的 `mps_at_max`。

---

## 8. 安全设计

| 层级 | 机制 | 失效后果 |
| :--- | :--- | :--- |
| L1 遥控器 | `s2` 打到底 = 硬急停，**压过串口** | 任何时刻人肉刹车 |
| L2 固件 | 串口 300 ms 无有效帧 → 发 0 | X5 死机/拔线，车 0.3 s 内停 |
| L3 节点 | `cmd_timeout` 300 ms 无新 `/cmd_vel` → 发 0 | 遥控程序崩了，车停 |
| L4 跟随器 | `/car/status` 报 estop 或断流 → 立即退出 | 底盘异常时不再发指令 |
| L5 跟随器 | TF 丢 >0.5 s → 发 0 并退出 | 定位跳变时不瞎跑 |

急停生效时 `status` 的 b3 置位，`bridge_node` 会打一次 `底盘报 ESTOP` 的红字，
`link_test` 也会把它打出来——**建图/走圈前先确认它不是 `estop:true`**。

---

## 9. 固件还差什么

`protocol.c` 已经写好且测过，剩下的是把它接进 Keil 工程。按这个顺序改：

- [ ] `Core/Src/usart.c` + `C8T6_DT7_car.ioc`：加 **USART3（PB10=TX, PB11=RX，115200 8N1，开 NVIC）**。
      必须同步改 `.ioc`，否则 CubeMX 一重生成就把 USART3 冲掉。
- [ ] 新增 `Hardware/UART/uart_rx.c`：`HAL_UART_RxCpltCallback` 只放这一个（现在在 `DBUS.c` 里），
      按句柄分发：`huart1 → DBUS_OnByte()`、`huart3 → X5_Link_OnByte()`。
      两处都定义回调会直接链接报 `multiply defined`。
- [ ] 新增 `Hardware/LINK/x5_link.c/h`：`HAL_UART_Receive_IT(&huart3,...)`、
      `proto_rx_feed()`、`X5_Link_Alive()`、10 Hz 发 `proto_build_status()`。
- [ ] `Core/Src/main.c`：改成 10 ms 调度 + 仲裁
      （`s2==3` 急停 > 串口 > 遥控 > 停），删掉现在这个**永远不成立**的
      `rc_tick` 超时判断（`rc_tick` 每轮都被刷新了）。
- [ ] `Hardware/CAN/can_bus.c`：把 `while(HAL_CAN_IsTxMessagePending(...))` 去掉
      （现在等于在主循环里以几 kHz 狂发，协议要求 5–20 ms）。
- [ ] `MDK-ARM/C8T6_DT7_car.uvprojx`：`protocol.c` 加进 Hardware 组，新文件同理。
- [ ] 烧写后跑 `link_test` 验验收项 A1/A2。

---

## 10. 验收

| # | 项目 | 判据 | 状态 |
| :--- | :--- | :--- | :--- |
| A1 | 链路 | `link_test` 500+ 帧、`crc_err=0`、status ≈10 Hz、停发后 `LINK_OK` 掉落 | ⬜ 待固件 |
| A2 | 键盘遥控 | 四向正确，松键 ≤300 ms 停，拔 USB 车停 | ⬜ 待固件 |
| A3 | 建图 | 产出 `.bin` + `path.yaml` + `map.pcd`；`map_bev.png` 与房间形状吻合 | ⬜ |
| A4 | 重定位 | `tf2_echo map imu` 有输出 | ⬜ |
| A5 | 走圈 | 跑完一圈无碰撞，横向误差 rms < 0.15 m，终点 < 0.3 m / 15° | ⬜ |
| A6 | 可复现 | 新人照 README 能走完 | ⬜ |

---

## 11. 排查

| 现象 | 先查什么 |
| :--- | :--- |
| `link_test` 一帧上行都没有 | TX/RX 是否交叉、GND 是否共地、固件烧了没、波特率是不是 115200 |
| 有上行但 `crc_err` 一直涨 | 杜邦线太长/接触不良；波特率两边不一致 |
| `bridge_node` 日志 `打开 /dev/car_link 失败` | USB-TTL 没插；`ls /dev/ttyUSB*`；重新 `sudo udevadm trigger` |
| `/car/status` 里 `estop:true` | 遥控器 s2 打在急停档 |
| `/car/status` 里 `src:"rc"` | 串口帧没被认，固件还是走了遥控分支（看 A1） |
| 车往反方向走 | `bridge_node -p invert_speed:=true`（或 `invert_angular`） |
| `cloud_slam` 没数据 | 把 `x5/config/control_command.mapping.yaml` 里 `senddtof: 0` 改回 `1` 再试 |
| 重定位一直不成功 | 起点不在建图位置 1 m 内；场景变化大；灯光太暗；轻轻晃一下设备 |
| `follow_path` 起步就报 TF 丢失 | 重定位没成功（`map` 帧不在 TF 树里） |
| 走圈时画龙/撞墙 | `v_max` 太大或 `mps_at_max` 没标定；先把 `VMAX` 降到 0.15 试 |

---

## 12. 已知不确定 / 待确认

1. **CAN 波特率**：`.ioc` 里算出来是 **250 kbit/s**（36 MHz / 8 / 18TQ），
   但厂商文档写 500K。既然遥控器版实测能跑，本项目按 250K 不动，
   等有空用示波器或 CAN 分析仪确认后统一。
2. **`senddtof: 0`**（关掉 cloud_raw 省 9.6 MB/s）：基于"`sendcloudslam` 是独立数据流"
   的判断。若建图时 `cloud_slam` 无数据，第一件事就是改回 `1`。
3. **Odin1 装车位置**：现在直接把 `imu` 系当车体中心。装得离转向中心远的话，
   转弯时会有系统性偏差，需要用静态 TF 补偏移（`follow_path` 的 `base_frame` 参数）。
4. **重定位成功后 map→odom 的发布频率**：未实测，若 TF 查得太频繁导致抖动，
   把 `follow_path` 的 `rate_hz` 降到 10。

---

## 13. 离线测试（不接硬件就能跑）

```powershell
pwsh -File car_slam_lap\tests\run_tests.ps1
```

跑三件事：gcc 编译 `protocol.c` + `protocol_test.c` 跑 70 项断言；
`test_protocol.py` 11 项、`test_path_io.py` 8 项。协议两侧断言同一组黄金向量，
所以"固件和 X5 各写一遍 CRC 然后对不上"这类问题在接线上车之前就会暴露。
