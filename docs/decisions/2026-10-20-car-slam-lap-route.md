# 小车底盘控制源换成 X5 串口，并用 Odin1 做建图+走圈

- 日期：2026-10-20
- 状态：X5 侧已实现并离线验证；固件 HAL 部分待接线确认
- 相关代码：`car_slam_lap/`

## 1. 背景

RDK X5 + Odin1 的感知链路已跑通（见 `x5_odin/README.md`），手上有现成的 C8T6
双驱底盘（XHSMCUW50 控制器，扩展帧 `0x1314` MIX 模式，原用 DT7 遥控器）。
目标：先用这套东西跑通 **房间建图 → 重定位 → 小车自己走一圈**，作为调比赛机器人的前置练习。

## 2. 决策

### D1 控制源：X5 串口 → C8T6 → CAN，遥控器保留

- **选择**：X5 通过 USB-TTL 接 **C8T6 USART3（PB10/PB11，115200 8N1）**，C8T6 收到指令后
  转成原有 `0x1314` CAN 帧；USART1 仍接 DT7 接收机，USART2 保留 printf 调试。
- **为什么不是 USART1**：USART1 是 DT7 的 100000 8E1 专用时序，且"保留遥控器"要求它继续工作。
- **仲裁**：`s2==3` 硬急停 > 串口（300 ms 内有帧）> 遥控器（100 ms 内有帧）> 发 0。
  遥控器留作建图时的手动驾驶和随时急停，这是练习阶段最实用的安全兜底。
- **放弃的方案**：X5 直连 CAN（需要 USB-CAN 适配器，且要自己实现 0x1314 帧与总线仲裁）；
  完全去掉遥控器（省钱省事，但建图时必须靠键盘远程开车，且丢了最后一道人肉刹车）。

### D2 建图与走圈：用 Odin1 原生 SLAM，不引 FAST-LIO2 / nav2

- **选择**：`custom_map_mode=1` 建图（回环 + `set_param.sh save_map`）、`=2` 重定位
  （加载 `.bin`，成功后才发 map→odom）；走圈用"建图时录轨迹 → 重定位后纯追踪回放"。
- **为什么**：Odin1 驱动自带 SLAM、存图、重定位，`/odin1/odometry` + `/odin1/cloud_slam`
  直接可用。自建 LIO 要处理外参、时间同步、标定，属另一个量级的工作量；
  nav2 需要 2D 激光或可靠的占据栅格，X5 只有 3 GB 内存、Odin1 点云 10 Hz，
  调参风险远大于收益。
- **代价**：路点靠"示教"，不是任意目标点导航；换布局要重录。练习阶段可接受。
- **复审条件**：需要"指哪去哪"时再上占据栅格 + 全局规划。

### D3 链路协议：二进制帧 + CRC8，字段与黄金向量固化

- 下行 9 字节 `A5 5A 05 01 speed(i16LE) angular(i16LE) CRC8`，50 Hz；
  上行 7 字节 `5A A5 03 81 status src CRC8`，10 Hz。CRC-8/ATM（poly 0x07）。
- **为什么不用文本协议**：文本上线快，但长度/校验字段是后续接云台、发射机构时的必需品，
  且这套结构与 RoboMaster 常见串口协议同构，练手价值更高。可调试性用
  `bridge_node -p dry_run:=true` 打 hex 补回来。
- **协议实现放在 HAL 之外**（`firmware/.../Hardware/LINK/protocol.c`），
  在 PC 上用 gcc 跑 70 项断言；Python 侧断言同一组向量，避免"两边各写一遍 CRC"。

### D4 `/cmd_vel` 用真实物理单位，靠实测标定

- 底盘只认 ±1000 无量纲指令。若直接拍一个 `mps_at_max`，纯追踪算出的曲率与实际不符，
  弯道会撞墙。因此提供 `calibrate_speed`：发固定指令跑 4 s，用 Odin1 里程计量实际
  位移/转角，反推"打满 1000 对应多少 m/s、rad/s"。
- **复审条件**：换电机/换轮胎/换电池电压后重新标定。

## 3. 过程中确认的事实

- **CAN 实际是 250 kbit/s**，不是厂商文档写的 500K：`.ioc` 里
  `CalculateBaudRate=250000`（36 MHz / 8 / (1+9+8)）。遥控器版实测能跑，故不动。
- **Odin1 的 TF 链是 `odom → imu → lidar`，`map` 是 `odom` 的子帧**，没有
  `odin1_base_link`（README FAQ 里的旧说法属于上一版驱动）。`/odin1/odometry` 的
  `child_frame_id` 是 `imu`，`cloud_slam` 在 `odom` 系。因此车体帧用 `imu`，
  点云累积默认在 `odom` 系（建图模式下 map 与 odom 同原点，不需要查 TF）。

## 4. 发现的既有固件缺陷（待随固件改造一并修）

| 位置 | 问题 | 后果 |
| :--- | :--- | :--- |
| `main.c` | `rc_tick = HAL_GetTick()` 每轮刷新，随后的 `HAL_GetTick() - rc_tick > 100` 永远为假 | 遥控器失联保护形同虚设 |
| `can_bus.c` | `CAN_Send()` 里 `while(HAL_CAN_IsTxMessagePending(...))`，主循环又无延时 | 实际以几 kHz 发帧，与协议要求的 5–20 ms 周期不符 |
| `DBUS.c` | `HAL_UART_RxCpltCallback` 定义在这里，再加 USART3 会重复定义 | 链接报 multiply defined |

## 5. 影响

- 新增 `car_slam_lap/`（固件工程 + ROS2 包 + 脚本 + 文档），原 `practice/` 目录已废弃。
- 板端新增 `~/car_ws`（colcon 工作区）、`/dev/car_link`（udev 固定串口名）、
  `~/car_ws/maps/<名字>/`（每次建图的产物）。
- `x5_odin/` 那套驱动部署不变，本方案只是给它换配置（`-p config_file:=`），
  不覆盖出厂 `control_command.yaml`。

## 6. 复审条件

1. 固件烧写完成、`link_test` 通过后，复核仲裁优先级是否符合实车手感（尤其 `s2` 档位语义）。
2. 若重定位成功率低，考虑改用 Odometry 模式（mode 0）+ 同会话回放。
3. `senddtof: 0` 若导致 `cloud_slam` 无数据，改回 1（性能换稳定）。
