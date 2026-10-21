# 小车底盘控制源换成 X5 串口，并用 Odin1 做建图+走圈

> [!warning] 修正 2026-10-21（来源：`CAN收发器原理图.pdf`）
> 实物是 **SN65HVD230 裸收发器**（H1: GND/3V3/R/D，H2: CANH/CANL），不是串口转CAN
> 模块。D1/D1b 里的「USART3 + slcan」作废，已改回片内 CAN1（PA11/PA12）+ 500K。
> 见 `docs/decisions/2026-10-21-can-direct-sn65hvd230.md` 与 `car_slam_lap/WIRING.md`。

- 日期：2026-10-20
- 状态：X5 侧已实现并离线验证；固件 HAL 部分待接线确认
- 相关代码：`car_slam_lap/`

## 1. 背景

RDK X5 + Odin1 的感知链路已跑通（见 `x5_odin/README.md`），手上有现成的 C8T6
双驱底盘（XHSMCUW50 控制器，扩展帧 `0x1314` MIX 模式，原用 DT7 遥控器）。
目标：先用这套东西跑通 **房间建图 → 重定位 → 小车自己走一圈**，作为调比赛机器人的前置练习。

## 2. 决策

### D1 控制源：X5 → C8T6 USART1，遥控器留在 USART2，电机走 USART3 的 slcan 模块

- **串口分配**（按实物接线，2026-10-20 修正）：USART1(PA9/PA10, 115200 8N1) = X5 40pin
  第 8/10 脚；USART2(PA2/PA3, 100000 8E1) = DT7 接收机；USART3(PB10/PB11, 115200 8N1) =
  串口转CAN 模块。
- **第一版设计错在哪**：曾按"遥控器在 USART1、X5 在 USART3、printf 在 USART2"设计，
  与实物不符。教训：串口分配要以**实物接线和用户说明**为准，不能从代码里的 `HAL_UART_*`
  调用反推——代码可能比接线旧。
- **电机指令改走外挂模块**：C8T6 不再初始化片内 CAN，改用 slcan 文本帧发给
  "串口转CAN 模块"，由模块上总线。好处是换板子不用焊 TJA1050；代价是多一层文本协议、
  且模块上电顺序变得敏感（模块晚启动不会自动重开通道）。
- **仲裁**：`s2==3` 硬急停 > 串口（300 ms 内有帧）> 遥控器（100 ms 内有帧）> 发 0。
  串口优先而不是遥控器优先：跑建图/走圈时人松手，摇杆回中会持续抢控制权。
- **放弃的方案**：X5 直连 CAN（仍需收发器）；完全去掉遥控器（丢了最后一道人肉刹车）。

### D1b 电机链路用 slcan 文本，不用自定义二进制

- **选择**：C8T6 用 `T<8位hex ID><1位hex DLC><16位hex 数据>\r` 发给模块，
  启动时发 `S5\r`(250K) + `O\r`(开通道)。
- **为什么**：slcan 是 CANable/candleLight 的既成事实标准，模块厂家普遍支持，
  且可读性极好——抓串口原始字节就能肉眼看出 ID 和数据对不对。
- **代价**：26 字节/帧 @10 ms ≈ 26 kbit/s，阻塞发送约 2.3 ms，对 10 ms 周期可接受；
  如果以后要提频，改 DMA 发送即可。
- **验证**：编码层 `Hardware/CAN/slcan.c` 与 HAL 无关，PC 上 21 项逐字节断言
  （`tests/slcan_test.c`）。这类文本协议错一个字符不会崩，只会静默丢帧。

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
  `CalculateBaudRate=250000`（36 MHz / 8 / (1+9+8)）。现在这个值通过 slcan 的
  `S5` 命令配置给模块；若实测电机不动而帧格式无误，第一个要怀疑的就是这里。
- **Odin1 的 TF 链是 `odom → imu → lidar`，`map` 是 `odom` 的子帧**，没有
  `odin1_base_link`（README FAQ 里的旧说法属于上一版驱动）。`/odin1/odometry` 的
  `child_frame_id` 是 `imu`，`cloud_slam` 在 `odom` 系。因此车体帧用 `imu`，
  点云累积默认在 `odom` 系（建图模式下 map 与 odom 同原点，不需要查 TF）。
- **RDK X5 40pin 第 8 脚 = UART_TX、第 10 脚 = UART_RX，节点 `/dev/ttyS1`**，3.3V 电平
  （D-Robotics 官方 UART 案例）。所以 X5 侧不需要 USB-TTL。

## 4. 原有固件的三处缺陷（已随本次改造修掉）

| 位置 | 问题 | 后果 | 处理 |
| :--- | :--- | :--- | :--- |
| `main.c` | `rc_tick = HAL_GetTick()` 每轮刷新，随后的判断永远为假 | 遥控器失联保护形同虚设 | 改为按 `DBUS_Alive()` 判活 |
| `can_bus.c` | `CAN_Send()` 里 `while(HAL_CAN_IsTxMessagePending(...))`，主循环无延时 | 几 kHz 狂发，违反 5–20 ms 周期 | 改为 10 ms 调度 + 单次发送 |
| `DBUS.c` | `HAL_UART_RxCpltCallback` 定义在业务文件里 | 加第二个串口就 multiply defined | 收敛到 `uart_rx.c` 统一分发 |

新增的一条同类风险：`HAL_UART_Receive_IT` 在溢出/帧错误后会停摆，所以补了
`HAL_UART_ErrorCallback` 重新挂接收——否则某个口会"静默死亡"，且没有任何报错。

## 5. 影响

- 新增 `car_slam_lap/`（固件工程 + ROS2 包 + 脚本 + 文档），原 `practice/` 目录已废弃。
- 板端新增 `~/car_ws`（colcon 工作区）、`/dev/car_link`（udev 软链，指向 ttyS1 或 USB-TTL）、
  `~/car_ws/maps/<名字>/`（每次建图的产物）。
- `x5_odin/` 那套驱动部署不变，本方案只是给它换配置（`-p config_file:=`），
  不覆盖出厂 `control_command.yaml`。

## 6. 复审条件

1. 固件烧写完成、`link_test` 通过后，复核仲裁优先级是否符合实车手感（尤其 `s2` 档位语义）。
2. 若电机不动而 USART3 上确实有正确的 slcan 帧，依次怀疑：模块波特率、模块 CAN 侧波特率、
   模块是否处于开通道状态（`O`）、上电顺序。
3. 若模块重启导致通道丢失，加"周期性重发 `O\r`"。
4. 若重定位成功率低，考虑改用 Odometry 模式（mode 0）+ 同会话回放。
5. `senddtof: 0` 若导致 `cloud_slam` 无数据，改回 1（性能换稳定）。
