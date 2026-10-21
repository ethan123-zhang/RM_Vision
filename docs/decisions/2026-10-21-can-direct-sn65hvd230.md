> [!warning] 实测修正 2026-10-21
> 这台控制器 500K 下 `can_ok=0`；把 `can.c` 的 Prescaler 改成 8（250K）后
> `can_ok`/`can_ack` 都出现。所以本车按 **250K** 运行；下文 500K 是 PDF 标称值。

# CAN 链路改回片内 CAN1 + SN65HVD230（500K）

- 日期：2026-10-21
- 状态：已采纳
- 关联：`car_slam_lap/WIRING.md`、`car_slam_lap/firmware/C8T6_DT7_car/Hardware/CAN/`、
  `docs/decisions/2026-10-20-car-slam-lap-route.md`（D1b 作废）

## 背景

2026-10-20 的 D1b 按「C8T6 USART3 → 串口转CAN 模块 → CAN」实现，依据是用户口述
「串口三下发指令给 CAN 收发器」。2026-10-21 用户补了实物资料：

- `CAN收发器原理图.pdf`：板上只有 **SN65HVD230DR**、R1=10K（RS 下拉）、R2=120Ω
  （CANH/CANL 终端）、H1 4 pin、H2 2 pin，**没有 MCU、没有 UART**。
- `CAN收发器说明书.pdf`：TI SN65HVD230 datasheet，3.3V、最高 1 Mbps。
- `双驱无刷电机控制器规格书A4pdf.pdf` 6.3：CAN **500K**、扩展 ID `0x1314`。

原理图坐标/网表确认 H1：1=GND、2=3V3、3=R（收发器输出）、4=D（收发器输入）；
H2：1=CANH、2=CANL。也就是说它是**裸收发器**，只能接 MCU 的 CAN_TX/CAN_RX，
不能用 USART3 驱动。旧方案在物理上不可能工作。

## 选项

| 选项 | 优点 | 缺点 |
| :--- | :--- | :--- |
| A 保留 USART3 + slcan | 不动旧代码 | 硬件根本不支持；再调也不会通 |
| B 片内 CAN1（PA11/PA12）→ SN65HVD230 | 与实物一致；少一层文本协议；实时性最好 | 需要恢复 `can.c`、改 Keil 工程/测试 |
| C 换一块真正的串口转CAN 模块 | 保留 USART3 方案 | 多买一个模块，且当前模块已焊好 |

## 结论

选 **B**：

- `CAN1`：PA12 是 `CAN_TX`，PA11 是 `CAN_RX`；`MX_CAN_Init()` + `CAN_filter_Init()`
  （配置过滤器 → `HAL_CAN_Start` → RX FIFO0 中断）。
- 帧编码独立成 `Hardware/CAN/can_frame.c`（无 HAL，PC 逐字节断言 12 项）：
  `CD AB | angular i16 LE | speed i16 LE | crc i16 LE`，
  `crc = 0xABCD ^ angular ^ speed`。
- `can_bus.c` 非阻塞 `HAL_CAN_AddTxMessage`，10 ms 一拍；邮箱满就跳过，不 `while` 等。
- 过滤器**全收**：控制器 ACK 的 CAN ID 文档未公开，靠 `speedL ^ speedR ^ voltage`
  校验和识别；最近 500 ms 有有效 ACK → status b4 `CAN_ACK`。
- 波特率按 PDF 的 **500K**（`Prescaler=4`，36 MHz / 4 / 18TQ）。旧 `.ioc` 是 250K，
  若首次上车 `can_ok` 有、`can_ack` 没有，先把 Prescaler 改回 8 再试。
- USART3 保留为备用/调试口（PB10/PB11），不接电机。

## 理由

- 实物是 3.3V 收发器，PA11/PA12 正好是 F103 的 CAN1 复用脚，`.ioc` 里也一直保留着
  `CAN` 外设和 `USB_LP_CAN1_RX0_IRQn`；恢复成本最低。
- 文本 slcan 多一层编码/解码，且这次是**用错硬件**，不是协议选型问题。
- ACK 是唯一能证明「CAN 总线真的通了」的反馈；`CAN_OK` 只证明帧进了邮箱。

## 影响

- 固件：新增 `can_frame.c/h`；重写 `can_bus.c/h`；`can.c` 改 500K + Start；
  `main.c` 调 `MX_CAN_Init`/`CAN_Bus_Init(&hcan)`；`uart_rx.c` 的 USART3 改为备用。
- 工程：`uvprojx` 用 `can_frame.c` 替换 `slcan.c`；`slcan.c/h` 与 `slcan_test.c` 删除；
  `check_keil_project.py` 增加 `.ioc` 的 CAN 500K/PA11/PA12 检查。
- 上位机：`protocol.py` 增加 `ST_CAN_ACK=0x10`；`bridge_node` 在 `can_ok` 但 2 s
  无 ACK 时告警。
- 文档：新增 `car_slam_lap/WIRING.md`；README 拓扑/接线/协议/排查全部改写。
- 验收：A1（X5↔C8T6）不受影响；A2 前先看 `/car/status` 是否出现 `can_ack`。

## 复审条件

1. 首次上电 `can_ok` 有、`can_ack` 无 → 先切 250K，再查 CANH/CANL、终端、PowerKey。
2. 控制器实际是独立模式（把 angular/speed 当左右轮）→ 找厂家确认模式切换方式。
3. 若以后换成真正的串口转CAN 模块，再评估是否恢复 USART3 方案（旧代码在 git 历史里）。
