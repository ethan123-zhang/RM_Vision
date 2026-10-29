# 项目进度

> 唯一“现在进行时”。每次会话结束更新；历史细节放 `docs/issues/` 与 `docs/decisions/`。
> 最后更新：2026-10-29

## 当前接续（优先于下方 10 月 26 日的历史状态）

- 比赛方向：高校联盟赛 3V3，先做重装机器人；当前主线是 C 板四轮全向底盘。
- 电调 ID 已人工确认：右前1、左前2、左后3、右后4，1/2 之间为车头。
  四轮正转从车外侧看轮心均为顺时针；方向表尚不据此修改，整车解算前须结合安装角度核对。
- C 板已固定在右侧1/4之间，使用板载 IMU，安装轴向待确认。
- 新增单轮速度模式3：KEY选轮，±600转子rpm正反阶跃和零速段，限流±1500，
  按键取消、掉线取消、超速取消，测试结束释放。详见 `cboard_chassis/README.md` 顶部。
- 主机速度测试通过，原映射/按键263项检查通过，Keil AC5实编译0错误0警告。
  用户已上板验证四轮正反转、KEY取消；2026-10-29直接CAN采集90秒、410146帧，四轮各一次完整测试。
  **功能验证通过，第一阶段定量验收未完成**：1/2/3号末段平均欠速，4号正转曾受外力。
  详细误差、曲线和3431条遥测见 `docs/experiments/2026-10-29-single-wheel/README.md`。
- 下一步：先录无外部接触基线，必要时延长恒速段区分瞬态与稳态，再逐项整定PI与滤波。
  之后基础运动→IMU姿态/解耦→功率保护→融合里程计。本次采集未修改固件参数。
- 已确认后续实际文件改动经检查后按明确路径提交推送，避免混入原有未跟踪资料。

## 一句话状态

RDK X5 单类 person YOLOv8n 定稿：**576 + 三段流水线 + C++ 后处理 = 104.1 FPS**，
person AP 0.7178（640 为 0.7244）。切尾巴、量化、板端测速全部跑通。
2026-10-20 原 SD 卡损坏 → 重灌镜像 + 板端 30 min 重建，Odin1 链路已恢复（10 Hz 数据、网页预览经端口转发在 PC 上验证）。
另起 `car_slam_lap/`：把这套感知装到 C8T6 双驱小车上，跑「建图 → 重定位 → 走一圈」。
X5 侧、链路协议、固件改造都已完成，**Keil AC5 实编译通过（0 error / 0 warning，Code 10.9 KB）**。
2026-10-21 按实物原理图修正：CAN 是 SN65HVD230 裸收发器（不是串口转CAN模块），
电机链路已改回片内 CAN1 + 250K（实测）；**X5 键盘控制 + CAN 已通**。
2026-10-22 遥控器链路已打通并实车验证：本款接收机是 **18 字节通道数据、帧首没有 `0xA9`**
（原来死等帧头，字节全被丢）。固件改成按帧间空闲定界，三档映射实测定稿
（GPS=全速 / 中间=1/3 速 / 下面=急停），`rc_ok` 稳定、遥控能开车。下一步用遥控器给全屋建图。

2026-10-26 **`car_slam_lap` 端到端跑通**：遥控示教建图（venue2，330 点 / 15.66 m）→ 重定位
（`map←imu` 与录制末点差 **3 cm / 0.6°**）→ 纯追踪自动回放（249 点 / 14.19 m，横向误差
**rms 0.103 m**、无碰撞）。同日修掉三个“伪装成硬件问题”的软件 bug：Odin1 驱动**不读**
`-p config_file`（重定位配置从未生效）、`save_map.sh` 找错存图目录且不等传输完成、
`follow_path` 启动竞态 / 名称遮蔽 / SIGINT 处理。详见
`docs/issues/2026-10-26-odin-config-and-follow-path.md`。

**2026-10-23 起换 C 板做底盘**：`cboard_chassis/`（官方 `17.chassis_task` 原样 + 调车层）。
本赛季底盘是 **4 轮全向（45° X 型），不是麦轮**——解算符号与麦轮相同、只差 1/√2，
官方公式可直接用。C620 的 CAN ID 仍是 1/2/3/4，但**哪个 ID 装在哪个角、正反如何，尚未标定**。
2026-10-26 **固件已烧进 C 板**（Keil F8，`Verify OK / Application running`），
现在进入 **阶段 0（按键验 ID）**：按 C 板 KEY N 次、停手 3 s，驱动 N 号电调 1.5 s。
下一步就是上电按 KEY 核对四个电调的编号。

## 里程碑

| # | 里程碑 | 状态 | 说明 |
| :--- | :--- | :--- | :--- |
| 1 | 仓库与协作规则初始化 | ✅ 完成 | 2026-10-17，首次推送 |
| 2 | 通读三份 RM2027 手册，产出视觉相关约束清单 | ⬜ 待开始 | 文本已提取到 `_extract/` |
| 3 | 确定赛季视觉目标与量化指标 | ⬜ 待开始 | 依赖里程碑 2 |
| 4 | RDK X5 环境跑通（相机采集 + BPU 推理） | ⬜ 待开始 | 镜像已备 |
| 5 | 单类 person YOLOv8n 切尾巴 + X5 .bin | ✅ 完成 | 2026-10-17，AP 0.78 不变、量化 cosine ≥0.997 |
| 6 | X5 板端 512 输入达 100+ FPS | ✅ 完成 | 2026-10-18，C++ 后处理 135.3 FPS |
| 7 | 576 输入定稿（精度/速度折中） | ✅ 完成 | 2026-10-18，104.1 FPS、AP 0.7178 |
| 8 | 坏卡重灌 + Odin1 数据链路恢复 | ✅ 完成 | 2026-10-20，镜像校验一致、30 min 重建、端口转发验证 |
| 9 | 小车底盘串口化 + 建图走圈（练习） | ✅ 完成 | 2026-10-26 端到端跑通：遥控示教建图（venue2：330 点 / 15.66 m）→ 重定位（`map←imu` 与录制末点差 3 cm / 0.6°）→ 纯追踪自动回放（249 点 / 14.19 m，横向误差 rms 0.103 m、无碰撞）。修掉 Odin1 `config_file` 不生效 / `save_map.sh` 找错目录且不等传输 / `follow_path` 三处 bug，详见 `docs/issues/2026-10-26-odin-config-and-follow-path.md` |
| 10 | C 板底盘（全向轮）首次上电与标定 | 🔶 进行中 | `cboard_chassis/`：官方 `17.chassis_task` 原样 + 调车层（key/map/scan/debug）。编译 0/0、Code 64.4 KB；纯逻辑 263 断言通过；**2026-10-26 已烧进 C 板**（Keil F8 `Verify OK`）。**当前阶段 0 按键验 ID** → 阶段 1 扫描填 `CHASSIS_WHEEL_CAN_ID_*` / `CHASSIS_WHEEL_DIR_*` → 阶段 2 遥控通道 → 阶段 3 整车 |

## 正在做

- **C 板底盘（当前主线）**：`cboard_chassis/`，**固件已烧进板子**，车已架起、电调已重编号。
  下一步（下个对话从这里开始）：**上电按 KEY 核对编号**。
  1. **阶段 0（默认模式）**：车轮离地，上电 → RGB 灯流动。
     按 KEY 1 次、停手 3 s → 应只有 **1 号电调**转 1.5 s；按 2 次 → 2 号，依此类推。
     蜂鸣器每按一次"嘀"一声；USB-CAN 分析仪看 `0x1F1[7]` = 当前计数、`0x1F1[5]` 应为 2。
  2. 记下"按 1/2/3/4 次分别转的是哪个角、正转时轮子顶部往车头还是车尾走"，
     填 `chassis_bringup_map.h` 的两张表（`CHASSIS_WHEEL_CAN_ID_*` / `CHASSIS_WHEEL_DIR_*`）。
  3. **阶段 1**：改 `CHASSIS_BRINGUP_MODE = SWEEP` 重烧（Keil F8），看 ID 1~4 依次正反转复核。
  4. **阶段 2**：改 `NORMAL` 重烧，S1 拨下（不动），用 `0x1F1` 确认四个在线 + 遥控通道。
  5. **阶段 3**：S1 拨中整车。
  判据、故障对照、烧写方式全在 `cboard_chassis/README.md` §2/§6/§7。
- 576 定稿；接入 USB 摄像头做端到端实测（720p60 / 480p120），之后换正式相机。
- Odin1 链路 2026-10-20 已恢复：板端起驱动 + 网页预览，PC 经 `plink -L 8080` 端口转发查看
  （`x5_odin/`，启动/打包/校验脚本齐全）。
- `car_slam_lap/`：三条链路都写完了 —— 协议（C+Python 同源，70+12 项断言）、`car_bridge`
  ROS2 包（桥接/键盘遥控/录轨迹/纯追踪/点云导出/速度标定）、固件（USART1=X5、
  USART2=遥控器、**CAN1=SN65HVD230 250K（实测）**、10 ms 仲裁、看门狗）、板端脚本与 Odin1 两份配置。
  自检五组全绿，其中第 5 组直接调 `UV4.exe` 真编译（Code 11152 / RAM 1408）。
  **2026-10-26 已端到端跑通**（建图 venue2 → 重定位 → 自动回放，rms 0.103 m）。接下来可选：
  - 掉头切角调参：max 误差 0.423 m 出现在远端 180° 掉头（纯追踪切角），降 `VMAX` / `lookahead_min`；
  - 用这套流程给**正式场地**建图：走完整环线、回到起点（<0.5 m）、**最后再** `save_map`；
  - 长期解：给 `bridge_node` 加 `passive:=true`（只收 status、不发帧），建图时遥控和状态可共存。
  X5 侧已部署，`/dev/car_link -> ttyS1` 已确认；键盘 `teleop_keys` 也能开车。

## 下一步（按优先级）

1. 通读 `_extract/rules.txt`、`entry.txt`、`build.txt`，产出《视觉相关规则约束清单》：目标类别、场地尺寸、分辨率/帧率限制、裁判系统接口、违规条款。
2. 依据约束清单确定本赛季视觉任务（检测/跟踪/测距等，以规则为准）与指标（mAP、端到端延迟、帧率、功耗）。
3. 在 RDK X5 上验证相机采集与 BPU 推理的最小链路，记录环境指纹与基线耗时。
4. 把可复用的量化/部署结论沉淀到 aiVault（按 `docs/conventions.md` §6）。

## 阻塞 / 风险

- **烧写只能用 Keil 界面按 F8**：`UV4 -f` 命令行在这台破解版 Keil 上必崩（`0xC0000409`），
  VS Code 只负责编译（已封成任务）。换电脑/重装 Keil 后需重新确认。
- **STM32CubeProgrammer 禁用**（2026-10-26 决定）：它与这颗 ST-Link 建立不了 SWD，
  但会报出 SN/FW/电压，看起来像"芯片没应答"，已把排查带偏一整轮。不要再用它烧录，
  也不要用它的输出判断硬件好坏。详见 `docs/issues/2026-10-26-cboard-flash-toolchain.md`。
- Keil 的 `Debug` 选项卡必须是 **ST-Link Debugger**（不是 ULINK2/ME），否则 F8 报
  `No ULINK2/ME Device found`；配置存在 `MDK-ARM/chassis_task.uvoptx`（用户级，换机要重设）。
- C 板底盘：**编号核对中**，C620 的 ID→轮位、轮子正反仍是未知数，默认表（1/2/3/4 + 全 +1）
  大概率需要改；改之前整车不会按预期方向走。
- 按键验 ID 只覆盖 1~4 号（`0x200` 一帧）；若实际编号是 5~8，需要再补 `0x1FF` 那条发送路径。
- 全向轮是否为 **45° X 型**未验证（手推侧面能平移才是；若是平行装法，解算要另写）。
- 底盘 `vx/vy/wz` 目前不是真实物理量（全向轮比麦轮快 1.41 倍），接 X5 导航前必须统一单位。
- RDK X5 镜像约 9.68 GiB 未入库：换机需按 `docs/environment.md` 重新获取并校验 SHA256。
- 规则为 RM2027 V1.0.0（20261015），后续版本可能调整，需定期核对。
- 相机型号、SDK/工具链版本尚未记录，补齐前实验不可复现。

## 最近变更

- 2026-10-26：**`car_slam_lap` 端到端跑通（建图 → 重定位 → 自动回放）**。venue2 实测：
  路径 330 点 / 15.66 m；重定位 `map←imu` 与录制末点差 **3 cm / 0.6°**；回放 249 点 /
  14.19 m / 167.6 s，横向误差 **rms 0.103 m**、无碰撞。修掉三处软件 bug：
  ① Odin1 驱动**不读** `-p config_file`（源码写死读
  `~/odin_ws/src/odin_ros_driver/config/control_command.yaml`），此前重定位从未真正进入
  mode 2；② `save_map.sh` 只看 `~/odin_ws` 且等 8 s 就拷（实际图存到 `~/car_ws/src/...`，
  传输是异步的）；③ `follow_path` 启动竞态、`save_actual` 名称遮蔽、SIGINT 抛
  `RuntimeError`。修复与验证见 `docs/issues/2026-10-26-odin-config-and-follow-path.md`。
  待改进：远端 180° 掉头切角（max 0.423 m），终点位置差 0.48 m 略超 A5 判据。
- 2026-10-26：**验 ID 的轮速调慢**（实测 +2500 空载约 3 A，轮子转太快、看不清是哪个角）。
  `CHASSIS_KEY_SPIN_CURRENT` 2500 → **900**（约 1.1 A），时长仍 1.5 s；
  阶段 1 的 `CHASSIS_SWEEP_CURRENT` 3000 → **1000**。两处都只改电流值，时序、状态机、
  263 项断言不变。转速基本正比于电流，**低于 ~500 会顶不过静摩擦**（只嗡嗡响不转，
  不是电机坏了）；调法写进 `cboard_chassis/README.md` §2「转得太快 / 想再慢一点」。
- 2026-10-26：**C 板固件烧进去了**（Keil F8，`Erase Done / Programming Done / Verify OK / Application running`）。
  烧写这条路上踩了三个独立的坑，全部记进 `docs/issues/2026-10-26-cboard-flash-toolchain.md`：
  ① 缺 `Keil.STM32F4xx_DFP` → `UV4 -f` 静默无动作；
  ② `Debug` 选项卡选成了 `ULINK2/ME` → F8 报 `No ULINK2/ME Device found`（Utilities 显示
  `-- Use Debug Driver --` 时烧写跟随 Debug 页）；
  ③ **STM32CubeProgrammer 在这台机器上连不上 ST-Link**，却照样报出 SN/FW/电压，
  看起来像芯片没应答，把排查方向带偏了一整轮 —— **以后禁用**。
  另外破解版 Keil 的 `UV4 -f` 批处理必崩（`0xC0000409`），所以最终定下：
  **VS Code 编译（`Ctrl+Shift+B`）+ Keil 界面 F8 烧写**，任务已封装好。
  下一步：上电按 KEY 核对四个电调的编号。
- 2026-10-25：**新增阶段 0「按键验 ID」**（当前默认模式）。底盘已架起、四个电调重新编号，
  需要核对"电调里设的编号"和"实际是哪一路"。做法：按 C 板 KEY（PA0，内部上拉、按下为低）
  N 次，**最后一次之后静默 3 s** 就驱动 N 号电调 1.5 s（+2500）；按超 4 次清零不驱动；
  转动中按键不打断但会记账。每按一次蜂鸣器"嘀"一声（调车模式下 `test_task` 已静音，
  否则它会对永远掉线的 yaw/pitch/trigger 一直报警），`0x1F1[7]` 同时显示计数。
  3 s 判定写成纯状态机，`tests/` 里精确卡到毫秒边界（含连按重置、转动中按键、超界），
  263 项断言通过；Keil 三种模式均 0 Error / 0 Warning。详见 `cboard_chassis/README.md` §2 阶段 0。
- 2026-10-23：**C 板底盘调车层完成（待上电）**。基线 `17.chassis_task` 原样编译 0/0
  （Code 63192），说明框架本身没问题；问题在于第一次上电时**看不见哪一步没通**。
  新增 `cboard_chassis/`：官方文件只插 5 处 `[BRINGUP]`，另加 3 个文件——
  单电机扫描（不依赖遥控器/陀螺仪，按裸 ID 1~4 正反转）、轮位↔CAN ID↔方向两张标定表、
  `0x1F0`/`0x1F1` 调试帧（USB-CAN 分析仪直接可读）、启动死等改 3 s 超时。
  底盘改**全向轮**后仍可用官方解算（符号与麦轮相同，只差 1/√2，给定 1 m/s 实际约 1.41 m/s）。
  纯逻辑自测 171 断言（恒等/打乱/重复 ID/越界四套表）通过；Keil 实编译 0 Error / 0 Warning，
  Code 64424。详见 `cboard_chassis/README.md`、`docs/decisions/2026-10-23-cboard-chassis-bringup.md`。
- 2026-10-22：**遥控器 DBUS 定位并修好（待烧写验证）**。现象是 `/car/status`
  `status=51`（`can_ok|link_ok|can_ack|uart2_rx`）、无 `rc_ok`/`dbus_frame`。
  抓包 `_extract/ttl.bin`（1026 B / 100000 8E1）周期严格 18 字节、`A9` 出现 **0 次**，
  按位布局解出来是 `ch=0,0,0,0, s1=1, s2=3`（摇杆居中、s2 急停档）——数据本身没问题，
  是**这一款接收机的帧首没有 `0xA9`**，而固件死等帧头，每个字节都被丢。
  修复：`DBUS.c` 改成按帧间空闲（>2 ms）定界，兼容 18 字节无帧头 / 19 字节带帧头；
  加保留字节校验防噪声假帧；新增 `tests/dbus_frame_test.c`（46 项），
  `run_tests.ps1` 五组全绿，Keil 实编译 0 Error / 0 Warning（Code 11152）。
  补记：第一版还要求 s1/s2 必须为 1/2/3，但这台遥控器 OFF 档编码就是 0，
  切到 OFF 后合法帧被误杀（status 127→115、`rc_ok` 消失）；已允许 s1/s2=0。
  另把 `uart2_rx` 从锁存位改成 **500 ms 实时位**，接收机中途停发时不再被旧帧掩盖。
  S2 映射按用户要求改为 **GPS(1)=全速 / 中间(3)=三分之一速 / 下面(2)=急停**
  （`DBUS_ESTOP_S2=2`，`gain` 判 3）。
  详见 `docs/issues/2026-10-22-rc-dbus-not-decoded.md`。
- 2026-10-21：**CAN 也通了（实测 250K）**：`/car/status` → `status=19`、
  `flags=["can_ok","link_ok","can_ack"]`、`src="serial"`、`crc_errors=0`；复位后轮子能转。
  踩坑：① PA9/PA10 插错 + 烧完没断电复位 → `/dev/ttyS1` 0 字节；
  ② 500K 下 `can_ok=0`，改 `can.c` Prescaler=8（250K）后 `can_ok`/`can_ack` 都出现。
  下一步：离地验方向、`calibrate_speed` 标定、建图走圈。
- 2026-10-21：**X5 侧已部署到 172.17.90.239**（`deploy.ps1 -Install`，colcon build 通过，
  `can_ack` 代码已在板端，`/dev/car_link -> ttyS1`）。C8T6 还需重烧
  `MDK-ARM/C8T6_CAN_car/C8T6_CAN_car.hex`，否则板上仍跑旧的 USART3/slcan 固件。
- 2026-10-21：**接线拓扑再修正一次：CAN 是 SN65HVD230 裸收发器，不是串口转CAN模块。**
  用户补了 `CAN收发器原理图.pdf` / `CAN收发器说明书.pdf`：板子只有 SN65HVD230 + 120Ω
  + 10K，H1 = GND/3V3/R/D，H2 = CANH/CANL，没有 MCU、没有 UART。原来的
  「USART3 → slcan → 模块」物理上不可能工作。已改回 **片内 CAN1（PA11/PA12）**：
  `can_frame.c` 纯帧编码 + `can_bus.c` 非阻塞发送 + ACK 解析（status b4）；
  `can.c`/`.ioc` 设 250K（实测；PDF 写 500K）；`slcan.c/h` 与测试删除；Keil 实编译 0 error / 0 warning。
  接线、上电顺序、终端电阻、控制器 PowerKey/Break 见 `car_slam_lap/WIRING.md`。
  决策记录：`docs/decisions/2026-10-21-can-direct-sn65hvd230.md`。
- 2026-10-20：**小车串口链路通了（验收 A1 通过）**。RDK(172.17.90.239) 上部署 `car_bridge` 并
  跑通 `link_test`：300 帧发 / 70 帧收、**10.00 Hz**、最大间隔 101 ms、`crc_err=0`、
  仲裁 `src=serial`、停发后 `LINK_OK` 正确掉落（300 ms 看门狗真实有效）。
  排障过程：① `ros2` 找不到 → 每个 shell 要 source；② `rm@local` 邮箱非法导致 colcon 解析
  manifest 失败、包装上但 `ros2 pkg` 找不到；③ 一帧上行都没有 → **串口 ISP 烧写后 BOOT0 忘了
  拨回 0**，芯片跑的是系统 bootloader，固件根本没执行。另外确认 RDK 40pin 串口确实是
  `/dev/ttyS1`（`pinctrl-handles` 里 `34070000.serial` 绑定 `uart1grp`），而 `ttyS5` 是蓝牙
  （`hciattach` 占着，写它会阻塞）。这几条已写进 `car_slam_lap/README.md` §11。

- 2026-10-20：新增 `car_slam_lap/`（原 `practice/`）——把 Odin1 + X5 装到 C8T6 双驱小车上。
  控制源从 DT7 遥控器改为 **X5 串口 → C8T6 → CAN**，遥控器保留作手动驾驶与硬急停。
  协议：9 字节下行 / 7 字节上行、CRC-8/ATM、50 Hz/10 Hz，C 与 Python 两侧断言同一组黄金向量
  （`tests/run_tests.ps1` 全绿）。建图/走圈走 Odin1 原生 SLAM（mode 1）+ 重定位（mode 2）+
  录路点纯追踪回放，不引 FAST-LIO2/nav2。顺带确认 **CAN 实际是 250 kbit/s**（文档写 500K 有误）、
  驱动 TF 链是 `odom→imu→lidar`（无 `odin1_base_link`）。决策见
  `docs/decisions/2026-10-20-car-slam-lap-route.md`。
  固件改动清单已列。
- 2026-10-20：**修正 `car_slam_lap/` 的链路拓扑**（第一版按代码反推，与实物不符）。
  实际是：USART1(PA9/PA10, 115200 8N1) ← RDK X5 40pin 第 8/10 脚（节点 `/dev/ttyS1`，
  官方文档确认）；USART2(PA2/PA3, 100000 8E1) ← DT7 接收机；USART3(PB10/PB11) → **外挂
  「串口转CAN 模块」**（slcan 文本，`T000013148…\r`，启动发 `S5`+`O`）→ 250K CAN。
  固件随之完成：`uart_rx.c` 统一接收中断、`x5_link.c`、`slcan.c`、10 ms 仲裁 + 双看门狗，
  并修掉原工程的 `rc_tick` 恒假超时、CAN 阻塞狂发、`HAL_UART_RxCpltCallback` 重复定义三处缺陷。
  离线检查四组全绿：C 断言 70+21、6 个固件模块语法检查、Python 19、Keil 工程一致性。

- 2026-10-20：原 SD 卡损坏 → 重灌 `3.5.0` 镜像（SHA256 复核一致，首启自动扩容到 29 GB）；
  板端 30 min 重建 odin_ros_driver + ROS2 依赖；Odin1 恢复 **10.11 Hz 点云 / 10.31 Hz 图像 / 355 Hz IMU**；
  网页预览经 `plink -L 8080:127.0.0.1:8080` 在 PC 端 `http://127.0.0.1:8080/` 验证通过。
  过程、脚本与坑见 `x5_odin/README.md` §11。

- 2026-10-17：初始化仓库；加入协作规则、环境清单、issue/decision 模板；首次推送。
- 2026-10-17：完成单类 person 切尾巴（`x5_yolo/`）；494 张 COCO person AP@0.5 0.7800 不变；
  opset 11 + hb_mapper 1.24.3 本地编出 3.7 MB `.bin`；决策见 `docs/decisions/2026-10-17-yolo-person-head-x5.md`。
- 2026-10-18：板端实测 640 73.9 FPS、512 115.2 FPS；512 `.bin`（3.4 MB）与快速包已交付。
- 2026-10-18：C++ 后处理（ctypes）上线；512 135.3 FPS、640 87.2 FPS。
- 2026-10-18：576 定稿，104.1 FPS、AP 0.7178；三档对比见
  `docs/decisions/2026-10-18-yolo-input-576.md`。
- 2026-10-18：USB2 MJPG 摄像头实测（软解瓶颈 24/48/11 FPS），待换 CSI/USB3；
  aiVault 草稿见 `docs/aiVault/2026-10-18-rdk-x5-yolo-int8-tailcut.md`。
