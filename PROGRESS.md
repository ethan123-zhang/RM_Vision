# 项目进度

> 唯一“现在进行时”。每次会话结束更新；历史细节放 `docs/issues/` 与 `docs/decisions/`。
> 最后更新：2026-10-21

## 一句话状态

RDK X5 单类 person YOLOv8n 定稿：**576 + 三段流水线 + C++ 后处理 = 104.1 FPS**，
person AP 0.7178（640 为 0.7244）。切尾巴、量化、板端测速全部跑通。
2026-10-20 原 SD 卡损坏 → 重灌镜像 + 板端 30 min 重建，Odin1 链路已恢复（10 Hz 数据、网页预览经端口转发在 PC 上验证）。
另起 `car_slam_lap/`：把这套感知装到 C8T6 双驱小车上，跑「建图 → 重定位 → 走一圈」。
X5 侧、链路协议、固件改造都已完成，**Keil AC5 实编译通过（0 error / 0 warning，Code 10.9 KB）**。
2026-10-21 按实物原理图修正：CAN 是 SN65HVD230 裸收发器（不是串口转CAN模块），
电机链路已改回片内 CAN1 + 500K；下一步是烧写 + 上车验收。

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
| 9 | 小车底盘串口化 + 建图走圈（练习） | 🔶 进行中 | 协议/固件/X5 侧完成，Keil AC5 编译 0 error/0 warning；CAN 已按实物改为 SN65HVD230 + CAN1 500K；待烧写 + A2，见 `car_slam_lap/WIRING.md` |

## 正在做

- 576 定稿；接入 USB 摄像头做端到端实测（720p60 / 480p120），之后换正式相机。
- Odin1 链路 2026-10-20 已恢复：板端起驱动 + 网页预览，PC 经 `plink -L 8080` 端口转发查看
  （`x5_odin/`，启动/打包/校验脚本齐全）。
- `car_slam_lap/`：三条链路都写完了 —— 协议（C+Python 同源，70+12 项断言）、`car_bridge`
  ROS2 包（桥接/键盘遥控/录轨迹/纯追踪/点云导出/速度标定）、固件（USART1=X5、
  USART2=遥控器、**CAN1=SN65HVD230 500K**、10 ms 仲裁、看门狗）、板端脚本与 Odin1 两份配置。
  自检五组全绿，其中第 5 组直接调 `UV4.exe` 真编译（Code 10944 / RAM 1400）。
  **下一步**：烧写 C8T6 → 按 `WIRING.md` 上电 → `link_test` 跑 A1，再离地跑 A2。
  X5 侧已部署（2026-10-21），`/dev/car_link -> ttyS1` 已确认。

## 下一步（按优先级）

1. 通读 `_extract/rules.txt`、`entry.txt`、`build.txt`，产出《视觉相关规则约束清单》：目标类别、场地尺寸、分辨率/帧率限制、裁判系统接口、违规条款。
2. 依据约束清单确定本赛季视觉任务（检测/跟踪/测距等，以规则为准）与指标（mAP、端到端延迟、帧率、功耗）。
3. 在 RDK X5 上验证相机采集与 BPU 推理的最小链路，记录环境指纹与基线耗时。
4. 把可复用的量化/部署结论沉淀到 aiVault（按 `docs/conventions.md` §6）。

## 阻塞 / 风险

- RDK X5 镜像约 9.68 GiB 未入库：换机需按 `docs/environment.md` 重新获取并校验 SHA256。
- 规则为 RM2027 V1.0.0（20261015），后续版本可能调整，需定期核对。
- 相机型号、SDK/工具链版本尚未记录，补齐前实验不可复现。

## 最近变更

- 2026-10-21：**X5 侧已部署到 172.17.90.239**（`deploy.ps1 -Install`，colcon build 通过，
  `can_ack` 代码已在板端，`/dev/car_link -> ttyS1`）。C8T6 还需重烧
  `MDK-ARM/C8T6_CAN_car/C8T6_CAN_car.hex`，否则板上仍跑旧的 USART3/slcan 固件。
- 2026-10-21：**接线拓扑再修正一次：CAN 是 SN65HVD230 裸收发器，不是串口转CAN模块。**
  用户补了 `CAN收发器原理图.pdf` / `CAN收发器说明书.pdf`：板子只有 SN65HVD230 + 120Ω
  + 10K，H1 = GND/3V3/R/D，H2 = CANH/CANL，没有 MCU、没有 UART。原来的
  「USART3 → slcan → 模块」物理上不可能工作。已改回 **片内 CAN1（PA11/PA12）**：
  `can_frame.c` 纯帧编码 + `can_bus.c` 非阻塞发送 + ACK 解析（status b4）；
  `can.c`/`.ioc` 设 500K；`slcan.c/h` 与测试删除；Keil 实编译 0 error / 0 warning。
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
