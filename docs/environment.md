# 环境与资料清单

> 变更板卡、镜像、SDK 或关键依赖时更新本文件。目的：换机/换人后能复现同一套环境。

## 1. 硬件

| 项 | 值 | 备注 |
| :--- | :--- | :--- |
| 主控板 | RDK X5（D-Robotics），8 核 aarch64 / 3062 MB / 32 GB SD | Ubuntu 22.04.5，kernel 6.1.83（2026-10-20 重灌实测） |
| 空间感知 | Odin1（奥丁之眼），USB3 `2207:0019 hawk` | 1600×1296 RGB + 49152 点 DTOF，10 Hz，SuperSpeed 5000M；见 `x5_odin/README.md` |
| 相机 | 待补充 | 型号、分辨率、接口、帧率（YOLO 链路另算，见 `x5_yolo/`） |
| 算力单元 | RDK X5 BPU | 工具链 hb_mapper 1.24.3 / hbdk 3.49.15 |
| 小车下位机 | STM32F103C8T6 最小系统 | USART1=X5、USART2=DT7、CAN1=PA11/PA12 |
| 电机控制器 | XHSMCUW50 双驱无刷 | 15–82V，CAN 250K（实测），扩展帧 `0x1314` |
| CAN 收发器 | SN65HVD230 模块 | 3.3V，H1: GND/3V3/R/D，H2: CANH/CANL，板载 120Ω |
| 小车电池 | 22.8V（6S） | 控制器直供；12V/5V 各一路降压 |
| 遥控 | DT7 + 接收机 | 100000 8E1，s2 到底 = 硬急停 |

## 2. 系统镜像

| 项 | 值 |
| :--- | :--- |
| 文件 | `rdk-x5-ubuntu22-preinstalled-desktop-3.5.0-arm64.img` |
| 大小 | 10,393,485,312 字节（约 9.68 GiB / 10.39 GB） |
| SHA256 | `02FD5223871C2D8D027B0C9AB3B57E6448F29C427C45C2D6AD7951622BDC2284` |
| 入库 | **否**：超过 GitHub 单文件 100 MB 限制，且 LFS 免费额度不足 |
| 获取 | 官方发布渠道（具体下载页待补充） |

校验：

```powershell
Get-FileHash "RDK_X5镜像/rdk-x5-ubuntu22-preinstalled-desktop-3.5.0-arm64.img" -Algorithm SHA256
```

重灌记录（2026-10-20，原卡损坏）：

- 镜像 SHA256 与上表一致（复核过）；Rufus / balenaEtcher 写卡，**首次上电自动扩容**，rootfs 29 GB。
- 板端指纹：kernel 6.1.83、内存 3062 MB（默认无 swap）、`/opt/ros/humble` 190 包。
- 网络：DHCP `172.17.90.239`，SSH `sunrise / sunrise`；plink 首次连接需 `-hostkey SHA256:K8oy…Bb4`。
- 完整恢复步骤与踩坑见 `x5_odin/README.md` §11。

## 3. 工具链

| 工具 | 版本 | 用途 |
| :--- | :--- | :--- |
| Python | 3.10.1 | `_extract/extractor.py`（仅标准库） |
| WSL Ubuntu | 22.04 / Python 3.10.12 | 本地训练与 ONNX 验证 |
| X5 模型转换 | hb_mapper 1.24.3（`pip install rdkx5-yolo-mapper`）+ hbdk 3.49.15 | 生成 Bayes-e `.bin` |
| 交叉编译工具链 | 待补充 | |

## 4. 官方资料

三份 RM2027 V1.0.0（20261015）文档。`相关文件/` 保留加密原件，`相关文件/未加密可AI访问/` 保留解密版（供 AI 读取与文本提取）。两版对应同一文档，字节数与哈希不同是加密容器差异。

| 文档 | 加密原件 | 未加密版 | 字节数（加密 / 未加密） |
| :--- | :---: | :---: | :--- |
| 比赛规则手册 | ✓ | ✓ | 13,455,593 / 13,456,852 |
| 参赛手册 | ✓ | ✓ | 1,130,880 / 1,129,443 |
| 机器人制作规范手册 | ✓ | ✓ | 18,240,763 / 18,243,661 |

SHA256：

```text
规则-加密      A8FD7B7F6EA5212B9E4DFA590B478827992598D91912ED671653C8BDD9727FD6
规则-未加密    44907DFFAA6EB7F15471A570C1DE30A3D25F94C4C8D7BB1B8BDDC08A8F252412
参赛-加密      969FB865B7DCA28360325D198FD47A7158CB7C8CFE3E0FD6D51BC556B67B2950
参赛-未加密    D7F846B25DC37FDEECACA672F81DE1910E4265471BCACDABAC4388793D143359
规范-加密      3287FE5F521F8801CE69632032EA17A9C8D8A099091558F58618CD7721154A3C
规范-未加密    585992A5575C2CB06CA89E1E29445A676BCD44C635A0576D5F2A2FCBCEF5E719
```

## 4.1 小车硬件资料（2026-10-21 入库）

| 文件 | 内容 | 用途 |
| :--- | :--- | :--- |
| `car_slam_lap/双驱无刷电机控制器规格书A4pdf.pdf` | XHSMCUW50 规格书 6.3/6.4 | CAN 文档 500K（实测 250K）、`0x1314`、混控帧/ACK |
| `car_slam_lap/双驱动CAN控制接口使用说明.pdf` | CAN 接口说明 | 5–20 ms 周期、信号丢失 |
| `car_slam_lap/CAN收发器原理图.pdf` | SN65HVD230 模块原理图 | H1/H2 引脚、120Ω、10K |
| `car_slam_lap/CAN收发器说明书.pdf` | TI SN65HVD230 datasheet | 3.3V、1 Mbps、RS 模式 |

接线与上电顺序见 `car_slam_lap/WIRING.md`。

## 5. 文本提取

```powershell
python _extract/extractor.py "相关文件/未加密可AI访问/<文档>.pdf" "_extract/<文档>.txt"
```

当前产物：

| 文件 | 来源 |
| :--- | :--- |
| `_extract/rules.txt` | 比赛规则手册 |
| `_extract/entry.txt` | 参赛手册 |
| `_extract/build.txt` | 机器人制作规范手册 |

## 6. 复现检查

- [x] 镜像 SHA256 与上表一致（2026-10-20 复核）
- [ ] Python 3 可用
- [ ] 三份 PDF 的 SHA256 与上表一致
- [x] 板端系统版本与镜像一致（2026-10-20：Ubuntu 22.04.5 / kernel 6.1.83）
