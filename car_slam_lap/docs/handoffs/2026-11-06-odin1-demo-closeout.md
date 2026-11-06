# 交接：Odin1 小车「建图→示教→自主巡线」演示收尾

- 日期：2026-11-06
- 分支：`codex/car-local-obstacle-avoidance`
- 现场：RDK X5 + Odin1 + C8T6 双驱底盘（带万向轮）+ DT7 遥控

---

## 一、故事：车是怎么开始不听话的

**开场。** 这套系统干的事很朴素：让车先自己走一圈记下场地（建图），把这份记忆存成一个
点名文件（`.bin`），下次开机认一下自己在哪（重定位），然后你手把手教它走一条路线（示教），
最后它自己照着这条路线跑（纯追踪）。雷达和相机装在车体转心**前方 0.267 m**处——
这个"手臂长度"后面会变成所有抽搐的根源。

**冲突。** 演示要的是一条龙：建图时同时录像给观众看，然后自主跑圈。结果每一步都出了岔子：
遥控推杆车不动；录像一帧没有；回起点时原地转半圈"又快又抽"；跑到场地最远端 180° 掉头时
轮子打滑转不动；最后连存地图都 17 分钟不回来。

**探索。** 最初的直觉全错在"把它当成调参问题"：
车不动 → 以为速度给小了；录像没帧 → 以为话题没发；转不动 → 以为角速度不够；
抽搐 → 以为角速度太大。逐个查日志才发现是四件完全不同的事在捣乱：
串口仲裁、图像队列、力臂几何、设备回传超时。

**真相（机制层）。**

1. **车不动**：底盘固件有仲裁——串口帧 300 ms 内优先于遥控器（`main.c:96-135`）。
   桥接节点以 50 Hz 发速度指令（哪怕是零），遥控器被永久压住。
   证据：起桥接后推杆无反应，停桥接立即恢复。
2. **录像 0 帧**：Odin1 驱动里 `sendrgb` 与 `sendrgbcompressed` 是**与门**关系
   （`host_sdk_sample.cpp:1935` 用 `g_sendrgb` 门控），只开压缩帧不出一帧。
3. **抽搐**：控制用的位姿是**传感器坐标系**，不是转心。原地一转，传感器沿半径 0.267 m
   的圆弧平移，起点判定永远差一截，于是"对齐→发现位置差→补位→再对齐"来回震荡。
   用 `calibrate_lever_arm` 实测：转 169.9° 传感器位移 0.531 m，理论 2·0.267·sin(84.95°)=0.532 m，
   误差 1 mm，确认偏移 `(+0.2626, -0.0471)`。
4. **转不动**：底盘静摩擦使转速 <0.3 rad/s 时轮子不启动；>0.8 rad/s 又抖。可用窗口 0.5–0.7 rad/s。
5. **存图 17 分钟不回来**：录像写盘跟不上 → 驱动报 `Queue RGB is full, dropping oldest data`
   → 帧计数停在 615 → 随后 `save_map` 停在 `waiting for device to finish generation...`，
   设备端再没回应；重发被 `Map transfer already in progress` 拒绝。
   对照成功那趟：设备生成 2042 ms、传 250 MB 用 5000 ms，全程 6 秒。
6. **示教"自己停了"**：录制器只有收到 SIGINT 才写 `route_map.yaml` + `manifest.json`；
   没有 manifest 就是没正常收尾，bundle 不能用。

**转折（改了什么）。**

| 文件 | 改动 | 为什么对得上机制 |
| :--- | :--- | :--- |
| `x5/scripts/start_mapping.sh` | `DT7=1` 跳过桥接；`VIDEO=1` 自动同时开 `sendrgb`/`sendrgbcompressed` | 解开仲裁、满足与门 |
| `x5/scripts/save_map.sh`（用法） | `SAVE_TIMEOUT=300` | 239 MB 回传要 2–3 分钟，默认 150 s 会在中途被杀 |
| `x5/car_bridge/car_bridge/follow_path.py` | 新增 `BASE_OFFSET_X/Y` 力臂补偿、`_rotation_center()`；`return_w_accel` 角速度变化率限幅；`return_reverse` 倒车三拍（默认关） | 控制量基于转心而非传感器；转速窗口内平滑 |
| `x5/scripts/drive_lap.sh` | 透传 `BASE_OFFSET_X/Y`、`RETURN_W_MAX`、`START_POS_TOL_M`、`RETURN_TIMEOUT_S`、`RETURN_REVERSE` 等 | 让验证过的参数可复现 |
| `x5/pc/frames_to_mp4.py` | 偶数裁剪 + ASCII 临时名 + 读回校验 | H.264 要求偶数宽高；OpenCV 中文路径会写坏 |
| `x5/scripts/record_odin_rgb.py` | 新增编码帧录制器（stride 可调） | 主机侧不解码，省 CPU |

**结局（数字）。**

- 自主跑圈在 `venue3` 地图上连续成功 2 次，横向误差 **rms 0.138 m**，回起点+起始对齐 **9.0 s**，
  "补位往返" **0 次**（改之前是转不动 + 无限补位）。
- 建图录像：1600×1296 → 1280×1036、**3309 帧 / 330.9 s / 86.7 MB**，板上 6 分钟出片。
- 地图：**400,000 点**（voxel 0.05 m），**29.46 m × 37.46 m**，覆盖 **81.58 m²**。
- 存图：正常 6 秒；卡死那趟 17 分钟 0 字节，只能重跑。

**教训（类级规则）。** 移动底盘上凡是"转不动/抽搐/回不到起点"，先查**控制量是不是基于传感器
而非转心**，再查**转速是否落在静摩擦与抖动之间**；凡是"存不回来/传不完"，先查**同一时间
还有谁在抢 I/O**——录制和回传抢的是同一条链路。

---

## 二、问题清单（现象 → 根因 → 修法 → 证据）

| # | 现象（证据） | 根因 | 修法 | 复验 |
| :--- | :--- | :--- | :--- | :--- |
| 1 | 建图时推杆车不动 | 桥接串口帧压过遥控（300 ms 窗口） | `DT7=1` 不起桥接 | 推杆即动 |
| 2 | 录像 0 帧 | `sendrgb` 未开，压缩帧被门控 | 两个开关都设 1 | 3309 帧 |
| 3 | 回起点原地转"太快、抽搐" | 力臂未补偿 + 转速偏离窗口 | 力臂 + `W_MAX/RETURN_W_MAX=0.6` + `START_POS_TOL_M=0.2` | 补位 0 次，rms 0.138 m |
| 4 | 远端 180° cusp 转不动 | 同上（缺力臂、转速低） | 同上 | 连续 2 次通过 |
| 5 | 存图 150 s 超时、地图目录空 | 239 MB 回传需 2–3 min | `SAVE_TIMEOUT=300` | `.bin` 203/239 MB 落盘 |
| 6 | 单帧 mp4 报 `Slice parameters invalid`、0 字节 | 缩放后高度 1037 为奇数 | `scale=W:-2` + 偶数裁剪 | ffprobe 1280×1036 |
| 7 | 存图卡死 17 分钟、重发被拒 | 录像把驱动帧队列打满，完成回调丢失 | 存图前先停录像；`VIDEO_STRIDE=2` 降负载 | 停录像后存图 6 s 成功 |
| 8 | 示教"自己停了"、无 manifest | 未收到 SIGINT，收尾未执行 | 停稳按一次 Ctrl-C；验收 `manifest.json` | 10:08 那次缺 manifest → 已弃用 |
| 9 | `pgrep/pkill` 把 SSH 自己杀掉（exit 128） | 模式匹配到自身命令行 | 模式锚定结尾 `$` | 之后未复现 |
| 10 | 终端里想回看录像 | 板端无 GUI | `ffmpeg -f caca`（需 rgb24 + TERM） | 实测出彩色字符画 |

---

## 三、当前状态（截至 2026-11-06 16:10）

**仓库（本机 `E:\RM\car_slam_lap`）**

- 分支 `codex/car-local-obstacle-avoidance`，本次提交包含：力臂/限速/倒车三拍代码与测试、
  三份演示文档、重置脚本、录制器、视频合成脚本、`.gitignore` 忽略大媒体。
- 演示素材：`demo/venue3-20261106b/`（地图 png、路线 yaml、日志）入库；
  **mp4 / bin / pcd / 帧序列按 `.gitignore` 不入库**（单文件最大 239 MB，GitHub 拒收），
  本机 `E:\RM\car_slam_lap\demo\` 与板端 `~/car_ws/maps/` 留有原件。

**板端 RDK X5**

- **当前离线**：11:15 起 SSH `connection abort` / 超时（16:10 仍不可达），疑似断电或断网。
- 最后已知状态（10:21）：`demo-1106` 地图完好（`.bin` 203 MB、`map.pcd`、`map_bev.png`、
  `mapping_1280.mp4` 83 MB）；示教路线已按要求删除（旧文件已备份到
  `demo/demo-1106/route_map_旧示教_20261106.yaml`）；驱动在重定位态；桥接已停。
- 磁盘：**可用 2.7 GB / 29 GB**（清理未执行）。

---

## 四、已完成 / 未完成

**已完成**

1. `venue3` 全链路演示跑通：建图+录像 → 存图 → 重定位 → 示教 → 自主 2 次成功（rms 0.138 m）。
2. 板端视频出片链路验证：`ffprobe` 实测 1280×1036 / 3309 帧 / 330.9 s / 86.7 MB。
3. 参数定版：`VMAX=0.2 W_MAX=0.6 RETURN_W_MAX=0.6 START_POS_TOL_M=0.2
   BASE_OFFSET_X=0.2626 BASE_OFFSET_Y=-0.0471 RETURN_TIMEOUT_S=90`。
4. 文档三件套：[demo-command-sheet](../demo-command-sheet.md)、
   [demo-day-fresh](../demo-day-fresh.md)、[reset-and-restart](../reset-and-restart.md)。
5. 板端 238 项测试通过（含 3 项新增：倒车收敛、默认不倒车、限幅生效）。

**未完成**

1. 板端清盘（`maps/*` 约 7 GB、驱动暂存副本、缓存）——脚本与命令已就绪，**未执行**。
2. `demo-1106` 的示教与自主**尚未做**（路线已删，需重示教）。
3. `reset_run.sh` **未在板端验证**（本机无可用 bash，未跑 `bash -n`）。
4. 倒车三拍（`RETURN_REVERSE=1`）实现了但实测不触发（终点已在容差内），按用户意见"先这样"，
   默认关闭保留。
5. 大媒体未入 GitHub（设计如此，见第三节）。

---

## 五、下次进入点（按顺序）

```bash
# 1) 上电后先确认可达，并部署/校验重置脚本
pscp -pw sunrise x5/scripts/reset_run.sh sunrise@<IP>:/home/sunrise/car_slam_lap/x5/scripts/
chmod +x ~/car_slam_lap/x5/scripts/reset_run.sh && bash -n ~/car_slam_lap/x5/scripts/reset_run.sh

# 2) 清盘（先 --full 备份式清理，确认无误再 --purge）
bash ~/car_slam_lap/x5/scripts/reset_run.sh                       # 先只停进程看一眼
du -sh ~/car_ws/maps/* | sort -h ; df -h ~
rm -rf ~/car_ws/maps/* ~/odin_ws/src/odin_ros_driver/map/*
rm -rf ~/.cache/pip ~/.ros/log ; sudo apt clean
df -h ~                                                           # 目标：可用 ≥ 9 GB

# 3) 重新建图（分钟级目录 + 减载录像）
MAP=~/car_ws/maps/demo-$(date +%m%d-%H%M)
VIDEO=1 VIDEO_STRIDE=2 DT7=1 bash ~/car_slam_lap/x5/scripts/start_mapping.sh "$MAP"
pkill -INT -f "record_odin_rgb[.]py$"; sleep 3                     # 存图前先停录像
SAVE_TIMEOUT=300 bash ~/car_slam_lap/x5/scripts/save_map.sh "$MAP"
ls -lh "$MAP"/odin_map/*.bin                                      # 200+ MB 才算成

# 4) 出片 → 重定位 → 示教 → 自主（详见 demo-day-fresh.md）
```

---

## 六、遗留风险

| 风险 | 触发条件 | 兜底 |
| :--- | :--- | :--- |
| 存图再次卡死 | 录像与回传抢 I/O | 存图前停录像；卡住 >3 分钟直接重启重跑 |
| 板端离线/断电 | 供电不稳、Wi-Fi 掉线 | 现场备 HDMI+键盘查 `ip a`；演示前 30 分钟先跑通一次 |
| 磁盘不足 | 每次建图约 2 GB 帧 + 0.2 GB 图 | 建图前 `df -h ~`，低于 3 GB 先清 `maps/` |
| 力臂参数不适用于别的车 | 传感器位置不同 | 新车上先跑 `calibrate_lever_arm` |
| 示教 bundle 缺 manifest | 连按 Ctrl-C | 验收 `ls -l route_map.yaml*` 两个文件都在 |