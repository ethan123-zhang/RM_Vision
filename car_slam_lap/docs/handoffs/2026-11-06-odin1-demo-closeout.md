# Odin1 演示项目：进度与问题总结

- 日期：2026-11-06
- 工作目录：`E:\RM\car_slam_lap`
- 分支：`codex/car-local-obstacle-avoidance`
- 硬件：RDK X5 + Odin1 + C8T6 双驱底盘（万向轮）+ DT7 遥控

## 一、本次目标

在无额外障碍物的场地上跑通演示链路：建图（同时录像给观众）→ 存图 → 重定位 → 手扶示教 →
纯追踪自主巡线；并把操作固化成可复现的指令与文档。

## 二、问题与解法

| # | 现象（证据） | 根因 | 修法 | 验证 |
| :--- | :--- | :--- | :--- | :--- |
| 1 | 建图时 DT7 推杆车不动 | 固件仲裁：串口帧 300 ms 内优先于遥控（`firmware/C8T6_DT7_car/Core/Src/main.c:96-135`），桥接 50 Hz 发帧压住遥控 | 建图用 `DT7=1` 跳过桥接 | 推杆即动 |
| 2 | 录像 0 帧 | 驱动里 `sendrgb` 与 `sendrgbcompressed` 是与门（`host_sdk_sample.cpp:1935`） | 两个开关都设 1（`start_mapping.sh` 自动） | 3309 帧写入 |
| 3 | 回起点原地转"太快、抽搐" | 控制用位姿是传感器坐标系、未扣 0.267 m 力臂；原地转导致传感器位姿平移，起点判定反复补位；同时转速偏离刻度 | `BASE_OFFSET_X=0.2626 BASE_OFFSET_Y=-0.0471`；`W_MAX=RETURN_W_MAX=0.6`；`START_POS_TOL_M=0.2` | 补位往返 0 次；rms 0.138 m |
| 4 | 远端 180° cusp 转不动 | 同上（缺力臂 + 转速低于静摩擦阈值 0.3 rad/s） | 同上 | 连续 2 次自主成功 |
| 5 | 存图 150 s 超时、`odin_map/` 为空 | `.bin` 约 203–239 MB，回传需 2–3 min；默认超时中途被杀 | `SAVE_TIMEOUT=300` | `.bin` 203 / 239 MB 落盘 |
| 6 | mp4 报 `Slice parameters invalid` 且 0 字节 | 缩放后高度为奇数（1037），H.264 要求偶数 | `-vf scale=1280:-2` | ffprobe 1280×1036 |
| 7 | 存图卡死 17 分钟：`odin.log` 停在 `waiting for device to finish generation...`，重发被 `Map transfer already in progress` 拒 | 录像写盘打满驱动帧队列（`Queue RGB is full, dropping oldest data`，帧计数停在 615），地图生成完成回调丢失 | 存图前先停录像（`pkill -INT -f "record_odin_rgb[.]py$"`）；录像加 `VIDEO_STRIDE=2` 降负载；卡死后只能重启驱动重跑（丢图） | 停录像后存图 6 s 成功（对照：设备生成 2042 ms + 传输 5000 ms） |
| 8 | 示教"自己停了"、无 `manifest.json` | 录制器只有收到 SIGINT 才写 bundle；被直接终止则不写 | 停稳后按一次 Ctrl-C；用 `ls -l route_map.yaml*` 验收两个文件 | 10:08 那次缺 manifest，bundle 弃用 |
| 9 | `pkill/pgrep -f` 把 SSH 会话自己杀掉（exit 128） | 模式文本出现在自身命令行里被匹配 | 模式锚定结尾，如 `pkill -f "lib/car_bridge/follow_path$"` | 未再复现 |
| 10 | 板端测速/回看录像无图形界面 | SSH 无 `DISPLAY` | `ffmpeg -f caca`（需 `-pix_fmt rgb24` 与 `TERM=xterm-256color`） | 实测输出彩色字符画 |
| 11 | 同目录重复建图导致录像串帧 | 录制器每趟从 `frame_000001.jpg` 重新编号；`start_mapping.sh` 只 `mkdir -p` 不清空 | 每趟用新目录（`demo-$(date +%m%d-%H%M)`） | — |
| 12 | 磁盘吃紧（29 GB 盘，可用 2.7 GB） | 每趟录像帧 1.5–2 GB + 地图副本 0.2–0.24 GB；驱动暂存目录与 `maps/` 各存一份 | 清 `maps/*`、`~/odin_ws/src/odin_ros_driver/map/*`、缓存；建图前 `df -h` 留 ≥ 3 GB | 待执行 |

## 三、定版参数与实测数据

**自主走圈参数（已验证可复现）**

```
VMAX=0.2 W_MAX=0.6 RETURN_W_MAX=0.6 RETURN_TIMEOUT_S=90 START_POS_TOL_M=0.2
BASE_OFFSET_X=0.2626 BASE_OFFSET_Y=-0.0471 MAX_TIME_S=400
```

**实测结果**

| 指标 | 数值 |
| :--- | :--- |
| 自主跑圈（`venue3` 地图） | 连续 2 次成功，横向误差 rms 0.138 m |
| 回起点 + 起始对齐 | 9.0 s，补位往返 0 次 |
| 建图点云 | 400,000 点（voxel 0.05 m），29.46 m × 37.46 m，覆盖 81.58 m²，1303 帧 / 354.4 s |
| 建图录像 | 1280×1036、3309 帧、330.9 s、86.7 MB（板端 ffmpeg 出片） |
| 力臂标定 | 转 169.9°，传感器位移 0.531 m vs 理论 0.532 m（误差 1 mm） |
| 板端测试 | 238 passed, 2 skipped |
| `demo-1106` 建图 | `.bin` 203 MB，972 帧，24.53 m × 36.43 m，覆盖 83.98 m² |
| `demo-1106` 示教（已删） | 537 点 / 15.57 m / 404.6 s；起点 (-0.39,0.13,-16°)、终点 (-0.33,0.08,-18°)；**缺 manifest** |

## 四、当前状态（2026-11-06 16:26）

**仓库**

- 分支 `codex/car-local-obstacle-avoidance` 已推送 GitHub，提交 `4f813e9`（94 文件，+14886/−33）。
- 大媒体按 `.gitignore` 不入库（`demo/**/*.mp4|*.bin|*.pcd|frames/|odin_map/`）；原件在本机
  `E:\RM\car_slam_lap\demo\` 与板端 `~/car_ws/maps/`。

**板端 RDK X5**

- **离线**：11:15 起 SSH 超时（16:26 仍不可达），疑似断电/断网。
- 最后已知状态（10:21）：`demo-1106` 地图完好（`.bin` 203 MB、`map.pcd`、`map_bev.png`、mp4 83 MB）；
  示教产物已按要求删除（旧文件备份在 `demo/demo-1106/route_map_旧示教_20261106.yaml`）；
  驱动处于重定位态；桥接已停；磁盘可用 2.7 GB。

## 五、已完成 / 未完成 / 下次进入点

**已完成**

1. `venue3` 全链路跑通：建图+录像 → 存图 → 重定位 → 示教 → 自主 2 次成功。
2. 板端出片链路验证（`ffprobe` 实测数值见上表）。
3. 参数定版（见第三节）；板端 238 项测试通过。
4. 文档交付：[指令单](../demo-command-sheet.md)、[明天全流程](../demo-day-fresh.md)、
   [断了怎么重来](../reset-and-restart.md)、重置脚本 `x5/scripts/reset_run.sh`。

**未完成**

1. 板端清盘未执行（`maps/*` 约 7 GB、驱动暂存副本、缓存），磁盘仍 2.7 GB 可用。
2. `demo-1106` 的重定位后示教、自主均未做（示教路线已删）。
3. `reset_run.sh` 未在板端验证（本机无可用 bash，未跑 `bash -n`）。
4. 倒车三拍 `RETURN_REVERSE=1` 已实现但实测不触发（终点已在容差内），默认关闭保留。
5. 大媒体未入 GitHub（按设计排除）。

**下次进入点**

```bash
# 1) 上电后部署并校验重置脚本
pscp -pw sunrise x5/scripts/reset_run.sh sunrise@<IP>:/home/sunrise/car_slam_lap/x5/scripts/
chmod +x ~/car_slam_lap/x5/scripts/reset_run.sh && bash -n ~/car_slam_lap/x5/scripts/reset_run.sh

# 2) 清盘（目标可用 ≥ 9 GB）
bash ~/car_slam_lap/x5/scripts/reset_run.sh                 # 先只停进程
rm -rf ~/car_ws/maps/* ~/odin_ws/src/odin_ros_driver/map/*
rm -rf ~/.cache/pip ~/.ros/log; sudo apt clean; df -h ~

# 3) 重新建图（分钟级新目录；存图前先停录像）
MAP=~/car_ws/maps/demo-$(date +%m%d-%H%M)
VIDEO=1 VIDEO_STRIDE=2 DT7=1 bash ~/car_slam_lap/x5/scripts/start_mapping.sh "$MAP"
pkill -INT -f "record_odin_rgb[.]py$"; sleep 3
SAVE_TIMEOUT=300 bash ~/car_slam_lap/x5/scripts/save_map.sh "$MAP"
ls -lh "$MAP"/odin_map/*.bin                                # 200+ MB 才算成

# 4) 出片 → 重定位 → 示教（一次 Ctrl-C，验 manifest）→ 自主
```

## 六、遗留风险

| 风险 | 触发条件 | 兜底 |
| :--- | :--- | :--- |
| 存图再次卡死 | 录像与地图回传同时进行 | 存图前停录像；卡住 >3 分钟直接重启重跑 |
| 板端离线 | 供电不稳 / Wi-Fi 掉线 | 现场备 HDMI+键盘执行 `ip a`；演示前 30 分钟先跑通一次 |
| 磁盘不足 | 每趟约 2 GB 帧 + 0.2 GB 图 | 建图前 `df -h ~`，可用 <3 GB 先清 `maps/` |
| 力臂参数不适用新车 | 传感器安装位置不同 | 新车先跑 `calibrate_lever_arm` |
| 示教 bundle 缺 manifest | 连按 Ctrl-C | 验收 `ls -l route_map.yaml*` 两个文件都在 |

## 七、相关文件

- 文档：[demo-command-sheet](../demo-command-sheet.md)、[demo-day-fresh](../demo-day-fresh.md)、
  [reset-and-restart](../reset-and-restart.md)、[demo-video-runbook](../demo-video-runbook.md)
- 脚本：`x5/scripts/{start_mapping,save_map,stop_mapping,start_relocal,teach_map_route,drive_lap,reset_run}.sh`、
  `x5/scripts/record_odin_rgb.py`、`x5/pc/frames_to_mp4.py`
- 代码：`x5/car_bridge/car_bridge/follow_path.py`（力臂补偿、转速限幅、倒车三拍）、
  `x5/car_bridge/test/test_follow_safety.py`
- 演示素材：`demo/venue3-20261106b/`（地图 png、路线 yaml、日志）、
  `demo/demo-1106/route_map_旧示教_20261106.yaml`