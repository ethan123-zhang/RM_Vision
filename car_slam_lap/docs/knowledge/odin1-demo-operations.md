# Odin1 演示：操作知识（规则与判据）

面向后续会话/接手者的速查。人读是清单，模型读是约束。

## 一、硬规则（违反必出问题）

1. **桥接与遥控互斥**：`bridge_node` 运行期间串口帧优先于 DT7（固件仲裁窗口 300 ms），
   手动开车/示教前必须先 `pkill -f "lib/car_bridge/bridge_node$"`。
   建图脚本必须带 `DT7=1`（跳过桥接）。
2. **每趟建图用新目录**：`start_mapping.sh` 只 `mkdir -p`；帧录制器每趟从 `frame_000001.jpg`
   重新编号，同目录重跑会覆盖旧帧、残留旧尾巴 → 成片串镜头。用
   `MAP=~/car_ws/maps/demo-$(date +%m%d-%H%M)`。
3. **存图前先停录像**：录像与地图回传共用链路；录像写盘打满驱动帧队列
   （`Queue RGB is full`）会让 `save_map` 永久停在
   `waiting for device to finish generation...`，重发被 `Map transfer already in progress` 拒绝，
   只能重启驱动（= 丢图）。
4. **示教收尾只能按一次 Ctrl-C**：录制器收到 SIGINT 才写 `route_map.yaml` + `manifest.json`；
   连按第二下等于 SIGKILL，bundle 作废。验收：两个文件都在。
5. **必须传力臂补偿**：`BASE_OFFSET_X=0.2626 BASE_OFFSET_Y=-0.0471`（Odin1 相对转心，
   `calibrate_lever_arm` 实测）。不传 → 原地转使传感器位姿漂移 → 起点判定无限"补位往返"（抽搐）。
6. **转速窗口 0.5–0.7 rad/s**：<0.3 静摩擦转不动，>0.8 抖/抽搐。用 `W_MAX=RETURN_W_MAX=0.6`。
7. **命令里 `pkill -f` 的模式锚定结尾**（`...$`），否则会匹配到自身 SSH 命令行把自己杀掉。

## 二、定版参数（已验证连续 2 次成功）

```
VMAX=0.2 W_MAX=0.6 RETURN_W_MAX=0.6 RETURN_TIMEOUT_S=90 START_POS_TOL_M=0.2
BASE_OFFSET_X=0.2626 BASE_OFFSET_Y=-0.0471 MAX_TIME_S=400
```

建图/存图：`VIDEO=1 VIDEO_STRIDE=2 DT7=1` + `SAVE_TIMEOUT=300`。

## 三、标准时序

1. 清场：`pkill` 掉 follow/bridge/driver 残留；`lsusb | grep 2207:0019`；`df -h ~`（≥3 GB）。
2. 建图+录像：`VIDEO=1 VIDEO_STRIDE=2 DT7=1 start_mapping.sh "$MAP"`，遥控走 5–10 分钟。
3. 停录像 → 存图：`pkill -INT -f "record_odin_rgb[.]py$"`；`SAVE_TIMEOUT=300 save_map.sh "$MAP"`；
   验收 `odin_map/*.bin ≥ 200 MB`；`stop_mapping.sh`。
4. 板端出片：`ffmpeg -framerate <实测fps> -pattern_type glob -i "video/frames/frame_*.jpg"
   -vf "scale=1280:-2" -c:v libx264 -preset veryfast -crf 24 -pix_fmt yuv420p
   -movflags +faststart mapping_1280.mp4`；`ffprobe` 验收宽高/帧数/时长。
5. 重定位：车停建图起点 1 m 内、朝向差 <10°；成功判据 = `map -> odom` TF 出现。
6. 示教：桥接必须停；遥控走一圈；末段向前开过起点 0.3–0.5 m 再停；一次 Ctrl-C。
7. 自主：起桥接（`setsid nohup`），用第二节参数跑 `drive_lap.sh`。

## 四、验收判据（数字）

| 环节 | 通过标准 |
| :--- | :--- |
| 建图有效 | 点云 40 万点、包围盒数十米、覆盖面积数十 m²；`path.yaml` 长度以米计 |
| 存图成功 | `odin_map/*.bin` 200+ MB；`odin.log` 出现 `Map transfer completed` |
| 重定位 | `relocalization success` / `map -> odom` TF |
| 示教 | `route_map.yaml` + `route_map.yaml.manifest.json` 同时存在；点数与长度合理 |
| 出片 | `ffprobe`：h264、宽高偶数、`nb_frames ≈ frames_written`、时长与录制时长一致 |
| 自主 | 日志"结果: 成功"；横向误差 rms ≲0.15 m；回起点补位 0 次 |

## 五、故障速查

| 现象 | 处理 |
| :--- | :--- |
| 遥控推杆车不动 | 桥接在跑 → 停掉 |
| 录像 0 帧 | `sendrgb` 与 `sendrgbcompressed` 都要为 1 |
| 存图目录空且日志停在 waiting | 停录像后重存；仍不动则重启驱动重跑（丢图） |
| 存图 150 s 超时 | `SAVE_TIMEOUT=300` |
| mp4 0 字节 / `Slice parameters invalid` | 缩放高度奇数 → `scale=W:-2` |
| 示教无 manifest | 只按一次 Ctrl-C；重录 |
| 自主抽搐 / 回不来 | 力臂没传 或 `RETURN_W_MAX` 未设 0.6 |
| 板端看录像 | `ffmpeg -i x.mp4 -pix_fmt rgb24 -f caca -`（ASCII）或开 HTTP 用浏览器 |
| 磁盘不足 | 清 `~/car_ws/maps/*`、`~/odin_ws/src/odin_ros_driver/map/*`、`~/.cache`、`~/.ros/log` |