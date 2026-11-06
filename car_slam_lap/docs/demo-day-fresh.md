# 明天现场：重新建图 → 板端出片 → 示教 → 自主（完整指令）

> 命令分两类：**【板端】**在 SSH 里跑，**【电脑】**在本机 PowerShell 里跑。
> **视频在板子上渲染**（板子有 ffmpeg + 8 核），电脑端不需要装 OpenCV/ffmpeg，只拉 83 MB 成片。

---

## 0. 【电脑】连板子

```powershell
$IP = "10.77.156.239"          # 明天换成实际 IP（板子接显示器跑 ip a 可查）
ssh sunrise@IP     # 密码 sunrise
```

---

## 1. 【板端】清场 + 看盘

```bash
# 杀残留进程（pkill 模式必须锚定结尾，否则会把这条 SSH 命令自己也杀掉）
pkill -f "lib/car_bridge/follow_path$"
pkill -f "lib/car_bridge/bridge_node$"
pkill -f "lib/odin_ros_driver/host_sdk_sample$"

lsusb | grep 2207:0019        # Odin1 在线
df -h ~                       # 至少留 4 GB（录像 2 GB + 成片 0.1 GB + 地图 0.24 GB）
```

---

## 2. 【板端】建图 + 同步录像（10–15 分钟）

```bash
MAP=~/car_ws/maps/demo-$(date +%m%d)
echo "$MAP"

# DT7=1 必须带：否则脚本会起桥接，桥接 50 Hz 发帧会压过遥控器（推杆车不动）
VIDEO=1 DT7=1 bash ~/car_slam_lap/x5/scripts/start_mapping.sh "$MAP"
```

可选参数：

| 变量 | 用途 |
| :--- | :--- |
| `VIDEO=1` | 开录像（写 `$MAP/video/frames/*.jpg`） |
| `DT7=1` | 遥控建图、**不起桥接**（必须） |
| `VIDEO_STRIDE=2` | 每 2 帧存 1 帧（5 fps，体积减半；卡慢时用，出片仍可 10 fps 播） |


---

## 3. 【板端】存图 + 收尾
```

# 239 MB 的地图要传 2–3 分钟；默认 150 s 会超时 → 驱动被杀 → 地图丢
SAVE_TIMEOUT=300 bash ~/car_slam_lap/x5/scripts/save_map.sh

ls -lh "$MAP/odin_map"          # 必须有一个 ~200+ MB 的 .bin，不能是 0
bash ~/car_slam_lap/x5/scripts/stop_mapping.sh
```

---

## 4. 建好的地图放在哪

**板端：`~/car_ws/maps/<地图名>/`**（本次就是 `~/car_ws/maps/demo-1106/`）

| 文件 | 是什么 | 给谁看 |
| :--- | :--- | :--- |
| `map.pcd` | 点云地图（ASCII xyz，40 万点，~8.5 MB） | CloudCompare / 自己渲染 |
| `map_bev.png` | 俯视快照（自动生成） | 观众 |
| `map_info.txt` | 统计：帧数、点数、包围盒、覆盖面积 | 讲解 |
| `path.yaml` | 建图时走过的轨迹 | 参考 |
| `odin_map/map_*.bin` | **Odin1 原始地图（239 MB）**，重定位必须用 | 不能删 |
| `video/frames/` | 2 万…3 千张 JPEG（~2 GB） | 出片用 |
| `video/meta.json` | 帧数、**实测帧率**、字节数 | 出片定帧率 |
| `*.log` | 各阶段日志 | 排障 |

**当前地图指针**：`~/car_ws/.current_map_dir`（`start_relocal.sh` 不传参时读它）

---

## 5. 【板端】渲染成 mp4（主路径，约 6 分钟）

板子自带 `ffmpeg 4.4.2`（含 libx264）+ 8 核，直接在板上把 JPEG 帧编成 H.264，
**电脑端不用装任何东西**，也不要拉 2 GB 帧回去。

```bash
cd "$MAP"

# ① 先看这趟录了多少帧、实测帧率是多少（决定 -framerate 填几）
cat video/meta.json | head -20
ls video/frames | wc -l
df -h .                            # 至少留 1 GB

# ② 编码（1600×1296 → 1280×1036，H.264）
ffmpeg -y -framerate 10 -pattern_type glob -i "video/frames/frame_*.jpg" \
  -vf "scale=1280:-2" -c:v libx264 -preset veryfast -crf 24 \
  -pix_fmt yuv420p -movflags +faststart mapping_1280.mp4

```

验收标准（上一趟的真实输出）：

```
codec_name=h264
width=1280
height=1036
r_frame_rate=10/1
nb_frames=3309
duration=330.900000
size=86738101          # ≈83 MB
```

## 播放

```
cd ~/car_ws/maps/demo-1105

ffmpeg -hide_banner -loglevel error -re -i mapping_1280.mp4 \
  -vf "scale=100:-1" -pix_fmt rgb24 -f caca -
```

### 参数说明

| 参数 | 为什么这么写 |
| :--- | :--- |
| `-framerate 10` | 填 `meta.json` 里的 `measured_fps`（实测 9.523 就写 9.523），写成 10 会轻微快放 |
| `scale=1280:-2` | **`-2` 不能改**：H.264 要求宽高都是偶数，奇数高度会报 `Slice parameters invalid` 并写出 0 字节文件 |
| `-c:v libx264 -crf 24` | 真 H.264，5.5 分钟片约 83 MB，画质够投屏 |
| `-pix_fmt yuv420p` | 不加的话有些播放器/投影不认 |
| `-movflags +faststart` | 把索引放文件头，双击就能秒开、可拖动 |


## 8. 【板端】重定位 → 示教 → 自主

```bash
MAP=~/car_ws/maps/demo-$(date +%m%d)
```

### 8.1 重定位（1–2 分钟）

```bash
bash ~/car_slam_lap/x5/scripts/start_relocal.sh "$MAP"
```

成功标志：`✓ 重定位成功：map -> odom 已发布`。
**车必须停在建图起点 1 m 内、朝向差 < 10°**；失败就挪车换朝向重跑。

### 8.2 示教（遥控走一圈，3–4 分钟）

```bash
pgrep -af "lib/car_bridge/bridge_node$"      # 必须没输出（示教要遥控）
bash ~/car_slam_lap/x5/scripts/teach_map_route.sh "$MAP"
```

1. 车摆到演示起点、车头朝出发方向；
2. 0.2–0.3 m/s，转弯半径 ≥ 0.5 m，走一圈；
3. **末段一路向前开过起点 0.3–0.5 m 再停**，不倒车、不原地搓；
4. 停稳 `Ctrl-C`。

成功标志：`✓ 严格示教完成` + 生成 `route_map.yaml`（想重教不覆盖：`OUT_PATH="$MAP/route_2.yaml"`）。

### 8.3 自主走圈（约 2 分钟）

```bash
//现在开第二个终端执行下面两行
source ~/car_slam_lap/x5/scripts/env.sh
ros2 run car_bridge bridge_node


VMAX=0.2 W_MAX=0.6 RETURN_W_MAX=0.6 RETURN_TIMEOUT_S=90 START_POS_TOL_M=0.2 \
BASE_OFFSET_X=0.2626 BASE_OFFSET_Y=-0.0471 MAX_TIME_S=400 \
bash ~/car_slam_lap/x5/scripts/drive_lap.sh "$MAP/route_map.yaml"
```

成功标志：

```
结果: 成功
结束原因: 到达终点（位置误差 …，朝向误差 …）
横向误差 max=… rms=…
```

参数不能省：`BASE_OFFSET_X/Y` 是 Odin1 相对转心的力臂（不补偿会原地"补位往返"抽搐）；
`W_MAX/RETURN_W_MAX=0.6` 是这台车的可用转速窗口（>0.8 抽搐、<0.3 转不动）。

---

## 9. 收尾

```bash
pkill -f "lib/car_bridge/follow_path$"
pkill -f "lib/car_bridge/bridge_node$"
pkill -f "lib/odin_ros_driver/host_sdk_sample$"
```

---
## 板端清理

### 1) 删掉所有地图目录（点云 / 俯视图 / 录像帧 / 示教路线 全在里面）
rm -rf ~/car_ws/maps/*
rm -f  ~/car_ws/.current_map_dir

### 2) 删驱动自己暂存的地图副本（save_map 时留下的，通常和 maps 里的重复）
rm -rf ~/odin_ws/src/odin_ros_driver/map/*

### 3) 清缓存和日志（这两块容易悄悄涨到几百 MB）
rm -rf ~/.cache/pip ~/.cache/colcon ~/.ros/log
sudo apt clean
sudo journalctl --vacuum-size=50M

### 4) 清临时文件
rm -f /tmp/odin_command.txt /tmp/odin_reloc.log /tmp/bridge_probe.log /tmp/map*.log

## 10. 一分钟排障

| 现象                             | 处理                                                                     |
| :----------------------------- | :--------------------------------------------------------------------- |
| 遥控推杆车不动                        | 桥接在跑：`pkill -f "lib/car_bridge/bridge_node$"`                          |
| 录像 0 帧                         | 驱动配置 `sendrgb` 与 `sendrgbcompressed` **都要** 1（`start_mapping.sh` 已自动改） |
| `odin_map/` 空                  | 存图超时：`SAVE_TIMEOUT=300` 重来                                             |
| `Slice parameters ... invalid` | 输出高度是奇数：`scale=1280:-2` 的 `-2` 别改                                      |
| ffmpeg 报 `No such file`（glob）  | 路径/引号问题：先 `ls video/frames \| head -3` 确认目录非空                          |
| 成片 0 字节 / 打不开                  | 多半是磁盘满或编码器报错：看 `df -h`、重跑并盯住 stderr                                    |
| 成片播放卡                          | 桌面播放器问题：`ffprobe` 看 `nb_frames` 是否等于 `meta.json` 的 `frames_written`    |
| 重定位失败                          | 车不在建图起点 1 m 内 / 朝向差大 / `lsusb` 没有 2207:0019                            |
| 自主抽动、回不来                       | 忘传 `BASE_OFFSET_X/Y`，或 `RETURN_W_MAX` 没设 0.6                           |
| `pgrep/pkill` 把 SSH 自己杀了       | 模式必须锚定结尾：`pkill -f "lib/car_bridge/follow_path$"`                      |
