# 演示跑法（无避障 · 带 Odin1 录像）——建图 → 重定位 → 示教 → 自主

> 适用：RDK X5 + Odin1 + C8T6 双驱练习车，`E:\RM\car_slam_lap` 这套代码。
> 本版本与 `docs/demo-runbook.md` 的区别只有两点：
> 1. **建图那趟可以把 Odin1 的 JPEG 视频录下来**（`VIDEO=1`），给观众看；
> 2. **全程不启用避障**（绕障代码保留，只是不加载参数文件）。
>
> 板端一律 `ssh sunrise@10.77.156.239`；PC 端在 `E:\RM`。

---

## 0. 为什么"不启用避障"就是什么都不用做

避障是**默认关闭**的，靠参数文件才打开：

- `x5/car_bridge/car_bridge/follow_path.py:202` —— `obstacle_avoidance` 默认 `False`
- `x5/scripts/drive_lap.sh:148-153` —— 只有设置了 `NAV_PARAMS_FILE` 才把参数文件传进去
- `x5/config/obstacle_avoidance.yaml:5` —— `obstacle_avoidance: true` 在这个文件里

所以：**只跑 `drive_lap.sh`、不要跑 `drive_lap_obstacles.sh` / `start_relocal_obstacles.sh`**，
避障就是关的，代码一行不用删。

运行时怎么证明它真没开：`drive_lap.sh` 启动横幅会打印

```
先回起点再跟踪；限起点附近空旷通道，无自动绕障。
```

收尾汇总里如果出现 `实时避障: {...}`，说明参数文件被带进去了 —— 停下来查 `NAV_PARAMS_FILE`。

---

## 1. 把新脚本同步到板端（PC 上做一次）

本次新增/改动只在 `x5/scripts/`，**不需要重新 colcon build**（录像脚本是独立 `python3` 脚本）。

```powershell
ssh sunrise@10.77.156.239 "ls -d ~/car_slam_lap/x5/scripts"
scp -r "E:\RM\car_slam_lap\x5\scripts" sunrise@10.77.156.239:~/car_slam_lap/x5/
```

如果你更想整套重装（会顺带 `colcon build`，约 1 分钟）：

```powershell
pwsh -File E:\RM\car_slam_lap\x5\pc\deploy.ps1 -X5Ip 10.77.156.239 -Install
```

同步完先在板端做一次语法自检（本次改的是 shell 脚本，本地没法替你验 bash）：

```bash
bash -n ~/car_slam_lap/x5/scripts/start_mapping.sh && echo start_ok
bash -n ~/car_slam_lap/x5/scripts/stop_mapping.sh  && echo stop_ok
python3 -c "import ast,sys; ast.parse(open('$HOME/car_slam_lap/x5/scripts/record_odin_rgb.py').read())" && echo py_ok
```

> `deploy.ps1` 默认钉死旧卡的 host key；换 IP 后如果报 host key 不匹配，
> 就把上面的 `scp` 用法顶掉它。

---

## 2. 上电 + 自检（板端）

```bash
ssh sunrise@10.77.156.239
```

车：电池上电 → 控制器 PowerKey 开机 → DT7 开机、**S2 拨中间**、油门回中 → e-stop 在手边。
Odin1：航插 **12 V/2 A** 独立供电（Type-C 只走数据）。

```bash
# 设备与底盘
lsusb | grep -E '2207'                    # 期望 2207:0019 ... hawk
source /opt/ros/humble/setup.bash
source ~/car_ws/install/setup.bash

# 磁盘（录像会写 ~1.3–2.7 GB，留够）
df -h ~

# 清场：别留旧进程
pgrep -af 'host_sdk_sample|bridge_node|record_path|map_from_cloud|record_odin_rgb'
```

有输出就说明上一轮没收拾干净，先跑 `bash ~/car_slam_lap/x5/scripts/stop_mapping.sh`。

---

## 3. 建图 + 录像（约 10–15 分钟）

```bash
MAP=~/car_ws/maps/venue3-20261106
VIDEO=1 DT7=1 bash ~/car_slam_lap/x5/scripts/start_mapping.sh "$MAP"
```

`DT7=1` 是必须的：`start_mapping.sh` 默认会起 `bridge_node`，而桥接每 50 Hz 发一帧
`set_vel`（哪怕是 0），固件仲裁里"300 ms 内有串口帧就压过遥控器"
（`firmware/C8T6_DT7_car/Core/Src/main.c:107-110`），遥控器会完全失灵。
2026-11-05 实测：桥接在跑的那 14 分钟，遥控推杆车一动不动。

`VIDEO=1` 做的事（看 `start_mapping.sh:31-43, 88-100`）：

- 只在**运行时生成的配置副本**里把 `sendrgb` 和 `sendrgbcompressed` **一起**从 0 改成 1，
  仓库模板不动。只开 `sendrgbcompressed` 不够：驱动里 RGB 流的开关是 `g_sendrgb`
  （`host_sdk_sample.cpp:1935`），实测只改 compressed 时 `/odin1/image/compressed` 一个字节都没有；
- 其余后处理（undistort / render / overlay / depth）仍然全关；
- 起 `record_odin_rgb.py`，把 `/odin1/image/compressed` 的 **JPEG 原样写盘**，
  不解码、不编码，CPU ≈ 0，代价只有 ~4.5 MB/s 的磁盘写。

帧率嫌高、怕占盘就加 `VIDEO_STRIDE=2`（每 2 帧存 1 帧 → 5 fps，约 1.2 GB）。

### 3.1 开车前必须先确认这三件事

```bash
# ① 图像话题真的有 10 Hz（这是本次唯一没在实车验证过的环节）
ros2 topic hz /odin1/image/compressed        # 期望 ≈10 Hz，按 Ctrl-C 停

# ② 录像进程在写盘
tail -f "$MAP/video/record_odin_rgb.log"     # 每 100 帧打一行 "已录 N 帧 / M MB"
ls "$MAP/video/frames" | wc -l               # 数字在涨就对

# ③ 遥控真的能开车（DT7=1 时没有 /car/status，别去找它）
#    推杆看车轮转不转；或者另开一个终端前后对比位姿：
ros2 topic echo /odin1/odometry --once
```

**① 没数据就不要开车**（录像会空）。按顺序查：

```bash
grep -E 'Custom map mode' "$MAP/odin.log" | tail -1                       # 要 1
grep -n sendrgb "$HOME/odin_ws/src/odin_ros_driver/config/control_command.yaml"
```

第 2 条里必须是 `sendrgbcompressed: 1`。如果还是 0，说明生成副本那步没生效，
直接手改驱动配置再重启驱动：`stop_mapping.sh` → 手动 `sed` → 重跑本步骤。

### 3.2 遥控走场（5–10 分钟）

- 匀速、转弯走大弧，**别长时间原地打转**（点云质量会差）；
- 演示路线尽量覆盖到，门框/走廊处慢一点、多走一点；
- 别站雷达正前方、别关灯。

### 3.3 收尾

```bash
SAVE_TIMEOUT=300 bash ~/car_slam_lap/x5/scripts/save_map.sh   # 等 "Map transfer completed"
bash ~/car_slam_lap/x5/scripts/stop_mapping.sh  # 停录像/录点/桥接/驱动
```

**`SAVE_TIMEOUT` 一定要放大**：默认只等 150 s，而 239 MB 的地图要传约 140–180 s。
2026-11-05 第一次就是 150 s 超时后 `stop_mapping.sh` 把驱动杀了，`odin_map/` 是空目录，
整张图白建。收尾后必须确认：

```bash
ls -lh "$MAP/odin_map"      # 应有一个 ~239 MB 的 map_*.bin，不是 0
```

录像停止 = `stop_mapping.sh` 里那句 `stop_pat "[r]ecord_odin_rgb"`（发 SIGINT，
脚本会写 `meta.json` 再退出）。这就是你说的"暂停录制后结束"。

产物自检：

```bash
ls -lh "$MAP"                       # map.pcd / map_bev.png / path.yaml / odin_map/ / video/
cat "$MAP/video/meta.json"          # frames_written / measured_fps 应该是 10 左右
du -sh "$MAP/video"                 # 大约 1.3–2.7 GB
```

---

## 4. 出片：板上 ffmpeg 直接编码（约 6 分钟），只拉 80 MB 回去

板上自带 `ffmpeg`（含 libx264），直接在板端把 3300 张 JPEG 编成 H.264，
比"2 GB 帧拉回 PC 再编"快得多（2026-11-05 实测：板端 6 分钟、输出 83 MB、5.5 分钟片）：

```bash
cd ~/car_ws/maps/venue3-20261106b
ffmpeg -y -framerate 10 -pattern_type glob -i "video/frames/frame_*.jpg" \
  -vf "scale=1280:-2" -c:v libx264 -preset veryfast -crf 24 \
  -pix_fmt yuv420p -movflags +faststart mapping_1280.mp4
```

`scale=1280:-2` 的 `-2` 不能省：H.264 要求宽高都是偶数，奇数高度直接报
`Slice parameters ... invalid` 并写出 0 字节文件。

PC 端只拉成片和地图产物：

```powershell
$MAP = "venue3-20261106b"
$R = "sunrise@10.77.156.239:/home/sunrise/car_ws/maps/$MAP"
New-Item -ItemType Directory -Force "E:\RM\car_slam_lap\demo\$MAP\odin_map" | Out-Null
foreach ($f in @("mapping_1280.mp4","map.pcd","map_bev.png","map_info.txt","path.yaml")) {
  scp "${R}/$f" "E:\RM\car_slam_lap\demo\$MAP\"
}
scp "${R}/odin_map/map_20261105_202349.bin" "E:\RM\car_slam_lap\demo\$MAP\odin_map\"
```

**PC 端备用方案**（板端 ffmpeg 不可用时）：把 `video/frames` 拉回来，用
`x5\pc\frames_to_mp4.py` —— 它自己会把宽高裁成偶数，并用"写完读回一帧"的方式校验编码器：

```powershell
python "E:\RM\car_slam_lap\x5\pc\frames_to_mp4.py" `
  --frames "E:\RM\car_slam_lap\demo\$MAP\video\frames" `
  --out    "E:\RM\car_slam_lap\demo\$MAP\mapping_pc.mp4" --fps 10 --width 1280
```

- 控制台可能先打一条 `Failed to load OpenH264` —— **无害**，会自动回退到 mp4v 并校验；
  最后必须看到 `[ok] ... (N 帧 ... )`。
- 这条路径的帧率是"实际录到的帧数 / 实际时长"：2026-11-05 第一趟 SD 写不过来，只录到
  2.16 fps，成片就是 2 fps 的幻灯片；第二趟稳定 9.5 fps，成片才顺。

合成完可以把板上的 `frames/` 删掉腾空间：

```bash
rm -rf ~/car_ws/maps/venue3-20261106b/video/frames
```

---

## 5. 重定位（1–2 分钟）——这就是"先检测自己在哪"

```bash
bash ~/car_slam_lap/x5/scripts/start_relocal.sh "$MAP"
```

成功标志：`✓ 重定位成功：map -> odom 已发布`。

- **车必须停在建图那趟的起点 1 m 内、朝向差 < 10°**，这是 Odin1 重定位的硬要求；
- 失败就把车挪到特征多的地方、换个朝向再跑一次。

---

## 6. 示教（手动示例巡航，3–4 分钟）

**先确认桥接没在跑**（示教必须遥控，桥接会抢控制权）：

```bash
pgrep -af 'bridge_node'          # 应该没有输出
bash ~/car_slam_lap/x5/scripts/teach_map_route.sh "$MAP"
```

看到提示后：

1. 把车摆到**演示起点**（车头朝出发方向）；
2. 速度 0.2–0.3 m/s，转弯半径 ≥ 0.5 m，按演示路线走一圈；
3. **末段一路向前开过起点 0.3–0.5 m 再停**，不要倒车、不要原地搓；
4. 停稳后 `Ctrl-C`。

成功标志：`✓ 严格示教完成` + 生成 `route_map.yaml` 和 `route_map.yaml.manifest.json`。

> 示教终点最好让**朝向与起点一致**（闭环）。否则每趟自主都要先原地转 180°——带上力臂
> 补偿 + `RETURN_W_MAX=0.6` 之后转得动（约 9 s），但看起来不如闭环干净。

> 如果报"原始里程计停更 0.5x s" = Odin1 回调队列溢出，重开示教即可，别改安全阈值。

---

## 7. 自主巡航（约 2 分钟）——"回到起点，然后自己走"

```bash
# ① 起底盘桥接（后台，关掉 SSH 也不影响）
setsid nohup ros2 run car_bridge bridge_node > "$MAP/bridge_drive.log" 2>&1 &

# ② 已验证可复现的参数（2026-11-05 连跑两遍都成功）
VMAX=0.2 W_MAX=0.6 RETURN_W_MAX=0.6 \
BASE_OFFSET_X=0.2626 BASE_OFFSET_Y=-0.0471 \
START_POS_TOL_M=0.2 RETURN_TIMEOUT_S=90 MAX_TIME_S=400 \
bash ~/car_slam_lap/x5/scripts/drive_lap.sh "$MAP/route_loop.yaml"
```

四个参数为什么是这个值：

- **`BASE_OFFSET_X/Y` 必带**：Odin1 离转心 0.27 m，原地转一圈传感器位姿就漂 0.2 m 以上，
  于是"回起点→对齐朝向→位置又差了→再补位"死循环，现场看起来就是原地抽搐。
  2026-11-04 `ros2 run car_bridge calibrate_lever_arm` 实测 `(+0.2626, -0.0471)`。
  带上之后回起点 9.0 s、补位往返 0 次。
- **`RETURN_W_MAX=0.6`**：>0.8 rad/s 转太快会抽搐（用户 2026-11-05 连续两次指出）；
  但 <0.3 rad/s 底盘静摩擦大、根本转不动（实测 0.28 rad/s 转 15 s 原地不动）。
  0.5–0.7 是这台车的可用区。
- **`START_POS_TOL_M=0.2`**：默认 0.10 m 太严，会在"对齐/补位"之间多往返一两轮。
- **`VMAX=0.2`**：演示够快又稳，14.5 m 一圈约 79 s。

启动横幅必须是：

```
先回起点再跟踪；限起点附近空旷通道，无自动绕障。
```

流程 = **回起点 → 起始对齐 → 跟踪示教路线 → 终点收口**，成功标志 `结果: 成功` +
`结束原因: 到达终点（位置与朝向均达标…）`。

参数含义：

| 变量 | 值 | 含义 |
| :--- | :--- | :--- |
| `VMAX` | 0.12 → 0.15 | 目标速度上限（m/s） |
| `W_MAX` | 0.35 | 跟踪段角速度上限；**不传默认 1.2，转弯会猛** |
| `RETURN_W_MAX` | 0.30 | 回起点/起始对齐角速度上限 |
| `BASE_OFFSET_X/Y` | +0.2626 / −0.0471 | 力臂补偿（本车已标定，换雷达安装要重标） |
| `START_MAX_OFFSET_M` | 2.0 | 允许离起点多远仍自动回去（只在空旷起点用） |
| `RETURN_TIMEOUT_S` | 150 | 回起点+对齐总预算，放大范围要同步放大 |
| `MAX_TIME_S` | 320 | 总时限，0 = 不限 |

---

## 8. 收尾

```bash
pgrep -f "[b]ridge_node" | xargs -r kill     # 停桥接 → 固件 0.3 s 超时归零
pgrep -x host_sdk_sample | xargs -r kill -INT
sudo shutdown -h now                          # 断电前先关 X5
```

---

## 9. 明天给观众看的清单

| 素材 | 路径 | 说明 |
| :--- | :--- | :--- |
| 建图录像 | `demo\venue3-20261106\建图录像.mp4` | 10 fps 车载视角，边开边建图 |
| 二维俯视地图 | `demo\venue3-20261106\map_bev.png` | 一张图看全场 |
| 三维点云地图 | `demo\venue3-20261106\map.pcd` | 用 CloudCompare 打开、转着看 |
| 轨迹 | 板上 `$MAP/path.yaml`、`lap_actual.yaml` | 示教线 vs 实际线 |

---

## 10. 出事怎么办

| 症状 | 处理 |
| :--- | :--- |
| `/odin1/image/compressed` 没数据 | 驱动配置没生效：查 `grep sendrgbcompressed ~/odin_ws/src/odin_ros_driver/config/control_command.yaml`，应为 1 |
| 录像帧数远低于 10 fps | 板端 I/O 被抢：`VIDEO_STRIDE=2` 降一半，或换高速 SD 卡 |
| `record_odin_rgb` 一帧没收到 | 退出时日志会明确报错；先解决第 ① 项再录 |
| 录像占盘太多 | 5 分钟约 1.3 GB；`--max-seconds` 可限制，或事后删 `frames/` |
| 重定位 90 s 不成功 | 车挪回建图起点 1 m 内 / 换朝向；确认 `lsusb` 还有 2207:0019 |
| 自主跑起来转弯很猛/抽搐 | 确认传了 `W_MAX=0.6 RETURN_W_MAX=0.6`，且 `BASE_OFFSET_X/Y` 非零 |
| 原地转不动、只听到轮子空转 | 转向速率太低（<0.3 rad/s）：抬到 0.6 |
| 回起点来回"补位往返"停不下来 | 没传力臂补偿 `BASE_OFFSET_X=0.2626 BASE_OFFSET_Y=-0.0471` |
| 回放报起点偏差过大 | 人工摆回起点，或放大 `START_MAX_OFFSET_M` 并同步放大 `RETURN_TIMEOUT_S` |
| 出现 `实时避障: {...}` | 误跑了 `drive_lap_obstacles.sh` 或设了 `NAV_PARAMS_FILE`；改用 `drive_lap.sh` |

---

## 11. 2026-11-05 实车验证结果（原"未验证项"已闭环）

1. **`sendrgb` + `sendrgbcompressed` 与建图同跑**：✅ 实测可行。建图那趟 16.26 m、
   点云 40 万点、`.bin` 239 MB 全部正常；`/odin1/image/compressed` 稳在 10 Hz，
   录像线程 CPU 开销可忽略。前提是**两个开关都要开**（见 3 节）。
   已知代价是磁盘：第一趟 9 分钟录了 1.17 GB，SD 写不过来时实测掉到 2.16 fps；
   第二趟稳定 9.5 fps（1.6 GB / 5.8 分钟）。掉帧严重时优先换卡，不是改代码。
2. **PC 端 `frames_to_mp4.py`**：✅ 已用 2037 张真实 1600×1296 帧跑通（宽高裁偶 + 读回校验，
   自动回退到 mp4v）。不过现在**优先用板端 ffmpeg**（见 4 节），又快又是真 H.264。
3. **原地转向**：`W_MAX=0.35 / RETURN_W_MAX=0.30` 在这台车上转不动（静摩擦），
   按第 7 节的 `0.6` 用；并且**必须带力臂补偿**，否则回起点会无限"补位往返"。