# 演示指令单（换电脑也能用）

> 一句话：**真正干活的全在车上那块 RDK X5 板子上**（ROS 2、Odin1 驱动、建图、录像、自主）。
> 你的电脑只当"遥控台 + 显示器"，**不需要装 ROS、不需要装厂商 SDK、不需要装驱动**。

---

## 0. 新电脑要准备什么

| 需要 | 说明 |
| :--- | :--- |
| **一个 SSH 客户端** | Windows 10/11 自带 `ssh` / `scp`（设置 → 应用 → 可选功能 → OpenSSH 客户端）。若系统没有，用免安装的 **PuTTY**（`plink.exe` + `pscp.exe` 两个文件，拷过去双击即用） |
| **能放 mp4 的播放器** | Windows 自带"电影和电视 / Media Player"就能放 H.264；不放心就装 **VLC**（免费） |
| **演示素材（拷贝，不用安装）** | 整个 `demo\venue3-20261106b\` 文件夹：`mapping_1280.mp4`、`map_bev_hd.png`、`map_only.png`、`map.pcd`、`route_vs_actual_final.png`、`lap_actual_final.yaml`、`route_loop.yaml`、各 `*.log` |
| 可选：Python 3 + numpy + Pillow | 只有想在新电脑上**重新渲染点云图**时才需要；成片和图已经出好了，演示不用 |

**不需要**：ROS 2、Nav2、Odin1 SDK、ROS 驱动、任何 C++ 编译环境。

---

## 1. 连上板子

先确认板子 IP（**换网络后 IP 会变**，见第 9 节）。

```powershell
# Windows 自带 OpenSSH（推荐，第一次会自动记指纹）
ssh sunrise@<板子IP>
# 密码：sunrise
```

PuTTY 版（不想装任何东西）：

```powershell
E:\RM\_downloads\putty\plink.exe -ssh -pw sunrise sunrise@<板子IP>
```

> 旧电脑用的 IP 是 `10.77.156.239`，hostkey `SHA256:K8oyEoddaZ4keNu39exAqMJLDEcncgoQGcRPQ898Bb4`。
> 新电脑/新网络请用 `-o StrictHostKeyChecking=accept-new` 让它自己记住新指纹。

---

## 2. 上电自检（板端）

```bash
# 设备在不在
lsusb | grep 2207:0019                 # Odin1 应该有一行

# 磁盘够不够（录像会写 1.5–3 GB）
df -h ~

# 清场：别留旧进程
pkill -f "lib/car_bridge/bridge_node$"
pkill -f "lib/odin_ros_driver/host_sdk_sample$"
pkill -f "lib/car_bridge/follow_path$"
```

---

## 3. 建图 + 同步录像（约 10–15 分钟）

```bash
MAP=~/car_ws/maps/demo-$(date +%m%d)
VIDEO=1 DT7=1 bash ~/car_slam_lap/x5/scripts/start_mapping.sh "$MAP"
```

- **`DT7=1` 必须带**：不带的话脚本会起桥接，桥接每 50 Hz 发一帧速度指令，
  固件仲裁里"串口优先于遥控器"，**遥控器会完全失灵**（实测推杆车不动）。
- 开车前确认图像流真的在出帧：

```bash
ros2 topic hz /odin1/image/compressed     # 期望 ≈10 Hz
ls "$MAP/video/frames" | wc -l            # 数字在涨
```

- 遥控走场 5–10 分钟：匀速、转弯走大弧、别长时间原地打转。

---

## 4. 存图 + 收尾 + 出片

```bash
# 存图：239 MB 的 .bin 要传 2–3 分钟，默认 150 s 会超时导致地图丢失
SAVE_TIMEOUT=300 bash ~/car_slam_lap/x5/scripts/save_map.sh
ls -lh "$MAP/odin_map"                     # 确认有 ~200+ MB 的 bin，不是 0

bash ~/car_slam_lap/x5/scripts/stop_mapping.sh

# 板端直接出 H.264（比把 2 GB 帧拉回电脑再编快得多）
cd "$MAP"
ffmpeg -y -framerate 10 -pattern_type glob -i "video/frames/frame_*.jpg" \
  -vf "scale=1280:-2" -c:v libx264 -preset veryfast -crf 24 \
  -pix_fmt yuv420p -movflags +faststart mapping_1280.mp4
```

> `scale=1280:-2` 的 `-2` 不能省：H.264 要求宽高都是偶数，奇数高度会报
> `Slice parameters invalid` 并写出 0 字节文件。

拉回电脑（在新电脑上执行）：

```powershell
$IP = "<板子IP>"
$M  = "demo-<日期>"
scp "sunrise@${IP}:~/car_ws/maps/$M/mapping_1280.mp4" .
scp "sunrise@${IP}:~/car_ws/maps/$M/map.pcd"          .
scp "sunrise@${IP}:~/car_ws/maps/$M/map_bev.png"      .
scp "sunrise@${IP}:~/car_ws/maps/$M/map_info.txt"     .
```

---

## 5. 重定位（1–2 分钟）

```bash
bash ~/car_slam_lap/x5/scripts/start_relocal.sh "$MAP"
```

成功标志：`✓ 重定位成功：map -> odom 已发布`。

**硬要求**：车停在建图那趟的起点 **1 m 内、朝向差 < 10°**。失败就挪车换朝向再跑。

---

## 6. 示教（遥控走一圈，3–4 分钟）

先确认桥接没在跑（示教必须遥控）：

```bash
pgrep -af "lib/car_bridge/bridge_node$"        # 应该没输出
bash ~/car_slam_lap/x5/scripts/teach_map_route.sh "$MAP"
```

1. 把车摆到演示起点、车头朝出发方向；
2. 速度 0.2–0.3 m/s，转弯半径 ≥ 0.5 m，按演示路线走一圈；
3. 末段**一路向前开过起点 0.3–0.5 m 再停**，不倒车、不原地搓；
4. 停稳后 `Ctrl-C`。

成功标志：`✓ 严格示教完成` + 生成 `route_map.yaml`。

> 想重教又不让覆盖旧文件：加 `OUT_PATH="$MAP/route_2.yaml"`。

---

## 7. 自主走圈（约 2 分钟）

```bash
# ① 起桥接（自主必须开，急停开关 S2 仍然最高优先级）
setsid nohup ros2 run car_bridge bridge_node > "$MAP/bridge.log" 2>&1 &

# ② 已验证参数（连跑多次成功，慢速、不抽搐）
VMAX=0.2 W_MAX=0.6 RETURN_W_MAX=0.6 RETURN_TIMEOUT_S=90 START_POS_TOL_M=0.2 \
BASE_OFFSET_X=0.2626 BASE_OFFSET_Y=-0.0471 MAX_TIME_S=400 \
bash ~/car_slam_lap/x5/scripts/drive_lap.sh "$MAP/route_map.yaml"
```

四个参数别省：

| 参数 | 为什么 |
| :--- | :--- |
| `BASE_OFFSET_X/Y` | Odin1 离转心 0.27 m，不补偿原地转会让位姿漂 0.2 m，回起点无限"补位往返"（看起来像抽搐） |
| `W_MAX=0.6 RETURN_W_MAX=0.6` | >0.8 转太快会抽搐；<0.3 底盘静摩擦转不动；0.5–0.7 是可用区 |
| `START_POS_TOL_M=0.2` | 默认 0.10 太严，会在"对齐/补位"之间多往返 |
| `RETURN_TIMEOUT_S=90` | 回正 + 对齐要留足时间 |

成功标志：

```
结果: 成功
结束原因: 到达终点（位置误差 …，朝向误差 …）
横向误差 max=… rms=…
```

可选：`RETURN_REVERSE=1` 打开"车尾对准→倒车→转正"的倒车三拍（默认关闭）。

---

## 8. 收尾

```bash
pkill -f "lib/car_bridge/follow_path$"
pkill -f "lib/car_bridge/bridge_node$"
pkill -f "lib/odin_ros_driver/host_sdk_sample$"
bash ~/car_slam_lap/x5/scripts/stop_mapping.sh   # 恢复厂商配置（若上次建图没恢复）
```

---

## 9. 换电脑 / 换网络必看

1. **板子 IP 会变**。三种拿法：
   - 给板子接显示器键盘，登录后 `ip a`（找 `inet 10.x` 或 `192.168.x`）；
   - 看路由器后台的已连接设备（主机名一般是 `ubuntu`/`rdkx5`）；
   - 手机热点：把板子和电脑连同一个热点，再 `ip a`。
2. **SSH 指纹会变**（同一个 IP 换了机器/重装时）。报 `REMOTE HOST IDENTIFICATION HAS CHANGED`
   就删掉旧记录：`ssh-keygen -R <IP>`。
3. 板子需要 **12 V 电源 + Odin1 的 12 V/2 A**，网线或同一 Wi-Fi 与电脑互通。
4. 演示素材已经烤在 `demo\venue3-20261106b\`，**不需要在演示现场重新建图**：
   现场只需第 5 步（重定位）+ 第 7 步（自主），3 分钟就能跑起来。

---

## 10. 故障速查

| 现象 | 处理 |
| :--- | :--- |
| 遥控推杆车不动 | 桥接还在跑：`pkill -f "lib/car_bridge/bridge_node$"` |
| 建图后 `odin_map/` 是空目录 | 存图超时被杀：`SAVE_TIMEOUT=300` 重来 |
| 录像一帧没有 | 驱动配置里 `sendrgb` 和 `sendrgbcompressed` 必须**同时**为 1 |
| 重定位 90 s 失败 | 车挪回建图起点 1 m 内、换朝向；`lsusb` 确认 2207:0019 在 |
| 自主原地转不动 / 抽搐 | 检查力臂补偿和 `RETURN_W_MAX=0.6` |
| 回起点来回补位停不下来 | 没传 `BASE_OFFSET_X/Y` |
| 命令里带 `pkill -f 某路径` 把 SSH 自己杀了 | 进程名要**锚定结尾**：`pkill -f "lib/car_bridge/follow_path$"` |