# Odin1 × RDK X5 联调记录

> 目标：把奥丁之眼 Odin1 的数据接入 RDK X5（ROS2 Humble），作为机器人的视觉/空间感知源。
> 状态：**已验证跑通**，2026-10-19。

## 1. 硬件连接

| 项 | 做法 |
| :--- | :--- |
| 数据 | Odin1 Type-C → X5 的 USB 3.0 口（蓝色）。实测枚举为 SuperSpeed 5000M，Bus 002 |
| 供电 | Odin1 航插 **12V / 2A（≥15W）**，与 Type-C 分开；Type-C 不供电 |
| 识别标志 | `lsusb` 出现 `2207:0019 Fuzhou Rockchip Electronics Company hawk`；厂商字符串 `manifold` |

## 2. 板端环境（实测）

| 项 | 值 |
| :--- | :--- |
| 系统 | Ubuntu 22.04.5 LTS / aarch64 / kernel **6.1.83**（2026-10-20 重灌新卡实测；旧记录的 5.10.209 属于上一张卡）|
| ROS | `/opt/ros/humble` 已预装（190 个 ros-humble-* deb）；另有 `/opt/tros/humble` |
| 注意 | 默认 `ROS_DISTRO` 为空，任何 shell 都要先 `source /opt/ros/humble/setup.bash` |
| 依赖 | PCL 1.12.1、OpenCV 4.5.4、Eigen 3.4、yaml-cpp 0.7、libusb-1.0-dev、OpenSSL 3.0 均已有 |
| 补装 | `ros-humble-image-transport`、`ros-humble-pcl-conversions`、`python3-colcon-common-extensions` |

## 3. 一次性部署步骤（换板/重装镜像时照做）

```bash
# 1) 依赖
sudo apt update
sudo apt install -y ros-humble-image-transport ros-humble-pcl-conversions python3-colcon-common-extensions

# 2) udev 规则：让普通用户可访问 2207:0019
sudo install -m 644 99-odin-usb.rules /etc/udev/rules.d/99-odin-usb.rules
sudo udevadm control --reload && sudo udevadm trigger

# 3) 源码 + 编译（X5 只有 3GB 内存，先加 swap，再 4 并发）
sudo fallocate -l 4G /swapfile && sudo chmod 600 /swapfile
sudo mkswap /swapfile && sudo swapon /swapfile
mkdir -p ~/odin_ws/src && cd ~/odin_ws/src
git clone https://github.com/manifoldsdk/odin_ros_driver.git   # 或从 PC 拷 odin_ros_driver-main
cd ~/odin_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select odin_ros_driver --parallel-workers 4 \
             --cmake-args -DCMAKE_BUILD_TYPE=Release             # 实测 32 min
```

CMake 用 `uname -m` 自动判别架构，aarch64 会链接 `lib/liblydHostApi_arm.a`，**不需要交叉编译**。

## 4. 日常启动

```bash
~/odin_ws/run_odin.sh          # 前台，Ctrl-C 退出
```

后台常驻：

```bash
nohup ~/odin_ws/run_odin.sh > ~/odin_run.log 2>&1 &
pkill host_sdk_sample          # 停止
```

> 不要用 `ros2 launch odin_ros_driver odin1_ros2.launch.py`：它会顺带拉起 `rviz2`，SSH 无 X11 时直接报错。

首次连接会通过 USB 从设备回读标定文件，写到 `~/.ros/odin_ros_driver/calib.yaml`（备用路径 `src/odin_ros_driver/config/`），所以要用 `sunrise` 身份跑，别用 root。

## 5. 数据验证（2026-10-19 实测）

| Topic | 内容 | 实测 |
| :--- | :--- | :--- |
| `/odin1/cloud_raw` | 原始 DTOF 点云 | **10.26 Hz**，49152 点 × 19 B = **0.93 MB/帧**，≈ **9.6 MB/s** |
| `/odin1/image/compressed` | JPEG 原图 1600×1296 | **10.26 Hz** |
| `/odin1/image` | 解码后 BGR | 同帧率 |
| `/odin1/imu` | IMU | **382 Hz** |
| `/odin1/odometry` / `_highfreq` | 位姿 | 正常 |
| `/odin1/cloud_slam` | SLAM 点云（`custom_map_mode: 1`） | 正常 |
| `/odin1/cloud_render` | 上色点云 | 正常 |

连接耗时约 11 s；驱动常驻内存 227 MB，CPU 约 1.3 核（8 核机器空闲 80%）；USB 实测 3.2（SuperSpeed）。

## 6. 调参与取舍

- 默认 `dtof_fps: 100` / `rgb_fps: 100` → 10 FPS。要 14.5 FPS：两者都改 `145`（必须成对）。
- 只要图像/点云、不要后处理时，关掉这些省 CPU：`sendcloudrender`、`senddepth`、`sendreprojection`、`sendoverlay`、`sendcloudslam`。
- `strict_usb3.0_check: 0`：USB2 也能连，但 SLAM 建图文件传输不可靠，正式用必须 USB3。
- 时间戳：`use_host_ros_time: 0`（设备时间，推荐）。

## 7. 本次会话解决的问题

| 现象 | 根因 | 处理 |
| :--- | :--- | :--- |
| Windows 设备管理器「其他设备 → hawk」黄色感叹号 | Odin1 是厂商自定义类（FF/AD/01），Windows 无内置驱动 | 用 OdinViewer 包内 `USBTools\zadig-2.9.exe` 绑 WinUSB |
| X5 上 `ros2` 命令找不到 | shell 未 source | `source /opt/ros/humble/setup.bash` |
| X5 缺 `image_transport` / `pcl_conversions` / `colcon` | 镜像只预装子集 | apt 补装 |
| 编译 OOM 风险（3 GB 内存） | PCL 模板实例化吃内存 | 4 GB swap + `--parallel-workers 4` |

## 8. `sendcloudrender` 开关实测（2026-10-19）

同一台 X5、同一个 Odin1、同一场景，每次改完 `control_command.yaml` 后重启驱动，等设备连接完成再测。
CPU 取自 `/proc/<pid>/stat` 的 utime+stime 在 10 s 内的增量；带宽取自 `ros2 topic bw`。

| 指标 | `sendcloudrender: 1` | `sendcloudrender: 0` | 差值 |
| :--- | ---: | ---: | ---: |
| 驱动 CPU（单核百分比） | **126.0 %** | **99.7 %** | −26.3 个百分点（约 0.26 核） |
| 常驻内存 RSS | 238.7 MB | 201.9 MB | −36.8 MB |
| `/odin1/cloud_render` 出流量 | 2.85 MB/s | 0 | −2.85 MB/s |
| `/odin1/cloud_raw` 出流量 | 9.80 MB/s | 9.60 MB/s | ≈ 0（波动） |
| `/odin1/image/compressed` | 5.18 MB/s | 5.16 MB/s | ≈ 0（波动） |

结论：
1. `sendcloudrender` 是**纯主机侧**开销，关掉它**不会减少设备→主机的 USB 数据量**（cloud_raw/image 完全不变）。
2. 代价是 0.26 个核 + 37 MB 内存 + 2.85 MB/s 的 ROS 序列化流量，换来 `/odin1/cloud_render` 的彩色点云。
3. 关掉后 topic 仍然存在（publisher 在初始化时创建），但**没有数据**（`ros2 topic hz` 无输出）——排查时别被 `ros2 topic list` 骗了。
4. `senddepth` / `sendreprojection` / `sendoverlay` 只影响 `pcd2depth_ros2_node` / `cloud_reprojection_ros2_node` / `image_overlay_node` 三个**独立进程**；用 `ros2 run` 只起主节点时，这三个开关改了也没有任何效果。

## 9. 效果展示

[`demo/odin1_x5_demo.png`](demo/odin1_x5_demo.png)：同一帧的
① RGB 原图、② 点云深度图（0.2–5.6 m 伪彩）、③ 深度叠在 RGB 上。
③ 中吸顶灯的轮廓与深度色块完全重合，说明内参/外参标定与时间同步都正确。

采集脚本 [`capture_one_frame.py`](capture_one_frame.py) 在板端用 rclpy 直接抓一帧
`/odin1/image/compressed`、`/odin1/cloud_raw`、`/odin1/cloud_render` 存成 npz，再拷回 PC 渲染。
投影用的是设备下发的 `~/.ros/odin_ros_driver/calib.yaml`（FishPoly 鱼眼模型 + `Tcl` 外参）。

## 10. 网页实时预览（2026-10-19）

板端跑一个轻量 web 服务，三路 MJPEG 推流，电脑浏览器直接看，不需要在 PC 装 ROS：

```
http://172.17.90.239:8080/          # 同网段直连（2026-10-20 重灌后的地址）
http://127.0.0.1:8080/              # 经 SSH 端口转发，见 §11
```

| 路径 | 内容 |
| :--- | :--- |
| `/rgb.mjpg` | RGB，1600x1296 解码后转正、缩到 648x800 |
| `/depth.mjpg` | `/odin1/cloud_render` 按标定投影成距离图（turbo 伪彩） |
| `/overlay.mjpg` | 深度叠加在 RGB 上，用来判断对齐 |
| `/stats` | JSON：帧数、点数、距离范围、单帧渲染耗时 |
| `/rgb.jpg`、`/depth.jpg`、`/overlay.jpg` | 单帧快照，方便截图 |

实测（X5，USB3 直连）：

| 指标 | 值 |
| :--- | ---: |
| RGB 推送 | 10.25 Hz |
| 点云推送 | 8.9 Hz（受渲染耗时限制，丢帧不排队，总是最新帧） |
| RGB 单帧渲染 | ~58 ms |
| 深度单帧渲染 | ~33 ms |
| 进程 CPU | ~111 %（合计约 1.1 核，含 OpenCV 多线程） |
| 单路 RGB 流带宽 | ~730 KB/s（三路全开约 1.6 MB/s ≈ 13 Mbps） |

实现要点（踩过的坑）：
1. **ROS 回调只负责存最新一帧**，解码/投影/编码全交给单个 worker 线程。第一版把渲染放在回调里，
   `PointCloud2` 订阅会被 GIL + 阻塞拖死（只收到 1 帧后就静默无数据，`ros2 topic list` 却一切正常）。
2. RGB 用 `cv2.IMREAD_REDUCED_COLOR_2` 直接按 1/2 解码（800x648），省掉全尺寸解码 + INTER_AREA 缩放，
   单帧从 124 ms 降到 58 ms。
3. 深度图在 1/4 网格上画点再放大（1600x1296 → 400x324），比全分辨率 canvas 便宜约 16 倍。
4. 设备侧着装的，服务端统一旋转 90°。

启停：

```bash
bash ~/odin_ws/start_all.sh     # 幂等，已在跑就只打印 URL
bash ~/odin_ws/stop_all.sh
```

> 电脑上打不开时先怀疑代理：把 `172.17.90.0/24` 和 `127.0.0.1` 加进 Clash/系统代理的直连（bypass）列表，
> 否则请求会被代理转发到外网而失败。`curl --noproxy '*'` 可以绕开代理做排查。

## 11. 换卡重灌（2026-10-20 实操）

原 SD 卡损坏，重新烧镜像后按本节从头恢复。镜像与校验值沿用 `docs/environment.md`
（`rdk-x5-ubuntu22-preinstalled-desktop-3.5.0-arm64.img`，10,393,485,312 B，SHA256 `02FD…C2284`）；
烧录用 Rufus / balenaEtcher 写卡，首次上电自动扩容 rootfs。

### 11.1 重灌后的板端指纹（实测）

| 项 | 值 |
| :--- | :--- |
| 系统 | Ubuntu 22.04.5 LTS / kernel **6.1.83** / aarch64 |
| 内存 | 3062 MB，**默认无 swap**（编译前必须加，脚本会自动建 4 GB） |
| rootfs | 29 GB，可用 19 GB（32 GB 卡） |
| 网络 | DHCP `172.17.90.239`，SSH `sunrise / sunrise` |
| host key | `SHA256:K8oyEoddaZ4keNu39exAqMJLDEcncgoQGcRPQ898Bb4`（换卡会变，plink 首次要 `-hostkey` 钉住） |
| Odin1 | `2207:0019 hawk` / `manifold`，Bus 002，SuperSpeed 5000M，serial `efc65c8722eaf74f` |
| ROS | `/opt/ros/humble` + `/opt/tros/humble`，190 个 ros-humble 包，numpy 1.26.4 / cv2 4.11.0 / rclpy 可用 |

### 11.2 一键重建

```powershell
# PC：打包（脚本 + 驱动源码，约 500 KB）
pwsh -File x5_odin\make_bundle.ps1
pscp -pw sunrise x5_odin\odin_deploy.zip sunrise@172.17.90.239:/home/sunrise/
```

```bash
# 板端：解包 + 安装（幂等，可重复跑）
cd ~ && unzip -o odin_deploy.zip -d odin_deploy && bash odin_deploy/install_odin.sh
```

`install_odin.sh` 依次做：apt 依赖（`image-transport` / `pcl-conversions` / `colcon`）→ 4 GB swap 并写进 fstab
→ udev 规则 → 解包源码到 `~/odin_ws/src/odin_ros_driver` → `colcon build --parallel-workers 4`
（实测 **30 min**，内存吃满 swap）→ 安装运行时脚本。日志：`~/odin_install.log`。

### 11.3 跑起来 + 端口转发到电脑

```bash
# 板端：起驱动 + 网页预览（先等 calib.yaml 再起 viewer）
bash ~/odin_ws/start_all.sh
bash ~/odin_ws/stop_all.sh        # 停止
```

```powershell
# 电脑：把板端 8080 转到本机 8080，并校验 /stats 与三路快照
pwsh -File x5_odin\pc\verify_view.ps1 -X5Ip 172.17.90.239
# 或双击 x5_odin\pc\odin_view.bat（后台开隧道 + 打开浏览器）
# 只要隧道：x5_odin\pc\odin_tunnel.bat 172.17.90.239
```

浏览器开 `http://127.0.0.1:8080/`，三路 MJPEG + `/stats` JSON。隧道就是一条
`plink -batch -hostkey … -pw sunrise -L 8080:127.0.0.1:8080 -N sunrise@<IP>`，不需要在 PC 上装 ROS。

### 11.4 重灌后实测（2026-10-20）

| 指标 | 本次 | 旧卡记录 |
| :--- | ---: | ---: |
| `/odin1/cloud_raw` | **10.11 Hz** | 10.26 Hz |
| `/odin1/image/compressed` | **10.31 Hz** | 10.26 Hz |
| `/odin1/imu` | **355 Hz** | 382 Hz |
| viewer `/stats` | n_rgb 829 / n_cloud 996，20079 点，0.15–2.85 m，渲染 52 / 56 ms | ~10.25 / 8.9 Hz |
| 单帧抓取 | cloud_raw 49152 点、cloud_render 19981 点、RGB 450,721 B | 49152 / 17024 |
| 设备连接耗时 | 11 s | ~11 s |

采集链路：`capture_one_frame.py`（板端）→ `pscp` 拉回 → `render_npz.py`（PC）出三联图
`demo/odin1_x5_demo.png`。本次产物在 `demo/`，原始 npz 在 `x5_odin/capture/2026-10-20/`（不入库）。

### 11.5 重灌才暴露的坑

1. **apt 在 IPv6 上假死**：校园网 IPv6 通但被黑洞，`apt-get update` 挂在连接上（实测 6 min 零输出、CPU 0.1%）。
   `install_odin.sh` 会写 `/etc/apt/apt.conf.d/99odin-ipv4`（`ForceIPv4` + 20 s 超时）解决。
2. **apt 索引约 200 MB + `apt-show-versions` post-invoke**：在 SD 卡上扫描要额外几分钟，
   整段 `update` 约 10 min，看着像卡死其实在干活；看 `ps` 里的 `apt-show-versions` 就知道。
3. **`start_all.sh` 必须等 `calib.yaml`**：驱动连上设备后才回读标定（约 11 s），viewer 只在启动时读一次；
   早于它起 viewer，`depth`/`overlay` 两路永远空白（`/stats` 的 `n_cloud` 为 0）。已改成等文件出现再起。
4. **plink 0.85 没有 `-keepalive`**：写了会直接 `unknown option "-keepalive"`。
5. **`plink -batch` 不会自动接受新 host key**：必须先裸跑一次拿到指纹，再用 `-hostkey SHA256:…` 连。
6. 编译期 3 GB 内存必然吃 swap（两个 PCL 重模板 TU 各占 1.3 GB），`--parallel-workers 4` 是实测折中。
