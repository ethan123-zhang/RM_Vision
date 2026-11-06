# TB3 + Nav2 官方示例 · WSL2/Gazebo Harmonic 运行记录

> 交接总结、使用教学（含"有没有中文版"）、RM3V3 地图导入的下一步路线，见
> **[任务总结与仿真使用指南.md](任务总结与仿真使用指南.md)**。
>
> **RMUL 2027 3V3 场地已在 Gazebo 里跑通**（12.600 × 8.600 m、台面 z=0、落差 0.503 m，
> 碰撞与比例均已量化验收）：见 **[rm_field/README.md](rm_field/README.md)**。
> 台面高度另有像素级验收（正立面亮带 1734×69 px → 墙高 501.3 mm、墙面近景占画面 62.6%），
> 见该文件 §9；证据图 `rm_field/shots/height_annotated_*.png`。
> GUI 默认加载**带 0.30 m 围挡**的变体世界（图纸外的补丁，只为观感与防掉落），
> 1:1 图纸世界仍是不带围挡的 `rm_field/worlds/rmul2027_3v3_field.sdf`；观感改造过程见 §10。
> 起 GUI：`bash rm_field/gui_final.sh`；跑验收：`bash rm_field/start_server.sh` +
> `python3 rm_field/verify_field.py`；台面高度：`bash rm_field/capture_height.sh`。
>
> **场地已接进 Nav2（2026-11-06）**：地图 `rm_field/maps/rmul2027_3v3_fenced.{pgm,yaml}`
> 按 CAD 几何生成、用顶视渲染独立校核（自由区 12.50 × 8.50 m = 106.25 m²，8/8 判据通过）；
> `bash rm_field/nav2_field.sh` 拉起后 `python3 rm_field/verify_nav2_field.py` 跑两点往返闭环
> （两个目标均 SUCCEEDED、0 次恢复、全程 z 恒定 0.010、不越界）。围挡碰撞也已补测
> （0.7 m/s 撞墙停在 6.1819 m，误差 0.4 mm）。细节见 `rm_field/README.md` §11。

**日期**：2026-11-01　**机器**：Windows `10.0.22631.4751` + WSL `3.0.1.0`（WSLg 1.0.79，内核 `6.18.40.1-microsoft-standard-WSL2`）
**主机 GPU**：NVIDIA GeForce RTX 4060 Laptop（驱动 616.56，24 vCPU / WSL 内 7.6 GiB 内存）

---

## 0. 结论速览

| 项目 | 结果 |
| --- | --- |
| ROS 2 | **Jazzy**（`ros-jazzy-ros-base 0.11.0`，`rclcpp 28.1.22`，`rmw_fastrtps_cpp 8.4.4`） |
| Gazebo | **Harmonic / gz sim 8.15.0**（`gz-harmonic 1.0.0`） |
| Nav2 | **1.3.13**（`navigation2` + `nav2-bringup`，新装） |
| TurtleBot3 | **2.3.6**（`turtlebot3-gazebo 2.3.7`、`nav2-minimal-tb3-sim 1.0.1`，新装） |
| 图形加速 | ❌ **无硬件加速**：OpenGL/Vulkan 全部走 **llvmpipe（CPU 软光栅）**，`Accelerated: no` |
| CUDA / GPU 可见性 | ✅ `nvidia-smi` 能看到 RTX 4060（dxcore 通道正常），**只有图形栈不加速** |
| 仿真实时因子 | **RTF ≈ 0.90**（软件渲染下基本实时） |
| 关键话题 | `/scan` 4.0 Hz、`/odom` 25.5 Hz、`/tf` 42.7 Hz |
| Nav2 端到端 | ✅ `/navigate_to_pose` 导航目标 **SUCCEEDED**（error_code 0，无恢复动作） |
| 场地 → Nav2 | ✅ **RMUL 2027 3V3 场地接进 Nav2**（2026-11-06）：自建地图 12.50×8.50 m 自由区、512×352 @0.025 m/px；两点往返均 SUCCEEDED、0 恢复、RTF 0.94、`/scan` 4.3–5.0 Hz；围挡 0.7 m/s 撞击不越界（停位误差 0.4 mm） |
| 现场 | 验收时在跑：Gazebo Sim 1000×845、RViz 1610×893 两个窗口都开在 WSLg 里。**会话结束前（17:50）已正常关闭**，用 `bringup.sh` 一条命令可原样重启 |

---

## 1. 环境探测

### 1.1 发行版（坑：默认发行版不是 Ubuntu）

```
$ wsl -l -v
  NAME              STATE      VERSION
* docker-desktop    Stopped    2      <-- 默认！
  Ubuntu-22.04      Stopped    2      无 /opt/ros，无 gz
  Ubuntu-24.04      Stopped    2      ROS 2 Jazzy + Gazebo Harmonic
```

此后**所有命令都显式 `-d Ubuntu-24.04`**，否则落进 `docker-desktop` 报
`execvpe(bash) failed: No such file or directory`。

### 1.2 ROS 2 / Gazebo

```
PRETTY_NAME=Ubuntu 24.04.5 LTS
kernel: 6.18.40.1-microsoft-standard-WSL2
ROS_DISTRO=jazzy
gz sim --versions -> 8.15.0
```

`ROS_DISTRO` 默认是空的，任何 shell 都要先 `source /opt/ros/jazzy/setup.bash`。
探测前系统只有 319 个 ROS 包、**0 个 nav2、0 个 turtlebot3** —— 官方示例依赖得自己装。

---

## 2. 图形加速：为什么是软件渲染

实测三处互相印证，结论是**这台机器的 WSL 里没有可用的 GPU OpenGL 通道**：

```
$ glxinfo -B
    Device: llvmpipe (LLVM 20.1.2, 256 bits)
    Accelerated: no
OpenGL renderer string: llvmpipe (LLVM 20.1.2, 256 bits)
OpenGL version string: 4.5 (Compatibility Profile) Mesa 25.2.8

$ vulkaninfo --summary
    deviceName = llvmpipe (LLVM 20.1.2, 256 bits)
    driverName = llvmpipe          # 即 lavapipe，也是 CPU

$ ls /dev/dri
ls: cannot access '/dev/dri': No such file or directory
$ ls /sys/class/drm         -> 不存在
$ grep -i drm /proc/devices -> 空
$ dmesg | grep dxg
misc dxg: dxgk: dxgkio_query_adapter_info: Ioctl failed: -22
```

判断链：

1. Mesa 的 D3D12 后端文件 `d3d12_dri.so` **在**（59 个 DRI 驱动都在），但它必须绑定
   `/dev/dri/renderD128` 才能出硬件 GL；本机**根本没有 DRM 设备节点**（`/dev` 里只有 `misc dxg`）。
2. `dxgkrnl` 的适配器查询持续 `Ioctl failed: -22` → 内核侧拿不到 D3D12 适配器 → 不注册 DRM 设备。
3. `nvidia-smi -L` 却正常（`GPU 0: NVIDIA GeForce RTX 4060 Laptop GPU`）——说明 **CUDA/计算通道通的，图形通道不通**。
4. `wsl --shutdown` 干净重启后再测，`/dev/dri` 依旧不存在 → 排除"内核更新后的残留状态"。

**含义**：Gazebo 的 ogre2 和 RViz 的 OpenGL 都在 CPU 上软跑（rviz2 吃 ~4.1 核、`gz sim -g` 吃 ~2.8 核、`gz sim -s` 吃 ~1.7 核），
但 24 vCPU 足够撑住：**RTF ≈ 0.90，导航闭环正常**。只是帧率低、笔记本会热。

**若之后要打开硬件加速**（本次未验证，供排查方向）：`wsl --update` 升到最新 GPU 组件、
确认 Windows 侧没在嵌套虚拟机里且 GPU 以"非 DDA/纯计算"方式透传、确认 D3D12 驱动支持 WSL GPU 半虚拟化。
判断标准只有一条：`ls /dev/dri` 出现 `card0`/`renderD128`，且 `glxinfo -B` 显示 `Accelerated: yes`。

---

## 3. 装了什么

`sudo` 需要密码，改用 WSL 原生 root 通道（免密）：

```bash
wsl -d Ubuntu-24.04 -u root -e bash -c "export DEBIAN_FRONTEND=noninteractive; apt-get install -y \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-rviz-plugins \
  ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-description \
  ros-jazzy-turtlebot3-msgs ros-jazzy-turtlebot3-teleop \
  mesa-utils vulkan-tools x11-utils imagemagick xdotool"
```

装完：`navigation2 1.3.13`、`nav2-bringup 1.3.13`、`nav2-minimal-tb3-sim 1.0.1`（官方示例的世界/机器人模型）、
`turtlebot3 2.3.6`、`turtlebot3-gazebo 2.3.7`。ROS 源已是清华镜像，`apt` 秒装。

---

## 4. 跑起来的步骤

```bash
# 一键：清场 → 启动 → 发初始位姿 → 等导航栈全 active
wsl -d Ubuntu-24.04 -e bash -c "bash /mnt/e/RM/tb3_gazebo_nav2/bringup.sh"

# 发一个导航目标（等价 RViz 里点 "Nav2 Goal"）
wsl -d Ubuntu-24.04 -e bash -c "bash /mnt/e/RM/tb3_gazebo_nav2/nav_goal.sh 1.0 -0.5"
```

`bringup.sh` 内部等价的官方原始命令是：

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle
export ROS_DOMAIN_ID=42
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False use_rviz:=True
```

### 实测证据

```
生命周期: /amcl active  /map_server active  /planner_server active  /controller_server active
          /bt_navigator active  /behavior_server active  /smoother_server active
          /velocity_smoother active  /waypoint_follower active      (30 个节点)

$ ros2 action send_goal /navigate_to_pose ... (map 1.0, -0.5)
  pose.position    x: 0.7499  y: -0.5512     <- 从 spawn 点 (-2.0,-0.5) 走到位
  navigation_time  6.975 s
  number_of_recoveries 0
  Result: error_code: 0 / Goal finished with status: SUCCEEDED

$ bash drive.sh   # 直接下发 /cmd_vel 原地转 2s
  /odom orientation.z = 0.802  (≈61°，物理与差速驱动正常)
```

截图与像素统计见 `shots/`（无视觉引擎，用颜色数/非黑占比/帧间差量化）：

| 文件 | 内容 | 统计 |
| --- | --- | --- |
| `02_after_bringup_desktop.png` | bringup 完成后的桌面（Gazebo + RViz 都在） | 1920×1080，非纯黑 98.8%，5791 色 |
| `04_after_nav_desktop.png` | 导航 + 旋转之后的桌面 | 与 02 逐像素比较 **30.3% 变化**，证明画面在实时刷新 |

---

## 5. 踩过的坑（按重要性）

1. **AMCL 不给初始位姿 → 整个 bringup 会中止**（最隐蔽）。
   现象：`global_costmap` 一直 `Timed out waiting for transform from base_link to map`，
   然后 → `Failed to change state for node: planner_server` → `Failed to bring up all requested nodes. Aborting bringup.`
   此时 `planner_server / bt_navigator / behavior_server` 停在 `inactive`，导航不可用。
   原因：AMCL 没初始位姿就不发 `map→odom`，全局代价地图就永远等不到 transform。
   正解：等价于 RViz 的 **2D Pose Estimate**，用话题发（机器人 spawn 在 `(-2.0, -0.5)`）：
   `ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: map}, pose: {pose: {position: {x: -2.0, y: -0.5}, orientation: {w: 1.0}}}}"`
   发完 1 s 内 `map→odom` 就出现，代价地图随即激活。
2. **默认发行版是 `docker-desktop`** —— 不带 `-d Ubuntu-24.04` 全部失败。
3. **`sudo` 要密码** —— 用 `wsl -u root` 装包。
4. **`set -u` 与 ROS `setup.bash` 冲突** —— 启动脚本里 `source` 前必须 `set +u`，否则
   `AMENT_TRACE_SETUP_FILES: unbound variable` 直接退出。
5. **WSL 是 NAT 模式且 localhost 转发被禁**（`wsl: 无法配置 localhost 转发到 WSL`）—— Windows 侧程序
   访问 WSL 内的 ROS 端口不能靠 `localhost`，要么用 WSL 的 IP，要么改镜像网络模式。本次全部在 WSL 内操作，不受影响。
6. **按 X 窗口 `import -window` 抓图不可靠**（WSLg + GL 直出，抓到的近似冻结帧，前后差异 0.02%）；
   要取证用 Windows 侧 `CopyFromScreen` 抓整屏（本目录 `grab.ps1`）。
7. **DSH 沙箱**：`wsl.exe` 在 workspace-write 模式下报 `Wsl/E_ACCESSDENIED`（打不开 WSL 服务的命名管道），
   所有 WSL 调用都需要 `danger-full-access` 提权。

---

## 6. 文件清单

| 文件 | 用途 |
| --- | --- |
| `recon.sh` / `recon2.sh` | 环境探测（OS/ROS/Gazebo/DRM/显示/网络/包统计） |
| `run_nav2_tb3.sh` | 启动官方示例（`headless:=False use_rviz:=True`，固定 `ROS_DOMAIN_ID=42`） |
| `bringup.sh` | **一键**：清场 → 启动 → 发 `/initialpose` → 等 `map→odom` → 打印生命周期 |
| `nav_goal.sh` | 发 `/navigate_to_pose` 目标（默认 `1.0,-0.5`），带反馈 |
| `drive.sh` | 直接下发 `/cmd_vel` 验证物理/渲染链路 |
| `verify.sh` | 话题频率 / 生命周期 / TF / CPU 快照 |
| `facts.sh` | 版本与运行状态汇总（本记录的原始数据） |
| `perf.sh` | 实时因子 RTF 与关键话题频率 |
| `capture_shot.sh` / `shoot.sh` | 抓 X 窗口（注意上面第 6 条局限） |
| `grab.ps1` | Windows 侧整屏截图 |
| `imgstats.ps1` / `imgdiff.ps1` | 截图像素统计 / 帧间差分（无视觉引擎时的量化取证） |
| `stop.sh` | 一键停掉仿真 |
| `logs/nav2_tb3_gui.log` | 完整 launch 日志 |

---

## 7. 现场怎么继续玩 / 怎么停

- 窗口已经在 WSLg 里：**Gazebo Sim**（3D 世界）与 **RViz**（Nav2 的 costmap/规划/导航面板）。<!-- 验收时状态；现已关闭 -->
- 重启：`wsl -d Ubuntu-24.04 -e bash -c "bash /mnt/e/RM/tb3_gazebo_nav2/bringup.sh"`
- 在 RViz 里点 **"2D Pose Estimate"** 重新定位，点 **"Nav2 Goal"** 随便指定目标点；或用 `bash nav_goal.sh x y`。
- 停：`wsl -d Ubuntu-24.04 -e bash -c "bash /mnt/e/RM/tb3_gazebo_nav2/stop.sh"`。
- 重来：`bringup.sh`（幂等，会先清理残留进程）。