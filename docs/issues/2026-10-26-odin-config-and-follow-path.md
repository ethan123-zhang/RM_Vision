# car_slam_lap 首次端到端跑通：三个“看起来像硬件/算法”的软件 bug

- 日期：2026-10-26
- 环境：RDK X5（10.115.150.239）/ Odin1 / C8T6 + SN65HVD230 + XHSMCUW50 / DT7
- 关联：`car_slam_lap/README.md`、`car_slam_lap/x5/scripts/*`、
  `car_slam_lap/x5/car_bridge/car_bridge/follow_path.py`、
  `docs/issues/2026-10-22-rc-dbus-not-decoded.md`

## 结论

**建图 → 重定位 → 遥控示教回放，整条链路首次跑通。** venue2 实测：

| 项 | 数值 |
| :--- | :--- |
| 录制轨迹 | 330 点 / 15.66 m / 131.6 s（`frame_id: odom`） |
| 地图 `.bin` | 147 MB，`Map transfer completed`，存图时刻在车停稳之后 |
| 重定位 | `map←imu` = (0.834, −0.117) / yaw +2.4°，与 path 末点 (0.83, −0.15) / +3.0° 相差 **3 cm / 0.6°** |
| 自动回放 | 249 点 / 14.19 m / 167.6 s；跟踪误差 **rms 0.103 m、max 0.423 m**；终点位置差 0.48 m、朝向差 3.0° |

过程中修掉三个软件 bug，**都不是地图或算法问题**：

1. Odin1 驱动**不读** `-p config_file:=...`，重定位配置从未生效（表现为“重定位失败”）；
2. `save_map.sh` 找错存图目录、且等固定 8 s 就拷（表现为“存图命令没生效”）；
3. `follow_path` 启动竞态 + `save_actual` 名称遮蔽 + SIGINT 异常（表现为“一启动就退出/退出时崩溃”）。

---

## 1. Odin1 驱动的 `config_file` 参数是空操作

### 现象

`start_relocal.sh` 生成 `custom_map_mode: 2` 的配置并用
`--ros-args -p config_file:=...` 启动驱动，等 90 s 也没有 `map` 帧；
驱动日志里却是 `Custom map mode: 1`、`relocalization_map_abs_path:` 为空。

### 根因

`odin_ros_driver/src/host_sdk_sample.cpp`：

```cpp
std::string config_file = get_package_source_directory() + "/config/control_command.yaml";
g_parser = std::make_shared<odin_ros_driver::YamlParser>(config_file);
```

`get_package_source_directory()` 用 `__FILE__` 回溯到 `~/odin_ws/src/odin_ros_driver`，
**代码里没有 `declare_parameter("config_file")`**。所以所有 `-p config_file:=` 都是空操作：
- 之前两次建图实际跑的是**厂商默认配置**（恰好 `custom_map_mode: 1`，所以“看起来正常”，
  但 RGB/原始点云全开，白烧 CPU/USB）；
- 重定位根本没进 mode 2，自然等不到 `map` 帧。

### 修复

- `env.sh` 新增 `ODIN_DRIVER_CFG` 与 `odin_use_config <yaml>`：备份一次厂商默认到
  `.vendor`，再把目标配置写进驱动真正读的那个文件；`odin_restore_vendor_config` 用于还原。
- `start_mapping.sh` / `start_relocal.sh`：启动驱动前调用 `odin_use_config`，
  删掉无效的 `-p config_file`。
- `stop_mapping.sh`：收尾时恢复厂商默认配置（避免影响 `x5_odin` 的网页预览）。

### 验证（板端，2026-10-26）

`start_relocal.sh ~/car_ws/maps/venue2` →
`Custom map mode: 2` → `relocalization success!`（约 17 s）→ `tf2_echo map imu` 有输出。
换回映射配置再跑 `start_mapping.sh` 时同样能从日志确认 `Custom map mode: 1`。

---

## 2. `save_map.sh` 只看一个目录、只等 8 s

### 现象

`✗ /home/sunrise/odin_ws/src/odin_ros_driver/map 是空的：存图命令没生效`，
但驱动日志里 `Map save triggered` / `Map transfer completed` 都正常。

### 根因

驱动按 `COLCON_PREFIX_PATH` 里**第一个** `/install` 推导存图目录：

```cpp
workspace_path = getenv("COLCON_PREFIX_PATH");
if (含 "/install") map_dir = <install 前缀>/src/odin_ros_driver/map;
```

`env.sh` 先 source `~/car_ws` 时，顺序是 `car_ws/install:odin_ws/install`，
于是图存到 `~/car_ws/src/odin_ros_driver/map/<启动时间>/`，而旧脚本只认 `~/odin_ws/...`。
另外存图是异步的（SDK 最多 120 s），旧脚本 `sleep 8` 就拷，可能拷到半个文件。

### 修复

- 优先从驱动日志里读 `Map transfer completed: <path>` 的真实路径，读不到再扫三个候选目录；
- 轮询等完成（默认 150 s，`SAVE_TIMEOUT` 可调），失败时打印排查用日志行；
- `env.sh` 末尾 source `~/odin_ws`，让 `COLCON_PREFIX_PATH` 里 odin_ws 排最前
  （以后图写回 `~/odin_ws/src/...`，与脚本默认一致）。

---

## 3. `follow_path.py` 的三处缺陷

### 现象

1. 刚启动就退出：`结束原因: 没收到 /car/status（car_bridge 没起？串口断了？）`；
2. 退出时 `TypeError: 'str' object is not callable`（`node.save_actual()`）。

### 根因

1. **启动竞态**：状态/TF 联锁判定在 `start_delay_s` 之前执行。节点刚起来 DDS 发现 +
   tf2 缓冲填充要 100–500 ms，第一条 `/car/status` 还没到就判“丢链”并退出；
2. **名称遮蔽**：`self.save_actual = <参数值字符串>` 把同名的 `save_actual()` 方法遮住，
   最后 `node.save_actual()` 变成“调用字符串”；
3. Humble 上 SIGINT 会让 rclpy 直接关 context，随后 `spin_once` 抛 `RuntimeError`
   （`Unable to convert call argument to Python object`）而不是 `ExternalShutdownException`。

### 修复

1. `start_delay` 期间只发 0（宽限期），过了宽限期才启用 status/TF 联锁与 `max_time`；
2. 参数值改存 `self._save_actual_path`；
3. 自己接管 SIGINT：置标志 → `_stop("手动 Ctrl-C")` 发 0 → 存实际轨迹 → 正常退出。

### 验证（板端安全测试）

`v_max=0`（车不动）跑 16 s 后发 SIGINT：

```
[INFO] 开始跟随
[INFO]   0.0/15.7 m  剩余 15.7 m  横向误差 0.001 m
结束原因: 手动 Ctrl-C
实际轨迹 1 点
[INFO] 实际轨迹写到 /tmp/actual_test2.yaml      ← 无 traceback
```

---

## 4. 回放结果与待改进

- 功能通过：完整跑完录制路线、无碰撞，`lap_actual.yaml` 已落盘；
- **rms 0.103 m / max 0.423 m**，max 出现在远端 180° 掉头处——纯追踪会切角
  （实际最远只到 6.50 m，录制时是 7.43 m），最终位置偏差 0.48 m，略超 A5 的
  “终点 < 0.3 m”判据；
- 待调方向：掉头段降低 `lookahead_min`、或对 180° 段单独降速；速度目前 `VMAX=0.10`。

## 5. 可复用结论

- **ROS 参数是否存在，以源码里的 `declare_parameter` 为准**；第三方驱动“文档说有参数”
  不代表真的有。接新驱动先 `ps -o args=` 对照日志里它实际读的配置文件路径。
- 异步落盘/传输的脚本不要“等固定秒数就拷”，要等对方日志里的**完成证据**。
- 节点的“启动宽限期”必须和“联锁超时”分开：DDS/TF 的发现时间不是故障。
- Python 实例属性会遮蔽同名方法，参数名别和方法名撞。
- **判断一张 SLAM 地图能不能用，最快的客观判据是：重定位出的位姿 vs 录制轨迹末点**
  （位置/朝向差）。本次 3 cm / 0.6°，同时验证了地图、轨迹、TF 三者一致。
- 把轨迹叠到 `map_bev.png` 上（`demo/venue2/_overlay.py`）比单看 PCD 更快发现
  “只走了一条直线/没闭环”。

## 6. 本次板端改动清单

| 文件 | 改动 |
| :--- | :--- |
| `x5/scripts/env.sh` | 末尾 source `~/odin_ws`；新增 `ODIN_DRIVER_CFG` / `odin_use_config` / `odin_restore_vendor_config` |
| `x5/scripts/start_mapping.sh` | 用 `odin_use_config` 装建图配置，去掉 `-p config_file` |
| `x5/scripts/start_relocal.sh` | 用 `odin_use_config` 装重定位配置，去掉 `-p config_file` |
| `x5/scripts/stop_mapping.sh` | 收尾恢复厂商默认配置 |
| `x5/scripts/save_map.sh` | 从日志读真实路径、等 `Map transfer completed`、多候选目录 |
| `x5/car_bridge/car_bridge/follow_path.py` | 启动宽限期、`_save_actual_path`、SIGINT 处理 |
| 板端 `~/odin_ws/src/odin_ros_driver/config/control_command.yaml` | 由脚本写入，厂商默认备份为 `.vendor` |
