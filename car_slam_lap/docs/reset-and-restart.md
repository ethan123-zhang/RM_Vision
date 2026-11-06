# 建图 / 示教断了：怎么清干净再来一遍

> 一条铁律：**先让进程自己收尾（SIGINT → stop_mapping.sh），再动文件。**
> 顺序反了，`record_path` 会把半截轨迹写回 yaml、`map_from_cloud` 会写出残缺的 `map.pcd`，
> 清完还带着"看起来有、其实不能用"的假数据。

---

## 0. 先判断断在哪一步

```bash
D=~/car_ws/maps/<地图名>          # 不知道名字就 ls -lt ~/car_ws/maps | head

ls -lh "$D"/odin_map/*.bin        # ① 地图本体在不在（~200 MB 才算成）
ls -l  "$D"/map.pcd "$D"/map_bev.png "$D"/map_info.txt   # ② 点云/俯视图在不在
ls -l  "$D"/route_map.yaml*       # ③ 示教 bundle：有 yaml 也要有 .manifest.json
tail -5 "$D"/odin.log             # ④ 是否卡在 "waiting for device to finish generation"
ls "$D"/video/frames | wc -l      # ⑤ 帧数还涨不涨（隔 10 秒再看一次）
```

| 症状 | 结论 | 走哪个分支 |
| :--- | :--- | :--- |
| 没有 `.bin`，或 `odin_map/` 是空目录 | 地图没存成 | **B 整趟重来** |
| 有 `.bin`，但 `map.pcd` 缺失 | 点云导出被硬杀（地图本身还能用） | **A 重示教**（点云就别要了） |
| `route_map.yaml` 有、`manifest.json` 没有 | 示教没正常收尾 | **A 重示教** |
| `odin.log` 停在 `waiting for device to finish generation` | 存图把驱动卡死了 | **C 存图卡死** |
| 帧数不再增长 | RGB 队列溢出，录像已断 | 建图可继续，但**存图前必须先停录像** |

---

## 1. 通用第 0 步：停干净（顺序不能乱）

```bash
# ① 录制器：只发一次 SIGINT，让它自己收尾写盘（连按第二下 = SIGKILL，bundle 就废了）
pkill -INT -f "record_path[.]py$"
pkill -INT -f "record_odin_rgb[.]py$"
sleep 2

# ② 点云导出 / 桥接 / 跟随
pkill -f "lib/car_bridge/map_from_cloud$"
pkill -f "lib/car_bridge/bridge_node$"
pkill -f "lib/car_bridge/follow_path$"

# ③ 停驱动 + 恢复厂商配置（这一步会把 Odin1 配置改回原样）
bash ~/car_slam_lap/x5/scripts/stop_mapping.sh

# ④ 确认真的干净了（必须为 0）
ps -ef | grep -E "[h]ost_sdk_sample|[b]ridge_node|[f]ollow_path|[r]ecord_path|[r]ecord_odin_rgb|[m]ap_from_cloud" | wc -l
```

> `pkill -f` 的模式一定要**锚定结尾**（`...$`）。不锚定的话，你这条 SSH 命令自身的命令行
> 也会被匹配到，`pkill` 会把当前会话杀掉（表现为 `exit=128`、连接直接断）。

---

## 2. 分支 A：只重来示教（地图是好的）

```bash
D=~/car_ws/maps/<地图名>

# 确认地图可用
ls -lh "$D"/odin_map/*.bin

# 删示教产物（保留地图 / 点云 / 录像）
rm -f "$D"/route_map.yaml "$D"/route_map.yaml.manifest.json "$D"/route_*.yaml "$D"/lap_actual*.yaml

# 重新重定位（示教的前置条件）
bash ~/car_slam_lap/x5/scripts/start_relocal.sh "$D"
#    成功标志：✓ 重定位成功：map -> odom 已发布

# 确认桥接没跑（示教必须遥控）
pgrep -af "lib/car_bridge/bridge_node$"

# 重示教：车摆到起点、车头朝出发方向
bash ~/car_slam_lap/x5/scripts/teach_map_route.sh "$D"
```

走完 **停稳 2 秒，按一次 Ctrl-C**。验收两个文件都在：

```bash
ls -l "$D"/route_map.yaml*      # 应该有 route_map.yaml 和 route_map.yaml.manifest.json
```

---

## 3. 分支 B：整趟重来（地图也不可信 / 要重录视频）

```bash
D=~/car_ws/maps/<地图名>

# 目录改名备份（比 rm 稳，出问题还能翻日志）
mv "$D" "${D}-aborted-$(date +%H%M)"

# 清驱动暂存的地图副本 + 当前地图指针
rm -rf ~/odin_ws/src/odin_ros_driver/map/*
rm -f  ~/car_ws/.current_map_dir

# 用新目录重新建图（分钟级命名，避免和旧目录撞名）
MAP=~/car_ws/maps/demo-$(date +%m%d-%H%M)
VIDEO=1 VIDEO_STRIDE=2 DT7=1 bash ~/car_slam_lap/x5/scripts/start_mapping.sh "$MAP"
```

**为什么用 `date +%m%d-%H%M`**：`start_mapping.sh` 对已存在的目录只做 `mkdir -p`，不会清空；
而录像帧是从 `frame_000001.jpg` 重新编号的，同目录重跑会把老帧顶掉、老帧多出来的部分残留，
成片就会串进上一次的画面。

---

## 4. 分支 C：存图卡死（设备没响应）

典型特征：`odin.log` 停在

```
lidar_save_map: save_map=1 set, waiting for device to finish generation...
```

十几分钟没有下一行，`odin_map/` 一直是空目录，再发一次存图会被回：

```
Map transfer already in progress, ignoring this save_map=1 request.
```

处理：

```bash
# 1) 别反复重发存图（只会被拒），也别继续等
# 2) 直接收场：驱动被杀 = 这次地图必然丢
bash ~/car_slam_lap/x5/scripts/stop_mapping.sh

# 3) 如果连 stop 都退不干净，或下次上电设备仍不响应 → 断电重启
#    整车断电 10 秒再上电（Odin1 是独立 12V 供电）
sudo poweroff        # 或者直接关电源

# 4) 重新建图，这次守两条规矩：
#    ① 存图前先停录像：pkill -INT -f "record_odin_rgb[.]py$"
#    ② 录制时加 VIDEO_STRIDE=2，把写盘压力减半
```

---

## 5. 三件不要做的事

| 不要 | 原因 |
| :--- | :--- |
| 在驱动还活着时 `rm -rf` 目录 | 进程会继续往里写，删了又冒出来，还可能写出残缺 `map.pcd` |
| 连按 Ctrl-C | 第一下是正常收尾，第二下是 SIGKILL —— 直接打断写盘，`manifest.json` 永远不出现 |
| 存图卡住后反复发 `save_map` | 驱动会以 "already in progress" 拒绝，真正要解决的是设备侧没响应 |

---

## 6. 清完的自检清单

```bash
# ① 进程为 0
ps -ef | grep -E "[h]ost_sdk_sample|[b]ridge_node|[f]ollow_path|[r]ecord_path|[r]ecord_odin_rgb" | wc -l

# ② 厂商配置已恢复（应输出"一致"）
C=~/odin_ws/src/odin_ros_driver/config
diff -q "$C/control_command.yaml" "$C/control_command.yaml.vendor" && echo 一致

# ③ 串口空闲（DT7 遥控可用）
fuser -v /dev/ttyS1 2>&1 | head -3

# ④ 空间
df -h ~
```

---

## 7. 一条命令版

脚本：`~/car_slam_lap/x5/scripts/reset_run.sh`

```bash
bash ~/car_slam_lap/x5/scripts/reset_run.sh                          # 只停进程，不动文件（先看）
bash ~/car_slam_lap/x5/scripts/reset_run.sh ~/car_ws/maps/demo-1106 --teach-only   # 重示教
bash ~/car_slam_lap/x5/scripts/reset_run.sh ~/car_ws/maps/demo-1106 --full         # 整趟重来（目录改名备份）
bash ~/car_slam_lap/x5/scripts/reset_run.sh ~/car_ws/maps/demo-1106 --purge        # 整个删掉
```

脚本做的事：按顺序发 SIGINT → 停导出/桥接/跟随 → `stop_mapping.sh` → 检查残留为 0 →
按模式清理 → 打印下一步命令。**只要残留进程不为 0 它就拒绝删文件**，避免带病操作。

---

## 8. 正常重来的完整时序（照着念）

```bash
# 1) 停
bash ~/car_slam_lap/x5/scripts/reset_run.sh ~/car_ws/maps/<旧目录> --full

# 2) 建图 + 录像（DT7 遥控，不启桥接）
MAP=~/car_ws/maps/demo-$(date +%m%d-%H%M)
VIDEO=1 VIDEO_STRIDE=2 DT7=1 bash ~/car_slam_lap/x5/scripts/start_mapping.sh "$MAP"

# 3) 走完：先停录像，再存图（顺序别反）
pkill -INT -f "record_odin_rgb[.]py$"; sleep 3
SAVE_TIMEOUT=300 bash ~/car_slam_lap/x5/scripts/save_map.sh "$MAP"
ls -lh "$MAP"/odin_map/*.bin          # 200+ MB 才算成
bash ~/car_slam_lap/x5/scripts/stop_mapping.sh

# 4) 板端出片
cd "$MAP" && ffmpeg -y -framerate 5 -pattern_type glob -i "video/frames/frame_*.jpg" \
  -vf "scale=1280:-2" -c:v libx264 -preset veryfast -crf 24 \
  -pix_fmt yuv420p -movflags +faststart mapping_1280.mp4

# 5) 重定位（车停建图起点 1 m 内、朝向差 < 10°）
bash ~/car_slam_lap/x5/scripts/start_relocal.sh "$MAP"

# 6) 示教（桥接必须停），走完按一次 Ctrl-C
bash ~/car_slam_lap/x5/scripts/teach_map_route.sh "$MAP"

# 7) 自主
setsid nohup ros2 run car_bridge bridge_node > "$MAP/bridge.log" 2>&1 &
VMAX=0.2 W_MAX=0.6 RETURN_W_MAX=0.6 RETURN_TIMEOUT_S=90 START_POS_TOL_M=0.2 \
BASE_OFFSET_X=0.2626 BASE_OFFSET_Y=-0.0471 MAX_TIME_S=400 \
bash ~/car_slam_lap/x5/scripts/drive_lap.sh "$MAP/route_map.yaml"
```