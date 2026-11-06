#!/usr/bin/env bash
# 建图一趟：起 Odin1（建图模式）+ 点云导出 + 轨迹录制 + 底盘桥接。
#
# 用法：  bash ~/car_slam_lap/x5/scripts/start_mapping.sh [地图目录名]
#         不给名字就自动用 lab_YYYYmmdd_HHMM
#
# 起来之后用遥控器（或另开一个终端 ros2 run car_bridge teleop_keys）把房间走一圈，
# 走完执行 save_map.sh 存图，最后 stop_mapping.sh 收尾。
#
# 录像（可选，给观众看）：本脚本默认不录。加 VIDEO=1 时，只把 Odin1 配置里的
# sendrgb 与 sendrgbcompressed 一起打开（设备侧 JPEG 直通，主机不解码/渲染），并起 record_odin_rgb.py
# 把 JPEG 帧原样写进 $MAP_DIR/video/。其余后处理仍全关，建图开销只多磁盘写。
#   VIDEO=1 bash start_mapping.sh demo_x
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"

MAP_DIR="${1:-$CAR_MAPS/lab_$(date +%Y%m%d_%H%M)}"
mkdir -p "$MAP_DIR"
echo "$MAP_DIR" > "$CAR_WS/.current_map_dir"
echo "== 建图目录: $MAP_DIR"

# ---------------------------------------------------------------- 1) Odin1
if pgrep -x host_sdk_sample >/dev/null; then
  echo "!! Odin1 驱动已在运行（pid $(pgrep -x host_sdk_sample | head -1)），沿用它的配置。"
  echo "   要保证是建图模式（custom_map_mode: 1）：先 stop_mapping.sh 再重跑本脚本。"
else
  echo "== 启动 Odin1（建图模式）..."
  # 驱动不读 -p config_file，只读 $ODIN_DRIVER_CFG，所以这里直接把建图配置装进去
  MAP_CFG="$CAR_CONFIG/control_command.mapping.yaml"
  if [ "${VIDEO:-0}" = "1" ]; then
    # 只在运行时生成的副本上改一个键，仓库里的模板保持不变。
    MAP_CFG="$MAP_DIR/control_command.mapping_video.yaml"
    sed -e 's/^\([[:space:]]*\)sendrgbcompressed:[[:space:]]*0/\1sendrgbcompressed: 1/' \
        -e 's/^\([[:space:]]*\)sendrgb:[[:space:]]*0/\1sendrgb: 1/' \
        "$CAR_CONFIG/control_command.mapping.yaml" > "$MAP_CFG"
    # 分辨率：SD 写只有 ~1.2 MB/s，1600x1296 约 540 KB/帧 -> 只能录 2 fps。
    # VIDEO_RES=800x648 将 JPEG 压到 ~135 KB/帧，才能跑满 10 Hz。
    # 建图不用 RGB，降分辨率不影响点云质量。
    if [ -n "${VIDEO_RES:-}" ]; then
      VW="${VIDEO_RES%x*}"
      VH="${VIDEO_RES#*x}"
      sed -i -e "s/^\([[:space:]]*\)rgb_width:[[:space:]]*[0-9]*/\1rgb_width: ${VW}/" \
             -e "s/^\([[:space:]]*\)rgb_height:[[:space:]]*[0-9]*/\1rgb_height: ${VH}/" \
             "$MAP_CFG"
    fi
    if ! grep -qE '^[[:space:]]*sendrgbcompressed:[[:space:]]*1' "$MAP_CFG" \
       || ! grep -qE '^[[:space:]]*sendrgb:[[:space:]]*[1-9]' "$MAP_CFG"; then
      echo "✗ 打开 sendrgbcompressed 失败（模板格式变了？）：$CAR_CONFIG/control_command.mapping.yaml"
      exit 1
    fi
    echo "== 录像模式：已打开 sendrgb + sendrgbcompressed（其余后处理仍关）"
  fi
  odin_use_config "$MAP_CFG"
  setsid nohup ros2 run odin_ros_driver host_sdk_sample \
      > "$MAP_DIR/odin.log" 2>&1 < /dev/null &
fi

CALIB="$HOME/.ros/odin_ros_driver/calib.yaml"
for _ in $(seq 1 60); do
  [ -f "$CALIB" ] && break
  sleep 1
done
if [ -f "$CALIB" ]; then
  echo "== 标定文件就绪（$(stat -c %s "$CALIB") B）"
else
  echo "!! 没等到 $CALIB，驱动可能没连上设备"
fi

if ! wait_for_topic /odin1/cloud_slam 120; then
  echo "✗ 120 s 内没等到 /odin1/cloud_slam。驱动日志尾部："
  tail -n 30 "$MAP_DIR/odin.log" || true
  echo "  排查顺序：USB3 线插蓝色口 / 12V 供电 / dmesg | grep -i hawk / 把 config 里 senddtof 改回 1"
  exit 1
fi
echo "== /odin1/cloud_slam 已出现"

# ---------------------------------------------------------------- 2) 点云导出
if pgrep -f "[m]ap_from_cloud" >/dev/null; then
  echo "== map_from_cloud 已在跑，跳过"
else
  setsid nohup ros2 run car_bridge map_from_cloud \
      --ros-args -p out_dir:="$MAP_DIR" \
      > "$MAP_DIR/map_from_cloud.log" 2>&1 < /dev/null &
  echo "== 点云导出已启动（Ctrl-C 时会写 map.pcd / map_bev.png）"
fi

# ---------------------------------------------------------------- 3) 轨迹录制
if pgrep -f "[r]ecord_path" >/dev/null; then
  echo "== record_path 已在跑，跳过"
else
  setsid nohup ros2 run car_bridge record_path \
      --ros-args -p out_path:="$MAP_DIR/path.yaml" \
      > "$MAP_DIR/record_path.log" 2>&1 < /dev/null &
  echo "== 轨迹录制已启动 -> $MAP_DIR/path.yaml"
fi

# ---------------------------------------------------------------- 3.5) RGB 录像
if [ "${VIDEO:-0}" = "1" ]; then
  if pgrep -f "[r]ecord_odin_rgb" >/dev/null; then
    echo "== record_odin_rgb 已在跑，跳过"
  else
    # 空数组展开用 "${ARR[@]}" 的写法与 drive_lap.sh 的 EXTRA_ROS_ARGS 一致
    STRIDE_ARGS=()
    if [ -n "${VIDEO_STRIDE:-}" ]; then
      STRIDE_ARGS=(--stride "$VIDEO_STRIDE")
    fi
    setsid nohup python3 -u "$SCRIPT_DIR/record_odin_rgb.py" \
        --out "$MAP_DIR/video" "${STRIDE_ARGS[@]}" \
        > "$MAP_DIR/record_odin_rgb.log" 2>&1 < /dev/null &
    echo "== RGB 录像已启动 -> $MAP_DIR/video/frames/"
  fi
fi

# ---------------------------------------------------------------- 4) 底盘桥接
# DT7=1：遥控建图，不启桥接。桥接一启就按 src=serial 压过遥控器
# （firmware/C8T6_DT7_car/Core/Src/main.c:107-110），遥控会被屏蔽。
if [ "${DT7:-0}" = "1" ]; then
  echo "== DT7 遥控建图：不启桥接"
elif pgrep -f "[b]ridge_node$" >/dev/null; then
  echo "== car_bridge 已在"
else
  setsid nohup ros2 run car_bridge bridge_node \
      > "$MAP_DIR/bridge.log" 2>&1 < /dev/null &
  echo "== 底盘桥接已启动（/cmd_vel -> 串口）"
fi

sleep 2
if [ "${DT7:-0}" = "1" ]; then
  HINT_DRIVE=" 1. DT7 遥控开车（桥接已按 DT7=1 跳过）"
  HINT_ALT=""
else
  HINT_DRIVE=" 1. 确认底盘在线：      ros2 topic echo /car/status --once"
  HINT_ALT="    （ok:true、src:serial；estop:true 遥控器 s2 急停档）"
fi
VIDEO_HINT=""
if [ "${VIDEO:-0}" = "1" ]; then
  VIDEO_HINT=" 5. 录像中：          $MAP_DIR/video/（进度 tail -f $MAP_DIR/record_odin_rgb.log）"
fi
cat <<EOF

------------------------------ 现在建图 ------------------------------
$HINT_DRIVE
$HINT_ALT
 2. 把房间走一圈：      遥控器开车
 3. 走完存地图：        bash $(dirname "${BASH_SOURCE[0]}")/save_map.sh
 4. 收尾（落盘并停）：bash $(dirname "${BASH_SOURCE[0]}")/stop_mapping.sh
$VIDEO_HINT
 日志: $MAP_DIR/
-------------------------------------------------------------------------
EOF
