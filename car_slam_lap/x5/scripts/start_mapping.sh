#!/usr/bin/env bash
# 建图一趟：起 Odin1（建图模式）+ 点云导出 + 轨迹录制 + 底盘桥接。
#
# 用法：  bash ~/car_slam_lap/x5/scripts/start_mapping.sh [地图目录名]
#         不给名字就自动用 lab_YYYYmmdd_HHMM
#
# 起来之后用遥控器（或另开一个终端 ros2 run car_bridge teleop_keys）把房间走一圈，
# 走完执行 save_map.sh 存图，最后 stop_mapping.sh 收尾。
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

MAP_DIR="${1:-$CAR_MAPS/lab_$(date +%Y%m%d_%H%M)}"
mkdir -p "$MAP_DIR"
echo "$MAP_DIR" > "$CAR_WS/.current_map_dir"
echo "== 建图目录: $MAP_DIR"

# ---------------------------------------------------------------- 1) Odin1
if pgrep -x host_sdk_sample >/dev/null; then
  echo "!! Odin1 驱动已在运行（pid $(pgrep -x host_sdk_sample | head -1)），沿用当前配置。"
  echo "   如果它跑的是出厂 control_command.yaml，先 stop_mapping.sh 再重跑本脚本。"
else
  echo "== 启动 Odin1（建图模式）..."
  setsid nohup ros2 run odin_ros_driver host_sdk_sample \
      --ros-args -p config_file:="$CAR_CONFIG/control_command.mapping.yaml" \
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

# ---------------------------------------------------------------- 4) 底盘桥接
if pgrep -f "[b]ridge_node" >/dev/null; then
  echo "== car_bridge 已在跑，跳过"
else
  setsid nohup ros2 run car_bridge bridge_node \
      > "$MAP_DIR/bridge.log" 2>&1 < /dev/null &
  echo "== 底盘桥接已启动（/cmd_vel -> 串口）"
fi

sleep 2
cat <<EOF

------------------------------ 现在开始建图 ------------------------------
 1. 确认底盘在线：      ros2 topic echo /car/status --once
    （ok:true、src:"serial" 才对；estop:true 说明遥控器 s2 打在急停档）
 2. 把房间走一圈：      遥控器开车
    或者键盘开车：       ros2 run car_bridge teleop_keys
 3. 走完存地图：        bash $(dirname "${BASH_SOURCE[0]}")/save_map.sh
 4. 收尾（落盘并停服务）：bash $(dirname "${BASH_SOURCE[0]}")/stop_mapping.sh
 日志都在: $MAP_DIR/
-------------------------------------------------------------------------
EOF
