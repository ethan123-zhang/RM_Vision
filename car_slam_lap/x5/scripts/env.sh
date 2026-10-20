#!/usr/bin/env bash
# 被其它脚本 source，不单独执行：统一路径、ROS 环境和小工具函数。
#
# 目录约定（都在板端 $HOME 下）：
#   ~/car_ws/              工作区（colcon build 的产物）
#   ~/car_ws/config/       Odin1 的两份配置 + 重定位时生成的临时配置
#   ~/car_ws/maps/<名字>/  一次建图的产物：odin_map/、path.yaml、map.pcd、map_bev.png、日志
#   ~/odin_ws/             Odin1 驱动（沿用 x5_odin 那套，不重复部署）

CAR_WS="${CAR_WS:-$HOME/car_ws}"
export CAR_WS
export CAR_MAPS="$CAR_WS/maps"
export CAR_CONFIG="$CAR_WS/config"
export CAR_LOGS="$CAR_WS/logs"
# 本脚本所在目录的上一级 = 仓库里的 x5/（部署后是 ~/car_slam_lap/x5/）
export CAR_SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

mkdir -p "$CAR_MAPS" "$CAR_CONFIG" "$CAR_LOGS"

# ROS2 默认没 source，每个 shell 都要来一遍
source /opt/ros/humble/setup.bash
if [ -f "$CAR_WS/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$CAR_WS/install/setup.bash"
fi

# 等某个 topic 出现（ros2 topic list 每次约 1 s，别设太小）
wait_for_topic() {
  local name="$1" timeout_s="${2:-120}" i
  for ((i = 1; i <= timeout_s; i++)); do
    if ros2 topic list 2>/dev/null | grep -qx "$name"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

# 等 TF 链连通。tf2_echo 在查不到时会一直打错误，出现 Translation: 才算通。
# 注意 subshell 里关掉 pipefail：tf2_echo 被 grep -m1 关掉会产生 SIGPIPE(141)。
wait_for_tf() {
  local parent="$1" child="$2" timeout_s="${3:-120}"
  (
    set +o pipefail
    timeout "$timeout_s" ros2 run tf2_ros tf2_echo "$parent" "$child" 2>/dev/null \
      | grep -m1 -q 'Translation:'
  )
}

# 最近一次 start_mapping.sh 用的目录
current_map_dir() {
  cat "$CAR_WS/.current_map_dir" 2>/dev/null || true
}

require_file() {
  if [ ! -f "$1" ]; then
    echo "✗ 缺少 $1"
    exit 1
  fi
}
