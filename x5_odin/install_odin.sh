#!/usr/bin/env bash
# install_odin.sh — 在 RDK X5（Ubuntu 22.04 + ROS2 Humble）上从零重建 Odin1 驱动环境
#
# 用法（板端，sunrise 用户）：
#   bash ~/odin_deploy/install_odin.sh
#
# 幂等：重复执行只补缺的步骤。首次约 30 min（colcon build 占绝大部分）。
# 注意：不要加 set -u —— ROS 的 setup.bash 里有未定义变量。
set -eo pipefail

BUNDLE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_ZIP="${1:-$BUNDLE/odin_ros_driver_main.zip}"
WS="$HOME/odin_ws"
SWAPFILE="/swapfile"

log() { printf '\n\033[1;36m==== %s ====\033[0m\n' "$*"; }
die() { printf '\n\033[1;31m[FAIL] %s\033[0m\n' "$*" >&2; exit 1; }

log "0/6 基本检查"
[ -f "$SRC_ZIP" ] || die "找不到驱动源码包 $SRC_ZIP"
. /etc/os-release
echo "系统   : $PRETTY_NAME ($(uname -m), kernel $(uname -r))"
echo "内存   : $(free -m | awk '/^Mem:/{print $2}') MB"
echo "源码包 : $SRC_ZIP ($(stat -c %s "$SRC_ZIP") bytes)"
[ -d /opt/ros/humble ] || die "镜像里没有 /opt/ros/humble"

log "1/6 apt 依赖"
# 板子所在的校园网 IPv6 被黑洞时，apt 会在 IPv6 连接上挂到超时（实测卡 6 min 不动）。
# 强制 IPv4 + 短超时 + 重试，写进 apt.conf.d 让后续所有 apt 调用都生效。
sudo tee /etc/apt/apt.conf.d/99odin-ipv4 >/dev/null <<'EOF'
Acquire::ForceIPv4 "true";
Acquire::Retries "3";
Acquire::http::Timeout "20";
Acquire::https::Timeout "20";
EOF
sudo apt-get update -y
sudo apt-get install -y --no-install-recommends \
  unzip ros-humble-image-transport ros-humble-pcl-conversions \
  python3-colcon-common-extensions

log "2/6 swap（3 GB 内存编 PCL 模板会 OOM）"
if swapon --show | grep -q "^${SWAPFILE}"; then
  echo "swap 已启用: $(swapon --show | grep "^${SWAPFILE}")"
else
  if [ ! -f "$SWAPFILE" ]; then
    sudo fallocate -l 4G "$SWAPFILE" || sudo dd if=/dev/zero of="$SWAPFILE" bs=1M count=4096 status=progress
    sudo chmod 600 "$SWAPFILE"
    sudo mkswap "$SWAPFILE"
  fi
  sudo swapon "$SWAPFILE"
  echo "swap 已创建并启用"
fi
grep -q "^${SWAPFILE} " /etc/fstab || echo "${SWAPFILE} none swap sw 0 0" | sudo tee -a /etc/fstab >/dev/null
echo "开机自动挂载: $(grep "^${SWAPFILE} " /etc/fstab)"

log "3/6 udev 规则（2207:0019 → group plugdev 可读写）"
sudo install -m 644 "$BUNDLE/99-odin-usb.rules" /etc/udev/rules.d/99-odin-usb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
echo "已安装 /etc/udev/rules.d/99-odin-usb.rules"

log "4/6 解包驱动源码 → $WS/src/odin_ros_driver"
mkdir -p "$WS/src"
TMP=$(mktemp -d)
unzip -q -o "$SRC_ZIP" -d "$TMP"
SRC_DIR=$(find "$TMP" -mindepth 1 -maxdepth 3 -name CMakeLists.txt -printf '%h\n' | head -1)
[ -n "$SRC_DIR" ] || die "源码包里没找到 CMakeLists.txt"
rsync -a --delete "$SRC_DIR/" "$WS/src/odin_ros_driver/"
rm -rf "$TMP"
echo "包名: $(grep -o '<name>[^<]*' "$WS/src/odin_ros_driver/package.xml" | head -1 | cut -d'>' -f2) $(grep -o '<version>[^<]*' "$WS/src/odin_ros_driver/package.xml" | head -1 | cut -d'>' -f2)"
echo "arm 静态库: $(ls -l "$WS/src/odin_ros_driver/lib/liblydHostApi_arm.a" | awk '{print $5}') bytes"

log "5/6 colcon build（首次约 30 min）"
cd "$WS"
source /opt/ros/humble/setup.bash
BUILD_START=$(date +%s)
colcon build --packages-select odin_ros_driver --parallel-workers 4 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
BUILD_END=$(date +%s)
echo "编译耗时: $(( (BUILD_END - BUILD_START) / 60 )) min"

log "6/6 安装运行时脚本"
for f in run_odin.sh start_all.sh stop_all.sh odin_web_viewer.py capture_one_frame.py; do
  if [ -f "$BUNDLE/$f" ]; then
    install -m 755 "$BUNDLE/$f" "$WS/$f"
    echo "  → $WS/$f"
  fi
done

echo
echo "可执行文件:"
ls -l "$WS/install/odin_ros_driver/lib/odin_ros_driver/" || die "build 产物缺失"
echo
echo "完成。下一步："
echo "  bash $WS/start_all.sh      # 起驱动 + 网页预览"
echo "  bash $WS/stop_all.sh       # 停止"
