#!/usr/bin/env bash
# 板端一次性安装（幂等，可重复跑）：
#   依赖 -> 目录 -> 源码/配置 -> udev 固定串口名 -> colcon build -> 自检
#
# 用法：  bash ~/car_slam_lap/x5/scripts/install_car.sh
#
# 前置：Odin1 驱动已经按 x5_odin/README.md 部署到 ~/odin_ws（本脚本不碰它）
set -eo pipefail

export CAR_WS="${CAR_WS:-$HOME/car_ws}"
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

echo "== 1/5 依赖"
need_apt=0
python3 -c "import serial" 2>/dev/null || need_apt=1
python3 -c "import yaml"   2>/dev/null || need_apt=1
if [ "$need_apt" = 1 ]; then
  # 校园网 IPv6 黑洞会让 apt 挂死，x5_odin 里也是这么绕的
  sudo tee /etc/apt/apt.conf.d/99ipv4 >/dev/null <<'EOF'
Acquire::ForceIPv4 "true";
Acquire::http::Timeout "20";
Acquire::https::Timeout "20";
EOF
  sudo apt-get update
  sudo apt-get install -y python3-serial python3-yaml
else
  echo "python3-serial / python3-yaml 已就绪"
fi
python3 -c "import serial, yaml; print('  pyserial', serial.__version__)"

echo "== 2/5 目录"
mkdir -p "$CAR_WS/src" "$CAR_MAPS" "$CAR_CONFIG" "$CAR_LOGS"
echo "  $CAR_WS"

echo "== 3/5 源码与配置"
rm -rf "$CAR_WS/src/car_bridge"
cp -a "$CAR_SRC/car_bridge" "$CAR_WS/src/car_bridge"
cp -a "$CAR_SRC"/config/*.yaml "$CAR_CONFIG/"
ls -1 "$CAR_CONFIG"

echo "== 4/5 udev 规则（USB-TTL 固定成 /dev/car_link）"
sudo install -m 644 "$CAR_SRC/scripts/99-car-link.rules" /etc/udev/rules.d/99-car-link.rules
sudo udevadm control --reload
sudo udevadm trigger

echo "== 5/5 colcon build"
cd "$CAR_WS"
colcon build --packages-select car_bridge --symlink-install
# shellcheck disable=SC1091
source "$CAR_WS/install/setup.bash"
ros2 pkg executables car_bridge

echo
echo "---- 自检 ----"
python3 -c "import serial, yaml, numpy; print('python 依赖 ok')"
if [ -e /dev/car_link ]; then
  echo "串口: /dev/car_link 存在 ✓"
else
  echo "串口: /dev/car_link 还没出现（USB-TTL 插上了吗？插上后再跑一次 udevadm trigger）"
  ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  连 ttyUSB 都没有：换根线或查 dmesg | tail"
fi
echo "装完了。下一步：把 x5/README 里的线接好 -> link_test"
