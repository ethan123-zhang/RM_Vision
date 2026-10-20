#!/usr/bin/env bash
# Start Odin1 driver + live web viewer on the RDK X5 (idempotent).
set -eo pipefail

cd "$HOME/odin_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash

if pgrep -x host_sdk_sample >/dev/null; then
  echo "driver already running (pid $(pgrep -x host_sdk_sample | head -1))"
else
  setsid nohup bash run_odin.sh > "$HOME/odin_run.log" 2>&1 < /dev/null &
  echo "driver starting ..."
fi

# 新烧的卡上，calib.yaml 要等设备连上后由驱动从硬件回读（实测约 11 s）。
# viewer 只在启动时读一次标定，太早起会拿不到 → 深度/叠加两路全空。
CALIB="$HOME/.ros/odin_ros_driver/calib.yaml"
for _ in $(seq 1 60); do
  [ -f "$CALIB" ] && break
  sleep 1
done
if [ -f "$CALIB" ]; then
  echo "calib ok ($(stat -c %s "$CALIB") bytes)"
else
  echo "WARN 没有 $CALIB，深度/叠加两路会是空的（Odin1 连上了吗？看 $HOME/odin_run.log）"
fi

if pgrep -f "[o]din_web_viewer.py" >/dev/null; then
  echo "viewer already running (pid $(pgrep -f "[o]din_web_viewer.py" | head -1))"
else
  setsid nohup python3 -u odin_web_viewer.py > "$HOME/web_viewer.log" 2>&1 < /dev/null &
  echo "viewer starting ..."
fi

sleep 3
IP=$(hostname -I | awk '{print $1}')
echo "--------------------------------------------------"
echo "  板端直连:   http://${IP}:8080/"
echo "  端口转发:   http://127.0.0.1:8080/   (在电脑上开 ssh -L 8080:127.0.0.1:8080)"
echo "  stats:      http://${IP}:8080/stats"
echo "--------------------------------------------------"
