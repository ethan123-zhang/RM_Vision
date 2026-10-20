#!/usr/bin/env bash
# probe_env.sh — 板端环境指纹（部署前后都跑一次，用于记录/比对）
set -eo pipefail
. /etc/os-release
echo "== system =="
echo "hostname : $(hostname)"
echo "os       : $PRETTY_NAME"
echo "kernel   : $(uname -r) $(uname -m)"
echo "uptime   : $(uptime -p)"
echo "== cpu/mem =="
echo "cores    : $(nproc)"
free -m | sed -n '1,3p'
echo "== disk =="
df -h / | tail -1
echo "== swap =="
swapon --show || echo "(no swap)"
echo "== ros =="
ls -d /opt/ros/* /opt/tros/* 2>/dev/null
echo "ros-humble packages: $(dpkg -l | grep -c '^ii  ros-humble')"
echo "== tools =="
for t in colcon cmake g++ git unzip python3; do printf '%-8s %s\n' "$t" "$(command -v $t || echo MISSING)"; done
echo "== python (ros sourced) =="
. /opt/ros/humble/setup.bash
python3 - <<'PY'
import sys
print("python", sys.version.split()[0])
for m in ("numpy", "cv2", "rclpy"):
    try:
        mod = __import__(m)
        print(f"  {m:6s} {getattr(mod, '__version__', 'ok')}")
    except Exception as e:
        print(f"  {m:6s} MISSING ({e.__class__.__name__}: {e})")
PY
echo "== odin usb =="
lsusb | grep -i 2207 || echo "(2207:0019 not present)"
echo "== odin_ws =="
ls "$HOME/odin_ws" 2>/dev/null || echo "(not built yet)"
