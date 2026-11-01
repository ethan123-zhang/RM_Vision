#!/usr/bin/env bash
# 场地导入前置探测：发行版 / ROS / Gazebo / CAD 网格工具 / 显示 / 网络
set +u
. /etc/os-release; echo "OS        : $PRETTY_NAME"
echo "KERNEL    : $(uname -r)"
echo "CPUS      : $(nproc)"
free -g | awk '/Mem:/{print "MEM       : "$2" GiB total / "$7" GiB available"}'
echo "DISK      : $(df -h / | awk 'NR==2{print $4" free on /"}')"

echo "--- executables ---"
for t in gz freecadcmd freecad freecadcmd.DEBUG gmsh python3 pip3 assimp meshlabserver blender convert import xdotool xvfb-run cmake g++ ros2; do
  p="$(command -v "$t" 2>/dev/null)"
  printf '%-16s %s\n' "$t" "${p:-MISSING}"
done

echo "--- python modules ---"
python3 - <<'PY'
import importlib
for m in ('trimesh','numpy','OCC','cadquery','OCP','PIL','lxml'):
    try:
        importlib.import_module(m); print(f'{m:10s} OK')
    except Exception:
        print(f'{m:10s} MISSING')
PY

echo "--- ros / gz ---"
ls /opt/ros 2>/dev/null
source /opt/ros/jazzy/setup.bash 2>/dev/null && { echo "ros2: $(command -v ros2)"; ros2 pkg list 2>/dev/null | wc -l | sed 's/^/ros pkg count: /'; }
gz sim --versions 2>&1 | head -3

echo "--- apt candidates (installed?) ---"
for p in freecad gmsh python3-occ freecad-python3 libocct-data-exchange-7.6; do
  st="$(dpkg-query -W -f='${Status}' "$p" 2>/dev/null || true)"
  echo "$p: ${st:-not-installed}"
done

echo "--- display ---"
echo "DISPLAY=$DISPLAY WAYLAND_DISPLAY=$WAYLAND_DISPLAY XDG_SESSION_TYPE=$XDG_SESSION_TYPE"
ls -l /dev/dri 2>&1 | head -3

echo "--- network probe (apt/pip) ---"
timeout 25 apt-get -s install -y gmsh >/tmp/apt_sim.log 2>&1; echo "apt simulate gmsh exit=$? ($(wc -l </tmp/apt_sim.log) lines)"
timeout 25 python3 -m pip download --no-deps -d /tmp/piptest trimesh >/tmp/pip.log 2>&1; echo "pip download exit=$?"; tail -2 /tmp/pip.log