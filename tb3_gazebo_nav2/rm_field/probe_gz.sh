#!/usr/bin/env bash
# Gazebo 侧能力探测：插件、CLI 子命令、tb3 模型、python gz 绑定
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
echo "=== gz-sim system plugins ==="
ls /usr/lib/x86_64-linux-gnu/gz-sim-8/plugins 2>/dev/null | head -40
echo
echo "=== gz model --help ==="
gz model --help 2>&1 | head -30
echo
echo "=== gz service --help (head) ==="
gz service --help 2>&1 | head -25
echo
echo "=== gz topic echo help ==="
gz topic -h 2>&1 | head -30
echo
echo "=== tb3 models ==="
ls /opt/ros/jazzy/share/turtlebot3_gazebo/models 2>/dev/null
ls /opt/ros/jazzy/share/nav2_minimal_tb3_sim/models 2>/dev/null
find /opt/ros/jazzy/share -maxdepth 3 -name 'turtlebot3_waffle*' 2>/dev/null | head -10
echo
echo "=== python gz bindings ==="
python3 - <<'PY'
import importlib
for m in ('gz.transport13','gz.msgs10','gz.sim8'):
    try:
        importlib.import_module(m); print(m,'OK')
    except Exception as e:
        print(m,'MISSING',type(e).__name__)
PY
echo
echo "=== physics engines installed ==="
dpkg -l 2>/dev/null | awk '/gz-physics/{print $2, $3}'
echo
echo "=== gz sim --help (iterations/headless) ==="
gz sim --help 2>&1 | grep -iE 'iterations|headless|server|render' | head -12