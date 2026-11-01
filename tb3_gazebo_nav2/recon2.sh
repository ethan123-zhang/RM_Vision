#!/usr/bin/env bash
echo "=========== A. DRM / GPU 设备 ==========="
echo "--- /proc/devices drm 相关"
grep -i drm /proc/devices
echo "--- /sys/class/drm"
ls -l /sys/class/drm/ 2>&1 | head -10
echo "--- /dev 里的 gpu 节点"
ls -l /dev 2>/dev/null | grep -iE 'dri|drm|dxg|nvidia|card'
echo "--- dmesg dxg/drm"
dmesg 2>/dev/null | grep -iE 'dxg|drm|hyperv.*gpu' | tail -15
echo "--- 内核模块"
cat /proc/modules 2>/dev/null | grep -iE 'dxg|drm|gpu' | head
ls /lib/modules/ 2>&1
echo "--- mesa DRI 驱动目录"
ls /usr/lib/x86_64-linux-gnu/dri/ 2>&1 | head -20
echo "--- /usr/lib/wsl"
ls /usr/lib/wsl/lib 2>&1 | head -10
cat /etc/ld.so.conf.d/*wsl* 2>/dev/null
echo "--- dxg 设备信息"
cat /sys/class/misc/dxg/uevent 2>/dev/null

echo
echo "=========== B. .wslconfig / wsl.conf ==========="
cat /etc/wsl.conf 2>&1
echo "--- /mnt/c/Users/*/.wslconfig"
for f in /mnt/c/Users/*/.wslconfig; do echo "== $f"; cat "$f" 2>/dev/null; done

echo
echo "=========== C. 显示/WSLg ==========="
ls -l /tmp/.X11-unix/ 2>&1
ls -l /mnt/wslg 2>&1 | head
echo "--- x11 socket 连试"
timeout 5 bash -c 'exec 3<>/dev/tcp/127.0.0.1/1' 2>&1
ls /usr/bin | grep -iE '^(x|wayland|weston|glx|egl|vulkan|gl)' | head -20

echo
echo "=========== D. 权限 / 网络 ==========="
sudo -n true 2>&1 && echo "sudo: 免密可用" || echo "sudo: 需要密码"
timeout 12 bash -c 'curl -sS -o /dev/null -w "packages.ros.org HTTP %{http_code}\n" https://packages.ros.org/ros2/ubuntu/dists/noble/Release' 2>&1
timeout 12 bash -c 'curl -sS -o /dev/null -w "github HTTP %{http_code}\n" https://github.com' 2>&1
grep -rhE '^deb ' /etc/apt/sources.list /etc/apt/sources.list.d/ 2>/dev/null | head -20

echo
echo "=========== E. ROS 包统计 ==========="
. /opt/ros/jazzy/setup.bash 2>/dev/null
echo "pkg 总数: $(ros2 pkg list 2>/dev/null | wc -l)"
echo "--- 已有 nav2/tb3"
ros2 pkg list 2>/dev/null | grep -icE 'nav2' | sed 's/^/nav2 包数: /'
ros2 pkg list 2>/dev/null | grep -icE 'turtlebot3' | sed 's/^/turtlebot3 包数: /'
ros2 pkg list 2>/dev/null | grep -iE 'slam_toolbox|cartographer|amcl' | head
echo "--- 已装 ros-jazzy 包数"
dpkg -l 2>/dev/null | grep -c '^ii  ros-jazzy'
echo "--- 内存/CPU"
nproc; free -h | head -2
echo "--- 磁盘"
df -h / | tail -1