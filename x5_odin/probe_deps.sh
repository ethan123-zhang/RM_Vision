#!/usr/bin/env bash
# probe_deps.sh — 编译期需要的头文件/库是否齐备
set -eo pipefail
for p in \
  /usr/include/libusb-1.0/libusb.h \
  /usr/include/pcl-1.12 \
  /usr/include/eigen3/Eigen/Core \
  /usr/include/yaml-cpp \
  /usr/include/openssl/ssl.h \
  /usr/include/opencv4/opencv2/opencv.hpp ; do
  if [ -e "$p" ]; then echo "OK   $p"; else echo "MISS $p"; fi
done
echo "---- packages ----"
dpkg -l | awk '/^ii/ {print $2}' | grep -E 'libusb-1.0-0-dev|libpcl-dev|libyaml-cpp-dev|libeigen3-dev|libssl-dev|libopencv-dev' || echo "(none matched)"
echo "---- install progress ----"
tail -n 3 "$HOME/odin_install.log"
