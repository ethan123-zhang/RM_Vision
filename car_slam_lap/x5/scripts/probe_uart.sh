#!/usr/bin/env bash
# 确认 RDK X5 40pin UART（第 8 脚 TX / 第 10 脚 RX）的节点能不能用、有没有被控制台占用。
#
# 用法：bash ~/car_slam_lap/x5/scripts/probe_uart.sh
#
# 为什么需要它：D-Robotics 文档说 40PIN 默认使能 UART1、节点是 /dev/ttyS1，
# 但不同镜像/教程里出现过 ttyS3 的说法。与其猜，不如在板子上跑一次：
#   - 节点是否存在、权限对不对
#   - 有没有 serial-getty 占着（被占着的话你发的帧会被 getty 吃掉）
#   - 短接 8/10 脚能不能回环（这一条能同时证明"节点选对了"和"线接对了"）
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

echo "== 1. /dev/ttyS* 列表"
ls -l /dev/ttyS* 2>/dev/null | sed 's/^/  /' || echo "  没有 ttyS* 设备"

echo
echo "== 2. 候选节点"
for d in /dev/ttyS1 /dev/ttyS3 /dev/car_link; do
  if [ -e "$d" ]; then
    printf '  %-16s -> %-12s %s\n' "$d" "$(readlink -f "$d")" "$(stat -c '%A %U:%G' "$d")"
  fi
done
for d in /dev/ttyUSB* /dev/ttyACM*; do
  [ -e "$d" ] && printf '  %-16s %s\n' "$d" "$(stat -c '%A %U:%G' "$d")"
done

echo
echo "== 3. 内核日志里的串口（可能需要 sudo）"
dmesg 2>/dev/null | grep -iE 'ttyS[0-9]|uart' | tail -n 8 | sed 's/^/  /' \
  || echo "  读不到 dmesg（权限）"

echo
echo "== 4. 谁占着 /dev/ttyS1"
if systemctl list-units 'serial-getty@*' --no-pager 2>/dev/null | grep -q 'ttyS1'; then
  echo "  !! serial-getty@ttyS1 在跑：它会抢走你发的字节。"
  echo "     临时关掉: sudo systemctl stop serial-getty@ttyS1"
  echo "     永久关掉: sudo systemctl disable serial-getty@ttyS1"
else
  echo "  没有 getty 占 ttyS1 ✓"
fi
if command -v fuser >/dev/null; then
  fuser -v /dev/ttyS1 2>&1 | sed 's/^/  /' || true
fi

echo
echo "== 5. 回环自测（把 40pin 的第 8 脚和第 10 脚用杜邦线短接，再跑这一节）"
python3 - <<'PY'
import sys
try:
    import serial
except ImportError:
    print("  缺 pyserial：sudo apt install python3-serial")
    sys.exit(0)

node = "/dev/ttyS1"
try:
    s = serial.Serial(node, 115200, timeout=0.5)
except Exception as exc:  # noqa: BLE001
    print(f"  打不开 {node}: {exc}")
    sys.exit(0)

s.reset_input_buffer()
s.write(b"CARLINK")
got = s.read(7)
s.close()
print(f"  发送 CARLINK，收到 {got!r}")
if got == b"CARLINK":
    print("  ✓ 回环正常：节点选对了、8/10 脚是通的")
else:
    print("  ✗ 没回环。三种可能：8/10 没短接、节点不是 ttyS1、权限不够")
PY

echo
echo "结论：回环通过后，把 C8T6 的 PA9(USART1_TX) 接第 10 脚、PA10(RX) 接第 8 脚、GND 互连。"
