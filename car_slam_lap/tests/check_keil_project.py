"""Keil 工程与 CubeMX .ioc 的一致性检查（纯 Python，不需要 Keil）。

为什么需要：固件在 PC 上编不了，最容易出的错不是语法，而是
    "文件写好了，但没加进 uvprojx" —— 表现是链接时找不到符号，
    或者更糟：某个 .c 根本没被编译，串口永远收不到数据。
这个脚本把"磁盘上有没有、工程里挂没挂、include 路径对不对"一次查完。

跑法：python3 check_keil_project.py
"""

from __future__ import annotations

import re
import sys
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FW = ROOT / "firmware" / "C8T6_DT7_car"
UVPROJX = FW / "MDK-ARM" / "C8T6_DT7_car.uvprojx"
IOC = FW / "C8T6_DT7_car.ioc"

#: 必须出现在 Hardware 组里的源文件
REQUIRED_SOURCES = ["can_bus.c", "slcan.c", "DBUS.c", "uart_rx.c", "protocol.c", "x5_link.c"]
#: 必须在 include 路径里的目录（相对 MDK-ARM）
REQUIRED_INCLUDES = ["../Hardware/CAN", "../Hardware/UART", "../Hardware/LINK"]
#: .ioc 里必须成立的串口参数
IOC_EXPECT = {
    "USART1.BaudRate": "115200",
    "USART1.Parity": "PARITY_NONE",
    "USART2.BaudRate": "100000",
    "USART2.Parity": "PARITY_EVEN",
    "USART3.BaudRate": "115200",
    "PB10.Signal": "USART3_TX",
    "PB11.Signal": "USART3_RX",
}


def check_uvprojx() -> list:
    problems = []
    if not UVPROJX.is_file():
        return [f"找不到 {UVPROJX}"]

    root = ET.parse(UVPROJX).getroot()
    mdk = UVPROJX.parent

    # 1) 工程里挂的文件都要在磁盘上存在
    files = []
    for f in root.iter("File"):
        name = (f.findtext("FileName") or "").strip()
        path = (f.findtext("FilePath") or "").strip()
        files.append(name)
        target = (mdk / path.replace("\\", "/")).resolve()
        if not target.exists():
            problems.append(f"工程引用了不存在的文件: {name} -> {path}")

    # 2) 新增的模块必须挂进工程
    for src in REQUIRED_SOURCES:
        if src not in files:
            problems.append(f"{src} 不在 uvprojx 里（写好了却没被编译）")

    # 3) include 路径
    inc_path = ""
    for tag in ("IncludePath",):
        for node in root.iter(tag):
            if node.text and "Hardware" in node.text:
                inc_path = node.text
                break
    for need in REQUIRED_INCLUDES:
        if need not in inc_path:
            problems.append(f"include 路径缺少 {need}")

    print(f"  uvprojx: {len(files)} 个文件，Hardware 组源文件 {sum(1 for s in REQUIRED_SOURCES if s in files)}/{len(REQUIRED_SOURCES)}")
    return problems


def check_ioc() -> list:
    problems = []
    if not IOC.is_file():
        return [f"找不到 {IOC}"]

    pairs = []
    for line in IOC.read_text(encoding="utf-8").splitlines():
        if "=" in line and not line.startswith("#"):
            k, v = line.split("=", 1)
            pairs.append((k, v))

    dup = [k for k, c in Counter(k for k, _ in pairs).items() if c > 1]
    if dup:
        problems.append(f".ioc 有重复键: {dup}")

    d = dict(pairs)
    for key, want in IOC_EXPECT.items():
        got = d.get(key)
        if got != want:
            problems.append(f".ioc {key} = {got!r}，期望 {want!r}")

    n_ip = d.get("Mcu.IPNb")
    if n_ip != "7":
        problems.append(f".ioc Mcu.IPNb = {n_ip}，应是 7（CAN/NVIC/RCC/SYS/USART1/2/3）")

    print(f"  .ioc: {len(pairs)} 个键，串口参数 {len(IOC_EXPECT)}/{len(IOC_EXPECT)} 项核对")
    return problems


def check_irq_handlers() -> list:
    """`.ioc` 里开了中断的外设，必须在 stm32f1xx_it.c 里有对应的 X_IRQHandler。

    为什么这条值得单独查：Keil 的 startup_stm32f103xb.s 把每个中断都声明成 [WEAK]
    的空实现，函数体只有一条 "B ."（原地死循环）。开了 NVIC 却忘了写 handler，
    编译链接全过、下载也成功，**第一个字节/第一个中断进来就整机卡死**，没有任何报错。
    """
    problems = []
    it_c = FW / "Core" / "Src" / "stm32f1xx_it.c"
    if not it_c.is_file():
        return [f"找不到 {it_c}"]
    text = it_c.read_text(encoding="utf-8", errors="replace")
    ioc = IOC.read_text(encoding="utf-8", errors="replace")

    core_exceptions = {
        "NonMaskableInt", "HardFault", "MemoryManagement", "BusFault", "UsageFault",
        "SVCall", "DebugMonitor", "PendSV", "SysTick",
    }
    enabled = re.findall(r"^NVIC\.(\w+)_IRQn=true", ioc, re.MULTILINE)
    checked = 0
    for irq in enabled:
        if irq in core_exceptions:
            continue
        checked += 1
        handler = f"{irq}_IRQHandler"
        # 必须匹配"完整的函数名 + 左括号"：单纯找子串会被 USART3_IRQHandler_x 这类改名骗过
        if re.search(rf"\b{handler}\s*\(", text) is None:
            problems.append(
                f".ioc 开了 {irq} 中断，但 stm32f1xx_it.c 里没有 {handler}("
                "（startup 里是弱符号 B .，中断一触发就挂死）"
            )

    print(f"  IRQ: .ioc 开了 {checked} 个外设中断，handler 全部存在"
          if not problems else f"  IRQ: 检查 {checked} 个外设中断")
    return problems


def main() -> int:
    print("check_keil_project — 工程一致性")
    problems = check_uvprojx() + check_ioc() + check_irq_handlers()
    if problems:
        print("\n发现问题:")
        for p in problems:
            # 用 ASCII 的 [x] 而不是 ✗：Windows 控制台默认 GBK，U+2717 编不出来会直接抛异常
            print(f"  [x] {p}")
        return 1
    print("\nOK：工程引用完整、新模块已挂载、include 路径与串口参数一致、中断入口齐全")
    return 0


if __name__ == "__main__":
    sys.exit(main())
