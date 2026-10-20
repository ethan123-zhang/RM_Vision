"""link_test —— 不依赖 ROS 的串口链路自检。

验证三件事：
    1. 下行：X5 连发 N 帧，MCU 不报错（CRC 错由 MCU 侧计数，本工具看不到，
       但能通过"MCU 是否持续上报 link_ok"间接确认帧被正确解析）；
    2. 上行：MCU 的 10 Hz status 帧能被完整接收，CRC 零错；
    3. 看门狗：停发 1 s 后，status 的 LINK_OK 位必须掉下去——这是"上位机死了
       车必须停"的最后一道保险，必须验。

默认 speed=0，车轮不会动。只有显式传 --speed 才会输出非零（会动！）。

用法：
    python3 -m car_bridge.link_test --port /dev/car_link
    python3 -m car_bridge.link_test --port /dev/car_link --seconds 20 --rate 50
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from typing import List

from . import protocol as p

try:
    import serial  # type: ignore
except ImportError:  # pragma: no cover
    print("缺少 pyserial：sudo apt install python3-serial", file=sys.stderr)
    raise SystemExit(2)


class LinkTester:
    def __init__(self, port: str, baud: int, rate: float, speed: int, angular: int) -> None:
        self.ser = serial.Serial(port, baud, timeout=0.05, write_timeout=0.2)
        self.rate = rate
        self.speed = speed
        self.angular = angular
        self.decoder = p.FrameDecoder(p.SOF_UP)

        self.sent = 0
        self.status_frames = 0
        self.status_times: List[float] = []
        self.last_status = None  # (status, src)
        self.last_status_t = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()

    # ------------------------------------------------------------ 读线程
    def _reader(self) -> None:
        while not self._stop.is_set():
            try:
                pending = self.ser.in_waiting
                data = self.ser.read(pending if pending else 1)
            except Exception as exc:  # noqa: BLE001
                print(f"\n读串口异常: {exc}")
                return
            if not data:
                continue
            for frame in self.decoder.feed_bytes(data):
                try:
                    status, src = p.parse_status(frame)
                except ValueError:
                    continue
                now = time.monotonic()
                with self._lock:
                    self.status_frames += 1
                    self.status_times.append(now)
                    self.last_status = (status, src)
                    self.last_status_t = now

    # ------------------------------------------------------------ 发送
    def send_for(self, seconds: float, speed: int, angular: int, label: str) -> None:
        frame = p.build_set_vel(speed, angular)
        period = 1.0 / self.rate
        deadline = time.monotonic() + seconds
        next_t = time.monotonic()
        last_print = 0.0

        while time.monotonic() < deadline:
            now = time.monotonic()
            if now < next_t:
                time.sleep(min(next_t - now, 0.005))
                continue
            next_t += period
            self.ser.write(frame)
            with self._lock:
                self.sent += 1

            if now - last_print > 0.5:
                last_print = now
                self._print_live(label)

        self._print_live(label, newline=True)

    def _print_live(self, label: str, newline: bool = False) -> None:
        with self._lock:
            sent, n, st, t = self.sent, self.status_frames, self.last_status, self.last_status_t
            crc_err = self.decoder.crc_errors
        age = (time.monotonic() - t) if st else float("nan")
        if st is None:
            desc = "无上行"
        else:
            desc = f"{p.status_text(st[0])} src={p.SRC_NAMES.get(st[1], st[1])} age={age:.2f}s"
        line = f"\r{label}  sent={sent:<5d} status={n:<5d} crc_err={crc_err:<3d} {desc}"
        print(line.ljust(96), end="\n" if newline else "", flush=True)

    # ------------------------------------------------------------ 主流程
    def run(self, seconds: float) -> int:
        reader = threading.Thread(target=self._reader, daemon=True)
        reader.start()

        print(f"发送 {seconds:.0f}s @ {self.rate:.0f} Hz, speed={self.speed} "
              f"angular={self.angular} (0 = 车轮不动)")
        self.send_for(seconds, self.speed, self.angular, "[发]")
        t_after = time.monotonic()

        # 停发，验证看门狗
        print("停发 1.0 s，检查 LINK_OK 是否掉下去（看门狗）...")
        time.sleep(1.0)

        with self._lock:
            sent, n = self.sent, self.status_frames
            times = list(self.status_times)
            last = self.last_status
        crc_err = self.decoder.crc_errors

        gaps = [b - a for a, b in zip(times, times[1:])]
        hz = (len(gaps) / sum(gaps)) if gaps and sum(gaps) > 0 else 0.0
        link_dropped = last is not None and not (last[0] & p.ST_LINK_OK)

        print("\n---------------- 结果 ----------------")
        print(f"发送帧数          : {sent}")
        print(f"收到 status 帧     : {n}  (CRC 错 {crc_err})")
        print(f"status 实测频率    : {hz:.2f} Hz  (期望 ~10 Hz)"
              + (f"，最大间隔 {max(gaps) * 1000:.0f} ms" if gaps else ""))
        if last is not None:
            print(f"最后一份 status    : {p.status_text(last[0])} src={p.SRC_NAMES.get(last[1], last[1])}")
            print(f"停发后 LINK_OK     : {'已掉落 ✓' if link_dropped else '仍然为 1 ✗'}")
        ok = True
        if n == 0:
            print("✗ 一帧上行都没收到：先查 TX/RX 是否交叉、GND 是否共地、波特率、固件是否烧了")
            ok = False
        if crc_err:
            print(f"✗ 有 {crc_err} 帧 CRC 错：查波特率偏差 / 杜邦线太长 / 共地")
            ok = False
        if 5.0 > hz > 0.0 or hz > 15.0:
            print("✗ status 频率偏离 10 Hz：查固件定时器/波特率")
            ok = False
        if last is not None and not link_dropped:
            print("✗ 停发后 LINK_OK 没掉：固件看门狗没生效，别让车跑起来")
            ok = False
        print("--------------------------------------")
        print("PASS" if ok else "FAIL")
        return 0 if ok else 1

    def close(self) -> None:
        self._stop.set()
        try:
            self.ser.write(p.build_set_vel(0, 0))
            time.sleep(0.05)
            self.ser.close()
        except Exception:  # noqa: BLE001
            pass


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="X5<->C8T6 串口链路自检")
    ap.add_argument("--port", default="/dev/ttyS1",
                    help="RDK X5 40pin UART 默认 /dev/ttyS1；用 udev 规则时也可写 /dev/car_link")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--speed", type=int, default=0, help="非 0 会让车轮转起来，谨慎")
    ap.add_argument("--angular", type=int, default=0)
    args = ap.parse_args(argv)

    t = LinkTester(args.port, args.baud, args.rate, args.speed, args.angular)
    try:
        return t.run(args.seconds)
    finally:
        t.close()


if __name__ == "__main__":
    sys.exit(main())
