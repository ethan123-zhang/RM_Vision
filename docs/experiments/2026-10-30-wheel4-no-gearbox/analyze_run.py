#!/usr/bin/env python3
"""4号轮拆减速箱后固定电流开环数据的离线分析（只读，不碰设备）。

输入：_probe 下的 capture.csv（列 pc_elapsed_s, device_us, can_id, kind, data_hex）。
时间基准：device_us（PCAN 设备时间戳），pc_elapsed_s 仅用于和用户观察时刻对照。

解析帧：
  0x204  ID4 反馈：angle(0..8191), rpm(int16), given_current(int16 实际电流), temp
  0x200  四轮电流指令：ID4 = data[6:8] 大端 int16
  0x1F2  开环诊断：[0]=CAN ID, [1]=状态(0空闲/1运行/故障码), [2:3]=指令,
         [4:5]=实测 rpm, [6:7]=实测电流
  0x1F1  状态帧：[0]=在线掩码, [5]=板上模式(应为 4)

判定口径（沿用 2026-10-29 已冻结的降速事件口径）：
  - 每段先去掉起转后 2 秒，才统计有效段；
  - 稳态参考 = 有效段 rpm 中位数；
  - 20ms 移动均值低于参考 90%、持续 >=50ms 记为候选顿挫；间隔 <100ms 合并；
  - 恒定电流下若持续加速/减速（首尾 1s 均值差 >15%）或有效窗口不足，
    明确输出「不可判定」，不硬给顿挫结论；
  - 退出状态 2/3/4 是故障码；0 无法区分「8s 到时正常释放」和「按键取消」，
    只有保持段时长与 8s 吻合（±0.15s）时才标「到时正常释放」。

用法：
  python analyze_run.py <capture.csv> [--out-dir DIR] [--json]
"""
import argparse
import csv
import json
import math
import statistics
import sys
from dataclasses import dataclass, field
from pathlib import Path

MOTOR_ID = 4
KICK_CURRENT = 1500
HOLD_CURRENTS = {450, 550, 650, 750}
HOLD_MS = 8000
KICK_MAX_MS = 2000
RPM_STATIC = 10          # |rpm| <= 10 视为静止
DROP_START_S = 2.0
STALL_RATIO = 0.90
STALL_MIN_MS = 50
MERGE_GAP_MS = 100
TREND_RATIO = 0.15
FAULT_TEXT = {2: "故障2 反馈掉线", 3: "故障3 超速", 4: "故障4 起转脉冲超时"}


def s16(buf, off):
    return int.from_bytes(buf[off:off + 2], "big", signed=True)


def u16(buf, off):
    return int.from_bytes(buf[off:off + 2], "big")


def parse(csv_path):
    """返回 (frames, 0x204, 0x200, 0x1F2, 0x1F1) 与时间跨度。坏行计数跳过。"""
    fb, cmd, diag, status = [], [], [], []
    bad = 0
    with open(csv_path, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        need = {"pc_elapsed_s", "device_us", "can_id", "data_hex"}
        if not need.issubset(reader.fieldnames or []):
            raise SystemExit(f"CSV 列不符，需包含 {sorted(need)}，实际 {reader.fieldnames}")
        for row in reader:
            try:
                cid = int(row["can_id"], 16)
                data = bytes.fromhex(row["data_hex"])
                t = int(row["device_us"])
                pc = float(row["pc_elapsed_s"])
            except (ValueError, TypeError):
                bad += 1
                continue
            if cid == 0x204 and len(data) >= 8:
                fb.append((t, pc, u16(data, 0), s16(data, 2), s16(data, 4), data[6]))
            elif cid == 0x200 and len(data) >= 8:
                cmd.append((t, pc, s16(data, 6)))
            elif cid == 0x1F2 and len(data) >= 8:
                diag.append((t, pc, data[0], data[1], s16(data, 2), s16(data, 4), s16(data, 6)))
            elif cid == 0x1F1 and len(data) >= 8:
                status.append((t, pc, data[0], data[5], data[7]))
    for seq in (fb, cmd, diag, status):
        seq.sort(key=lambda x: x[0])
    return dict(fb=fb, cmd=cmd, diag=diag, status=status, bad=bad)


def unwrap_deg(angles):
    """把 0..8191 的编码器角展开成累计编码器计数（带符号）。"""
    total = 0.0
    prev = None
    out = []
    for a in angles:
        if prev is not None:
            total += ((a - prev + 4096) % 8192) - 4096
        out.append(total)
        prev = a
    return out


def moving_mean(values, times_s, win_s):
    """按时间窗的滑动均值（双指针，时间单位秒）。"""
    out = []
    lo = 0
    run = 0.0
    for i, t in enumerate(times_s):
        run += values[i]
        while times_s[lo] <= t - win_s:
            run -= values[lo]
            lo += 1
        out.append(run / (i - lo + 1))
    return out


@dataclass
class Segment:
    start_us: int
    start_pc: float
    end_us: int
    end_pc: float
    statuses: list = field(default_factory=list)
    cmds: list = field(default_factory=list)
    hold_cmd: int = 0
    kick_ms: float = 0.0
    hold_ms: float = 0.0

    @property
    def dur_ms(self):
        return (self.end_us - self.start_us) / 1000.0


def split_segments(diag):
    """0x1F2 状态 1 -> 非1 切段；非 1 可能直接是故障码。"""
    segments = []
    cur = None
    for t, pc, mid, st, c, rpm, cur_i in diag:
        if st == 1:
            if cur is None:
                cur = Segment(t, pc, t, pc)
            cur.end_us, cur.end_pc = t, pc
            cur.statuses.append(st)
            cur.cmds.append(c)
        elif cur is not None:
            cur.end_us, cur.end_pc = t, pc
            cur.statuses.append(st)
            cur.cmds.append(c)
            segments.append(cur)
            cur = None
    if cur is not None:
        segments.append(cur)
    # 阶段时长按诊断帧里指令值对应的首末时间算
    for seg in segments:
        rows = [d for d in diag if seg.start_us <= d[0] <= seg.end_us]
        kick_rows = [d for d in rows if d[4] == KICK_CURRENT]
        hold_rows = [d for d in rows if d[4] in HOLD_CURRENTS]
        seg.kick_ms = ((kick_rows[-1][0] - kick_rows[0][0]) / 1000.0) if kick_rows else 0.0
        seg.hold_ms = ((hold_rows[-1][0] - hold_rows[0][0]) / 1000.0) if hold_rows else 0.0
        seg.hold_cmd = hold_rows[0][4] if hold_rows else 0
    return segments


def analyze_segment(seg, fb, cmd):
    rows = [x for x in fb if seg.start_us <= x[0] <= seg.end_us]
    cmds = [x for x in cmd if seg.start_us <= x[0] <= seg.end_us]
    res = dict(dur_s=round(seg.dur_ms / 1000, 3),
               kick_ms=round(seg.kick_ms, 1),
               hold_ms=round(seg.hold_ms, 1),
               hold_cmd=seg.hold_cmd,
               terminal_status=seg.statuses[-1],
               status_seq=sorted(set(seg.statuses)))
    if not rows:
        res.update(verdict="无 0x204 样本：不可判定")
        return res

    t0 = rows[0][0]
    times = [(x[0] - t0) / 1e6 for x in rows]
    rpms = [x[3] for x in rows]
    curs = [x[4] for x in rows]
    temp = rows[-1][5]
    res.update(
        rpm_max=max(rpms), rpm_min=min(rpms),
        rpm_mean=round(statistics.fmean(rpms), 1),
        rpm_last=rpms[-1],
        static_frac=round(sum(1 for v in rpms if abs(v) <= RPM_STATIC) / len(rpms), 3),
        current_mean=round(statistics.fmean(curs), 1) if curs else 0,
        current_max=max(curs) if curs else 0,
        temp_end=temp,
    )
    if cmds:
        res["cmd_0x200_mean"] = round(statistics.fmean(c[2] for c in cmds), 1)

    # 角度与转数：net 为带符号净转数，travel 为最大行程（判断是否真转过）
    rev = unwrap_deg([x[2] for x in rows])
    res["rev_net"] = round((rev[-1] - rev[0]) / 8192.0, 3)
    res["rev_travel"] = round((max(rev) - min(rev)) / 8192.0, 3)
    res["rpm_p95"] = sorted(abs(v) for v in rpms)[int(len(rpms) * 0.95)]

    # 去掉起转 2 秒
    i0 = next((i for i, t in enumerate(times) if t >= DROP_START_S), None)
    if i0 is None or len(times) - i0 < 30:
        res["verdict"] = f"去掉起转 {DROP_START_S:.0f}s 后有效窗口不足：不可判定"
        return res
    wt = times[i0:]
    wr = rpms[i0:]
    wrev = rev[i0:]
    res["valid_window_s"] = round(wt[-1] - wt[0], 3)

    ref = statistics.median(wr)
    res["steady_ref_rpm"] = round(ref, 1)
    ma = moving_mean(wr, wt, 0.020)
    res["ma20_min"] = round(min(ma), 1)
    res["ma20_max"] = round(max(ma), 1)

    # 是否持续加速/减速
    n1 = max(1, int(len(wr) * 0.15))
    head = statistics.fmean(wr[:n1])
    tail = statistics.fmean(wr[-n1:])
    res["head1_rpm"] = round(head, 1)
    res["tail1_rpm"] = round(tail, 1)
    trend = (tail - head) / max(abs(ref), 1.0)
    res["trend_ratio"] = round(trend, 3)

    if abs(ref) < 50:
        res["verdict"] = (
            f"稳态参考转速 {ref:.1f}rpm <50rpm（行程 {res['rev_travel']} 转、静止占比 "
            f"{res['static_frac']*100:.1f}%）：转子基本未连续转动，降速事件口径不适用，不可判定")
        return res

    # 候选顿挫：ma < 90% 参考 持续 >=50ms，间隔 <100ms 合并
    lo, hi = 0.90 * ref, 1.10 * ref
    events = []
    below = [i for i, v in enumerate(ma) if v < lo]
    groups = []
    for i in below:
        if groups and wt[i] - wt[groups[-1][-1]] <= MERGE_GAP_MS / 1000.0:
            groups[-1].append(i)
        else:
            groups.append([i])
    for g in groups:
        if (wt[g[-1]] - wt[g[0]]) * 1000.0 < STALL_MIN_MS:
            continue
        events.append(dict(
            start_s=round(wt[g[0]], 3), end_s=round(wt[g[-1]], 3),
            depth_pct=round((1 - min(ma[i] for i in g) / ref) * 100, 1),
            min_rpm=round(min(wr[i] for i in g), 1),
            delta_rev=round(abs(wrev[g[0]] - wrev[g[-1]]) / 8192.0, 3),
        ))
    res["stall_events"] = events
    res["stall_count"] = len(events)

    accelerating = abs(trend) > TREND_RATIO
    stable = (max(ma) - min(ma)) <= 0.10 * abs(ref)
    if accelerating and not stable:
        res["verdict"] = ("恒定电流下持续%s（首尾1s均值 %+.1f rpm，趋势 %.0f%%），"
                          "无稳定窗口：不可判定顿挫") % (
            "加速" if trend > 0 else "减速", tail - head, trend * 100)
    elif not stable:
        res["verdict"] = "波动带 >10%% 参考、无稳定窗口：不可判定"
    elif events:
        res["verdict"] = f"有 {len(events)} 个候选顿挫事件（口径见脚本头）"
    else:
        res["verdict"] = "稳定窗口内无候选顿挫事件"
    return res


def classify_exit(seg, res):
    st = seg.statuses[-1]
    if st in FAULT_TEXT:
        return FAULT_TEXT[st]
    if st == 0:
        if seg.hold_cmd and abs(seg.hold_ms - HOLD_MS) <= 150:
            return "到时正常释放（保持段时长与 8s 吻合）"
        return "提前退出，状态 0：0x1F2 无法区分按键取消与其他正常取消"
    return f"未知状态 {st}"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", type=Path)
    ap.add_argument("--out-dir", type=Path, default=None)
    args = ap.parse_args()
    if not args.csv.is_file():
        raise SystemExit(f"找不到 {args.csv}")
    out_dir = args.out_dir or args.csv.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    data = parse(args.csv)
    fb, cmd, diag, status = data["fb"], data["cmd"], data["diag"], data["status"]
    modes = sorted({s[3] for s in status})
    online = sorted({s[2] for s in status})
    span = (fb[-1][0] - fb[0][0]) / 1e6 if fb else 0.0
    print(f"文件 {args.csv}")
    print(f"帧: 0x204={len(fb)} 0x200={len(cmd)} 0x1F2={len(diag)} 0x1F1={len(status)} 坏行={data['bad']}")
    print(f"时长 {span:.3f}s；0x1F1 模式={modes} 在线掩码={online}")

    segments = split_segments(diag)
    print(f"检测到 {len(segments)} 个开环输出段\n")

    summary = dict(file=str(args.csv), span_s=round(span, 3), modes=modes,
                   online_masks=online, segments=[])
    for k, seg in enumerate(segments, 1):
        res = analyze_segment(seg, fb, cmd)
        res["index"] = k
        res["t_start_pc_s"] = round(seg.start_pc, 3)
        res["t_end_pc_s"] = round(seg.end_pc, 3)
        res["exit"] = classify_exit(seg, res)
        summary["segments"].append(res)

        print(f"[段{k}] t={seg.start_pc:.3f}..{seg.end_pc:.3f}s 共{res['dur_s']}s "
              f"(起转{res['kick_ms']}ms/保持{res['hold_ms']}ms 指令{res['hold_cmd']})")
        print(f"  退出: 状态{res['terminal_status']} -> {res['exit']}")
        if "rpm_mean" not in res:
            print(f"  结论: {res['verdict']}\n")
            continue
        print(f"  rpm: 最大{res['rpm_max']} 最小{res['rpm_min']} 均值{res['rpm_mean']} "
              f"p95|rpm|={res['rpm_p95']} 末值{res['rpm_last']} 静止占比{res['static_frac']*100:.1f}%")
        print(f"  实际电流: 均值{res['current_mean']} 最大{res['current_max']} "
              f"温度末{res['temp_end']}℃ 净转数{res['rev_net']} 行程{res['rev_travel']}转")
        if "valid_window_s" in res:
            print(f"  有效段 {res['valid_window_s']}s 参考{res['steady_ref_rpm']}rpm "
                  f"首/尾1s={res['head1_rpm']}/{res['tail1_rpm']} 趋势{res['trend_ratio']*100:.0f}%")
            print(f"  候选顿挫 {res.get('stall_count', 0)} 个: "
                  f"{[(e['start_s'], e['depth_pct']) for e in res.get('stall_events', [])]}")
        print(f"  结论: {res['verdict']}\n")

    jp = out_dir / "analysis_summary.json"
    jp.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    ep = out_dir / "stall_events.csv"
    with ep.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["segment", "start_s", "end_s", "depth_pct", "min_rpm", "delta_rev"])
        for s in summary["segments"]:
            for e in s.get("stall_events", []):
                w.writerow([s["index"], e["start_s"], e["end_s"], e["depth_pct"],
                            e["min_rpm"], e["delta_rev"]])
    print(f"已写 {jp}\n已写 {ep}")


if __name__ == "__main__":
    sys.exit(main())
