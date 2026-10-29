"""Startup breakaway scan (exploratory; not part of acceptance).

Finds intervals where the rotor angle stays within +-6 counts (about 2 rpm) for
>= 60 ms in 20 ms bins while the mean |measured current| is >= 400 protocol units.
Parameters are frozen here and identical for both speeds; they are not tuned per run.

Interpretation limits: this only detects startup breakaway (phase start), when the
commanded current ramps from ~Kp*target. It does not detect the steady-state slow
drops (those never fully freeze) and it is not proof of a mechanical root cause.

Run: python analyze_startup_breakaway.py
Output: startup-breakaway.json
"""
import csv, json
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
CAPTURES = {
    (400, 'run1'): ROOT/'_probe/speed400_capture/run1-1793268106993/capture.csv',
    (400, 'run2'): ROOT/'_probe/speed400_capture/run2-1793274040118/capture.csv',
    (600, 'run1'): ROOT/'_probe/stage1_acceptance_20261029/repeat-1793266358266/capture.csv',
    (600, 'run2'): ROOT/'_probe/stage1_acceptance_20261029/repeat2-1793266687532/capture.csv',
}
TARGETS = {400: [400, 0, -400, 0], 600: [600, 0, -600, 0]}
PARAMS = {'bin_s': 0.02, 'max_angle_counts_per_bin': 6, 'min_mean_abs_current': 400,
          'merge_gap_s': 0.03, 'min_duration_s': 0.06,
          'note': 'angle +-6 of 8192 counts per 20 ms bin is about 2 rpm; current units are C620 protocol units'}


def load(path):
    raw = {i: [] for i in range(1, 5)}
    diag = []
    base = None
    for r in csv.DictReader(path.open()):
        t0 = int(r['device_us'])
        if base is None:
            base = t0
        t = (t0 - base) / 1e6
        b = bytes.fromhex(r['data_hex'])
        cid = int(r['can_id'], 16)
        if len(b) != 8:
            continue
        s = lambda j: int.from_bytes(b[j:j+2], 'big', signed=True)
        if 0x201 <= cid <= 0x204:
            raw[cid-0x200].append((t, int.from_bytes(b[:2], 'big'), s(2), s(4)))
        elif cid == 0x1f2:
            diag.append((t, b[0], b[1], s(2), s(4), s(6)))
    return {i: np.array(v, dtype=float) for i, v in raw.items()}, np.array(diag, dtype=float)


def sessions(diag):
    out, active = [], []
    for d in diag:
        if d[2] == 1:
            active.append(d)
        elif active:
            out.append((active, d)); active = []
    if active:
        out.append((active, None))
    return out


def frozen_intervals(t, angle, current):
    delta = (np.diff(angle) + 4096) % 8192 - 4096
    delta = np.r_[delta, 0.0]  # align with sample indices
    bins = np.floor((t - t[0]) / PARAMS['bin_s']).astype(int)
    hits = []
    for b in np.unique(bins):
        sel = np.flatnonzero(bins == b)
        if len(sel) < 2:
            continue
        # absolute angle motion inside the bin (sum of |delta|)
        motion = float(np.abs(delta[sel[1:]]).sum())
        cur = float(np.mean(np.abs(current[sel])))
        if motion <= PARAMS['max_angle_counts_per_bin'] and cur >= PARAMS['min_mean_abs_current']:
            hits.append((float(t[sel[0]]), float(t[sel[-1]]), cur))
    merged = []
    for hit in hits:
        if merged and hit[0] - merged[-1][1] <= PARAMS['merge_gap_s']:
            merged[-1] = (merged[-1][0], hit[1], max(merged[-1][2], hit[2]))
        else:
            merged.append(list(hit))
    return [{'start_rel_s': x[0] - t[0], 'end_rel_s': x[1] - t[0],
             'duration_s': x[1] - x[0], 'mean_abs_current': x[2]}
            for x in merged if x[1] - x[0] >= PARAMS['min_duration_s']]


def main():
    result = {'purpose': 'exploratory startup breakaway scan; not acceptance, not root-cause proof',
              'parameters': PARAMS, 'captures': {}}
    for (speed, run), path in CAPTURES.items():
        assert path.exists(), path
        raw, diag = load(path)
        phases = []
        for active, exitrow in sessions(diag):
            a = np.array(active)
            motor = int(a[0, 1])
            groups, start = [], 0
            for k in range(1, len(a) + 1):
                if k == len(a) or a[k, 3] != a[start, 3]:
                    groups.append(a[start:k]); start = k
            targets = [int(g[0, 3]) for g in groups]
            complete = (exitrow is not None and int(exitrow[2]) == 0
                        and targets == TARGETS[speed]
                        and abs((exitrow[0] - a[0, 0]) - 18) <= 0.08)
            if not complete:
                continue
            for g in groups:
                target = int(g[0, 3])
                if target == 0:
                    continue
                lo, hi = float(g[0, 0]), float(g[-1, 0])
                m = raw[motor]
                m = m[(m[:, 0] >= lo) & (m[:, 0] <= hi)]
                if len(m) < 10:
                    continue
                phases.append({'motor': motor, 'target': target,
                               'intervals': frozen_intervals(m[:, 0], m[:, 1], m[:, 3])})
        result['captures'][f'{speed}-{run}'] = {'capture': str(path.relative_to(ROOT)), 'phases': phases}
    (HERE / 'startup-breakaway.json').write_text(json.dumps(result, indent=2), encoding='utf8')
    for name, cap in result['captures'].items():
        for ph in cap['phases']:
            for iv in ph['intervals']:
                print(f"{name} motor {ph['motor']} target {ph['target']:+d}: "
                      f"frozen {iv['start_rel_s']:.3f}-{iv['end_rel_s']:.3f}s "
                      f"({iv['duration_s']:.3f}s) |I|~{iv['mean_abs_current']:.0f}")
    print('WROTE startup-breakaway.json')


if __name__ == '__main__':
    main()

