"""Independent pure-Python recompute of summarize400.py outputs.

usage: python verify_summary.py <capture.csv> <summary.json>
No numpy; tail mean/std use running sum and sum of squares.
Exits non-zero on any mismatch.
"""
import csv, json, math, sys
from pathlib import Path

csv_path = Path(sys.argv[1]); summary_path = Path(sys.argv[2])
summary = json.loads(summary_path.read_text())

counts = {}; base = None; n = 0
diag = []; raw = {i: [] for i in range(1, 5)}
for row in csv.DictReader(csv_path.open()):
    t0 = int(row['device_us'])
    if base is None:
        base = t0
    t = (t0 - base) / 1e6
    b = bytes.fromhex(row['data_hex'])
    cid = int(row['can_id'], 16)
    counts[hex(cid)] = counts.get(hex(cid), 0) + 1
    n += 1
    if len(b) != 8:
        continue
    s = lambda j: int.from_bytes(b[j:j+2], 'big', signed=True)
    if 0x201 <= cid <= 0x204:
        raw[cid-0x200].append((t, s(2)))
    elif cid == 0x1f2:
        diag.append((t, b[0], b[1], s(2), s(4), s(6)))

meta = json.loads(csv_path.with_suffix('.json').read_text())
errors = []
def check(name, got, want, tol=1e-9):
    if isinstance(want, (int, float)) and got is not None:
        ok = abs(float(got) - float(want)) <= tol
    else:
        ok = got == want
    if not ok:
        errors.append(f'{name}: recomputed={got!r} summary={want!r}')

check('frame count', n, meta['frames'])
check('frame count vs summary', n, summary['frames'])
check('counts vs capture.json', counts, meta['counts'])
check('counts sum', sum(counts.values()), meta['frames'])

sessions = []; active = []
for d in diag:
    if d[2] == 1:
        active.append(d)
    elif active:
        sessions.append((active, d)); active = []
if active:
    sessions.append((active, None))

def tail_stats(motor, lo, hi):
    values = [v for t, v in raw[motor] if lo <= t <= hi]
    n_v = len(values); s1 = sum(values); s2 = sum(v*v for v in values)
    mean = s1 / n_v
    var = s2 / n_v - mean * mean
    return n_v, mean, math.sqrt(max(var, 0.0))

if len(sessions) != len(summary['runs']):
    errors.append(f'sessions {len(sessions)} != summary runs {len(summary["runs"])}')
for (rows, exitrow), run in zip(sessions, summary['runs']):
    motor = int(rows[0][1])
    check(f'run motor', motor, run['motor'])
    groups = []
    for d in rows:
        if not groups or groups[-1][-1][3] != d[3]:
            groups.append([])
        groups[-1].append(d)
    targets = [g[0][3] for g in groups]
    check(f'motor {motor} targets', targets, run['targets'])
    ends = [g[0][0] for g in groups[1:]] + [exitrow[0] if exitrow else None]
    durations = [e - g[0][0] if e else None for g, e in zip(groups, ends)]
    for k, (d, want) in enumerate(zip(durations, run['durations_s'])):
        check(f'motor {motor} duration {k}', d, want, 1e-9)
    for g, phase in zip(groups, run['phases']):
        target = g[0][3]; hi = g[-1][0]
        n_v, mean, std = tail_stats(motor, hi - (2 if target else .5), hi)
        check(f'motor {motor} target {target} tail_samples', n_v, phase['tail_samples'])
        check(f'motor {motor} target {target} tail_mean', mean, phase['tail_mean_rpm'], 5e-6)
        check(f'motor {motor} target {target} tail_std', std, phase['tail_std_rpm'], 5e-6)
    if run.get('command_peak') is not None:
        check(f'motor {motor} release_nonzero', run['release_nonzero_frames'], 0)

if errors:
    print('MISMATCH')
    for e in errors:
        print(' ', e)
    sys.exit(1)
print(f'VERIFY OK {summary_path.name}: {n} frames, {len(sessions)} runs, counts match capture.json, tail mean/std recomputed within 5e-6')
