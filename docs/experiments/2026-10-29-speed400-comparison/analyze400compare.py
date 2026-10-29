"""400 vs 600 rotor-rpm single-variable comparison (diagnostic only, not acceptance).

Reuses the frozen 600rpm acceptance methods:
  tail window = last 2 s (0.5 s for zero) before the last diagnostic sample of the phase
  overshoot = max(0, max(sign*target*raw_rpm)-|target|)/|target| over whole phase
  settling = trailing 0.25 s mean enters +-5% and holds >= 1 s to phase end
Frozen here, identical for both speeds:
  drop detection on trailing 20 ms mean below 90% of |target|, min 50 ms,
  merge gaps < 100 ms, exclude first 1.0 s and last 0.2 s
  per event: start / minimum / recovery, depth, recovery time, dt to previous,
  dRev from unwrapped 8192-count angle, dRev from rpm integration
  spectrum on last 2 s, uniform 2 ms resample, linear detrend, Hann, peak 2..100 Hz,
  angle-locked harmonics orders 1..12 (order 7 reported)

Outputs (its own products only; raw captures are never modified):
  comparison400vs600.json, events.csv,
  curves400-600.png, events400.png, spectrum400-600.png

Run: python analyze400compare.py
"""
import csv, json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]

SOURCES = {
    400: [('run1', ROOT / '_probe/speed400_capture/run1-1793268106993/capture.csv'),
          ('run2', ROOT / '_probe/speed400_capture/run2-1793274040118/capture.csv')],
    600: [('run1', ROOT / '_probe/stage1_acceptance_20261029/repeat-1793266358266/capture.csv'),
          ('run2', ROOT / '_probe/stage1_acceptance_20261029/repeat2-1793266687532/capture.csv')],
}
TARGETS = {400: [400, 0, -400, 0], 600: [600, 0, -600, 0]}
EVENTS = {'smooth_s': 0.02, 'drop_frac': 0.10, 'min_duration_s': 0.05,
          'merge_gap_s': 0.10, 'exclude_start_s': 1.0, 'exclude_end_s': 0.2,
          'recover_frac': 0.97}
CLUSTER = {'merge_gap_s': 0.8}
SETTLE = {'window_s': 0.25, 'frac': 0.05, 'hold_s': 1.0}


def load_capture(path):
    raw = {i: [] for i in range(1, 5)}
    cmd, diag, status = [], [], []
    base = None; counts = {}
    with path.open() as f:
        for row in csv.DictReader(f):
            t0 = int(row['device_us'])
            if base is None:
                base = t0
            t = (t0 - base) / 1e6
            b = bytes.fromhex(row['data_hex'])
            cid = int(row['can_id'], 16)
            counts[hex(cid)] = counts.get(hex(cid), 0) + 1
            if len(b) != 8:
                continue
            s = lambda j: int.from_bytes(b[j:j+2], 'big', signed=True)
            if 0x201 <= cid <= 0x204:
                raw[cid-0x200].append((t, int.from_bytes(b[:2], 'big'), s(2), s(4), b[6]))
            elif cid == 0x200:
                cmd.append((t, *[s(j) for j in (0, 2, 4, 6)]))
            elif cid == 0x1f2:
                diag.append((t, b[0], b[1], s(2), s(4), s(6)))
            elif cid == 0x1f1:
                status.append((t, b[0], b[5]))
    return {'raw': {i: np.array(v, dtype=float) for i, v in raw.items()},
            'cmd': np.array(cmd, dtype=float), 'diag': np.array(diag, dtype=float),
            'status': np.array(status, dtype=float), 'counts': counts, 'duration': t}


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


def moving_average(t, v, window):
    csum = np.r_[0.0, np.cumsum(v)]
    left = np.searchsorted(t, t - window, side='left')
    den = np.arange(len(v)) + 1 - left
    return (csum[1:] - csum[left]) / den


def unwrap_angle(m):
    a = m[:, 1]
    delta = (np.diff(a) + 4096) % 8192 - 4096
    return np.r_[a[0], a[0] + np.cumsum(delta)]


def rev_between(t, unwrapped, t0, t1, sign):
    return float(sign * (np.interp(t1, t, unwrapped) - np.interp(t0, t, unwrapped)) / 8192)


def integrated_rev(t, v, sign, t0, t1):
    sel = (t >= t0) & (t <= t1)
    if sel.sum() < 2:
        return None
    return float(np.trapezoid(sign * v[sel], t[sel]) / 60)


def settling_time(t, v, target, lo, hi):
    ma = moving_average(t, v, SETTLE['window_s'])
    eligible = np.flatnonzero(t - lo >= SETTLE['window_s'])
    if not len(eligible):
        return None
    bad = eligible[np.abs(ma[eligible] - target) > abs(target) * SETTLE['frac']]
    cand = int(bad[-1]) + 1 if len(bad) else int(eligible[0])
    if cand < len(v) and hi - t[cand] >= SETTLE['hold_s']:
        return float(t[cand] - lo)
    return None


def detect_events(t, v, target, lo, hi, unwrapped_angle):
    sign = 1 if target > 0 else -1
    signed = sign * v
    tgt = abs(target)
    ma = moving_average(t, signed, EVENTS['smooth_s'])
    below = ma < tgt * (1 - EVENTS['drop_frac'])
    allowed = (t >= lo + EVENTS['exclude_start_s']) & (t <= hi - EVENTS['exclude_end_s'])
    idx = np.flatnonzero(below & allowed)
    events = []
    if len(idx):
        groups = np.split(idx, np.flatnonzero(np.diff(t[idx]) > EVENTS['merge_gap_s']) + 1)
        for g in groups:
            if t[g[-1]] - t[g[0]] < EVENTS['min_duration_s']:
                continue
            k = int(g[np.argmin(ma[g])])
            candidates = np.flatnonzero((np.arange(len(t)) > k) & (ma >= tgt * EVENTS['recover_frac']))
            rec = candidates[candidates <= np.searchsorted(t, hi)]
            events.append({'start': float(t[g[0]]), 'minimum': float(t[k]),
                           'minimum_ma_rpm': float(ma[k]),
                           'depth_pct': float((tgt - ma[k]) / tgt * 100),
                           'recovery': float(t[rec[0]]) if len(rec) else None,
                           'recovery_s': float(t[rec[0]] - t[k]) if len(rec) else None,
                           'duration_s': float(t[rec[0]] - t[g[0]]) if len(rec) else None,
                           'signed_min_rpm': float(ma[k] * sign),
                           'rev_cumulative': rev_between(t, unwrapped_angle, lo, float(t[g[0]]), sign)})
    for i, ev in enumerate(events):
        if i == 0:
            ev['dt_prev_s'] = None; ev['drev_prev'] = None; ev['drev_prev_integrated'] = None
        else:
            ev['dt_prev_s'] = ev['start'] - events[i-1]['start']
            ev['drev_prev'] = rev_between(t, unwrapped_angle, events[i-1]['start'], ev['start'], sign)
            ev['drev_prev_integrated'] = integrated_rev(t, v, sign, events[i-1]['start'], ev['start'])
    return events


def spectrum(t, v, unwrapped_angle, hi):
    grid = np.arange(hi - 2.0, hi, 0.002)
    speed = np.interp(grid, t, v)
    angle = np.interp(grid, t, unwrapped_angle)
    detrended = speed - np.polyval(np.polyfit(grid, speed, 1), grid)
    power = abs(np.fft.rfft(detrended * np.hanning(len(detrended)))) ** 2
    freqs = np.fft.rfftfreq(len(detrended), 0.002)
    band = (freqs >= 2) & (freqs <= 100)
    dominant = float(freqs[band][np.argmax(power[band])])
    phase = angle * 2 * np.pi / 8192
    harmonics = []
    for order in range(1, 13):
        design = np.column_stack([np.ones(len(phase)), grid - grid.mean(),
                                  np.sin(order * phase), np.cos(order * phase)])
        coef = np.linalg.lstsq(design, speed, rcond=None)[0]
        residual = speed - design @ coef
        harmonics.append({'order': order,
                          'amplitude_rpm': float(np.hypot(coef[-2], coef[-1])),
                          'explained_frac': float(1 - np.sum(residual ** 2) / np.sum(detrended ** 2))})
    rotor_hz = float(abs(np.mean(speed)) / 60)
    return {'dominant_hz': dominant, 'rotor_hz': rotor_hz,
            'order7_expected_hz': 7 * rotor_hz,
            'order7_amplitude_rpm': harmonics[6]['amplitude_rpm'],
            'order7_explained_frac': harmonics[6]['explained_frac'],
            'harmonics': harmonics}


def cluster_events(events, t, unwrapped, sign):
    groups = []
    for ev in events:
        if groups and ev['start'] - groups[-1][-1]['start'] <= CLUSTER['merge_gap_s']:
            groups[-1].append(ev)
        else:
            groups.append([ev])
    clusters = []
    for ci, group in enumerate(groups):
        for ev in group:
            ev['cluster_index'] = ci
        deepest = max(group, key=lambda e: e['depth_pct'])
        clusters.append({'onset': group[0]['start'], 'deepest_start': deepest['start'],
                         'deepest_depth_pct': deepest['depth_pct'],
                         'events_in_cluster': len(group)})
    for i, c in enumerate(clusters):
        c['dt_prev_s'] = c['onset'] - clusters[i-1]['onset'] if i else None
        c['drev_prev'] = (rev_between(t, unwrapped, clusters[i-1]['onset'], c['onset'], sign)
                          if i else None)
    return clusters


def slow_spectrum(t, v, sign, lo, hi):
    ma = moving_average(t, sign * v, EVENTS['smooth_s'])
    sel = (t >= lo + EVENTS['exclude_start_s']) & (t <= hi - EVENTS['exclude_end_s'])
    if sel.sum() < 100:
        return None
    grid = np.arange(t[sel][0], t[sel][-1], 0.01)
    y = np.interp(grid, t[sel], ma[sel])
    y = y - np.polyval(np.polyfit(grid, y, 1), grid)
    y = y - y.mean()
    power = abs(np.fft.rfft(y * np.hanning(len(y)))) ** 2
    freqs = np.fft.rfftfreq(len(y), 0.01)
    band = (freqs >= 0.1) & (freqs <= 10)
    if not band.any():
        return None
    k = int(np.argmax(power[band]))
    return {'peak_hz': float(freqs[band][k]),
            'peak_power_frac': float(power[band][k] / np.sum(power[band])),
            'window_s': [float(grid[0] - lo), float(grid[-1] - lo)]}


def analyze_phase(m, lo, hi, target):
    sign = 1 if target > 0 else -1
    t = m[:, 0]; v = m[:, 2]
    window = 2 if target else 0.5
    tail = v[(t >= hi - window) & (t <= hi)]
    out = {'target': target, 'samples': int(len(t)),
           'tail_samples': int(len(tail)), 'tail_mean_rpm': float(np.mean(tail)),
           'tail_std_rpm': float(np.std(tail)),
           'tail_min_rpm': float(np.min(tail)), 'tail_max_rpm': float(np.max(tail)),
           'feedback_max_gap_ms': float(np.max(np.diff(t)) * 1000)}
    trace = None
    if not target:
        out['zero_peak_abs_rpm'] = float(np.max(np.abs(tail)))
        return out, None
    out['mean_error_pct'] = float(abs(np.mean(tail) - target) / abs(target) * 100)
    out['error_95pct_rpm'] = np.percentile(tail - target, [2.5, 97.5]).tolist()
    out['tail_std_pct'] = float(np.std(tail) / abs(target) * 100)
    out['overshoot_pct'] = float(max(0.0, np.max(sign * v) - abs(target)) / abs(target) * 100)
    out['settling_025s_s'] = settling_time(t, v, target, lo, hi)
    unwrapped = unwrap_angle(m)
    out['events'] = detect_events(t, v, target, lo, hi, unwrapped)
    out['clusters'] = cluster_events(out['events'], t, unwrapped, sign)
    out['slow'] = slow_spectrum(t, v, sign, lo, hi)
    out['rev_total_angle'] = rev_between(t, unwrapped, lo, hi, sign)
    out['rev_total_integrated'] = integrated_rev(t, v, sign, lo, hi)
    out['spectrum'] = spectrum(t, v, unwrapped, hi)
    trace = {'t_rel': (t - lo).tolist()[::5], 'rpm': v.tolist()[::5],
             'ma_signed': moving_average(t, sign * v, EVENTS['smooth_s']).tolist()[::5]}
    return out, trace


def analyze_capture(speed, run_label, path):
    d = load_capture(path)
    meta = {'speed': speed, 'run': run_label, 'capture': str(path.relative_to(ROOT)),
            'duration_s': d['duration'], 'counts': d['counts'],
            'status_values': sorted({(int(row[1]), int(row[2])) for row in d['status']}),
            'fault_diagnostic_samples': int(np.sum(d['diag'][:, 2] >= 2)),
            'command_peak': float(np.max(np.abs(d['cmd'][:, 1:]))),
            'saturation_frames': int(np.sum(np.any(np.abs(d['cmd'][:, 1:]) >= 1500, axis=1))),
            'other_motor_nonzero_frames': int(np.sum(np.count_nonzero(d['cmd'][:, 1:], axis=1) > 1))}
    phases, traces = [], {}
    for active, exitrow in sessions(d['diag']):
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
        phase_records = []
        for g in groups:
            target = int(g[0, 3])
            lo = float(g[0, 0]); hi = float(g[-1, 0])
            m = d['raw'][motor]
            m = m[(m[:, 0] >= lo) & (m[:, 0] <= hi)]
            if not len(m):
                continue
            rec, trace = analyze_phase(m, lo, hi, target)
            rec.update({'phase_start_s': lo, 'phase_end_s': hi, 'motor': motor,
                        'direction': '+' if target > 0 else ('-' if target < 0 else '0'),
                        'speed': speed, 'run': run_label})
            phase_records.append(rec)
            if trace is not None:
                traces[(speed, run_label, motor, rec['direction'])] = trace
        phases.append({'motor': motor, 'complete': bool(complete),
                       'start_s': float(a[0, 0]),
                       'end_s': float(exitrow[0]) if exitrow is not None else None,
                       'targets': targets, 'phases': phase_records})
    return meta, phases, traces


def mean_of(values):
    vals = [x for x in values if x is not None]
    return float(np.mean(vals)) if vals else None


def main():
    result = {'purpose': 'speed-only diagnostic comparison 400 vs 600 rotor rpm; not acceptance',
              'date': '2026-10-29',
              'method': {'tail_window_s': 2, 'zero_window_s': 0.5,
                         'overshoot': 'max(0, max(sign*target*raw_rpm)-|target|)/|target| over whole phase',
                         'settling': SETTLE,
                         'events': EVENTS,
                         'clusters': {'merge_gap_s': CLUSTER['merge_gap_s'],
                                      'note': 'events whose onsets are within 0.8 s are one drop cluster; cluster onset/dRev used for periodicity'},
                         'slow_spectrum': {'smoothing_s': EVENTS['smooth_s'], 'window': 'phase minus first 1 s and last 0.2 s',
                                           'resample_s': 0.01, 'band_hz': [0.1, 10], 'detrend': 'linear',
                                           'window_fn': 'Hann', 'note': 'low-frequency drop periodicity; not the 7th-order ripple'},
                         'spectrum': {'resample_s': 0.002, 'window_s': 2, 'band_hz': [2, 100],
                                      'detrend': 'linear', 'window_fn': 'Hann',
                                      'harmonics': 'angle-locked sin/cos orders 1..12'},
                         'angle': 'feedback bytes 0..1 mod 8192; delta=(d+4096)%8192-4096 unwrapped; rev=angle/8192',
                         'time_base': 'device_us zeroed per file; no interpolation across gaps'},
              'sources': {}, 'conditions': [], 'comparison': {}, 'notes': []}
    all_events = []
    traces = {}
    for speed in (400, 600):
        result['sources'][str(speed)] = []
        for run_label, path in SOURCES[speed]:
            assert path.exists(), path
            meta, phases, tr = analyze_capture(speed, run_label, path)
            traces.update(tr)
            meta['runs'] = [{'motor': r['motor'], 'complete': r['complete']} for r in phases]
            result['sources'][str(speed)].append(meta)
            for run in phases:
                if not run['complete']:
                    result['notes'].append(f'{speed} {run_label} motor {run["motor"]} incomplete; excluded from comparison')
                    continue
                for ph in run['phases']:
                    result['conditions'].append(ph)
                    for i, ev in enumerate(ph.get('events', [])):
                        all_events.append({'speed': speed, 'run': run_label, 'motor': run['motor'],
                                           'direction': ph['direction'], 'target': ph['target'],
                                           'phase_start_s': ph['phase_start_s'],
                                           'event_index': i, **ev})

    for motor in (1, 2, 4):
        result['comparison'][str(motor)] = {}
        for direction in ('+', '-'):
            entry = {}
            for speed in (400, 600):
                recs = [c for c in result['conditions']
                        if c['speed'] == speed and c['motor'] == motor
                        and c['direction'] == direction and c['target']]
                evs = [ev for ev in all_events if ev['speed'] == speed and ev['motor'] == motor
                       and ev['direction'] == direction]
                entry[str(speed)] = {
                    'runs': [c['run'] for c in recs],
                    'tail_mean_rpm': [c['tail_mean_rpm'] for c in recs],
                    'tail_std_rpm': [c['tail_std_rpm'] for c in recs],
                    'tail_min_rpm': [c['tail_min_rpm'] for c in recs],
                    'tail_max_rpm': [c['tail_max_rpm'] for c in recs],
                    'overshoot_pct': [c['overshoot_pct'] for c in recs],
                    'settling_025s_s': [c['settling_025s_s'] for c in recs],
                    'event_count': len(evs),
                    'event_dt_s': [ev['dt_prev_s'] for ev in evs if ev['dt_prev_s'] is not None],
                    'event_drev': [ev['drev_prev'] for ev in evs if ev['drev_prev'] is not None],
                    'event_drev_integrated': [ev['drev_prev_integrated'] for ev in evs if ev['drev_prev_integrated'] is not None],
                    'event_depth_pct': [ev['depth_pct'] for ev in evs],
                    'event_recovery_s': [ev['recovery_s'] for ev in evs],
                    'dominant_hz': [c['spectrum']['dominant_hz'] for c in recs],
                    'order7_amplitude_rpm': [c['spectrum']['order7_amplitude_rpm'] for c in recs],
                    'order7_explained_frac': [c['spectrum']['order7_explained_frac'] for c in recs],
                    'slow_peak_hz': [c['slow']['peak_hz'] for c in recs if c.get('slow')],
                    'cluster_count': sum(len(c.get('clusters', [])) for c in recs),
                    'cluster_dt_s': [cc['dt_prev_s'] for c in recs for cc in c.get('clusters', []) if cc['dt_prev_s'] is not None],
                    'cluster_drev': [cc['drev_prev'] for c in recs for cc in c.get('clusters', []) if cc['drev_prev'] is not None],
                    'rev_total_angle': [c['rev_total_angle'] for c in recs],
                    'rev_total_integrated': [c['rev_total_integrated'] for c in recs],
                }
            e400, e600 = entry['400'], entry['600']
            m400 = mean_of([abs(x) for x in e400['tail_mean_rpm']])
            m600 = mean_of([abs(x) for x in e600['tail_mean_rpm']])
            speed_ratio = (m600 / m400) if (m400 and m600) else None
            mean_dt400 = mean_of(e400['event_dt_s'])
            mean_dt600 = mean_of(e600['event_dt_s'])
            mean_drev400 = mean_of(e400['event_drev'])
            mean_drev600 = mean_of(e600['event_drev'])
            dom400 = mean_of(e400['dominant_hz'])
            dom600 = mean_of(e600['dominant_hz'])
            slow400 = mean_of(e400['slow_peak_hz'])
            slow600 = mean_of(e600['slow_peak_hz'])
            cl_dt400 = mean_of(e400['cluster_dt_s'])
            cl_dt600 = mean_of(e600['cluster_dt_s'])
            cl_drev400 = mean_of(e400['cluster_drev'])
            cl_drev600 = mean_of(e600['cluster_drev'])
            entry['aggregate'] = {
                'tail_mean_rpm_mean': {s: mean_of(entry[s]['tail_mean_rpm']) for s in ('400', '600')},
                'tail_std_rpm_mean': {s: mean_of(entry[s]['tail_std_rpm']) for s in ('400', '600')},
                'measured_speed_ratio_600_over_400': speed_ratio,
                'event_dt_mean_s': {'400': mean_dt400, '600': mean_dt600},
                'event_drev_mean': {'400': mean_drev400, '600': mean_drev600},
                'event_dt_ratio_400_over_600': (mean_dt400 / mean_dt600) if (mean_dt400 and mean_dt600) else None,
                'event_drev_ratio_400_over_600': (mean_drev400 / mean_drev600) if (mean_drev400 and mean_drev600) else None,
                'cluster_count': {s: entry[s]['cluster_count'] for s in ('400', '600')},
                'cluster_dt_mean_s': {'400': cl_dt400, '600': cl_dt600},
                'cluster_drev_mean': {'400': cl_drev400, '600': cl_drev600},
                'cluster_dt_ratio_400_over_600': (cl_dt400 / cl_dt600) if (cl_dt400 and cl_dt600) else None,
                'cluster_drev_ratio_400_over_600': (cl_drev400 / cl_drev600) if (cl_drev400 and cl_drev600) else None,
                'dominant_hz_mean': {'400': dom400, '600': dom600},
                'dominant_hz_ratio_400_over_600': (dom400 / dom600) if (dom400 and dom600) else None,
                'slow_peak_hz_mean': {'400': slow400, '600': slow600},
                'slow_peak_ratio_400_over_600': (slow400 / slow600) if (slow400 and slow600) else None,
            }
            result['comparison'][str(motor)][direction] = entry

    (HERE / 'comparison400vs600.json').write_text(
        json.dumps({**result, 'events': all_events}, indent=2, ensure_ascii=False), encoding='utf-8')

    with (HERE / 'events.csv').open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['speed', 'run', 'motor', 'direction', 'target', 'event_index',
                    'start_s_rel_phase', 'start_s_abs', 'minimum_s_rel', 'minimum_ma_rpm',
                    'depth_pct', 'recovery_s_rel', 'recovery_time_s', 'duration_s',
                    'dt_prev_s', 'drev_prev', 'drev_prev_integrated', 'rev_cumulative', 'signed_min_rpm', 'cluster_index'])
        for ev in all_events:
            w.writerow([ev['speed'], ev['run'], ev['motor'], ev['direction'], ev['target'],
                        ev['event_index'], ev['start'] - ev['phase_start_s'], ev['start'],
                        ev['minimum'] - ev['phase_start_s'], ev['minimum_ma_rpm'],
                        ev['depth_pct'],
                        (ev['recovery'] - ev['phase_start_s']) if ev['recovery'] else None,
                        ev['recovery_s'], ev['duration_s'], ev['dt_prev_s'], ev['drev_prev'],
                        ev['drev_prev_integrated'], ev['rev_cumulative'], ev['signed_min_rpm'],
                        ev.get('cluster_index')])
    print('CONDITIONS', len(result['conditions']), 'EVENTS', len(all_events))
    for motor in (1, 2, 4):
        for d in ('+', '-'):
            e = result['comparison'][str(motor)][d]
            ag = e['aggregate']
            print(f"motor {motor} {d}: std 400={ag['tail_std_rpm_mean']['400']:.1f} 600={ag['tail_std_rpm_mean']['600']:.1f} | "
                  f"events 400={e['400']['event_count']} 600={e['600']['event_count']} | "
                  f"clusters {ag['cluster_count']['400']}/{ag['cluster_count']['600']} "
                  f"cluster_dt_ratio={ag['cluster_dt_ratio_400_over_600']} drev_ratio={ag['cluster_drev_ratio_400_over_600']} "
                  f"slow_ratio={ag['slow_peak_ratio_400_over_600']} dom_ratio={ag['dominant_hz_ratio_400_over_600']}")
    return result, all_events, traces


def make_plots(result, traces, events):
    fig, axes = plt.subplots(3, 2, figsize=(13, 10), sharex=True)
    for row, motor in enumerate((1, 2, 4)):
        for col, direction in enumerate(('+', '-')):
            ax = axes[row][col]
            for speed, run, style in ((400, 'run1', '-'), (400, 'run2', '--'),
                                      (600, 'run1', '-'), (600, 'run2', '--')):
                tr = traces.get((speed, run, motor, direction))
                if not tr:
                    continue
                color = '#1f77b4' if speed == 400 else '#d35400'
                ax.plot(tr['t_rel'], tr['rpm'], style, color=color, linewidth=.8,
                        alpha=.75, label=f'{speed} {run}')
            ax.axhline(400 if direction == '+' else -400, color='#1f77b4', lw=.7, ls=':')
            ax.axhline(600 if direction == '+' else -600, color='#d35400', lw=.7, ls=':')
            ax.set_title(f'motor {motor} direction {direction}: raw feedback rpm')
            ax.set_ylabel('rpm'); ax.grid(alpha=.3)
    axes[2][0].set_xlabel('time in phase (s)'); axes[2][1].set_xlabel('time in phase (s)')
    axes[0][0].legend(fontsize=7, ncol=2)
    fig.tight_layout(); fig.savefig(HERE / 'curves400-600.png', dpi=130); plt.close(fig)

    fig, axes = plt.subplots(3, 2, figsize=(13, 9), sharex=True)
    for row, motor in enumerate((1, 2, 4)):
        for col, direction in enumerate(('+', '-')):
            ax = axes[row][col]
            for speed, run, style in ((400, 'run1', '-'), (400, 'run2', '--'),
                                      (600, 'run1', '-'), (600, 'run2', '--')):
                tr = traces.get((speed, run, motor, direction))
                if not tr:
                    continue
                color = '#1f77b4' if speed == 400 else '#d35400'
                ax.plot(tr['t_rel'], tr['ma_signed'], style, color=color, linewidth=.9,
                        alpha=.8, label=f'{speed} {run}')
            for ev in events:
                if ev['motor'] != motor or ev['direction'] != direction:
                    continue
                color = '#1f77b4' if ev['speed'] == 400 else '#d35400'
                ax.scatter([ev['start'] - ev['phase_start_s']], [ev['minimum_ma_rpm']],
                           color=color, marker='o', s=22, zorder=5)
            ax.set_title(f'motor {motor} direction {direction}: 20 ms mean, events marked')
            ax.set_ylabel('rpm'); ax.grid(alpha=.3)
    axes[2][0].set_xlabel('time in phase (s)'); axes[2][1].set_xlabel('time in phase (s)')
    axes[0][0].legend(fontsize=7, ncol=2)
    fig.tight_layout(); fig.savefig(HERE / 'events400.png', dpi=130); plt.close(fig)

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.2))
    labels, dom, amp, slow = [], {400: [], 600: []}, {400: [], 600: []}, {400: [], 600: []}
    for motor in (1, 2, 4):
        for direction in ('+', '-'):
            labels.append(f'{motor}{direction}')
            for speed in (400, 600):
                recs = [c for c in result['conditions'] if c['speed'] == speed
                        and c['motor'] == motor and c['direction'] == direction]
                dom[speed].append(float(np.mean([c['spectrum']['dominant_hz'] for c in recs])) if recs else np.nan)
                amp[speed].append(float(np.mean([c['spectrum']['order7_amplitude_rpm'] for c in recs])) if recs else np.nan)
                slow[speed].append(float(np.mean([c['slow']['peak_hz'] for c in recs if c.get('slow')])) if recs else np.nan)
    x = np.arange(len(labels)); w = 0.38
    for ax, data, ylabel in ((axes[0], dom, 'dominant frequency in 2..100 Hz of raw rpm (Hz)'),
                             (axes[1], slow, 'dominant slow-drop frequency 0.1..10 Hz (Hz)'),
                             (axes[2], amp, 'angle-locked order-7 amplitude (rpm)')):
        ax.bar(x - w/2, data[400], w, color='#1f77b4', label='400 rpm')
        ax.bar(x + w/2, data[600], w, color='#d35400', label='600 rpm')
        ax.set_xticks(x); ax.set_xticklabels(labels)
        ax.set_ylabel(ylabel); ax.legend(); ax.grid(axis='y', alpha=.3)
    fig.tight_layout(); fig.savefig(HERE / 'spectrum400-600.png', dpi=130); plt.close(fig)


if __name__ == '__main__':
    result, events, traces = main()
    make_plots(result, traces, events)
    print('WROTE comparison400vs600.json, events.csv, curves400-600.png, events400.png, spectrum400-600.png')
