"""Lubrication A/B comparison at 400rpm (2/4 wheels lubricated by user).

Reuses the frozen methods and parameters from analyze400compare.py (tail windows,
drop events, clusters, slow spectrum, order-7 regression) and the frozen startup
breakaway scanner from analyze_startup_breakaway.py. Nothing is re-tuned.

Pre-lubrication : 400rpm run1 (18:01) and run2 (19:44), motors 1/2/4.
Post-lubrication: lube window1 (motors 1/3/4; motor 2 missed by extra KEY press)
                  and lube window2 (motors 1/2/4).
Motor 1 is the untouched reference (not lubricated).
Motor 3 has no pre-lubrication 400rpm record; reported separately, not as a controlled pair.

Outputs (its own products only):
  lube-comparison.json, lube-curves.png, lube-events.png

Run: python analyze_lube_compare.py
"""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import analyze400compare as A
import analyze_startup_breakaway as B

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
LUBE = HERE / 'lube'

WINDOWS = {
    'pre-run1': ('pre', ROOT / '_probe/speed400_capture/run1-1793268106993/capture.csv'),
    'pre-run2': ('pre', ROOT / '_probe/speed400_capture/run2-1793274040118/capture.csv'),
    'post-window1': ('post', ROOT / '_probe/speed400_capture/run3-lube1-1793276985937/capture.csv'),
    'post-window2': ('post', ROOT / '_probe/speed400_capture/run3-lube2-1793277188520/capture.csv'),
}
TARGETS = A.TARGETS[400]


def analyze_window(label, phase, path):
    d = A.load_capture(path)
    meta = {'window': label, 'phase_group': phase,
            'capture': str(path.relative_to(ROOT)),
            'duration_s': d['duration'], 'frames': sum(d['counts'].values()),
            'status_values': sorted({(int(r[1]), int(r[2])) for r in d['status']}),
            'fault_diagnostic_samples': int(np.sum(d['diag'][:, 2] >= 2)),
            'command_peak': float(np.max(np.abs(d['cmd'][:, 1:]))),
            'saturation_frames': int(np.sum(np.any(np.abs(d['cmd'][:, 1:]) >= 1500, axis=1))),
            'other_motor_nonzero_frames': int(np.sum(np.count_nonzero(d['cmd'][:, 1:], axis=1) > 1))}
    phases, traces = [], {}
    for active, exitrow in A.sessions(d['diag']):
        a = np.array(active)
        motor = int(a[0, 1])
        groups, start = [], 0
        for k in range(1, len(a) + 1):
            if k == len(a) or a[k, 3] != a[start, 3]:
                groups.append(a[start:k]); start = k
        targets = [int(g[0, 3]) for g in groups]
        complete = (exitrow is not None and int(exitrow[2]) == 0
                    and targets == TARGETS
                    and abs((exitrow[0] - a[0, 0]) - 18) <= 0.08)
        run_phases = []
        for g in groups:
            target = int(g[0, 3])
            lo, hi = float(g[0, 0]), float(g[-1, 0])
            m = d['raw'][motor]
            m = m[(m[:, 0] >= lo) & (m[:, 0] <= hi)]
            if not len(m):
                continue
            rec, trace = A.analyze_phase(m, lo, hi, target)
            rec.update({'phase_start_s': lo, 'phase_end_s': hi, 'motor': motor,
                        'direction': '+' if target > 0 else ('-' if target < 0 else '0'),
                        'window': label, 'phase_group': phase})
            rec['startup'] = B.frozen_intervals(m[:, 0], m[:, 1], m[:, 3]) if target else []
            run_phases.append(rec)
            if trace is not None:
                traces[(phase, label, motor, rec['direction'])] = trace
        phases.append({'motor': motor, 'complete': bool(complete), 'phases': run_phases})
    return meta, phases, traces


def mean_of(values):
    vals = [x for x in values if x is not None]
    return float(np.mean(vals)) if vals else None


def build():
    result = {'purpose': 'lubrication A/B at 400rpm; 2/4 wheels lubricated between pre and post; not acceptance',
              'date': '2026-10-29',
              'method': 'identical frozen functions imported from analyze400compare.py and analyze_startup_breakaway.py',
              'intervention': 'user applied lubricating oil to 2/4 wheels; location/amount/time not independently recorded',
              'control': 'motor 1 untouched; motor 3 recorded post only (no pre 400rpm record)',
              'windows': {}, 'conditions': [], 'comparison': {}, 'startup_intervals': {}}
    traces_all = {}
    for label, (phase, path) in WINDOWS.items():
        assert path.exists(), path
        meta, phases, traces = analyze_window(label, phase, path)
        meta['runs'] = [{'motor': r['motor'], 'complete': r['complete']} for r in phases]
        result['windows'][label] = meta
        traces_all.update(traces)
        for run in phases:
            if not run['complete']:
                continue
            for ph in run['phases']:
                result['conditions'].append(ph)
                if ph.get('startup'):
                    result['startup_intervals'].setdefault(label, []).append(
                        {'motor': run['motor'], 'target': ph['target'],
                         'intervals': ph['startup']})

    for motor in (1, 2, 4):
        result['comparison'][str(motor)] = {}
        for direction in ('+', '-'):
            entry = {}
            for phase in ('pre', 'post'):
                recs = [c for c in result['conditions']
                        if c['phase_group'] == phase and c['motor'] == motor
                        and c['direction'] == direction and c['target']]
                wins = [c['window'] for c in recs]
                clusters = [cl for c in recs for cl in c.get('clusters', [])]
                entry[phase] = {
                    'windows': wins,
                    'tail_mean_rpm': [c['tail_mean_rpm'] for c in recs],
                    'tail_std_rpm': [c['tail_std_rpm'] for c in recs],
                    'mean_abs_error_pct': [abs(c['tail_mean_rpm'] - c['target']) / abs(c['target']) * 100 for c in recs],
                    'overshoot_pct': [c['overshoot_pct'] for c in recs],
                    'settling_025s_s': [c['settling_025s_s'] for c in recs],
                    'event_count': sum(len(c.get('events', [])) for c in recs),
                    'event_depth_pct': [ev['depth_pct'] for c in recs for ev in c.get('events', [])],
                    'cluster_count': len(clusters),
                    'cluster_dt_s': [cl['dt_prev_s'] for cl in clusters if cl['dt_prev_s'] is not None],
                    'cluster_drev': [cl['drev_prev'] for cl in clusters if cl['drev_prev'] is not None],
                    'slow_peak_hz': [c['slow']['peak_hz'] for c in recs if c.get('slow')],
                    'order7_amplitude_rpm': [c['spectrum']['order7_amplitude_rpm'] for c in recs],
                    'rev_total_angle': [c['rev_total_angle'] for c in recs],
                    'startup_frozen_total_s': [sum(iv['duration_s'] for iv in c.get('startup', [])) for c in recs],
                    'startup_frozen_max_s': [max([iv['duration_s'] for iv in c.get('startup', [])], default=0.0) for c in recs],
                }
            entry['delta'] = {
                'tail_std_rpm_mean_pre': mean_of(entry['pre']['tail_std_rpm']),
                'tail_std_rpm_mean_post': mean_of(entry['post']['tail_std_rpm']),
                'tail_std_change_pct': ((mean_of(entry['post']['tail_std_rpm']) / mean_of(entry['pre']['tail_std_rpm']) - 1) * 100)
                                       if mean_of(entry['pre']['tail_std_rpm']) else None,
                'mean_abs_error_pct_pre': mean_of(entry['pre']['mean_abs_error_pct']),
                'mean_abs_error_pct_post': mean_of(entry['post']['mean_abs_error_pct']),
                'cluster_drev_mean_pre': mean_of(entry['pre']['cluster_drev']),
                'cluster_drev_mean_post': mean_of(entry['post']['cluster_drev']),
                'cluster_dt_mean_pre': mean_of(entry['pre']['cluster_dt_s']),
                'cluster_dt_mean_post': mean_of(entry['post']['cluster_dt_s']),
                'event_depth_mean_pre': mean_of(entry['pre']['event_depth_pct']),
                'event_depth_mean_post': mean_of(entry['post']['event_depth_pct']),
                'startup_frozen_max_pre': max(entry['pre']['startup_frozen_max_s']),
                'startup_frozen_max_post': max(entry['post']['startup_frozen_max_s']),
                'order7_amp_mean_pre': mean_of(entry['pre']['order7_amplitude_rpm']),
                'order7_amp_mean_post': mean_of(entry['post']['order7_amplitude_rpm']),
            }
            result['comparison'][str(motor)][direction] = entry

    # motor 3 (post only, extra reference)
    result['motor3_post_only'] = {}
    for direction in ('+', '-'):
        recs = [c for c in result['conditions'] if c['motor'] == 3 and c['direction'] == direction and c['target']]
        result['motor3_post_only'][direction] = {
            'windows': [c['window'] for c in recs],
            'tail_mean_rpm': [c['tail_mean_rpm'] for c in recs],
            'tail_std_rpm': [c['tail_std_rpm'] for c in recs],
            'cluster_count': sum(len(c.get('clusters', [])) for c in recs),
        }

    (LUBE / 'lube-comparison.json').write_text(
        json.dumps(result, indent=2, ensure_ascii=False), encoding='utf-8')

    for motor in (1, 2, 4):
        for d in ('+', '-'):
            e = result['comparison'][str(motor)][d]['delta']
            print(f"motor {motor} {d}: std {e['tail_std_rpm_mean_pre']:.1f}->{e['tail_std_rpm_mean_post']:.1f} "
                  f"({e['tail_std_change_pct']:+.1f}%) | meanerr {e['mean_abs_error_pct_pre']:.2f}->{e['mean_abs_error_pct_post']:.2f}% | "
                  f"clusters {result['comparison'][str(motor)][d]['pre']['cluster_count']}->{result['comparison'][str(motor)][d]['post']['cluster_count']} "
                  f"drev {e['cluster_drev_mean_pre']}->{e['cluster_drev_mean_post']} | "
                  f"startup_max {e['startup_frozen_max_pre']:.2f}->{e['startup_frozen_max_post']:.2f}s")
    return result, traces_all


def make_plots(result, traces):
    colors = {'pre': '#7f7f7f', 'post': '#d35400'}
    fig, axes = plt.subplots(3, 2, figsize=(13, 9), sharex=True)
    for row, motor in enumerate((1, 2, 4)):
        for col, direction in enumerate(('+', '-')):
            ax = axes[row][col]
            for phase in ('pre', 'post'):
                labels = sorted({lab for (p, lab, mo, di) in traces if p == phase and mo == motor and di == direction})
                for i, lab in enumerate(labels):
                    tr = traces[(phase, lab, motor, direction)]
                    ax.plot(tr['t_rel'], tr['ma_signed'], '-', color=colors[phase],
                            linewidth=1.0, alpha=0.55 if i else 0.95,
                            label=f'{phase} {lab}')
            ax.set_title(f'motor {motor} direction {direction}: 20 ms mean, pre vs post lubrication')
            ax.set_ylabel('rpm'); ax.grid(alpha=.3)
    axes[2][0].set_xlabel('time in phase (s)'); axes[2][1].set_xlabel('time in phase (s)')
    handles, labels = axes[0][0].get_legend_handles_labels()
    uniq = dict(zip(labels, handles))
    axes[0][0].legend(uniq.values(), uniq.keys(), fontsize=7, ncol=2)
    fig.tight_layout(); fig.savefig(LUBE / 'lube-curves.png', dpi=130); plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    labels = []
    pre_std, post_std, pre_ev, post_ev = [], [], [], []
    for motor in (1, 2, 4):
        for direction in ('+', '-'):
            labels.append(f'{motor}{direction}')
            e = result['comparison'][str(motor)][direction]['delta']
            pre_std.append(e['tail_std_rpm_mean_pre']); post_std.append(e['tail_std_rpm_mean_post'])
            pre_ev.append(result['comparison'][str(motor)][direction]['pre']['event_count'])
            post_ev.append(result['comparison'][str(motor)][direction]['post']['event_count'])
    x = np.arange(len(labels)); w = 0.38
    axes[0].bar(x - w/2, pre_std, w, color='#7f7f7f', label='pre-lubrication')
    axes[0].bar(x + w/2, post_std, w, color='#d35400', label='post-lubrication')
    axes[0].set_xticks(x); axes[0].set_xticklabels(labels)
    axes[0].set_ylabel('tail 2 s std (rpm)'); axes[0].legend(); axes[0].grid(axis='y', alpha=.3)
    axes[1].bar(x - w/2, pre_ev, w, color='#7f7f7f', label='pre-lubrication')
    axes[1].bar(x + w/2, post_ev, w, color='#d35400', label='post-lubrication')
    axes[1].set_xticks(x); axes[1].set_xticklabels(labels)
    axes[1].set_ylabel('detected drop events'); axes[1].legend(); axes[1].grid(axis='y', alpha=.3)
    fig.tight_layout(); fig.savefig(LUBE / 'lube-events.png', dpi=130); plt.close(fig)


if __name__ == '__main__':
    result, traces = build()
    make_plots(result, traces)
    print('WROTE lube/lube-comparison.json, lube/lube-curves.png, lube/lube-events.png')
