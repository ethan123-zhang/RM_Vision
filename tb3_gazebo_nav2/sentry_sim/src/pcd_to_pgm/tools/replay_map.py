#!/usr/bin/env python3
"""Replay a dumped cloud run through the same mapper, offline.

The node can dump its filtered, map-frame clouds (``--dump-dir``); this tool
re-runs the occupancy grid over them with different parameters, so tuning does
not need another simulator run.

    python3 tools/replay_map.py --dump /tmp/s2b/dump --out /tmp/s2b/replay \
        --truth /path/truth.yaml --end-margin 3 --occ-hit 0.5
"""

import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'src', 'pcd_to_pgm'))

from pcd_to_pgm.grid import OccGrid            # noqa: E402
from pcd_to_pgm.compare import (compare, format_report,  # noqa: E402
                                write_json)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--dump', required=True, help='dir with index.json')
    parser.add_argument('--out', required=True, help='output base path')
    parser.add_argument('--truth', default='', help='truth yaml -> compare')
    parser.add_argument('--json', default='')
    parser.add_argument('--diff', default='')
    parser.add_argument('--free-miss', type=float, default=-0.4)
    parser.add_argument('--free-exit', type=float, default=-0.6)
    parser.add_argument('--min-occ-hits', type=int, default=2)
    parser.add_argument('--block-hits', type=int, default=3)
    parser.add_argument('--behind-cells', type=int, default=1)
    parser.add_argument('--end-margin', type=int, default=2)
    parser.add_argument('--despeckle', type=int, default=1)
    parser.add_argument('--tol', type=int, default=1)
    args = parser.parse_args(argv)

    with open(os.path.join(args.dump, 'index.json')) as handle:
        index = json.load(handle)
    grid = OccGrid(
        resolution=index['resolution'],
        origin=tuple(index['origin']),
        free_miss=args.free_miss, free_exit=args.free_exit,
        min_occ_hits=args.min_occ_hits, block_hits=args.block_hits,
        behind_cells=args.behind_cells, end_margin=args.end_margin,
        despeckle=args.despeckle)

    poses = []
    import time
    started = time.time()
    for entry in index['scans']:
        points = np.load(os.path.join(args.dump, entry['file']))
        if points.size == 0:
            continue
        grid.add_scan(points[:, 0], points[:, 1], entry['ox'], entry['oy'])
        poses.append({'sx': entry['ox'], 'sy': entry['oy']})
    stats = grid.save(args.out, comment='offline replay of %s' % args.dump)
    stats['replay_seconds'] = round(time.time() - started, 1)
    print(json.dumps(stats, indent=2))

    traj = args.out + '_traj.json'
    with open(traj, 'w') as handle:
        json.dump({'poses': poses}, handle)

    if args.truth:
        report = compare(args.truth, args.out + '.yaml', traj_path=traj,
                         tol=args.tol,
                         diff_path=args.diff or (args.out + '_diff.png'))
        print(format_report(report))
        write_json(report, args.json or (args.out + '_compare.json'))
    return 0


if __name__ == '__main__':
    sys.exit(main())