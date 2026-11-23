"""``ros2 run pcd_to_pgm compare_maps``: offline truth-vs-generated comparison."""

import argparse
import os
import sys

from .compare import compare, format_report, write_json


def parse_args(argv):
    parser = argparse.ArgumentParser(
        prog='compare_maps',
        description='Compare a generated Nav2 map against a truth map: '
                    'occupancy recall, free-area error, connectivity.')
    parser.add_argument('--truth', required=True, help='truth map.yaml')
    parser.add_argument('--map', required=True, help='generated map.yaml')
    parser.add_argument('--traj', default='',
                        help='<out>_traj.json from pcd_to_pgm (enables the '
                             'visible-cell recall ceiling)')
    parser.add_argument('--json', default='', help='write the report here')
    parser.add_argument('--diff', default='', help='write a diff image here')
    parser.add_argument('--tol', type=int, default=1)
    parser.add_argument('--min-component-cells', type=int, default=25)
    parser.add_argument('--max-range', type=float, default=10.0)
    parser.add_argument('--sweep-rays', type=int, default=2048)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    report = compare(args.truth, args.map,
                     traj_path=args.traj or None,
                     tol=args.tol,
                     min_component_cells=args.min_component_cells,
                     max_range=args.max_range,
                     n_rays=args.sweep_rays,
                     diff_path=args.diff or None)
    print(format_report(report))
    if args.json:
        write_json(report, args.json)
        print('report: %s' % args.json)
    if 'diff_image' in report:
        print('diff  : %s' % report['diff_image'])
    return 0