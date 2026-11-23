"""``ros2 run pcd_to_pgm pcd_to_pgm``: map while it runs, then save.

    # 90 s of mapping, then save + compare against the truth map
    ros2 run pcd_to_pgm pcd_to_pgm --out ~/sentry_ws/results/odin_map \
        --duration 90 --compare /path/truth.yaml

    # run until Ctrl-C (incremental), save on exit
    ros2 run pcd_to_pgm pcd_to_pgm --duration 0

The node publishes ~/map (OccupancyGrid) while it runs and offers
``ROS_DOMAIN_ID=... ros2 service call /pcd_to_pgm/save_map std_srvs/srv/Trigger``
for an on-demand save from another shell.
"""

import argparse
import json
import os
import sys
import time

DEFAULTS = {
    'out': os.path.expanduser('~/sentry_ws/results/odin_map'),
    'cloud_topic': '/livox/lidar',
    'map_frame': 'map',
    'base_frame': 'base_footprint',
    'resolution': 0.025,
    'origin_x': -6.4,
    'origin_y': -4.4,
    'width': 512,
    'height': 352,
    'z_min': 0.10,
    'z_max': 1.20,
    'min_range': 0.35,
    'max_range': 10.0,
    'max_points': 4000,
    'ray_origin': 'sensor',
    'behind_cells': 1,
    'end_margin': 2,
    'despeckle': 1,
    'dump_dir': '',
    'dump_every': 2,
    'ground_z': 0.0,
    'publish_every': 10,
    'duration': 0.0,
    'compare': '',
    'compare_json': '',
    'diff': '',
    'tol': 1,
    'min_component_cells': 25,
    'sweep_rays': 2048,
}


def parse_args(argv):
    parser = argparse.ArgumentParser(
        prog='pcd_to_pgm',
        description='PointCloud2 + TF -> Nav2 map (pgm/yaml) in the map frame.')
    parser.add_argument('--out', default=DEFAULTS['out'],
                        help='output base path (writes .pgm/.yaml/_traj.json/_stats.json)')
    parser.add_argument('--duration', type=float, default=DEFAULTS['duration'],
                        help='mapping seconds; 0 = until Ctrl-C then save')
    parser.add_argument('--cloud-topic', default=DEFAULTS['cloud_topic'])
    parser.add_argument('--map-frame', default=DEFAULTS['map_frame'])
    parser.add_argument('--base-frame', default=DEFAULTS['base_frame'])
    parser.add_argument('--resolution', type=float, default=DEFAULTS['resolution'])
    parser.add_argument('--origin-x', type=float, default=DEFAULTS['origin_x'])
    parser.add_argument('--origin-y', type=float, default=DEFAULTS['origin_y'])
    parser.add_argument('--width', type=int, default=DEFAULTS['width'])
    parser.add_argument('--height', type=int, default=DEFAULTS['height'])
    parser.add_argument('--z-min', type=float, default=DEFAULTS['z_min'],
                        help='keep points at least this high above ground (m)')
    parser.add_argument('--z-max', type=float, default=DEFAULTS['z_max'],
                        help='keep points at most this high above ground (m)')
    parser.add_argument('--min-range', type=float, default=DEFAULTS['min_range'],
                        help='drop returns closer than this (self hits, m)')
    parser.add_argument('--max-range', type=float, default=DEFAULTS['max_range'])
    parser.add_argument('--max-points', type=int, default=DEFAULTS['max_points'],
                        help='stride downsample cap per cloud')
    parser.add_argument('--ground-z', type=float, default=DEFAULTS['ground_z'],
                        help='map-frame z of the floor')
    parser.add_argument('--ray-origin', default=DEFAULTS['ray_origin'],
                        choices=['sensor', 'base'])
    parser.add_argument('--behind-cells', type=int,
                        default=DEFAULTS['behind_cells'],
                        help='extra occupied cells behind each surface (solid '
                             'assumption; 0 = surface only)')
    parser.add_argument('--end-margin', type=int, default=DEFAULTS['end_margin'],
                        help='do not mark the last N cells before a return as '
                             'free (grazing returns would erase the surface)')
    parser.add_argument('--despeckle', type=int, default=DEFAULTS['despeckle'],
                        help='drop occupied cells with fewer than N occupied '
                             'neighbours (1 = isolated dots only, 0 = off)')
    parser.add_argument('--dump-dir', default=DEFAULTS['dump_dir'],
                        help='save the filtered clouds here for offline replay')
    parser.add_argument('--dump-every', type=int,
                        default=DEFAULTS['dump_every'])
    parser.add_argument('--publish-every', type=int,
                        default=DEFAULTS['publish_every'],
                        help='publish ~/map every N clouds')
    parser.add_argument('--compare', default='',
                        help='truth map.yaml; runs the comparison after saving')
    parser.add_argument('--compare-json', default='',
                        help='write the compare report here (default <out>_compare.json)')
    parser.add_argument('--diff', default='', help='write a diff image here')
    parser.add_argument('--tol', type=int, default=DEFAULTS['tol'],
                        help='+-cells tolerance for recall/precision')
    parser.add_argument('--min-component-cells', type=int,
                        default=DEFAULTS['min_component_cells'])
    parser.add_argument('--sweep-rays', type=int, default=DEFAULTS['sweep_rays'],
                        help='rays per pose for the visibility ceiling')
    args, unknown = parser.parse_known_args(argv)
    if unknown:
        # tolerate --ros-args ... passed through `ros2 run`
        print('[pcd_to_pgm] ignoring extra args: %s' % ' '.join(unknown))
    return args


def _overrides(args):
    return {
        'cloud_topic': args.cloud_topic,
        'map_frame': args.map_frame,
        'base_frame': args.base_frame,
        'resolution': args.resolution,
        'origin_x': args.origin_x,
        'origin_y': args.origin_y,
        'width': args.width,
        'height': args.height,
        'z_min': args.z_min,
        'z_max': args.z_max,
        'min_range': args.min_range,
        'max_range': args.max_range,
        'max_points': args.max_points,
        'ground_z': args.ground_z,
        'ray_origin': args.ray_origin,
        'behind_cells': args.behind_cells,
        'end_margin': args.end_margin,
        'despeckle': args.despeckle,
        'dump_dir': args.dump_dir,
        'dump_every': args.dump_every,
        'publish_every': args.publish_every,
        'save_path': args.out,
    }


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        import rclpy
        from .node import PcdToPgmNode
    except ImportError as exc:
        print('[pcd_to_pgm] ROS import failed (%s); source /opt/ros/<distro>/'
              'setup.bash first' % exc)
        return 2

    rclpy.init(args=None)
    node = None
    stats = None
    started = time.time()
    try:
        node = PcdToPgmNode(_overrides(args))
        print('[pcd_to_pgm] mapping for %s s -> %s'
              % (args.duration if args.duration > 0 else 'Ctrl-C', args.out))
        deadline = started + args.duration if args.duration > 0 else None
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if deadline is not None and time.time() >= deadline:
                break
    except KeyboardInterrupt:
        print('[pcd_to_pgm] interrupted, saving what we have')
    finally:
        if node is not None:
            try:
                stats = node.save(args.out)
            except Exception as exc:      # never lose a finished map to IO
                print('[pcd_to_pgm] SAVE FAILED: %s' % exc)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if stats is None:
        return 1
    stats['wall_seconds'] = round(time.time() - started, 2)
    print(json.dumps({k: stats[k] for k in
                      ('scans', 'points', 'rays', 'occ_cells', 'free_cells',
                       'unknown_cells', 'occ_area_m2', 'free_area_m2',
                       'wall_seconds') if k in stats}, indent=2))

    if not args.compare:
        return 0

    from .compare import compare, format_report, write_json
    report = compare(
        args.compare, args.out + '.yaml',
        traj_path=stats.get('traj_path'),
        tol=args.tol,
        min_component_cells=args.min_component_cells,
        max_range=args.max_range,
        n_rays=args.sweep_rays,
        diff_path=args.diff or (args.out + '_diff.png'))
    report['map_stats'] = {k: stats[k] for k in stats
                           if k not in ('traj_path', 'pgm', 'yaml')}
    print(format_report(report))
    json_path = args.compare_json or (args.out + '_compare.json')
    write_json(report, json_path)
    print('report: %s' % json_path)
    if 'diff_image' in report:
        print('diff  : %s' % report['diff_image'])
    return 0