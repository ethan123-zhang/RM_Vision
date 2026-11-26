"""S1a: RM sentry minimum loop (S1b reuses it and adds Nav2 on top).

gz sim (headless) + 4-mecanum-wheel sentry with Mid-360 + /cmd_vel, /odom
bridges + odin1 stand-in + pointcloud_to_laserscan -> /scan.

S1b changes kept here:
  * /gt_odom bridged in, so odin1_sim_bridge can publish map -> odom;
  * pointcloud_to_laserscan targets base_footprint with a 0.05 .. 0.60 m band,
    which drops the ground returns that used to look like a wall ring;
  * no AMCL: localization is the Odin1 stand-in (map -> odom).

S2c: map_offset_x / map_offset_y / map_offset_yaw (default 0) are forwarded to
odin1_sim_bridge to inject an Odin1 map-frame vs field-frame misalignment.
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            TimerAction)
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

WORLD_XACRO = '/mnt/e/RM/tb3_gazebo_nav2/rm_field/worlds/rmul2027_3v3_up_nav2.sdf.xacro'
MODELS_DIR = '/mnt/e/RM/tb3_gazebo_nav2/rm_field/models'
WORLD_SDF = '/tmp/sentry_ws/world.sdf'

BRIDGE_ARGS = [
    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
    '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
    '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
    # S1b ground truth for map -> odom (world frame == map frame).
    '/gt_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
    # S3a low safety lidar (0.18 m high, 0.05..2.5 m) -> safety_scan_relay -> /scan_safety.
    # The gz sensor names its frame "sentry/base_footprint/safety_lidar", which no TF
    # knows, so the raw topic is bridged and the relay rewrites header.frame_id.
    '/scan_safety_raw@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
]

# /livox/imu note (S1b): the livox gz plugin reports linear_acceleration in g
# (~0.999 at rest, not 9.8). It is not connected to Nav2 or to any EKF here.


def _env_with(var, extra):
    old = os.environ.get(var, '')
    return (extra + ':' + old) if old else extra


def _launch_setup(context, *args, **kwargs):
    gui = context.perform_substitution(LaunchConfiguration('gui')).lower() == 'true'
    world_xacro = context.perform_substitution(LaunchConfiguration('world'))
    models_dir = context.perform_substitution(LaunchConfiguration('models'))
    camera = context.perform_substitution(LaunchConfiguration('camera')).lower() == 'true'
    spawn = [context.perform_substitution(LaunchConfiguration(a))
             for a in ('spawn_x', 'spawn_y', 'spawn_z')]
    # S3e: 相机话题只在开相机时桥接（不桥的话 gz 侧多一路大图，DDS 也白占带宽）。
    bridge_args = list(BRIDGE_ARGS)
    if camera:
        bridge_args += [
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ]

    os.makedirs(os.path.dirname(WORLD_SDF), exist_ok=True)
    conv = subprocess.run(['xacro', world_xacro, 'headless:=true'],
                          capture_output=True, text=True)
    if conv.returncode != 0:
        raise RuntimeError('xacro failed on %s:\n%s' % (world_xacro, conv.stderr))
    with open(WORLD_SDF, 'w') as handle:
        handle.write(conv.stdout)

    livox_share = get_package_share_directory('livox_gz_sim')
    livox_prefix = os.path.dirname(livox_share)
    desc_share = get_package_share_directory('sentry_description')
    xacro_path = os.path.join(desc_share, 'urdf', 'sentry.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_path,
                 ' lidar_slices:=', LaunchConfiguration('lidar_slices'),
                 ' lidar_resolution:=', LaunchConfiguration('lidar_resolution'),
                 # S3a: safety lidar on/off, forwarded from run_sentry_nav.sh
                 ' safety_lidar:=', LaunchConfiguration('safety_lidar'),
                 # S3e: 前视相机（装甲板检测用），默认关
                 ' camera:=', LaunchConfiguration('camera')]),
        value_type=str)

    gz_cmd = ['gz', 'sim', '-r'] + ([] if gui else ['-s']) + ['-v', '2', WORLD_SDF]

    return [
        ExecuteProcess(
            cmd=gz_cmd,
            output='screen',
            additional_env={
                'GZ_SIM_RESOURCE_PATH': _env_with('GZ_SIM_RESOURCE_PATH',
                                                       livox_prefix + ':' + models_dir),
                'GZ_SIM_SYSTEM_PLUGIN_PATH': _env_with('GZ_SIM_SYSTEM_PLUGIN_PATH',
                                                       os.path.normpath(os.path.join(livox_prefix, os.pardir, 'lib'))),
            },
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True,
                         'frame_prefix': ''}],
            output='screen',
        ),
        TimerAction(period=3.0, actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-name', 'sentry', '-topic', 'robot_description',
                           '-x', spawn[0], '-y', spawn[1], '-z', spawn[2]],
                output='screen',
            ),
        ]),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=bridge_args,
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        # S3a: gz frame "sentry/base_footprint/safety_lidar" -> base_footprint.
        Node(
            package='sentry_bringup',
            executable='safety_scan_relay.py',
            parameters=[{
                'use_sim_time': True,
                'input_topic': '/scan_safety_raw',
                'output_topic': '/scan_safety',
                'frame_id': 'base_footprint',
            }],
            output='screen',
        ),
        Node(
            package='odin1_sim_bridge',
            executable='odin1_sim_bridge',
            parameters=[{
                'use_sim_time': True,
                'rate': 10.0,
                'noise_xy': 0.0,
                'noise_yaw': 0.0,
                'publish_cloud': False,
                'odom_topic': '/odom',
                'truth_topic': '/gt_odom',
                'publish_odom_tf': True,
                # S2c: injected Odin1 map-frame vs field-frame offset, default 0.
                'map_offset_x': ParameterValue(
                    LaunchConfiguration('map_offset_x'), value_type=float),
                'map_offset_y': ParameterValue(
                    LaunchConfiguration('map_offset_y'), value_type=float),
                'map_offset_yaw': ParameterValue(
                    LaunchConfiguration('map_offset_yaw'), value_type=float),
            }],
            output='screen',
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/livox/lidar'), ('scan', '/scan')],
            parameters=[{
                'use_sim_time': True,
                # base_footprint (not base_link) sits on the floor, so the
                # height band below is "0.05 m .. 0.60 m above the ground".
                'target_frame': 'base_footprint',
                'transform_tolerance': 0.02,
                'min_height': ParameterValue(LaunchConfiguration('p2l_min_height'), value_type=float),
                'max_height': ParameterValue(LaunchConfiguration('p2l_max_height'), value_type=float),
                'angle_min': -3.14159265,
                'angle_max': 3.14159265,
                'angle_increment': 0.00872665,
                'scan_time': 0.1,
                'range_min': 0.10,
                'range_max': 60.0,
                'use_inf': True,
            }],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=WORLD_XACRO),
        DeclareLaunchArgument('models', default_value=MODELS_DIR),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.09'),
        DeclareLaunchArgument('lidar_slices', default_value='1'),
        DeclareLaunchArgument('lidar_resolution', default_value='1.0'),
        # S3a: low safety lidar; false = old behaviour (reproduce the blind spot).
        DeclareLaunchArgument('safety_lidar', default_value='true'),
        # S3e: front camera for armor-plate detection (default off: costs RTF on llvmpipe).
        DeclareLaunchArgument('camera', default_value='false'),
        DeclareLaunchArgument('p2l_min_height', default_value='0.05'),
        DeclareLaunchArgument('p2l_max_height', default_value='0.60'),
        # S2c: Odin1 map -> field offset injected into odin1_sim_bridge.
        DeclareLaunchArgument('map_offset_x', default_value='0.0'),
        DeclareLaunchArgument('map_offset_y', default_value='0.0'),
        DeclareLaunchArgument('map_offset_yaw', default_value='0.0'),
        OpaqueFunction(function=_launch_setup),
    ])