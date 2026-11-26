"""S2a/S2c: sentry simulation + Nav2, no AMCL, trimmed to the nodes in use.

Same stack as S1b (sentry_sim + map_server + Nav2, localization from
odin1_sim_bridge / odom), except that nav2_bringup/navigation_launch.py is
replaced by an explicit node list. Dropped (never used by NavigateToPose):
  * route_server      (nav2_route, ComputeAndTrackRoute)
  * docking_server    (opennav_docking, DockRobot/UndockRobot)
  * waypoint_follower (FollowWaypoints; NavigateThroughPoses lives in
                       bt_navigator, not here)
Params, remaps and lifecycle order are otherwise identical to nav2_bringup
(Jazzy). Start through run_sentry_nav.sh: it cleans the ROS domain first and
waits until /navigate_to_pose is ready.

S2c: map_offset_x / map_offset_y / map_offset_yaw are forwarded to
sentry_sim.launch.py (and from there to odin1_sim_bridge) so an Odin1 map-frame
misalignment can be injected, e.g.
    ./run_sentry_nav.sh map_offset_x:=0.30 map_offset_y:=-0.20 map_offset_yaw:=0.05236

S3: map_click:=true adds the "click the map, robot goes there" window
(tools/map_click.py). It retries until /navigate_to_pose shows up, so it may be
started at the same time as the rest of the stack.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MAP_YAML = '/mnt/e/RM/tb3_gazebo_nav2/rm_field/maps/rmul2027_3v3_up.yaml'
# S3: 点地图窗口的脚本（仓库是 source of truth；WSL 部署副本只是 rsync 的结果）
MAP_CLICK_PY = '/mnt/e/RM/tb3_gazebo_nav2/sentry_sim/tools/map_click.py'

REMAPPINGS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
CMD_VEL_NAV = [('cmd_vel', 'cmd_vel_nav')]

# same order as nav2_bringup, minus route_server / waypoint_follower / docking_server
NAV_NODES = ['controller_server', 'smoother_server', 'planner_server',
             'behavior_server', 'velocity_smoother', 'collision_monitor',
             'bt_navigator']


def generate_launch_description():
    bringup = get_package_share_directory('sentry_bringup')
    params_file = os.path.join(bringup, 'config', 'sentry_nav2.yaml')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, 'launch', 'sentry_sim.launch.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            # S3a: low safety lidar switch, forwarded to sentry_sim.launch.py
            'safety_lidar': LaunchConfiguration('safety_lidar'),
            # S3e: front camera switch (armor-plate detection), forwarded to sentry_sim.launch.py
            'camera': LaunchConfiguration('camera'),
            'map_offset_x': LaunchConfiguration('map_offset_x'),
            'map_offset_y': LaunchConfiguration('map_offset_y'),
            'map_offset_yaw': LaunchConfiguration('map_offset_yaw'),
        }.items())

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': LaunchConfiguration('map'),
            'topic_name': 'map',
            'frame_id': 'map',
        }])

    map_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server'],
        }])

    def nav_node(package, executable, name, extra_remaps=(), extra_params=None):
        params = [params_file, {'use_sim_time': True}]
        if extra_params:
            params.append(extra_params)
        return Node(
            package=package,
            executable=executable,
            name=name,
            output='screen',
            parameters=params,
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=REMAPPINGS + list(extra_remaps),
        )

    # S3c: 5 Hz 重规划的 BT（默认 XML 是 1 Hz）
    bt_xml = os.path.join(bringup, 'bt', 'sentry_navigate_to_pose.xml')
    nav = [
        nav_node('nav2_controller', 'controller_server', 'controller_server', CMD_VEL_NAV),
        nav_node('nav2_smoother', 'smoother_server', 'smoother_server'),
        nav_node('nav2_planner', 'planner_server', 'planner_server'),
        nav_node('nav2_behaviors', 'behavior_server', 'behavior_server', CMD_VEL_NAV),
        nav_node('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother', CMD_VEL_NAV),
        nav_node('nav2_collision_monitor', 'collision_monitor', 'collision_monitor'),
        nav_node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator',
                 extra_params={'default_nav_to_pose_bt_xml': bt_xml}),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'],
            parameters=[{'autostart': True, 'node_names': NAV_NODES,
                         'use_sim_time': True}]),
    ]

    # S3: 地图点选窗口（可选）。栈还没就绪时它自己在状态栏里等 action server，不挡其他节点。
    map_click = ExecuteProcess(
        cmd=['python3', LaunchConfiguration('map_click_script'),
             '--map', LaunchConfiguration('map'),
             '--ppm', LaunchConfiguration('map_click_ppm')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('map_click')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('map', default_value=MAP_YAML),
        # S3: 点地图窗口
        DeclareLaunchArgument('map_click', default_value='false'),
        DeclareLaunchArgument('map_click_script', default_value=MAP_CLICK_PY),
        DeclareLaunchArgument('map_click_ppm', default_value='70'),
        # S2c: Odin1 map -> field offset injected into odin1_sim_bridge.
        DeclareLaunchArgument('map_offset_x', default_value='0.0'),
        DeclareLaunchArgument('map_offset_y', default_value='0.0'),
        DeclareLaunchArgument('map_offset_yaw', default_value='0.0'),
        # S3a: low safety lidar; safety_lidar:=false reproduces the old blind spot.
        DeclareLaunchArgument('safety_lidar', default_value='true'),
        # S3e: front camera (armor-plate detection); off by default.
        DeclareLaunchArgument('camera', default_value='false'),
        sim,
        map_click,
        # map_server only needs the yaml; give gz a few seconds to settle so the
        # first costmap activation already has map -> odom from the odin bridge.
        TimerAction(period=4.0, actions=[map_server, map_lifecycle]),
        TimerAction(period=12.0, actions=nav),
    ])