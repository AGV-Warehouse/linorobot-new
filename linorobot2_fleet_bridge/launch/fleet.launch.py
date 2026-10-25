"""Standalone fleet launch. Requires an explicit real map; no automatic motion."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def start(context):
    package = Path(get_package_share_directory('linorobot2_fleet_bridge'))
    nav = Path(get_package_share_directory('nav2_bringup'))
    base = Path(get_package_share_directory('linorobot2_bringup'))
    map_path = Path(LaunchConfiguration('map').perform(context)).expanduser()
    if not map_path.is_file():
        raise RuntimeError('map:= must point to a real map-server YAML file')
    params = RewrittenYaml(
        source_file=str(package / 'config/fleet_navigation.yaml'),
        param_rewrites={'default_nav_to_pose_bt_xml': str(package / 'config/fleet_navigate.xml')},
        convert_types=True)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(base / 'launch/robot.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_hardware')),
            launch_arguments={'slam': 'false', 'map_viewer': 'false'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav / 'launch/localization_launch.py')),
            launch_arguments={'map': str(map_path), 'params_file': params,
                              'use_composition': 'false', 'use_sim_time': LaunchConfiguration('sim')}.items()),
        # Explicit nodes/remaps: do not run navigation_launch's direct cmd_vel_nav
        # connection between controller and smoother, or the authorization gate is bypassed.
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             parameters=[params, {'use_sim_time': LaunchConfiguration('sim')}], output='screen'),
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             parameters=[params, {'use_sim_time': LaunchConfiguration('sim')}], output='screen',
             remappings=[('cmd_vel', 'fleet/cmd_vel_requested')]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
             parameters=[params, {'use_sim_time': LaunchConfiguration('sim')}], output='screen',
             remappings=[('cmd_vel', 'fleet/cmd_vel_authorized'), ('cmd_vel_smoothed', 'cmd_vel_smoothed')]),
        Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor',
             parameters=[params, {'use_sim_time': LaunchConfiguration('sim')}], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_fleet', output='screen',
             parameters=[{'autostart': True, 'use_sim_time': LaunchConfiguration('sim'),
                          'node_names': ['planner_server', 'controller_server',
                                         'velocity_smoother', 'collision_monitor']}]),
        Node(package='linorobot2_fleet_bridge', executable='fleet_bridge', output='screen',
             parameters=[LaunchConfiguration('bridge_config'),
                         {'use_sim_time': LaunchConfiguration('sim')}]),
    ]


def generate_launch_description():
    package = Path(get_package_share_directory('linorobot2_fleet_bridge'))
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path to commissioned map YAML'),
        DeclareLaunchArgument('bridge_config', default_value=str(package / 'config/bridge.yaml')),
        DeclareLaunchArgument('start_hardware', default_value='false'),
        DeclareLaunchArgument('sim', default_value='false'),
        OpaqueFunction(function=start),
    ])
