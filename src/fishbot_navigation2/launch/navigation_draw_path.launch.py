"""Independent drawn-path profile with the project's existing Nav2 integration."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    navigation_dir = get_package_share_directory('fishbot_navigation2')
    custom_path_dir = get_package_share_directory('custom_path_navigation')
    nav2_dir = get_package_share_directory('nav2_bringup')
    with open(os.path.join(navigation_dir, 'urdf', 'navigation_footprint.urdf'),
              encoding='utf-8') as robot_urdf:
        robot_description = robot_urdf.read()

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    custom_path_params_file = LaunchConfiguration('custom_path_params_file')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use the simulation clock'),
        DeclareLaunchArgument('map',
                              default_value=os.path.join(navigation_dir, 'maps', 'pcb_map222.yaml'),
                              description='Full path to the map YAML'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            navigation_dir, 'config', 'nav2_params_height_obstacle.yaml'),
            description='Full path to the Nav2 parameter YAML'),
        DeclareLaunchArgument('custom_path_params_file', default_value=os.path.join(
            custom_path_dir, 'config', 'custom_path.yaml'),
            description='Full path to the custom path executor parameter YAML'),
        GroupAction([
            SetRemap(src='/cmd_vel', dst='/command/cmd_twist'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'bringup_launch.py')),
                launch_arguments={
                    'map': map_file,
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                }.items()),
        ]),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='navigation_footprint_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
            remappings=[('robot_description', '/navigation_footprint/robot_description')],
            output='screen'),
        Node(
            package='custom_path_navigation', executable='custom_path_executor',
            name='custom_path_executor',
            parameters=[custom_path_params_file, {'use_sim_time': use_sim_time}],
            output='screen'),
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', os.path.join(navigation_dir, 'rviz', 'navigation_draw_path.rviz')],
            parameters=[{'use_sim_time': use_sim_time}], output='screen'),
    ])
