from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            emulate_tty=True,
            remappings=[
                ('cloud_in', '/cloud_registered'),
                ('scan', '/scan')
            ],
            parameters=[{
                'use_sim_time': False,
                'target_frame': 'line_frame',
                'transform_tolerance': 5.0,
                'min_height': -0.3,
                'max_height': 1.0,
                'angle_min': -3.14159,
                'angle_max':  3.14159,
                'angle_increment': 0.0087,
                'scan_time': 0.1,
                'range_min': 0.15,
                'range_max': 5.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }]
        )
    ])

