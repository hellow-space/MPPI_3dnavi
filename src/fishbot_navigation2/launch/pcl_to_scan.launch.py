from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/points'),  # 确保这里和你的 Gazebo 输出对齐
                ('scan', '/scan')
            ],
            parameters=[{
                'use_sim_time': True,           # 【绝对核心】直接写在字典里，强行同步 Gazebo 时间！
                'target_frame': 'radar_link',   # 必须写雷达的 TF 名字，否则 AMCL 找不到它
                'transform_tolerance': 0.1,     # 容忍 0.1 秒的仿真延迟
                'min_height': -0.1,             # 往下看 0.5 米
                'max_height': 0.1,              # 往上看 0.5 米
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0087,
                'scan_time': 0.1,
                'range_min': 0.8,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }]
        )
    ])