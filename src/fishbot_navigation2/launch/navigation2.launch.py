import os
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取默认路径
    fishbot_navigation2_dir = get_package_share_directory('fishbot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    # 创建 Launch 配置
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='true')
    map_yaml_path = launch.substitutions.LaunchConfiguration('map', default=os.path.join(fishbot_navigation2_dir, 'maps', 'room.yaml'))
    nav2_param_path = launch.substitutions.LaunchConfiguration('params_file', default=os.path.join(fishbot_navigation2_dir, 'config', 'nav2_params.yaml'))

    # 定义 Nav2 的启动组（包含重映射逻辑）
    # 使用 GroupAction 包裹，可以在内部统一做话题重映射
    nav2_group = GroupAction([
        # 🟢 核心修正：在这里进行话题重映射
        SetRemap(src='/cmd_vel', dst='/command/cmd_twist'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        )
    ])

    return LaunchDescription([
        # 声明 Launch 参数
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                             description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path,
                                             description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                                             description='Full path to param file to load'),

        # 加载 Nav2 组（包含重映射）
        nav2_group,

        # 启动 Rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])