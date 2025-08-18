# Neobotix GmbH
# Author: Pradheep Padmanabhan

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    declare_map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='corridor',
        description='Name of the map file in tr_real/maps directory (without .yaml extension)'
    )

    map_name = LaunchConfiguration('map_name')

    map_dir = [
        os.path.join(
            get_package_share_directory('tr_real'),
            'maps',
            ''
        ),
        map_name,
        '.yaml'
    ]

    use_multi_robots = LaunchConfiguration('use_multi_robots', default='False')
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    namespace = LaunchConfiguration('namespace', default='')
    use_namespace = LaunchConfiguration('use_namespace', default='False')
    use_rviz = LaunchConfiguration('use_rviz', default='True')

    param_file_name = 'tr_navigation.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('tr_real'),
            'configs/',
            param_file_name))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('tr_nav2_bringup'), 'launch')
    merge_launch_file_dir = os.path.join(get_package_share_directory('laser_scan_integrator'), 'launch')

    return LaunchDescription([
        # 위에서 선언한 'map_name' 인자를 런치 설명에 추가합니다.
        declare_map_name_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/localization_amcl.launch.py']),
            launch_arguments={
                'map': map_dir, # 동적으로 생성된 맵 경로를 전달합니다.
                'use_sim_time': use_sim_time,
                'use_multi_robots': use_multi_robots,
                'params_file': param_dir,
                'namespace': namespace}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation_tr.launch.py']),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'params_file': param_dir,
                              'use_rviz': use_rviz}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([merge_launch_file_dir, '/integrate_2_scan.launch.py']),
        ),
    ])