# Neobotix GmbH
# Author: Pradheep Padmanabhan

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    declare_use_speed_zone_arg = DeclareLaunchArgument(
        'use_speed_zone',
        default_value='True',
        description='Set to "False" to disable the speed limit zones.'
    )
    declare_map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='fac_final',
        description='Name of the map file in tr_real/maps directory (without .yaml extension)'
    )

    use_speed_zone = LaunchConfiguration('use_speed_zone')
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
    real_launch_file_dir = os.path.join(get_package_share_directory('tr_real'), 'launch')


    return LaunchDescription([
        declare_use_speed_zone_arg,
        declare_map_name_arg,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/localization_amcl.launch.py']),
            launch_arguments={
                'map': map_dir,
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
            PythonLaunchDescriptionSource([real_launch_file_dir, '/speed_limit.launch.py']),
            condition=IfCondition(use_speed_zone),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time}.items()),
    ])