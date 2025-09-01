import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    tr_real_dir = get_package_share_directory('tr_real')
    tr_description_dir = get_package_share_directory('tr_description')
    tr_driver_dir = get_package_share_directory('tr_driver')
    tr_kinematics_dir = get_package_share_directory('tr_kinematics_differential2')
    tr_mux_dir = get_package_share_directory('tr_twist_mux')
    sick_safetyscanners_dir = get_package_share_directory('sick_safetyscanners2')
    teleop_twist_joy_dir = get_package_share_directory('teleop_twist_joy')
    ekf_config_path = os.path.join(tr_real_dir, 'configs', 'robot_localization_ekf.yaml')
    integrate_laser_launch_file_dir = os.path.join(get_package_share_directory('laser_scan_integrator'), 'launch')

    launch_tr_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tr_description_dir, 'launch', 'tr_description.launch.py')
        )
    )

    launch_tr_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tr_driver_dir, 'launch', 'driving_motor_launch.py')
        )
    )

    launch_tr_kinematics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tr_kinematics_dir, 'launch', 'kinematics.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false'
        }.items()
    )

    launch_sick_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sick_safetyscanners_dir, 'launch', 'sick_safetyscanners2_launch.py')
        )
    )

    launch_teleop_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(teleop_twist_joy_dir, 'launch', 'teleop-launch.py')
        ),
        launch_arguments={
            'joy_config': 'ps5',
            'joy_vel': 'cmd_vel_joy'
            }.items()
    )

    launch_twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tr_mux_dir, 'launch', 'twist_mux_launch.py')
        )
    )

    launch_integrate_scan = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([integrate_laser_launch_file_dir, '/integrate_2_scan.launch.py']),
        )

    node_imu = Node(package='tr_driver_imu', executable="imu_node")
    node_pgv = Node(package='tr_driver_pgv', executable="pgv_node")

    node_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_localization',
        parameters=[ekf_config_path],
        remappings=[('odometry/filtered', 'odom')]
    )

    ld = LaunchDescription()

    ld.add_action(launch_tr_driver)
    ld.add_action(launch_tr_kinematics)
    ld.add_action(node_imu)
    ld.add_action(node_pgv)
    ld.add_action(launch_sick_lidar)
    ld.add_action(node_ekf)
    ld.add_action(launch_tr_description)
    ld.add_action(launch_teleop_joy)
    ld.add_action(launch_twist_mux)
    ld.add_action(launch_integrate_scan)

    return ld