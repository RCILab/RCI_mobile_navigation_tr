from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share_dir = get_package_share_directory('tr_driver')

    # Declare launch arguments
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN interface to use (e.g., can0)'
    )

    dcf_path_arg = DeclareLaunchArgument(
        'dcf_path',
        default_value=PathJoinSubstitution([
            pkg_share_dir,
            'config',
            'master.dcf'
        ]),
        description='Absolute path to the master.dcf configuration file'
    )

    motor_freq_arg = DeclareLaunchArgument(
        'motor_freq',
        default_value='100.0',
        description='Motor control frequency in Hz'
    )

    num_driver_arg = DeclareLaunchArgument(
        'num_driver',
        default_value='1',
        description='Number of motor drivers'
    )

    # Node for driving_motor_node
    driving_motor_node = Node(
        package='tr_driver',
        executable='driving_motor_node',
        name='driving_motor_node',
        output='screen',
        parameters=[{
            'can_interface': LaunchConfiguration('can_interface'),
            'dcf_path': LaunchConfiguration('dcf_path'),
            'motor_freq': LaunchConfiguration('motor_freq'),
            'num_driver': LaunchConfiguration('num_driver')
        }]
    )

    return LaunchDescription([
        can_interface_arg,
        dcf_path_arg,
        motor_freq_arg,
        num_driver_arg,
        driving_motor_node
    ])
