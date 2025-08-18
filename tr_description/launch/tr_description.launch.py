from asyncio.proactor_events import _ProactorSocketTransport
import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from glob import glob

def generate_launch_description():

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'amr.urdf'
    packageName = 'tr_description'
    urdfDirectory = os.path.join(get_package_share_path(packageName), 'urdf')
    urdfPath = os.path.join(urdfDirectory, urdf_file_name)

    with open(urdfPath, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            #conditions='True',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            #arguments=[urdfPath],
            remappings=remappings,
            arguments=[urdfPath]
            )
    ])