# Neobotix GmbH
# Author: Pradheep Padmanabhan
# Modified for use_sim_time fix

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
    RegisterEventHandler
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
import os
import xacro

def launch_setup(context: LaunchContext, world_name_arg, use_sim_time_arg):
    """
    This function is executed by OpaqueFunction and sets up all nodes and actions.
    """
    # Get the actual values from the launch arguments
    world_name = world_name_arg.perform(context)

    # Package paths
    tr_sim_share = get_package_share_directory('tr_sim')
    tr_description_share = get_package_share_directory('tr_description')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    ekf_config_path = os.path.join(get_package_share_directory('tr_sim'), 'configs', 'robot_localization_ekf.yaml')

    # ---------- Gazebo world ----------------------------------------------
    if world_name in ("test", "warehouse"):
        world_path = os.path.join(tr_sim_share, 'worlds', f'{world_name}.world')
    else:
        world_path = world_name

    # Launch Gazebo server and client
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'verbose': 'true',
            'use_sim_time': use_sim_time_arg
        }.items()
    )

    robot_description_xacro = os.path.join(tr_description_share, 'urdf', 'amr_sim.urdf.xacro')
    controller_yaml_path = os.path.join(tr_sim_share, 'configs', 'amr_controller.yaml')
    lidar_macro_path = os.path.join(tr_sim_share, 'components', 'common_macro', 'gazebo_lidar_macro.xacro')

    robot_description_content = xacro.process_file(
        robot_description_xacro,
        mappings={'use_gazebo': 'true', 'controller_yaml_path': controller_yaml_path, 'lidar_macro_path': lidar_macro_path}
    ).toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time_arg,
            'robot_description': robot_description_content
        }]
    )

    # Spawn Entity Node
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'amr', '-topic', '/robot_description'],
        output='screen'
    )

    kinematics_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tr_kinematics_differential2'), 'launch', 'kinematics.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time_arg
        }.items()
    )

    launch_twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tr_twist_mux'), 'launch', 'twist_mux_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time_arg
        }.items()
    )

    node_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_localization',
        parameters=[ekf_config_path, {'use_sim_time': use_sim_time_arg}],
        remappings=[('odometry/filtered', 'odom')]
    )

    launch_laser_integrator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('laser_scan_integrator'), 'launch', 'integrate_2_scan.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time_arg
        }.items()
    )

    spawn_jsb_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen'
    )

    delay_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_jsb_node,
            on_exit=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['velocity_controller', '-c', '/controller_manager'],
                    output='screen',
                )
            ]
        )
    )

    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        prefix='xterm -e',
        name='teleop',
        output='screen',
        remappings=[('cmd_vel', 'cmd_vel_key')]
    )

    return [
        gazebo,
        robot_state_publisher_node,
        spawn_entity_node,
        kinematics_launch,
        teleop_node,
        spawn_jsb_node,
        delay_controller_spawner,
        node_ekf,
        launch_twist_mux,
        launch_laser_integrator
    ]


def generate_launch_description():
    """
    Main launch function.
    """
    return LaunchDescription([
        # Launch argument for selecting the world
        DeclareLaunchArgument(
            'world',
            default_value='test',
            description='Available worlds: "test", "warehouse" or a full path to a world file'
        ),

        # Launch argument for using simulation time
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        # OpaqueFunction to access launch arguments
        OpaqueFunction(
            function=launch_setup,
            args=[
                LaunchConfiguration('world'),
                LaunchConfiguration('use_sim_time')
            ]
        )
    ])