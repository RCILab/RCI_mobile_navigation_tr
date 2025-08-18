# Neobotix GmbH
# Author: Pradheep Padmanabhan
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
    ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable ### <--- 수정: SetEnvironmentVariable 추가
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.event_handlers import OnShutdown, OnProcessExit
from launch_ros.substitutions import FindPackageShare
import os
import xacro


def launch_setup(context: LaunchContext, my_neo_env_arg):
    launch_actions = []

    my_neo_environment = my_neo_env_arg.perform(context)
    use_sim_time = True

    # ---------- Gazebo world ----------------------------------------------
    if my_neo_environment in ("neo_workshop", "neo_track1"):
        world_path = os.path.join(
            get_package_share_directory('tr_sim'),
            'worlds',
            f'{my_neo_environment}.world')
    else:
        world_path = my_neo_environment

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py')),
        launch_arguments={
            'world':   world_path,
            'verbose': 'true',
        }.items())

    # ---------- Robot description / spawn ---------------------------------
    robot_description_xacro = os.path.join(
        get_package_share_directory('tr_description'),
        'urdf',
        'amr_sim.urdf.xacro')

    controller_yaml_path = os.path.join(
        get_package_share_directory('tr_sim'),
        'configs',
        'amr_controller.yaml')

    lidar_macro_path = os.path.join(
        get_package_share_directory('tr_sim'),
        'components', 'common_macro',
        'gazebo_lidar_macro.xacro')

    robot_description = xacro.process_file(
        robot_description_xacro,
        mappings={'use_gazebo': 'true', 'controller_yaml_path': controller_yaml_path, 'lidar_macro_path': lidar_macro_path}).toxml()

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'amr', '-topic', '/robot_description'],
        output='screen')

    state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description}],
        output='screen')

    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        prefix='xterm -e',
        name='teleop',
        output='screen')

    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen')

    # Delay start of joint_trajectory_controller until joint_state_broadcaster is running
    delay_jtc_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_jsb,
            on_exit=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_trajectory_controller', '-c', '/controller_manager'],
                    output='screen',
                )
            ]
        )
    )

    # ---------- Kinematics ------------------------------------------------
    kinematics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('tr_kinematics_differential2'),
                'launch',
                'kinematics.launch.py')),
        launch_arguments={
            'use_sim': 'true'
        }.items())

    # ---------- Append all actions ----------------------------------------
    launch_actions += [
        state_pub,
        gazebo,
        spawn_entity,
        teleop,
        spawn_jsb,
        delay_jtc_spawner,
        kinematics
    ]
    return launch_actions


def generate_launch_description():
    ### --- 시작: GAZEBO_MODEL_PATH 설정 추가 --- ###
    # 로봇 모델 파일(.stl, .dae)이 있는 패키지의 경로를 찾습니다.
    pkg_tr_description = get_package_share_directory('tr_description')

    # 기존 환경 변수에 새 경로를 추가합니다.
    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path =  os.environ['GAZEBO_MODEL_PATH'] + ':' + os.path.join(pkg_tr_description, 'meshes')
    else:
        model_path =  os.path.join(pkg_tr_description, 'meshes')

    # 환경 변수를 설정하는 launch 액션을 생성합니다.
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=model_path
    )
    ### --- 종료: GAZEBO_MODEL_PATH 설정 추가 --- ###

    ld = LaunchDescription()

    # LaunchDescription에 가장 먼저 환경 변수 설정을 추가합니다.
    ld.add_action(set_gazebo_model_path)

    ld.add_action(DeclareLaunchArgument(
        'world',
        default_value='neo_workshop',
        description='Available worlds: "neo_track1", "neo_workshop"'))

    ld.add_action(OpaqueFunction(
        function=launch_setup,
        args=[LaunchConfiguration('world')]))

    return ld