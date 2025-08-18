import launch
import launch.actions
import launch.substitutions
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions

def generate_launch_description():
    # 1. 패키지의 공유 디렉터리 경로를 가져옵니다.
    pkg_share = get_package_share_directory('tr_kinematics_differential2')

    # 2. 'use_sim' 런치 인자를 선언합니다. 기본값은 'false'입니다.
    use_sim_arg = launch.actions.DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Set to "true" for simulation, "false" for real robot.'
    )

    # 3. 'use_sim' 값에 따라 다른 파라미터 파일을 조건부로 선택합니다.
    #    - true: kinematics_sim.yaml
    #    - false: kinematics.yaml
    param_file = launch.substitutions.PythonExpression([
        '"', os.path.join(pkg_share, 'launch', 'kinematics_sim.yaml'), '" if "',
        launch.substitutions.LaunchConfiguration('use_sim'), '" == "true" else "',
        os.path.join(pkg_share, 'launch', 'kinematics.yaml'), '"'
    ])

    # 4. 노드를 정의하고, 위에서 선택된 파라미터 파일을 전달합니다.
    node_action = launch_ros.actions.Node(
        package='tr_kinematics_differential2',
        executable='tr_differential_node',
        output='screen',
        name='tr_differential_node',
        parameters=[param_file]
    )

    # 5. 선언한 런치 인자와 노드 액션을 LaunchDescription에 담아 반환합니다.
    return launch.LaunchDescription([
        use_sim_arg,
        node_action
    ])