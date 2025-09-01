import launch
import launch.actions
import launch.substitutions
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions

def generate_launch_description():
    pkg_share = get_package_share_directory('tr_kinematics_differential2')

    use_sim_time_arg = launch.actions.DeclareLaunchArgument('use_sim_time', default_value='false')

    param_file = launch.substitutions.PythonExpression([
        '"', os.path.join(pkg_share, 'config', 'kinematics_sim.yaml'), '" if "',
        launch.substitutions.LaunchConfiguration('use_sim_time'), '" == "true" else "',
        os.path.join(pkg_share, 'config', 'kinematics.yaml'), '"'
    ])

    node_action = launch_ros.actions.Node(
        package='tr_kinematics_differential2',
        executable='tr_differential_node',
        output='screen',
        name='tr_differential_node',
        parameters=[
            param_file,
            {'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')}
        ]
    )

    return launch.LaunchDescription([
        use_sim_time_arg,
        node_action
    ])