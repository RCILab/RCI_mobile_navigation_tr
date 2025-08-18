from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="sick_safetyscanners2",
            executable="sick_safetyscanners2_node",
            name="sick_safetyscanners2_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"frame_id": "scan1",
                 "sensor_ip": "192.168.192.101",
                 "host_ip": "192.168.192.10",
                 "interface_ip": "0.0.0.0",
                 "host_udp_port": 6061,
                 "channel": 0,
                 "channel_enabled": True,
                 "skip": 0,
                 "angle_start": 0.0,
                 "angle_end": 0.0,
                 "time_offset": 0.0,
                 "general_system_state": True,
                 "derived_settings": True,
                 "measurement_data": True,
                 "intrusion_data": True,
                 "application_io_data": True,
                 "use_persistent_config": False,
                 "min_intensities": 0.0}
            ],
            remappings=[('/scan', '/scan1')]
        ),
        Node(
            package="sick_safetyscanners2",
            executable="sick_safetyscanners2_node",
            name="sick_safetyscanners2_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"frame_id": "scan2",
                 "sensor_ip": "192.168.192.100",
                 "host_ip": "192.168.192.10",
                 "interface_ip": "0.0.0.0",
                 "host_udp_port": 6060,
                 "channel": 0,
                 "channel_enabled": True,
                 "skip": 0,
                 "angle_start": 0.0,
                 "angle_end": 0.0,
                 "time_offset": 0.0,
                 "general_system_state": True,
                 "derived_settings": True,
                 "measurement_data": True,
                 "intrusion_data": True,
                 "application_io_data": True,
                 "use_persistent_config": False,
                 "min_intensities": 0.0}
            ],
            remappings=[('/scan', '/scan2')]
        )
    ])