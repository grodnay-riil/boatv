import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the sub_pub package
    sub_pub_dir = get_package_share_directory('vera_ui')

    # Corrected path to zenoh.config inside the sub_pub package
    zenoh_config_path = os.path.join(sub_pub_dir, 'config', 'zenoh.config.json5')

    # Get the IP address of the Vera host
    vera_host_ip = os.environ['VERA_HOST_IP']
    end_point_url = f"udp/{vera_host_ip}:7447"

    return LaunchDescription([
        # Launch Zenoh bridge as an external process
        ExecuteProcess(
            cmd=[
                'zenoh-bridge-ros2dds',
                '-c', zenoh_config_path,
                '-l', end_point_url
            ],
            name='zenoh_bridge',
            output='screen'
        )
    ])
