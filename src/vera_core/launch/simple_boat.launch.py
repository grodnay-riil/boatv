import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    zenoh_publisher_path = os.path.join(get_package_share_directory('vera_core'), 'launch', 'zenoh.launch.py')
    zenoh_publisher_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(zenoh_publisher_path))
    rosbridge_path = os.path.join(get_package_share_directory('vera_tools'),'launch','rosbridge_websocket.launch.py')
    ros_bridge_launch=IncludeLaunchDescription(PythonLaunchDescriptionSource(rosbridge_path),launch_arguments={'port': '9092'}.items())
    return LaunchDescription([
        zenoh_publisher_launch,
        ros_bridge_launch,
        Node(package='rosboard',executable='rosboard_node', name='rosboard', output='screen', parameters=[{'port': 8882}] ),      
    ])
