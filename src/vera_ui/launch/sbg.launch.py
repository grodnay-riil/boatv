import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    subscriber_launch = os.path.join(get_package_share_directory('vera_ui'), 'launch', 'subscriber.launch.py')
    rviz_config = os.path.join(get_package_share_directory('vera_ui'), 'config', 'sbg.rviz')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(subscriber_launch)),
        Node(
            package='vera_tools',
            executable='sbg_tf_publisher',  # must match your entry point in setup.py
            name='sbg_tf_publisher_node',
            output='screen'
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config],
            output='screen'
        )
    ])
