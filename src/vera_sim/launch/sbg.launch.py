import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vera_sim',  # Replace with your package name
            executable='simple_boat_sim',  # Your node's executable name
            name='sbg_simulator',
            output='screen',
            parameters=[{
                # Add parameters here if needed
            }]
        )
    ])
