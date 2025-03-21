import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    #We have no robot logic, so just run Zenoh bridge to UI
    zenoh_publisher = os.path.join(get_package_share_directory('vera_core'), 'launch', 'zenoh.launch.py')
    include_other_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zenoh_publisher)
    )
    return LaunchDescription([
        include_other_launch
    ])
