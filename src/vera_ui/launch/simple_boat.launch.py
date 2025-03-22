import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_dir = get_package_share_directory('vera_ui')
    subscriber_launch = os.path.join(package_dir, 'launch', 'zenoh.launch.py')
    rviz_config = os.path.join(package_dir, 'config', 'simple_boat.rviz')
    rqt_perspective = os.path.join(package_dir, 'config', 'simple_boat.perspective')
    rosbridge_launch = os.path.join(get_package_share_directory('vera_tools'),'launch','rosbridge_websocket.launch.py')

    urdf_file = os.path.join(
        get_package_share_directory('vera_description'),
        'urdf',
        'bullshark.urdf'
    )
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
        

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(subscriber_launch)),
        Node(package='vera_tools',executable='tf_publisher', name='tf_publisher_node',output='screen'),
        ExecuteProcess(cmd=['rviz2', '-d', rviz_config],output='screen'),
        ExecuteProcess(cmd=['rqt', '--perspective-file', rqt_perspective],output='screen'),
        # ExecuteProcess(cmd=['ros2', 'topic', 'pub', '/listener', 'std_msgs/msg/String','{data: "Hello Michal!"}'],output='screen'),
        Node(package='rosboard',executable='rosboard_node', name='rosboard', output='screen', parameters=[{'port': 8881}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(rosbridge_launch),launch_arguments={'port': '9091'}.items()),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(package='foxglove_bridge',executable='foxglove_bridge',output='screen' )
    ])
