from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    declared_args = [
        DeclareLaunchArgument('port', default_value='9090'),
        DeclareLaunchArgument('address', default_value=''),
        DeclareLaunchArgument('ssl', default_value='false'),
        DeclareLaunchArgument('certfile', default_value=''),
        DeclareLaunchArgument('keyfile', default_value=''),
        DeclareLaunchArgument('retry_startup_delay', default_value='5.0'),
        DeclareLaunchArgument('fragment_timeout', default_value='600'),
        DeclareLaunchArgument('delay_between_messages', default_value='0'),
        DeclareLaunchArgument('max_message_size', default_value='10000000'),
        DeclareLaunchArgument('unregister_timeout', default_value='10.0'),
        DeclareLaunchArgument('use_compression', default_value='false'),
        DeclareLaunchArgument('call_services_in_new_thread', default_value='false'),
        DeclareLaunchArgument('send_action_goals_in_new_thread', default_value='false'),
        DeclareLaunchArgument('topics_glob', default_value=''),
        DeclareLaunchArgument('services_glob', default_value=''),
        DeclareLaunchArgument('params_glob', default_value=''),
        DeclareLaunchArgument('bson_only_mode', default_value='false'),
        DeclareLaunchArgument('binary_encoder', default_value='default')  # only used in XML if bson_only_mode is false
    ]

    # Common parameters
    common_params = {
        'port': LaunchConfiguration('port'),
        'address': LaunchConfiguration('address'),
        'retry_startup_delay': LaunchConfiguration('retry_startup_delay'),
        'fragment_timeout': LaunchConfiguration('fragment_timeout'),
        'delay_between_messages': LaunchConfiguration('delay_between_messages'),
        'max_message_size': LaunchConfiguration('max_message_size'),
        'unregister_timeout': LaunchConfiguration('unregister_timeout'),
        'use_compression': LaunchConfiguration('use_compression'),
        'call_services_in_new_thread': LaunchConfiguration('call_services_in_new_thread'),
        'send_action_goals_in_new_thread': LaunchConfiguration('send_action_goals_in_new_thread'),
        'topics_glob': LaunchConfiguration('topics_glob'),
        'services_glob': LaunchConfiguration('services_glob'),
        'params_glob': LaunchConfiguration('params_glob'),
        'bson_only_mode': LaunchConfiguration('bson_only_mode'),
    }

    # rosbridge_websocket (SSL group)
    ssl_group = GroupAction(
        actions=[
            Node(
                package='rosbridge_server',
                executable='rosbridge_websocket',
                name='rosbridge_websocket',
                output='screen',
                parameters=[
                    {
                        **common_params,
                        'certfile': LaunchConfiguration('certfile'),
                        'keyfile': LaunchConfiguration('keyfile'),
                    }
                ]
            )
        ],
        condition=IfCondition(LaunchConfiguration('ssl'))
    )

    # rosbridge_websocket (non-SSL group)
    non_ssl_group = GroupAction(
        actions=[
            Node(
                package='rosbridge_server',
                executable='rosbridge_websocket',
                name='rosbridge_websocket',
                output='screen',
                parameters=[common_params]
            )
        ],
        condition=UnlessCondition(LaunchConfiguration('ssl'))
    )

    # rosapi node
    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        output='screen',
        parameters=[
            {
                'topics_glob': LaunchConfiguration('topics_glob'),
                'services_glob': LaunchConfiguration('services_glob'),
                'params_glob': LaunchConfiguration('params_glob'),
            }
        ]
    )

    return LaunchDescription(declared_args + [ssl_group, non_ssl_group, rosapi_node])
