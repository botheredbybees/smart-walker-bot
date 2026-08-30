from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    http_port_arg = DeclareLaunchArgument(
        'http_port',
        default_value='8080',
        description='Port for the dashboard HTTP server.',
    )

    dashboard_app_node = Node(
        package='walker_companion_app',
        executable='dashboard_app_node',
        name='walker_companion_app',
        output='screen',
        parameters=[{
            'http_port': LaunchConfiguration('http_port'),
            'conversation_log_path': '~/.walker_companion_app/conversation.jsonl',
            'conversation_buffer_size': 50,
        }],
    )

    return LaunchDescription([http_port_arg, dashboard_app_node])
