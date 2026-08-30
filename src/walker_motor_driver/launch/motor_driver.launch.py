from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    backend_arg = DeclareLaunchArgument(
        'backend',
        default_value='sim',
        description="Motor backend to use: 'sim' (default) or 'real' (not yet implemented - added at hardware bring-up).",
    )

    motor_driver_node = Node(
        package='walker_motor_driver',
        executable='motor_driver_node',
        name='walker_motor_driver',
        output='screen',
        parameters=[{
            'wheel_radius_m': 0.03,
            'wheel_separation_m': 0.2,
            'max_wheel_speed_rad_s': 10.0,
            'publish_rate_hz': 20.0,
            'backend': LaunchConfiguration('backend'),
        }],
    )

    return LaunchDescription([backend_arg, motor_driver_node])
