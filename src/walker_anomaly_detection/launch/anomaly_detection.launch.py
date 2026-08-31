from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='USB-serial device path for the ESP32 IMU bridge.',
    )

    anomaly_detection_node = Node(
        package='walker_anomaly_detection',
        executable='anomaly_detection_node',
        name='walker_anomaly_detection',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': 115200,
            'free_fall_threshold_g': 0.3,
            'free_fall_min_duration_s': 0.05,
            'impact_threshold_g': 2.0,
            'impact_window_s': 0.5,
            'tilt_threshold_deg': 45.0,
            'tilt_sustained_duration_s': 3.0,
        }],
    )

    return LaunchDescription([serial_port_arg, anomaly_detection_node])
