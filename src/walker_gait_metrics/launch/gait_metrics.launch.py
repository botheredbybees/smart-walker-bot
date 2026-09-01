from launch import LaunchDescription
from launch_ros.actions import Node

GAIT_STEP_THRESHOLD_G = 1.2
GAIT_MIN_STEP_INTERVAL_S = 0.3
GAIT_PUBLISH_RATE_HZ = 1.0


def generate_launch_description():
    gait_metrics_node = Node(
        package='walker_gait_metrics',
        executable='gait_metrics_node',
        name='walker_gait_metrics',
        output='screen',
        parameters=[{
            'step_threshold_g': GAIT_STEP_THRESHOLD_G,
            'min_step_interval_s': GAIT_MIN_STEP_INTERVAL_S,
            'publish_rate_hz': GAIT_PUBLISH_RATE_HZ,
        }],
    )

    return LaunchDescription([gait_metrics_node])
