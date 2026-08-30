import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Must match config/slam_toolbox_params.yaml's max_laser_range - nothing
# else keeps these two in sync (see walker_nav's README "Known limitations").
FAKE_LIDAR_MAX_RANGE_M = 8.0
FAKE_LIDAR_NUM_BEAMS = 360
FAKE_LIDAR_SCAN_RATE_HZ = 5.0


def generate_launch_description():
    walker_nav_share = get_package_share_directory('walker_nav')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')
    params_file = os.path.join(walker_nav_share, 'config', 'slam_toolbox_params.yaml')

    fake_lidar_node = Node(
        package='walker_nav',
        executable='fake_lidar_node',
        name='walker_nav_fake_lidar',
        output='screen',
        parameters=[{
            'num_beams': FAKE_LIDAR_NUM_BEAMS,
            'max_range_m': FAKE_LIDAR_MAX_RANGE_M,
            'scan_rate_hz': FAKE_LIDAR_SCAN_RATE_HZ,
        }],
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': params_file,
            # online_async_launch.py declares use_sim_time with
            # default_value='true' and applies it AFTER slam_params_file,
            # so it silently overrides anything set in the YAML - must be
            # passed here explicitly. Every other node in this project
            # (fake_lidar_node, walker_motor_driver) stamps with the wall
            # clock; leaving slam_toolbox on simulated time with no
            # /clock publisher works today only by accident.
            'use_sim_time': 'false',
        }.items(),
    )

    return LaunchDescription([fake_lidar_node, slam_toolbox_launch])
