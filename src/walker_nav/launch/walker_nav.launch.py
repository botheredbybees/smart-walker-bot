import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    walker_nav_share = get_package_share_directory('walker_nav')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')
    params_file = os.path.join(walker_nav_share, 'config', 'slam_toolbox_params.yaml')

    fake_lidar_node = Node(
        package='walker_nav',
        executable='fake_lidar_node',
        name='walker_nav_fake_lidar',
        output='screen',
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'slam_params_file': params_file}.items(),
    )

    return LaunchDescription([fake_lidar_node, slam_toolbox_launch])
