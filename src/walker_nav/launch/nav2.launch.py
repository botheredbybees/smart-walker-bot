import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    walker_nav_share = get_package_share_directory('walker_nav')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    default_params_file = os.path.join(walker_nav_share, 'config', 'nav2_params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the nav2 params YAML to use.',
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
            # navigation_launch.py's own use_sim_time launch argument
            # already defaults to 'false' (verified against the
            # installed nav2_bringup package) - unlike slam_toolbox's
            # online_async_launch.py, no override is strictly required,
            # but it's passed explicitly here for clarity and so a
            # future nav2_bringup version change can't silently flip it.
            'use_sim_time': 'false',
        }.items(),
    )

    return LaunchDescription([params_file_arg, navigation_launch])
