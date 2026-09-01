import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Must match config/slam_toolbox_params.yaml's max_laser_range - nothing
# else keeps these two in sync (see walker_nav's README "Known limitations").
# max_range_m below is overridable at launch (see max_range_m_arg); these
# constants are only its default.
FAKE_LIDAR_MAX_RANGE_M = 8.0
FAKE_LIDAR_NUM_BEAMS = 360
FAKE_LIDAR_SCAN_RATE_HZ = 5.0
FAKE_LIDAR_FOV_DEG = 360.0


def generate_launch_description():
    walker_nav_share = get_package_share_directory('walker_nav')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')
    params_file = os.path.join(walker_nav_share, 'config', 'slam_toolbox_params.yaml')

    fov_deg_arg = DeclareLaunchArgument(
        'fov_deg',
        default_value=str(FAKE_LIDAR_FOV_DEG),
        description=(
            "fake_lidar_node's horizontal field of view in degrees. 360 (default) "
            "reproduces the full-circle sim; 57 is the documented Kinect-realistic "
            "profile (docs/superpowers/specs/2026-09-01-walker-nav-kinect-design.md Sec 2.3)."
        ),
    )
    max_range_m_arg = DeclareLaunchArgument(
        'max_range_m',
        default_value=str(FAKE_LIDAR_MAX_RANGE_M),
        description=(
            "fake_lidar_node's max sensing range in meters. 8.0 (default) matches "
            "config/slam_toolbox_params.yaml's max_laser_range; 4.0 is the documented "
            "Kinect-realistic profile."
        ),
    )

    fake_lidar_node = Node(
        package='walker_nav',
        executable='fake_lidar_node',
        name='walker_nav_fake_lidar',
        output='screen',
        parameters=[{
            'num_beams': FAKE_LIDAR_NUM_BEAMS,
            'max_range_m': ParameterValue(LaunchConfiguration('max_range_m'), value_type=float),
            'scan_rate_hz': FAKE_LIDAR_SCAN_RATE_HZ,
            'fov_deg': ParameterValue(LaunchConfiguration('fov_deg'), value_type=float),
        }],
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            # online_async_launch.py declares use_sim_time with
            # default_value='true' and applies it AFTER slam_params_file,
            # so it silently overrides anything set in the YAML - must be
            # passed here explicitly. Every other node in this project
            # (fake_lidar_node, walker_motor_driver) stamps with the wall
            # clock; leaving slam_toolbox on simulated time with no
            # /clock publisher works today only by accident.
            'slam_params_file': params_file,
            'use_sim_time': 'false',
        }.items(),
    )

    return LaunchDescription([fov_deg_arg, max_range_m_arg, fake_lidar_node, slam_toolbox_launch])
