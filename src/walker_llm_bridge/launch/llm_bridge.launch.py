from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# NOTE: the default 'text' backend (text_io_backend.py's TextIoBackend) reads
# sys.stdin, but `ros2 launch` never connects a launched node's stdin to
# anything - the launch service holds it open and does not feed it, so a
# node started this way hangs forever waiting for input that never arrives.
# For interactive use with the 'text' backend, run the node directly instead:
#     ros2 run walker_llm_bridge llm_bridge_node
# This launch file remains for future backends (e.g. real STT/TTS) that
# don't depend on a real stdin.


def generate_launch_description():
    backend_arg = DeclareLaunchArgument(
        'voice_io_backend',
        default_value='text',
        description="Voice I/O backend: 'text' (default, stdin/stdout) - "
                    "a real STT/TTS backend is added at hardware bring-up.",
    )

    llm_bridge_node = Node(
        package='walker_llm_bridge',
        executable='llm_bridge_node',
        name='walker_llm_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'voice_io_backend': LaunchConfiguration('voice_io_backend'),
            'ollama_host': '192.168.1.20',
            'ollama_port': 11434,
            'ollama_model': 'qwen2.5:14b',
            'ollama_timeout_s': 30.0,
            'max_history_messages': 20,
        }],
    )

    return LaunchDescription([backend_arg, llm_bridge_node])
