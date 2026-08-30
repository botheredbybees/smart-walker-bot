"""walker_llm_bridge's ROS2 node: bridges a VoiceIOBackend
(text_io_backend.py's TextIoBackend for now) to an Ollama chat model,
publishing conversation and stop-intent events on ROS2 topics. See
docs/superpowers/specs/2026-08-30-walker-llm-bridge-design.md for the
full design.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Empty, String

from walker_llm_bridge.ollama_client import OllamaClient, OllamaError
from walker_llm_bridge.stop_intent import is_stop_utterance
from walker_llm_bridge.text_io_backend import TextIoBackend

STOP_ACK_MESSAGE = (
    "Stop noted - this is a convenience signal only and isn't wired to "
    "the motors; the hardware E-stop is what actually stops the robot."
)
OLLAMA_UNREACHABLE_MESSAGE = "I can't reach the LLM server right now."


class LlmBridgeNode(Node):
    def __init__(self):
        super().__init__('walker_llm_bridge')

        self.declare_parameter('voice_io_backend', 'text')
        self.declare_parameter('ollama_host', '192.168.1.20')
        self.declare_parameter('ollama_port', 11434)
        self.declare_parameter('ollama_model', 'qwen2.5:14b')
        self.declare_parameter('ollama_timeout_s', 30.0)
        self.declare_parameter(
            'system_prompt',
            "You are a friendly companion robot's conversational voice. "
            "Keep replies short and warm.",
        )
        self.declare_parameter('max_history_messages', 20)

        backend_name = self.get_parameter('voice_io_backend').value
        ollama_host = self.get_parameter('ollama_host').value
        ollama_port = self.get_parameter('ollama_port').value
        ollama_model = self.get_parameter('ollama_model').value
        ollama_timeout_s = self.get_parameter('ollama_timeout_s').value
        self._system_prompt = self.get_parameter('system_prompt').value
        self._max_history_messages = self.get_parameter('max_history_messages').value

        if backend_name == 'text':
            self._backend = TextIoBackend()
        else:
            raise ValueError(
                f"Unknown voice_io_backend '{backend_name}' - only 'text' is implemented; "
                "a real STT/TTS backend is added at the hardware bring-up checkpoint."
            )

        self._ollama_client = OllamaClient(ollama_host, ollama_port, ollama_model, ollama_timeout_s)
        self._history = []

        self._text_in_pub = self.create_publisher(String, '/llm_bridge/text_in', 10)
        self._text_out_pub = self.create_publisher(String, '/llm_bridge/text_out', 10)
        self._stop_pub = self.create_publisher(Empty, '/llm_bridge/stop_requested', 10)

        self._backend.start(self._on_utterance)

    def _on_utterance(self, text):
        self._text_in_pub.publish(String(data=text))

        if is_stop_utterance(text):
            self._stop_pub.publish(Empty())
            self.get_logger().warning(
                f"Stop utterance detected ({text!r}) - convenience signal only, "
                "not wired to any motor/safety topic."
            )
            self._backend.speak(STOP_ACK_MESSAGE)
            return

        messages = [{'role': 'system', 'content': self._system_prompt}]
        messages.extend(self._history)
        messages.append({'role': 'user', 'content': text})

        try:
            response_text = self._ollama_client.chat(messages)
        except OllamaError as e:
            self.get_logger().error(f'Ollama call failed: {e}')
            response_text = OLLAMA_UNREACHABLE_MESSAGE
        else:
            self._history.append({'role': 'user', 'content': text})
            self._history.append({'role': 'assistant', 'content': response_text})
            if len(self._history) > self._max_history_messages:
                overflow = len(self._history) - self._max_history_messages
                del self._history[:overflow]

        self._text_out_pub.publish(String(data=response_text))
        self._backend.speak(response_text)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LlmBridgeNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node._backend.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
