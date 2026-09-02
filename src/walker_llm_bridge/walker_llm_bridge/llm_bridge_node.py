"""walker_llm_bridge's ROS2 node: bridges a VoiceIOBackend
(text_io_backend.py's TextIoBackend for now) to an Ollama chat model,
publishing conversation and stop-intent events on ROS2 topics. See
docs/superpowers/specs/2026-08-30-walker-llm-bridge-design.md for the
full design.
"""
import json
import re
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Empty, String

from walker_llm_bridge.ollama_client import OllamaClient, OllamaError
from walker_llm_bridge.stop_intent import is_stop_utterance
from walker_llm_bridge.text_io_backend import TextIoBackend
from walker_llm_bridge.wellness_context import build_wellness_context_message

STOP_ACK_MESSAGE = (
    "Stop noted - this is a convenience signal only and isn't wired to "
    "the motors; the hardware E-stop is what actually stops the robot."
)
OLLAMA_UNREACHABLE_MESSAGE = "I can't reach the LLM server right now."
REQUIRED_GAIT_KEYS = ('step_count', 'total_distance_m', 'avg_step_length_m')
REQUIRED_ALERT_KEYS = ('type', 'timestamp')
# type is embedded verbatim into an LLM system-role message (wellness_context.py) -
# restrict it to a short identifier shape so a hostile/misbehaving publisher on
# /anomaly_detected can't inject prompt-steering text into that message.
ALERT_TYPE_PATTERN = re.compile(r'^[a-zA-Z0-9_-]{1,32}$')


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
            "Keep replies short and warm. If asked about steps, walking, or "
            "falls, answer plainly and warmly from the wellness data you're "
            "given, never clinically or alarmingly.",
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
        # Guards _gait/_alert_counts/_latest_alert_type: _on_gait_metrics and
        # _on_anomaly_detected run on the rclpy spin thread, while _on_utterance
        # (which reads all three) runs on TextIoBackend's background daemon
        # thread - same two-thread shape shared_state.py's lock guards against.
        self._wellness_lock = threading.Lock()
        self._gait = None
        self._alert_counts = {}
        self._latest_alert_type = None

        self._text_in_pub = self.create_publisher(String, '/llm_bridge/text_in', 10)
        self._text_out_pub = self.create_publisher(String, '/llm_bridge/text_out', 10)
        self._stop_pub = self.create_publisher(Empty, '/llm_bridge/stop_requested', 10)
        self.create_subscription(String, '/gait_metrics', self._on_gait_metrics, 10)
        self.create_subscription(String, '/anomaly_detected', self._on_anomaly_detected, 10)

        self._backend.start(self._on_utterance)

    def _on_gait_metrics(self, msg):
        try:
            gait = json.loads(msg.data)
        except (ValueError, TypeError):
            gait = None
        if (
            not isinstance(gait, dict)
            or not all(key in gait for key in REQUIRED_GAIT_KEYS)
            or not all(
                isinstance(gait.get(key), (int, float)) and not isinstance(gait.get(key), bool)
                for key in REQUIRED_GAIT_KEYS
            )
        ):
            self.get_logger().warn(
                'Ignoring malformed /gait_metrics payload.', throttle_duration_sec=5.0,
            )
            return
        with self._wellness_lock:
            self._gait = gait

    def _on_anomaly_detected(self, msg):
        try:
            alert = json.loads(msg.data)
        except (ValueError, TypeError):
            alert = None
        if (
            not isinstance(alert, dict)
            or not all(key in alert for key in REQUIRED_ALERT_KEYS)
            or not isinstance(alert.get('type'), str)
            or not ALERT_TYPE_PATTERN.match(alert.get('type', ''))
            or not isinstance(alert.get('timestamp'), (int, float))
            or isinstance(alert.get('timestamp'), bool)
        ):
            self.get_logger().warn(
                'Ignoring malformed /anomaly_detected payload.', throttle_duration_sec=5.0,
            )
            return
        alert_type = alert['type']
        with self._wellness_lock:
            self._alert_counts[alert_type] = self._alert_counts.get(alert_type, 0) + 1
            self._latest_alert_type = alert_type

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

        with self._wellness_lock:
            gait_snapshot = self._gait
            alert_counts_snapshot = dict(self._alert_counts)
            latest_alert_type_snapshot = self._latest_alert_type

        messages = [{'role': 'system', 'content': self._system_prompt}]
        wellness_message = build_wellness_context_message(
            gait_snapshot, alert_counts_snapshot, latest_alert_type_snapshot
        )
        if wellness_message is not None:
            messages.append(wellness_message)
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
