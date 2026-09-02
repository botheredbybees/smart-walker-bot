#!/usr/bin/env python3
"""Scripted end-to-end check for walker_llm_bridge - not a pytest test.

llm_bridge_node.py's only real utterance path is its VoiceIOBackend (the
`text` backend reads stdin) - see
docs/superpowers/specs/2026-08-30-walker-llm-bridge-design.md Sec 2.1/2.4.
So driving the conversation from a script means writing into the node's
actual stdin, not publishing to a topic (/llm_bridge/text_in is
published BY the node, an echo for observability - subscribing a test
publisher to it would create a publish/subscribe loop back onto the
node's own callback).

This script launches the node itself (via subprocess, stdin redirected
from a named pipe it creates) rather than assuming a separately-launched
node the way other packages' verify scripts do, since stdin redirection
can only be wired up at process-creation time.

Requires the real Ollama server (config default: 192.168.1.20:11434,
model qwen2.5:14b) to be reachable - there is no mocking here, unlike
test_ollama_client.py.

Usage (after `colcon build --packages-select walker_llm_bridge` and
`source install/setup.bash` from src/):

    python3 tools/verify_llm_bridge.py

Exits 0 and prints PASS on success, exits 1 and prints FAIL otherwise.
"""
import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String

from walker_llm_bridge.llm_bridge_node import OLLAMA_UNREACHABLE_MESSAGE


class VerifyNode(Node):
    def __init__(self):
        super().__init__('walker_llm_bridge_verify')
        self.text_in_messages = []
        self.text_out_messages = []
        self.stop_requested_count = 0
        self.create_subscription(String, '/llm_bridge/text_in', self._on_text_in, 10)
        self.create_subscription(String, '/llm_bridge/text_out', self._on_text_out, 10)
        self.create_subscription(Empty, '/llm_bridge/stop_requested', self._on_stop, 10)
        self.gait_pub = self.create_publisher(String, '/gait_metrics', 10)
        self.anomaly_pub = self.create_publisher(String, '/anomaly_detected', 10)

    def _on_text_in(self, msg):
        self.text_in_messages.append(msg.data)

    def _on_text_out(self, msg):
        self.text_out_messages.append(msg.data)

    def _on_stop(self, msg):
        self.stop_requested_count += 1


def _spin_until(node, predicate, timeout_s):
    deadline = time.monotonic() + timeout_s
    while not predicate() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
    return predicate()


def main():
    fifo_dir = tempfile.mkdtemp(prefix='walker_llm_bridge_verify_')
    fifo_path = os.path.join(fifo_dir, 'stdin_fifo')
    os.mkfifo(fifo_path)

    node_process = subprocess.Popen(
        f'exec ros2 run walker_llm_bridge llm_bridge_node < {fifo_path}',
        shell=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )

    # Blocks until node_process's shell redirection opens the FIFO's read
    # end - standard FIFO open-pairing, no race or O_NONBLOCK trick needed.
    fifo_write = open(fifo_path, 'w')

    rclpy.init()
    node = VerifyNode()

    try:
        time.sleep(2.0)  # let the node finish declaring parameters/subscriptions

        # Wellness data published before the utterance below - not asserted
        # on semantically (too flaky against a real LLM's exact wording),
        # just confirms the node stays up and responsive with this context
        # wired in. Eyeball the printed round-trip response to sanity-check
        # it actually reflects this data.
        gait_payload = json.dumps({'step_count': 42, 'total_distance_m': 84.0, 'avg_step_length_m': 2.0})
        node.gait_pub.publish(String(data=gait_payload))
        alert_payload = json.dumps({'type': 'fall', 'timestamp': time.time()})
        node.anomaly_pub.publish(String(data=alert_payload))
        time.sleep(1.0)

        try:
            fifo_write.write('hello there\n')
            fifo_write.flush()
        except BrokenPipeError:
            print('FAIL: node process exited before reading stdin - check the package is '
                  'built and the workspace is sourced')
            return 1
        if not _spin_until(node, lambda: len(node.text_in_messages) >= 1, timeout_s=5.0):
            print('FAIL: no /llm_bridge/text_in echo received within 5s')
            return 1
        if node.text_in_messages[0] != 'hello there':
            print(f"FAIL: /llm_bridge/text_in echoed {node.text_in_messages[0]!r}, expected 'hello there'")
            return 1

        # 40s, not the client's own 30s ollama_timeout_s - a wide-enough
        # margin that a slow-but-working server's real reply and the node's
        # own OLLAMA_UNREACHABLE_MESSAGE fallback (checked below) don't race
        # this loop's own timeout for which failure message wins.
        if not _spin_until(node, lambda: len(node.text_out_messages) >= 1, timeout_s=40.0):
            print('FAIL: no /llm_bridge/text_out response received within 40s (Ollama round-trip)')
            return 1
        if node.text_out_messages[0] == OLLAMA_UNREACHABLE_MESSAGE:
            print('FAIL: node returned its Ollama-unreachable fallback - the server was '
                  'not actually reached (this check does not exercise that path)')
            return 1
        print(f'Round-trip response: {node.text_out_messages[0]!r}')

        try:
            fifo_write.write('stop\n')
            fifo_write.flush()
        except BrokenPipeError:
            print('FAIL: node process exited before reading stdin - check the package is '
                  'built and the workspace is sourced')
            return 1
        if not _spin_until(node, lambda: node.stop_requested_count >= 1, timeout_s=5.0):
            print('FAIL: /llm_bridge/stop_requested did not fire within 5s')
            return 1

        time.sleep(3.0)
        rclpy.spin_once(node, timeout_sec=0.5)
        if len(node.text_out_messages) != 1:
            print(
                'FAIL: expected no new /llm_bridge/text_out after stop utterance, '
                f'got {len(node.text_out_messages)} total messages'
            )
            return 1

        print(
            'PASS: text_in echo, Ollama round-trip response (with wellness context wired in), '
            'and stop-intent short-circuit all verified'
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
        fifo_write.close()
        shutil.rmtree(fifo_dir, ignore_errors=True)
        # node_process.pid is only the `ros2` wrapper - `ros2 run` forks the
        # actual llm_bridge_node as a separate child via subprocess.Popen()
        # internally (not an exec-replace), so signaling node_process.pid
        # alone would orphan the real node. start_new_session=True above put
        # the wrapper (and its forked child, which inherits the same process
        # group) in their own process group, so signal that group instead.
        try:
            os.killpg(os.getpgid(node_process.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            node_process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(node_process.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            node_process.wait()


if __name__ == '__main__':
    sys.exit(main())
