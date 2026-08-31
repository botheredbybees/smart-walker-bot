#!/usr/bin/env python3
"""Scripted end-to-end check for walker_anomaly_detection - not a
pytest test.

Fully automated: uses Python's stdlib `pty` module to create a virtual
serial device pair, points the node's serial_port param at one end,
and writes synthetic JSON-line IMU samples to the other end - no real
ESP32/IMU hardware required. See
docs/superpowers/specs/2026-08-31-walker-anomaly-detection-design.md
Sec 2.10 for why this is possible (the node's own wiring logic doesn't
depend on real hardware, only genuine sensor validation does - see
docs/bring_up.md for that separate manual step, not covered by this
script).

Usage (after `colcon build --packages-select walker_anomaly_detection`
and `source install/setup.bash` from src/):

    python3 tools/verify_anomaly_detection.py

Exits 0 and prints PASS on success, exits 1 and prints FAIL otherwise.
"""
import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VerifyNode(Node):
    def __init__(self):
        super().__init__('walker_anomaly_detection_verify')
        self.events = []
        self.create_subscription(String, '/anomaly_detected', self._on_event, 10)

    def _on_event(self, msg):
        self.events.append(json.loads(msg.data))


def _sample_line(ax, ay, az, t_ms):
    return json.dumps({
        'ax': ax, 'ay': ay, 'az': az,
        'gx': 0.0, 'gy': 0.0, 'gz': 0.0,
        'mx': 0.0, 'my': 0.0, 'mz': 0.0,
        't_ms': t_ms,
    }) + '\n'


def main():
    import pty
    controller_fd, device_fd = pty.openpty()
    device_path = os.ttyname(device_fd)

    node_process = subprocess.Popen(
        [
            'ros2', 'run', 'walker_anomaly_detection', 'anomaly_detection_node',
            '--ros-args', '-p', f'serial_port:={device_path}',
        ],
        start_new_session=True,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    os.close(device_fd)  # the node's own pyserial.Serial() opens device_path itself

    rclpy.init()
    node = VerifyNode()

    try:
        time.sleep(2.0)  # let the node declare parameters and open the serial port

        # --- Fall scenario: free-fall (low accel) then impact spike ---
        os.write(controller_fd, _sample_line(0.0, 0.0, 1.0, 0).encode())
        time.sleep(0.05)
        os.write(controller_fd, _sample_line(0.05, 0.0, 0.1, 50).encode())   # enters free-fall
        time.sleep(0.1)                                                      # exceeds 0.05s min duration
        os.write(controller_fd, _sample_line(0.05, 0.0, 0.1, 150).encode())  # confirms free-fall
        time.sleep(0.05)
        os.write(controller_fd, _sample_line(0.0, 0.0, 3.0, 200).encode())   # impact spike

        deadline = time.monotonic() + 10.0
        got_fall = False
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
            if any(e['type'] == 'fall' for e in node.events):
                got_fall = True
                break
        if not got_fall:
            print(f'FAIL: no fall event received within 10s (events so far: {node.events})')
            return 1
        print('Fall event received.')

        # --- Tilt scenario: sustained tilt past the 3.0s duration threshold ---
        # tilt_from_accel_deg(1.0, 0.0, 0.0) == 90 degrees, well past the 45-degree default.
        for i in range(40):  # ~4s at 0.1s spacing, past the 3.0s sustained-duration default
            os.write(controller_fd, _sample_line(1.0, 0.0, 0.0, 300 + i * 100).encode())
            time.sleep(0.1)
            rclpy.spin_once(node, timeout_sec=0.1)

        deadline = time.monotonic() + 10.0
        got_tilt = False
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
            if any(e['type'] == 'tilt' for e in node.events):
                got_tilt = True
                break
        if not got_tilt:
            print(f'FAIL: no tilt event received within 10s (events so far: {node.events})')
            return 1
        print('Tilt event received.')

        print('PASS: fall event and tilt event both verified via a virtual serial pair')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.close(controller_fd)
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
