#!/usr/bin/env python3
"""Scripted end-to-end check for walker_gait_metrics - not a pytest
test.

Fully automated: publishes synthetic /imu/raw_sample and /odom messages
directly - this node has no serial/hardware dependency of its own,
unlike walker_anomaly_detection's node, so no pty trick is needed. See
docs/superpowers/specs/2026-09-01-walker-gait-metrics-design.md Sec 2.11.

Usage (after `colcon build --packages-select walker_gait_metrics` and
`source install/setup.bash` from src/):

    python3 tools/verify_gait_metrics.py

Exits 0 and prints PASS on success, exits 1 and prints FAIL otherwise.
"""
import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

# Must match launch/gait_metrics.launch.py's defaults (this script relies
# on the node's default parameters, launched via plain `ros2 run` below).
STEP_THRESHOLD_G = 1.2
MIN_STEP_INTERVAL_S = 0.3

NUM_STEPS = 5
TOTAL_DISTANCE_M = 10.0
EXPECTED_AVG_STEP_LENGTH_M = TOTAL_DISTANCE_M / NUM_STEPS


class VerifyNode(Node):
    def __init__(self):
        super().__init__('walker_gait_metrics_verify')
        self.latest_metrics = None
        self.imu_pub = self.create_publisher(String, '/imu/raw_sample', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(String, '/gait_metrics', self._on_metrics, 10)

    def _on_metrics(self, msg):
        self.latest_metrics = json.loads(msg.data)

    def publish_imu_sample(self, ax, ay, az):
        payload = json.dumps({
            'ax': ax, 'ay': ay, 'az': az,
            'gx': 0.0, 'gy': 0.0, 'gz': 0.0,
            'mx': 0.0, 'my': 0.0, 'mz': 0.0,
            't_ms': 0,
        })
        self.imu_pub.publish(String(data=payload))

    def publish_odom_pose(self, x_m, y_m):
        msg = Odometry()
        msg.pose.pose.position.x = x_m
        msg.pose.pose.position.y = y_m
        msg.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(msg)


def main():
    node_process = subprocess.Popen(
        ['ros2', 'run', 'walker_gait_metrics', 'gait_metrics_node'],
        start_new_session=True,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )

    rclpy.init()
    node = VerifyNode()

    try:
        time.sleep(2.0)  # let the node declare parameters and subscribe

        # --- Odometry: a single 10m displacement ---
        node.publish_odom_pose(0.0, 0.0)
        time.sleep(0.1)
        node.publish_odom_pose(TOTAL_DISTANCE_M, 0.0)
        time.sleep(0.1)

        # --- IMU: five steps, each spaced past the debounce interval ---
        for _ in range(NUM_STEPS):
            node.publish_imu_sample(0.0, 0.0, 1.5)  # magnitude 1.5g, above the 1.2g threshold
            time.sleep(MIN_STEP_INTERVAL_S + 0.05)

        # publish_rate_hz defaults to 1.0 - give it time to publish at least
        # once after all the synthetic input above has been processed.
        # Keep spinning until the metrics have actually converged to the
        # expected step_count, not just until any message arrives -
        # gait_metrics_node's 1Hz timer publishes on schedule regardless of
        # input completeness, so several early, incomplete snapshots can
        # queue up while this script was busy publishing synthetic input
        # above. Stopping at the first non-None message risks grabbing one
        # of those stale backlog snapshots instead of the converged state.
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
            if node.latest_metrics is not None and node.latest_metrics['step_count'] >= NUM_STEPS:
                break

        if node.latest_metrics is None:
            print('FAIL: no /gait_metrics message received within 10s')
            return 1

        metrics = node.latest_metrics
        if metrics['step_count'] != NUM_STEPS:
            print(f"FAIL: step_count={metrics['step_count']}, expected {NUM_STEPS}")
            return 1
        if metrics['total_distance_m'] != TOTAL_DISTANCE_M:
            print(f"FAIL: total_distance_m={metrics['total_distance_m']}, expected {TOTAL_DISTANCE_M}")
            return 1
        if metrics['avg_step_length_m'] != EXPECTED_AVG_STEP_LENGTH_M:
            print(
                f"FAIL: avg_step_length_m={metrics['avg_step_length_m']}, "
                f"expected {EXPECTED_AVG_STEP_LENGTH_M}"
            )
            return 1

        print(
            f"PASS: step_count={metrics['step_count']}, "
            f"total_distance_m={metrics['total_distance_m']}, "
            f"avg_step_length_m={metrics['avg_step_length_m']}"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
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
