#!/usr/bin/env python3
"""Scripted end-to-end check for walker_motor_driver - not a pytest test.

motor_driver_node.py needs a live rclpy context and the sim backend's
real-time clock, which doesn't fit the fast, deterministic pytest suite
the rest of this package uses (see this package's README). Unlike
walker_safety's hardware bring-up, this doesn't need any physical
hardware - just the node running with the sim backend.

Usage (after `colcon build --packages-select walker_motor_driver` and
`source install/setup.bash` from src/, with the node already launched
via `ros2 launch walker_motor_driver motor_driver.launch.py`):

    python3 tools/verify_motor_driver.py

Exits 0 and prints PASS on success, exits 1 and prints FAIL otherwise.
"""
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class VerifyNode(Node):
    def __init__(self):
        super().__init__('walker_motor_driver_verify')
        self.latest_odom = None
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)

    def _on_odom(self, msg):
        self.latest_odom = msg


def main():
    rclpy.init()
    node = VerifyNode()

    try:
        twist = Twist()
        twist.linear.x = 1.0
        node.cmd_pub.publish(twist)

        deadline = time.monotonic() + 5.0
        while node.latest_odom is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)

        if node.latest_odom is None:
            print('FAIL: no /odom message received within 5s')
            return 1

        first_x = node.latest_odom.pose.pose.position.x
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
        second_x = node.latest_odom.pose.pose.position.x

        if not (second_x > first_x):
            print(f'FAIL: pose.position.x did not increase ({first_x} -> {second_x})')
            return 1

        twist_x = node.latest_odom.twist.twist.linear.x
        if not (twist_x > 0.0):
            print(f'FAIL: twist.twist.linear.x should be positive, got {twist_x}')
            return 1

        print(f'PASS: odom.pose.position.x increased ({first_x:.4f} -> {second_x:.4f}), '
              f'twist.linear.x={twist_x:.4f}')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
