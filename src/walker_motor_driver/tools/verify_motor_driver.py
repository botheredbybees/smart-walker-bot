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

(On this project's dev workstation, `python3` on PATH is an Anaconda
install that can't import rclpy's C extension - use `/usr/bin/python3`
instead if you hit `ModuleNotFoundError: No module named
'rclpy._rclpy_pybind11'`. This is a workstation-specific quirk, not
something this script can detect or fix.)

Exits 0 and prints PASS on success, exits 1 and prints FAIL otherwise.
"""
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

# With wheel_radius_m=0.03, wheel_separation_m=0.2, max_wheel_speed_rad_s=10.0
# (this package's placeholder defaults), a Twist(linear.x=1.0) command
# saturates both wheels to 10.0 rad/s, giving a body-frame linear velocity
# of wheel_radius_m * 10.0 = 0.3 m/s - not the commanded 1.0 m/s. Asserting
# against this clamped value (rather than just "greater than zero") is what
# actually catches a wrong wheel_radius_m, a broken clamp, or a scale error.
EXPECTED_LINEAR_X_M_S = 0.3
LINEAR_X_TOLERANCE_M_S = 0.05


class VerifyNode(Node):
    def __init__(self):
        super().__init__('walker_motor_driver_verify')
        self.latest_odom = None
        self.saw_tf = False
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(TFMessage, '/tf', self._on_tf, 10)

    def _on_odom(self, msg):
        self.latest_odom = msg

    def _on_tf(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == 'odom' and transform.child_frame_id == 'base_link':
                self.saw_tf = True

    def publish_cmd_vel(self):
        twist = Twist()
        twist.linear.x = 1.0
        self.cmd_pub.publish(twist)


def main():
    rclpy.init()
    node = VerifyNode()

    try:
        # Republish each spin iteration, not just once - a single publish
        # immediately after creating the publisher can race DDS discovery
        # of the node's subscriber and be silently dropped.
        deadline = time.monotonic() + 5.0
        while node.latest_odom is None and time.monotonic() < deadline:
            node.publish_cmd_vel()
            rclpy.spin_once(node, timeout_sec=0.5)

        if node.latest_odom is None:
            print('FAIL: no /odom message received within 5s')
            return 1

        first_x = node.latest_odom.pose.pose.position.x
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            node.publish_cmd_vel()
            rclpy.spin_once(node, timeout_sec=0.5)
        second_x = node.latest_odom.pose.pose.position.x

        if not (second_x > first_x):
            print(f'FAIL: pose.position.x did not increase ({first_x} -> {second_x})')
            return 1

        twist_x = node.latest_odom.twist.twist.linear.x
        if abs(twist_x - EXPECTED_LINEAR_X_M_S) > LINEAR_X_TOLERANCE_M_S:
            print(f'FAIL: twist.twist.linear.x={twist_x:.4f}, expected '
                  f'{EXPECTED_LINEAR_X_M_S:.4f} +/- {LINEAR_X_TOLERANCE_M_S} '
                  '(check wheel_radius_m/max_wheel_speed_rad_s or the clamp)')
            return 1

        if not node.saw_tf:
            print('FAIL: no odom->base_link transform seen on /tf within the test window')
            return 1

        print(f'PASS: odom.pose.position.x increased ({first_x:.4f} -> {second_x:.4f}), '
              f'twist.linear.x={twist_x:.4f}, odom->base_link TF observed')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
