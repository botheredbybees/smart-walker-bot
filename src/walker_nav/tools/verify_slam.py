#!/usr/bin/env python3
"""Scripted end-to-end check for walker_nav's SLAM pipeline - not a
pytest test. Assumes walker_motor_driver's node (backend:=sim) and
walker_nav's fake_lidar_node + slam_toolbox are already running (see
this package's README for the launch commands). Drives the simulated
robot from its start pose, through the doorway, into Room 2, then
checks that /map has actually accumulated known cells and that
slam_toolbox is publishing map->odom on /tf - confirming SLAM is
genuinely running and building a map, not just that topics are wired
up.

Usage: python3 tools/verify_slam.py
(On this project's dev workstation, use /usr/bin/python3 if plain
python3 can't import rclpy - see walker_motor_driver's
verify_motor_driver.py docstring for why.)

Exits 0 and prints PASS on success, exits 1 and prints FAIL otherwise.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

# Robot starts at (0,0,0) facing +x; the doorway is reached by heading
# +y (spec Sec 3.1). Turn ~90 degrees at pi/2 rad/s for 1s (an exact
# 90-degree turn if timing were perfect; a few degrees of loop-timing
# jitter is fine - the 1m-wide doorway comfortably absorbs errors up to
# about 18 degrees given the ~1.5m distance to it, worked out from the
# room geometry in room_map.py).
TURN_ANGULAR_Z_RAD_S = math.pi / 2
TURN_DURATION_S = 1.0

# linear.x=1.0 is clamped by walker_motor_driver's placeholder
# max_wheel_speed_rad_s=10.0/wheel_radius_m=0.03 to an actual ~0.3 m/s
# (see walker_motor_driver's verify_motor_driver.py for the same
# arithmetic). ~2.7m at 0.3 m/s takes 9s, landing well inside Room 2
# (which spans y in [1.5, 3.5]) without reaching its far wall.
DRIVE_LINEAR_X = 1.0
DRIVE_DURATION_S = 9.0

SETTLE_DURATION_S = 3.0

MINIMUM_KNOWN_CELLS = 50  # /map cells that are free (0) or occupied (100), not unknown (-1)

# Republish every command at this interval - walker_motor_driver has a
# cmd_vel_timeout_s (default 0.5s) that zeroes wheel speeds if no
# command arrives in time, so a single publish-and-wait would stop the
# robot mid-maneuver.
REPUBLISH_INTERVAL_S = 0.1


class VerifySlamNode(Node):
    def __init__(self):
        super().__init__('walker_nav_verify_slam')
        self.latest_map = None
        self.saw_map_to_odom_tf = False
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, 10)
        self.create_subscription(TFMessage, '/tf', self._on_tf, 10)

    def _on_map(self, msg):
        self.latest_map = msg

    def _on_tf(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == 'map' and transform.child_frame_id == 'odom':
                self.saw_map_to_odom_tf = True

    def publish_cmd(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)


def _drive_phase(node, linear_x, angular_z, duration_s):
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        node.publish_cmd(linear_x, angular_z)
        rclpy.spin_once(node, timeout_sec=REPUBLISH_INTERVAL_S)


def main():
    rclpy.init()
    node = VerifySlamNode()

    try:
        _drive_phase(node, linear_x=0.0, angular_z=TURN_ANGULAR_Z_RAD_S, duration_s=TURN_DURATION_S)
        _drive_phase(node, linear_x=DRIVE_LINEAR_X, angular_z=0.0, duration_s=DRIVE_DURATION_S)
        _drive_phase(node, linear_x=0.0, angular_z=0.0, duration_s=SETTLE_DURATION_S)

        if node.latest_map is None:
            print('FAIL: no /map message received')
            return 1

        known_cells = sum(1 for cell in node.latest_map.data if cell != -1)
        if known_cells < MINIMUM_KNOWN_CELLS:
            print(f'FAIL: /map has only {known_cells} known cells, expected at least '
                  f'{MINIMUM_KNOWN_CELLS} - slam_toolbox may not be receiving /scan data')
            return 1

        if not node.saw_map_to_odom_tf:
            print('FAIL: no map->odom transform seen on /tf - slam_toolbox may not be running')
            return 1

        print(f'PASS: /map has {known_cells} known cells, map->odom TF observed')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
