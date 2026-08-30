#!/usr/bin/env python3
"""Scripted end-to-end check for walker_nav's Nav2 pass - not a pytest
test. Assumes walker_motor_driver's node (backend:=sim), walker_nav's
SLAM launch (fake_lidar_node + slam_toolbox), and walker_nav's Nav2
launch are all already running (see this package's README for the
launch commands).

Reuses the SLAM pass's exact drive-through-the-doorway maneuver
(tools/verify_slam.py's constants) to give slam_toolbox's map real
coverage of both rooms before handing control to Nav2 - sending a
navigate_to_pose goal immediately on a mostly-unknown map would be a
less deterministic first test. Then sends a navigate_to_pose action
goal back near the start pose (0, 0), letting Nav2 plan and drive the
return trip through the doorway on its own, and confirms the action
reports SUCCEEDED with the final /odom pose close to the goal.

Usage: python3 tools/verify_nav2.py
(On this project's dev workstation, use /usr/bin/python3 if plain
python3 can't import rclpy - see walker_motor_driver's
verify_motor_driver.py docstring for why.)

Exits 0 and prints PASS on success, exits 1 and prints FAIL otherwise.
"""
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node

# Same maneuver walker_nav's SLAM pass verification uses
# (tools/verify_slam.py) to give the map real coverage before Nav2
# needs to plan through it.
TURN_ANGULAR_Z_RAD_S = math.pi / 2
TURN_DURATION_S = 1.0
DRIVE_LINEAR_X = 1.0
DRIVE_DURATION_S = 9.0
SETTLE_DURATION_S = 3.0
REPUBLISH_INTERVAL_S = 0.1
FIRST_ODOM_TIMEOUT_S = 5.0

GOAL_X_M = 0.0
GOAL_Y_M = 0.0
GOAL_XY_TOLERANCE_M = 0.5
NAV2_ACTION_TIMEOUT_S = 60.0


class VerifyNav2Node(Node):
    def __init__(self):
        super().__init__('walker_nav_verify_nav2')
        self.latest_odom = None
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _on_odom(self, msg):
        self.latest_odom = msg

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


def _send_nav_goal_and_wait(node):
    """Send a navigate_to_pose goal and block (via spinning) until it
    completes. Returns the GoalStatus constant, or None on a timeout,
    rejection, or unavailable action server."""
    if not node.nav_to_pose_client.wait_for_server(timeout_sec=10.0):
        return None

    goal_msg = NavigateToPose.Goal()
    goal_msg.pose = PoseStamped()
    goal_msg.pose.header.frame_id = 'map'
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = GOAL_X_M
    goal_msg.pose.pose.position.y = GOAL_Y_M
    goal_msg.pose.pose.orientation.w = 1.0

    send_goal_future = node.nav_to_pose_client.send_goal_async(goal_msg)
    deadline = time.monotonic() + NAV2_ACTION_TIMEOUT_S
    while not send_goal_future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)

    if not send_goal_future.done():
        return None
    goal_handle = send_goal_future.result()
    if not goal_handle.accepted:
        return None

    result_future = goal_handle.get_result_async()
    while not result_future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)

    if not result_future.done():
        return None
    return result_future.result().status


def main():
    rclpy.init()
    node = VerifyNav2Node()

    try:
        # Wait for the first /odom before priming - a publish immediately
        # after node creation can race DDS discovery of walker_motor_driver's
        # subscriber and be silently dropped (same guard verify_motor_driver.py
        # and verify_slam.py already use).
        deadline = time.monotonic() + FIRST_ODOM_TIMEOUT_S
        while node.latest_odom is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)

        if node.latest_odom is None:
            print('FAIL: no /odom message received within '
                  f'{FIRST_ODOM_TIMEOUT_S}s - is walker_motor_driver running?')
            return 1

        # Prime the map: drive through the doorway into Room 2 (same
        # maneuver as walker_nav's SLAM pass verification), so
        # slam_toolbox's map covers the path Nav2 will need to plan
        # the return trip through.
        _drive_phase(node, linear_x=0.0, angular_z=TURN_ANGULAR_Z_RAD_S, duration_s=TURN_DURATION_S)
        _drive_phase(node, linear_x=DRIVE_LINEAR_X, angular_z=0.0, duration_s=DRIVE_DURATION_S)
        _drive_phase(node, linear_x=0.0, angular_z=0.0, duration_s=SETTLE_DURATION_S)

        # Hand off to Nav2 for the return trip - don't publish /cmd_vel
        # ourselves from here on, or we'd fight with Nav2's own output.
        status = _send_nav_goal_and_wait(node)

        if status is None:
            print('FAIL: navigate_to_pose action did not complete within '
                  f'{NAV2_ACTION_TIMEOUT_S}s (or the action server/goal was '
                  'rejected) - is the Nav2 stack running and active?')
            return 1

        if status != GoalStatus.STATUS_SUCCEEDED:
            print(f'FAIL: navigate_to_pose finished with status {status}, '
                  f'expected STATUS_SUCCEEDED ({GoalStatus.STATUS_SUCCEEDED})')
            return 1

        final_x = node.latest_odom.pose.pose.position.x
        final_y = node.latest_odom.pose.pose.position.y
        distance_from_goal_m = math.hypot(final_x - GOAL_X_M, final_y - GOAL_Y_M)
        if distance_from_goal_m > GOAL_XY_TOLERANCE_M:
            print(f'FAIL: navigate_to_pose reported SUCCEEDED but final pose '
                  f'({final_x:.2f}, {final_y:.2f}) is {distance_from_goal_m:.2f}m '
                  f'from the goal ({GOAL_X_M}, {GOAL_Y_M}), expected within '
                  f'{GOAL_XY_TOLERANCE_M}m')
            return 1

        print(f'PASS: navigate_to_pose SUCCEEDED, final pose ({final_x:.2f}, '
              f'{final_y:.2f}) is {distance_from_goal_m:.2f}m from the goal')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
