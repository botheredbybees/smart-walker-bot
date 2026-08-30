#!/usr/bin/env python3
"""Scripted end-to-end check for walker_nav's Nav2 pass - not a pytest
test. Assumes walker_motor_driver's node (backend:=sim), walker_nav's
SLAM launch (fake_lidar_node + slam_toolbox), and walker_nav's Nav2
launch are all already running (see this package's README for the
launch commands, in that order).

Reuses the SLAM pass's exact drive-through-the-doorway maneuver
(tools/verify_slam.py's constants) to give slam_toolbox's map real
coverage of both rooms before handing control to Nav2 - sending a
navigate_to_pose goal immediately on a mostly-unknown map would be a
less deterministic first test. Then sends a navigate_to_pose action
goal back near the start pose (0, 0), letting Nav2 plan and drive the
return trip through the doorway on its own, and confirms the action
reports SUCCEEDED with the final pose (looked up via TF, map frame -
see _lookup_map_to_base_link) close to the goal.

If the goal doesn't succeed for any reason - timeout, rejection, a
FAIL on the final check, or Ctrl-C - the goal is explicitly cancelled
before this script exits. Without that, an abandoned goal leaves
bt_navigator autonomously driving the robot with no supervisor once
this process is gone; walker_motor_driver's cmd_vel_timeout_s does NOT
catch this, since it only fires on an ABSENCE of /cmd_vel, and Nav2's
own velocity_smoother keeps actively publishing throughout navigation.

Two things are needed to make that hold on the Ctrl-C path
specifically, both verified by interrupting a real run mid-navigation:

- This script installs its OWN SIGINT handler (see _install_sigint_
  handler), replacing the one rclpy.init() installs. rclpy's handler
  shuts the context down immediately, which makes the in-flight
  spin_once raise ExternalShutdownException and leaves the context
  dead - so the cancellation this script needs to send on the way out
  could no longer be sent at all. Ours only sets a flag; the spin
  loops notice it and unwind with the context still alive.
- The accepted goal handle is stored on the node (node.goal_handle) as
  soon as the server accepts it, not just returned from
  _send_nav_goal_and_wait. An exception unwinding out of that function
  never reaches its return statement, so a handle that only travelled
  by return value would be invisible to the cleanup path - which is
  exactly the case where cancelling matters most.

Usage: python3 tools/verify_nav2.py
(On this project's dev workstation, use /usr/bin/python3 if plain
python3 can't import rclpy - see walker_motor_driver's
verify_motor_driver.py docstring for why.)

Exits 0 and prints PASS on success, exits 1 and prints FAIL otherwise.
"""
import math
import signal
import sys
import time

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException

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
# Tighter than the original 0.5m now that the check is frame-correct
# (map->base_link via TF, not odom) - matches nav2_params.yaml's own
# general_goal_checker.xy_goal_tolerance (0.25m) plus margin for the
# map->odom correction this check now actually accounts for.
GOAL_XY_TOLERANCE_M = 0.3
NAV2_ACTION_TIMEOUT_S = 60.0
TF_LOOKUP_TIMEOUT_S = 5.0
GOAL_CANCEL_TIMEOUT_S = 5.0


# Set by this script's own SIGINT handler; polled by every spin loop
# below so a Ctrl-C unwinds through the cleanup path (which cancels the
# goal) instead of killing the ROS context out from under it.
_interrupted = False


def _install_sigint_handler():
    """Replace rclpy's SIGINT handler with one that only raises a flag.

    rclpy.init() installs a handler that shuts the context down on
    SIGINT. That makes the in-flight spin_once raise
    ExternalShutdownException and, worse, leaves the context dead - so
    goal_handle.cancel_goal_async() and the spinning needed to deliver
    it can no longer work, and Ctrl-C would leave Nav2 driving the
    robot with nothing supervising it. Keeping the context alive is
    what lets the cleanup path actually cancel.

    A second Ctrl-C restores the default handler, so an impatient
    operator can still force the process down.
    """
    def _handler(signum, frame):
        global _interrupted
        if _interrupted:
            signal.signal(signal.SIGINT, signal.SIG_DFL)
            return
        _interrupted = True
        print('\nInterrupted - winding down (the in-flight goal will be '
              'cancelled first; Ctrl-C again to force quit).')

    signal.signal(signal.SIGINT, _handler)


class VerifyNav2Node(Node):
    def __init__(self):
        super().__init__('walker_nav_verify_nav2')
        self.latest_odom = None
        # Held on the node, not just returned from
        # _send_nav_goal_and_wait, so main()'s cleanup can still find
        # (and cancel) it if an exception unwinds past that return.
        self.goal_handle = None
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _on_odom(self, msg):
        self.latest_odom = msg

    def publish_cmd(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)


def _drive_phase(node, linear_x, angular_z, duration_s):
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline and not _interrupted:
        node.publish_cmd(linear_x, angular_z)
        rclpy.spin_once(node, timeout_sec=REPUBLISH_INTERVAL_S)


def _send_nav_goal_and_wait(node):
    """Send a navigate_to_pose goal and block (via spinning) until it
    completes. Returns (status, goal_handle):
    - status is a GoalStatus constant, or None on timeout/rejection/
      unavailable action server.
    - goal_handle is the accepted handle if one exists, or None - the
      caller must cancel it on any non-success exit path (see module
      docstring). It is also recorded on node.goal_handle the moment
      the server accepts it, so the cleanup path can find it even if
      this function never reaches a return statement."""
    if not node.nav_to_pose_client.wait_for_server(timeout_sec=10.0):
        return None, None

    goal_msg = NavigateToPose.Goal()
    goal_msg.pose = PoseStamped()
    goal_msg.pose.header.frame_id = 'map'
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = GOAL_X_M
    goal_msg.pose.pose.position.y = GOAL_Y_M
    goal_msg.pose.pose.orientation.w = 1.0

    send_goal_future = node.nav_to_pose_client.send_goal_async(goal_msg)
    deadline = time.monotonic() + NAV2_ACTION_TIMEOUT_S
    while not send_goal_future.done() and time.monotonic() < deadline and not _interrupted:
        rclpy.spin_once(node, timeout_sec=0.5)

    if not send_goal_future.done():
        return None, None
    goal_handle = send_goal_future.result()
    if not goal_handle.accepted:
        return None, None
    # Record it before the (potentially long) wait below, so a Ctrl-C
    # or any other unwind during navigation still has a handle to cancel.
    node.goal_handle = goal_handle

    result_future = goal_handle.get_result_async()
    while not result_future.done() and time.monotonic() < deadline and not _interrupted:
        rclpy.spin_once(node, timeout_sec=0.5)

    if not result_future.done():
        return None, goal_handle
    return result_future.result().status, goal_handle


def _cancel_goal_and_wait(node, goal_handle):
    """Best-effort cancellation, called on any non-success exit path so
    an abandoned goal doesn't leave Nav2 autonomously driving with no
    supervisor once this script exits."""
    if goal_handle is None:
        return
    print('Cancelling navigate_to_pose goal...')
    cancel_future = goal_handle.cancel_goal_async()
    deadline = time.monotonic() + GOAL_CANCEL_TIMEOUT_S
    while not cancel_future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
    if cancel_future.done():
        print('Goal cancelled.')
    else:
        print('WARNING: goal cancellation did not confirm within '
              f'{GOAL_CANCEL_TIMEOUT_S}s - the robot may still be moving autonomously.')


def _lookup_map_to_base_link(node, timeout_s=TF_LOOKUP_TIMEOUT_S):
    """Look up the map->base_link transform via TF, spinning until it's
    available or timeout_s elapses. Returns (x, y) in the map frame, or
    None if the transform never became available.

    Checking the goal (sent in the map frame) against this - rather
    than raw /odom, which is in the odom frame, a DIFFERENT frame from
    the goal - makes this a genuinely independent, frame-correct check
    of where Nav2 actually left the robot, not just a re-read of what
    the action server itself already claimed.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and not _interrupted:
        try:
            transform = node.tf_buffer.lookup_transform('map', 'base_link', Time())
            return transform.transform.translation.x, transform.transform.translation.y
        except TransformException:
            rclpy.spin_once(node, timeout_sec=0.2)
    return None


def main():
    rclpy.init()
    # Must come after rclpy.init(), which installs the context-killing
    # handler this replaces - see _install_sigint_handler.
    _install_sigint_handler()
    node = VerifyNav2Node()

    try:
        # Wait for the first /odom before priming - a publish immediately
        # after node creation can race DDS discovery of walker_motor_driver's
        # subscriber and be silently dropped (same guard verify_motor_driver.py
        # and verify_slam.py already use).
        deadline = time.monotonic() + FIRST_ODOM_TIMEOUT_S
        while node.latest_odom is None and time.monotonic() < deadline and not _interrupted:
            rclpy.spin_once(node, timeout_sec=0.5)

        if _interrupted:
            print('FAIL: interrupted')
            return 1

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

        if _interrupted:
            print('FAIL: interrupted')
            return 1

        # Hand off to Nav2 for the return trip - don't publish /cmd_vel
        # ourselves from here on, or we'd fight with Nav2's own output.
        status, _ = _send_nav_goal_and_wait(node)

        if _interrupted:
            # The cleanup path below cancels node.goal_handle if the
            # goal was already accepted when the interrupt landed.
            print('FAIL: interrupted')
            return 1

        if status is None:
            print('FAIL: navigate_to_pose action did not complete within '
                  f'{NAV2_ACTION_TIMEOUT_S}s (or the action server/goal was '
                  'rejected) - is the Nav2 stack running and active?')
            return 1

        if status != GoalStatus.STATUS_SUCCEEDED:
            print(f'FAIL: navigate_to_pose finished with status {status}, '
                  f'expected STATUS_SUCCEEDED ({GoalStatus.STATUS_SUCCEEDED})')
            return 1

        node.goal_handle = None  # succeeded - nothing left to cancel

        final_pose = _lookup_map_to_base_link(node)
        if final_pose is None:
            print('FAIL: could not look up the map->base_link transform '
                  'for the final pose check')
            return 1

        final_x, final_y = final_pose
        distance_from_goal_m = math.hypot(final_x - GOAL_X_M, final_y - GOAL_Y_M)
        if distance_from_goal_m > GOAL_XY_TOLERANCE_M:
            print(f'FAIL: navigate_to_pose reported SUCCEEDED but final pose '
                  f'({final_x:.2f}, {final_y:.2f}) [map frame] is '
                  f'{distance_from_goal_m:.2f}m from the goal ({GOAL_X_M}, {GOAL_Y_M}), '
                  f'expected within {GOAL_XY_TOLERANCE_M}m')
            return 1

        print(f'PASS: navigate_to_pose SUCCEEDED, final pose ({final_x:.2f}, '
              f'{final_y:.2f}) [map frame] is {distance_from_goal_m:.2f}m from the goal')
        return 0
    except KeyboardInterrupt:
        print('FAIL: interrupted')
        return 1
    except ExternalShutdownException:
        # Something outside this script tore the context down (rclpy's
        # own SIGINT handler is replaced above, so this is no longer
        # the Ctrl-C path). Cancellation can't be delivered on a dead
        # context, so say so rather than implying a clean stop.
        print('FAIL: the ROS context was shut down externally - the '
              'in-flight goal could NOT be cancelled and Nav2 may still '
              'be driving the robot.')
        return 1
    finally:
        if node.goal_handle is not None:
            _cancel_goal_and_wait(node, node.goal_handle)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
