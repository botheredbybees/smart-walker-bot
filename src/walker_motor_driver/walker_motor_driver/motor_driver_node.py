"""walker_motor_driver's ROS2 node: subscribes /cmd_vel, publishes /odom
and an odom->base_link TF, driving a MotorBackend (sim_backend.py's
SimMotorBackend for now) in between. See
docs/superpowers/specs/2026-08-30-walker-motor-driver-design.md for the
full design.

Unlike walker_safety's main.py, this node runs on ordinary desktop
Python + rclpy - there's no missing-hardware-module problem here, so
it's verified by actually running it (see tools/verify_motor_driver.py)
rather than requiring physical hardware.
"""
import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from walker_motor_driver.diff_drive_kinematics import (
    OdometryTracker,
    clamp_wheel_speeds,
    twist_to_wheel_speeds,
    yaw_to_quaternion,
)
from walker_motor_driver.sim_backend import SimMotorBackend

# Placeholder Odometry covariance diagonals (order: x, y, z, roll, pitch, yaw
# for pose; vx, vy, vz, vroll, vpitch, vyaw for twist). A 2D ground robot has
# no real information about z/roll/pitch, so those get a large "unknown"
# variance rather than 0.0 (all-zero covariance reads as "perfectly known" to
# consumers like robot_localization's EKF, which is never true). Like the
# physical parameters below, these are placeholders - recalibrate at bring-up
# once real sensor noise can be measured.
ODOM_POSE_COVARIANCE_DIAGONAL = (0.01, 0.01, 1e6, 1e6, 1e6, 0.05)
ODOM_TWIST_COVARIANCE_DIAGONAL = (0.01, 1e6, 1e6, 1e6, 1e6, 0.05)


def _set_diagonal_covariance(covariance_array, diagonal_values):
    for i, value in enumerate(diagonal_values):
        covariance_array[i * 6 + i] = value


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('walker_motor_driver')

        self.declare_parameter('wheel_radius_m', 0.03)
        self.declare_parameter('wheel_separation_m', 0.2)
        self.declare_parameter('max_wheel_speed_rad_s', 10.0)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('cmd_vel_timeout_s', 0.5)
        self.declare_parameter('backend', 'sim')

        self._wheel_radius_m = self.get_parameter('wheel_radius_m').value
        self._wheel_separation_m = self.get_parameter('wheel_separation_m').value
        self._max_wheel_speed_rad_s = self.get_parameter('max_wheel_speed_rad_s').value
        self._publish_rate_hz = self.get_parameter('publish_rate_hz').value
        self._cmd_vel_timeout_s = self.get_parameter('cmd_vel_timeout_s').value
        backend_name = self.get_parameter('backend').value

        if self._publish_rate_hz <= 0:
            raise ValueError("publish_rate_hz must be positive")

        # A steady (monotonic) clock for backend/odometry timing - never
        # steps backward or jumps forward on an NTP correction or manual
        # clock change the way the ROS/system clock can. MotorBackend's
        # contract (motor_backend.py) explicitly requires a monotonic
        # now_s; using the wall clock here previously meant a backwards
        # step raised inside a timer callback (killing the node) and a
        # forward step could integrate an implausible distance in one
        # tick. Message timestamps still use the ROS/wall clock below,
        # which is correct - only integration timing needs to be steady.
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        now_s = self._steady_clock.now().nanoseconds / 1e9

        if backend_name == 'sim':
            self._backend = SimMotorBackend(now_s=now_s)
        else:
            raise ValueError(
                f"Unknown backend '{backend_name}' - only 'sim' is implemented; "
                "a 'real' GPIO backend is added at the hardware bring-up checkpoint."
            )

        self._odometry = OdometryTracker(self._wheel_radius_m, self._wheel_separation_m)

        self._cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._last_update_s = now_s
        self._last_cmd_vel_s = now_s
        self._timer = self.create_timer(1.0 / self._publish_rate_hz, self._on_timer)

    def _on_cmd_vel(self, msg):
        left_rad_s, right_rad_s = twist_to_wheel_speeds(
            msg.linear.x, msg.angular.z, self._wheel_radius_m, self._wheel_separation_m
        )
        left_rad_s, right_rad_s = clamp_wheel_speeds(
            left_rad_s, right_rad_s, self._max_wheel_speed_rad_s
        )
        self._backend.apply_wheel_speeds(left_rad_s, right_rad_s)
        self._last_cmd_vel_s = self._steady_clock.now().nanoseconds / 1e9

    def _on_timer(self):
        now_s = self._steady_clock.now().nanoseconds / 1e9

        if now_s - self._last_cmd_vel_s > self._cmd_vel_timeout_s:
            # No /cmd_vel received recently (commander crashed, network
            # dropped, or none ever sent) - stop rather than keep applying
            # the last-known speed forever. This is a local, self-contained
            # safety behavior, distinct from (and not a substitute for) the
            # hardware E-stop/Pico watchdog in walker_safety - see design
            # spec Sec 2.6.
            self._backend.apply_wheel_speeds(0.0, 0.0)

        dt_s = now_s - self._last_update_s
        if dt_s <= 0:
            return

        # Cap how much time a single tick can cover, so a scheduling stall
        # can't integrate an implausible distance in one step. The steady
        # clock above already prevents clock-jump causes of this; this is
        # defense in depth against a stalled process/scheduler.
        max_dt_s = 5.0 / self._publish_rate_hz
        if dt_s > max_dt_s:
            now_s = self._last_update_s + max_dt_s
            dt_s = max_dt_s

        left_delta_rad, right_delta_rad = self._backend.read_wheel_deltas(now_s)
        self._last_update_s = now_s
        linear_x_m_s, angular_z_rad_s = self._odometry.update(left_delta_rad, right_delta_rad, dt_s)
        self._publish_odometry(linear_x_m_s, angular_z_rad_s)

    def _publish_odometry(self, linear_x_m_s, angular_z_rad_s):
        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quaternion(self._odometry.theta_rad)

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self._odometry.x_m
        odom_msg.pose.pose.position.y = self._odometry.y_m
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        _set_diagonal_covariance(odom_msg.pose.covariance, ODOM_POSE_COVARIANCE_DIAGONAL)
        odom_msg.twist.twist.linear.x = linear_x_m_s
        odom_msg.twist.twist.angular.z = angular_z_rad_s
        _set_diagonal_covariance(odom_msg.twist.covariance, ODOM_TWIST_COVARIANCE_DIAGONAL)
        self._odom_pub.publish(odom_msg)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = self._odometry.x_m
        tf_msg.transform.translation.y = self._odometry.y_m
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MotorDriverNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # A clean Ctrl-C (or the SIGINT `ros2 launch` sends on shutdown)
        # is a normal exit, not an error - swallow it so the process ends
        # quietly with status 0 after stop() runs below, rather than
        # printing a traceback.
        pass
    finally:
        if node is not None:
            node._backend.stop()
            node.destroy_node()
        # rclpy's own signal handler has usually already shut the context
        # down by the time a Ctrl-C reaches here; calling shutdown() again
        # raises RCLError, which would turn a clean exit into a failure.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
