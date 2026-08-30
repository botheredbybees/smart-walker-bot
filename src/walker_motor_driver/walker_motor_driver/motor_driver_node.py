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
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from walker_motor_driver.diff_drive_kinematics import (
    OdometryTracker,
    twist_to_wheel_speeds,
    yaw_to_quaternion,
)
from walker_motor_driver.sim_backend import SimMotorBackend


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('walker_motor_driver')

        self.declare_parameter('wheel_radius_m', 0.03)
        self.declare_parameter('wheel_separation_m', 0.2)
        self.declare_parameter('max_wheel_speed_rad_s', 10.0)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('backend', 'sim')

        self._wheel_radius_m = self.get_parameter('wheel_radius_m').value
        self._wheel_separation_m = self.get_parameter('wheel_separation_m').value
        self._max_wheel_speed_rad_s = self.get_parameter('max_wheel_speed_rad_s').value
        publish_rate_hz = self.get_parameter('publish_rate_hz').value
        backend_name = self.get_parameter('backend').value

        now_s = self.get_clock().now().nanoseconds / 1e9

        if backend_name == 'sim':
            self._backend = SimMotorBackend(now_s=now_s)
        else:
            raise ValueError(
                f"Unknown backend '{backend_name}' - only 'sim' is implemented; "
                "a 'real' GPIO backend is added at the hardware bring-up checkpoint."
            )

        self._odometry = OdometryTracker(self._wheel_radius_m, self._wheel_separation_m)

        self._cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._last_update_s = now_s
        self._timer = self.create_timer(1.0 / publish_rate_hz, self._on_timer)

    def _on_cmd_vel(self, msg):
        left_rad_s, right_rad_s = twist_to_wheel_speeds(
            msg.linear.x, msg.angular.z, self._wheel_radius_m, self._wheel_separation_m
        )
        left_rad_s = max(-self._max_wheel_speed_rad_s, min(self._max_wheel_speed_rad_s, left_rad_s))
        right_rad_s = max(-self._max_wheel_speed_rad_s, min(self._max_wheel_speed_rad_s, right_rad_s))
        self._backend.apply_wheel_speeds(left_rad_s, right_rad_s)

    def _on_timer(self):
        now_s = self.get_clock().now().nanoseconds / 1e9
        left_delta_rad, right_delta_rad = self._backend.read_wheel_deltas(now_s)
        dt_s = now_s - self._last_update_s
        self._last_update_s = now_s
        if dt_s <= 0:
            return
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
        odom_msg.twist.twist.linear.x = linear_x_m_s
        odom_msg.twist.twist.angular.z = angular_z_rad_s
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
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
