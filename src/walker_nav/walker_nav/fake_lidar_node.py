"""walker_nav's fake LiDAR node: subscribes walker_motor_driver's
/odom, publishes a sensor_msgs/LaserScan on /scan built from the
robot's real, live-tracked pose against the fixed room in room_map.py.
See docs/superpowers/specs/2026-08-30-walker-nav-design.md Sec 2.4.
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from walker_nav.room_map import scan_room, yaw_from_quaternion


class FakeLidarNode(Node):
    def __init__(self):
        super().__init__('walker_nav_fake_lidar')

        self.declare_parameter('num_beams', 360)
        self.declare_parameter('max_range_m', 8.0)
        self.declare_parameter('scan_rate_hz', 5.0)

        self._num_beams = self.get_parameter('num_beams').value
        self._max_range_m = self.get_parameter('max_range_m').value
        scan_rate_hz = self.get_parameter('scan_rate_hz').value

        if self._num_beams <= 0:
            raise ValueError("num_beams must be positive")
        if self._max_range_m <= 0:
            raise ValueError("max_range_m must be positive")
        if scan_rate_hz <= 0:
            raise ValueError("scan_rate_hz must be positive")

        self._angle_min_rad = -math.pi
        self._angle_increment_rad = (2.0 * math.pi) / self._num_beams

        # Defaults match the room's origin (spec Sec 2.3) - if /odom
        # never arrives, the node still publishes a valid, if
        # stationary-at-the-origin, scan rather than erroring.
        self._x_m = 0.0
        self._y_m = 0.0
        self._theta_rad = 0.0

        self._odom_sub = self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self._scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self._timer = self.create_timer(1.0 / scan_rate_hz, self._on_timer)

    def _on_odom(self, msg):
        self._x_m = msg.pose.pose.position.x
        self._y_m = msg.pose.pose.position.y
        self._theta_rad = yaw_from_quaternion(
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        )

    def _on_timer(self):
        ranges = scan_room(
            self._x_m, self._y_m, self._theta_rad,
            self._angle_min_rad, self._angle_increment_rad,
            self._num_beams, self._max_range_m,
        )
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.angle_min = self._angle_min_rad
        msg.angle_max = self._angle_min_rad + (self._num_beams - 1) * self._angle_increment_rad
        msg.angle_increment = self._angle_increment_rad
        msg.range_min = 0.05
        msg.range_max = self._max_range_m
        msg.ranges = ranges
        self._scan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FakeLidarNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
