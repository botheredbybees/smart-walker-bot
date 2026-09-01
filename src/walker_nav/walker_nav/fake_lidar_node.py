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

from walker_nav.room_map import fov_to_scan_params, scan_room, yaw_from_quaternion


class FakeLidarNode(Node):
    def __init__(self):
        super().__init__('walker_nav_fake_lidar')

        self.declare_parameter('num_beams', 360)
        self.declare_parameter('max_range_m', 8.0)
        self.declare_parameter('scan_rate_hz', 5.0)
        self.declare_parameter('fov_deg', 360.0)

        self._num_beams = self.get_parameter('num_beams').value
        self._max_range_m = self.get_parameter('max_range_m').value
        scan_rate_hz = self.get_parameter('scan_rate_hz').value
        fov_deg = self.get_parameter('fov_deg').value

        if self._num_beams <= 0:
            raise ValueError("num_beams must be positive")
        if self._max_range_m <= 0:
            raise ValueError("max_range_m must be positive")
        if scan_rate_hz <= 0:
            raise ValueError("scan_rate_hz must be positive")

        # fov_deg's own >0 / num_beams>=2-for-narrow-arc validation lives in
        # fov_to_scan_params itself (walker_nav.room_map) - see its docstring.
        self._angle_min_rad, self._angle_increment_rad = fov_to_scan_params(fov_deg, self._num_beams)

        # Defaults match the room's origin (spec Sec 2.3) - if /odom
        # never arrives, the node still publishes a valid, if
        # stationary-at-the-origin, scan rather than erroring.
        self._x_m = 0.0
        self._y_m = 0.0
        self._theta_rad = 0.0
        self._odom_stamp = None
        self._received_first_odom = False

        self._odom_sub = self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self._scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self._timer = self.create_timer(1.0 / scan_rate_hz, self._on_timer)

    def _on_odom(self, msg):
        x_m = msg.pose.pose.position.x
        y_m = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        if not (math.isfinite(x_m) and math.isfinite(y_m) and math.isfinite(qz) and math.isfinite(qw)):
            # A malformed pose must never corrupt the room simulation or
            # get ray-cast against - keep the last known-good pose
            # instead. A non-finite pose would otherwise make every beam
            # report max range, which slam_toolbox reads as a long "all
            # clear" ray straight through already-mapped walls.
            self.get_logger().warn(
                'Ignoring non-finite /odom pose - keeping last known pose.',
                throttle_duration_sec=5.0,
            )
            return

        if not self._received_first_odom:
            self._received_first_odom = True
            distance_from_origin_m = math.hypot(x_m, y_m)
            if distance_from_origin_m > 0.5:
                self.get_logger().warn(
                    f'First /odom pose ({x_m:.2f}, {y_m:.2f}) is '
                    f'{distance_from_origin_m:.2f}m from the origin - '
                    'room_map.py assumes the room origin coincides with the '
                    'odometry origin (design spec Sec 2.3); that assumption '
                    'may no longer hold.'
                )

        self._x_m = x_m
        self._y_m = y_m
        self._theta_rad = yaw_from_quaternion(qz, qw)
        self._odom_stamp = msg.header.stamp

    def _on_timer(self):
        ranges = scan_room(
            self._x_m, self._y_m, self._theta_rad,
            self._angle_min_rad, self._angle_increment_rad,
            self._num_beams, self._max_range_m,
        )
        msg = LaserScan()
        # Stamp with the pose's own odometry time when available, not
        # "now" - the pose can be up to one /odom period old by the time
        # this timer fires, and stamping with the pose's real time keeps
        # the scan and the odom->base_link TF lookup slam_toolbox does
        # for it consistent.
        msg.header.stamp = self._odom_stamp if self._odom_stamp is not None else self.get_clock().now().to_msg()
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
