"""walker_gait_metrics's ROS2 node: subscribes to walker_anomaly_detection's
/imu/raw_sample and walker_motor_driver's /odom, feeds both into a
GaitTracker, and publishes cumulative gait metrics on /gait_metrics on a
timer. See
docs/superpowers/specs/2026-09-01-walker-gait-metrics-design.md for the
full design.
"""
import json

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from walker_gait_metrics.gait_tracker import GaitTracker

REQUIRED_IMU_KEYS = ('ax', 'ay', 'az')


def _parse_imu_sample(data_str):
    """Parse one /imu/raw_sample JSON payload. Returns a dict with at
    least ax, ay, az on success, or None on malformed JSON, a missing
    key, or a non-numeric value - never raises. A small, deliberate
    duplicate of walker_anomaly_detection.imu_serial.parse_sample_line's
    validation, not a cross-package import - see design spec Sec 2.2."""
    try:
        data = json.loads(data_str)
    except (ValueError, TypeError):
        return None
    if not isinstance(data, dict):
        return None
    if not all(key in data for key in REQUIRED_IMU_KEYS):
        return None
    if not all(
        isinstance(data.get(key), (int, float)) and not isinstance(data.get(key), bool)
        for key in REQUIRED_IMU_KEYS
    ):
        return None
    return data


class GaitMetricsNode(Node):
    def __init__(self):
        super().__init__('walker_gait_metrics')

        self.declare_parameter('step_threshold_g', 1.2)
        self.declare_parameter('min_step_interval_s', 0.3)
        self.declare_parameter('publish_rate_hz', 1.0)

        step_threshold_g = self.get_parameter('step_threshold_g').value
        min_step_interval_s = self.get_parameter('min_step_interval_s').value
        publish_rate_hz = self.get_parameter('publish_rate_hz').value

        if step_threshold_g <= 0:
            raise ValueError("step_threshold_g must be positive")
        if min_step_interval_s <= 0:
            raise ValueError("min_step_interval_s must be positive")
        if publish_rate_hz <= 0:
            raise ValueError("publish_rate_hz must be positive")

        self._tracker = GaitTracker(step_threshold_g, min_step_interval_s)

        self.create_subscription(String, '/imu/raw_sample', self._on_imu_sample, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self._metrics_pub = self.create_publisher(String, '/gait_metrics', 10)
        self._timer = self.create_timer(1.0 / publish_rate_hz, self._on_timer)

    def _on_imu_sample(self, msg):
        sample = _parse_imu_sample(msg.data)
        if sample is None:
            self.get_logger().warn(
                'Ignoring malformed /imu/raw_sample payload.', throttle_duration_sec=5.0,
            )
            return
        now_s = self.get_clock().now().nanoseconds / 1e9
        self._tracker.on_imu_sample(sample, now_s)

    def _on_odom(self, msg):
        x_m = msg.pose.pose.position.x
        y_m = msg.pose.pose.position.y
        self._tracker.on_odom_pose(x_m, y_m)

    def _on_timer(self):
        payload = json.dumps({
            'step_count': self._tracker.step_count,
            'total_distance_m': self._tracker.total_distance_m,
            'avg_step_length_m': self._tracker.avg_step_length_m,
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
        })
        self._metrics_pub.publish(String(data=payload))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GaitMetricsNode()
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
