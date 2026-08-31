"""walker_anomaly_detection's ROS2 node: reads IMU samples from a
serial-connected ESP32 on a background thread, runs FallDetector and
TiltDetector against the stream, and publishes /anomaly_detected on a
detected event. See
docs/superpowers/specs/2026-08-31-walker-anomaly-detection-design.md
for the full design.
"""
import json
import math
import threading

import rclpy
import serial
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from walker_anomaly_detection.fall_detector import FallDetector
from walker_anomaly_detection.imu_serial import read_samples
from walker_anomaly_detection.tilt_detector import TiltDetector, tilt_from_accel_deg


class AnomalyDetectionNode(Node):
    def __init__(self):
        super().__init__('walker_anomaly_detection')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('free_fall_threshold_g', 0.3)
        self.declare_parameter('free_fall_min_duration_s', 0.05)
        self.declare_parameter('impact_threshold_g', 2.0)
        self.declare_parameter('impact_window_s', 0.5)
        self.declare_parameter('tilt_threshold_deg', 45.0)
        self.declare_parameter('tilt_sustained_duration_s', 3.0)

        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value

        self._fall_detector = FallDetector(
            free_fall_threshold_g=self.get_parameter('free_fall_threshold_g').value,
            free_fall_min_duration_s=self.get_parameter('free_fall_min_duration_s').value,
            impact_threshold_g=self.get_parameter('impact_threshold_g').value,
            impact_window_s=self.get_parameter('impact_window_s').value,
        )
        self._tilt_detector = TiltDetector(
            tilt_threshold_deg=self.get_parameter('tilt_threshold_deg').value,
            tilt_sustained_duration_s=self.get_parameter('tilt_sustained_duration_s').value,
        )

        self._alert_pub = self.create_publisher(String, '/anomaly_detected', 10)

        self._serial_conn = serial.Serial(serial_port, baud_rate, timeout=1.0)
        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()

    def _read_loop(self):
        try:
            read_samples(self._serial_conn, self._on_sample)
        except Exception as e:
            self.get_logger().error(
                f'IMU serial read loop terminated unexpectedly: {e}. '
                'Anomaly detection has stopped - the node needs to be restarted.'
            )

    def _on_sample(self, sample):
        now_s = self.get_clock().now().nanoseconds / 1e9
        accel_magnitude_g = math.sqrt(
            sample['ax'] ** 2 + sample['ay'] ** 2 + sample['az'] ** 2
        )

        if self._fall_detector.update(accel_magnitude_g, now_s):
            self._publish_alert('fall')

        tilt_deg = tilt_from_accel_deg(sample['ax'], sample['ay'], sample['az'])
        if self._tilt_detector.update(tilt_deg, now_s):
            self._publish_alert('tilt')

    def _publish_alert(self, alert_type):
        payload = json.dumps({
            'type': alert_type,
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
        })
        self._alert_pub.publish(String(data=payload))
        self.get_logger().warning(f'Anomaly detected: {alert_type}')

    def stop(self):
        try:
            self._serial_conn.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AnomalyDetectionNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
