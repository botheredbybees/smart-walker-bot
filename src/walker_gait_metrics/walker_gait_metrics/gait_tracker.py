"""Pure cumulative gait-metrics tracking for walker_gait_metrics. No ROS
or hardware imports - shared between gait_metrics_node.py and the
pytest suite. See
docs/superpowers/specs/2026-09-01-walker-gait-metrics-design.md Sec 2.6.
"""
import math

from walker_gait_metrics.step_counter import StepCounter


class GaitTracker:
    """Combines step counting (from IMU samples) with distance
    accumulation (from odometry poses) into cumulative gait metrics:
    step_count, total_distance_m, and avg_step_length_m =
    total_distance_m / step_count (0.0 while step_count is 0, never a
    ZeroDivisionError)."""

    def __init__(self, step_threshold_g, min_step_interval_s):
        self._step_counter = StepCounter(step_threshold_g, min_step_interval_s)
        self._step_count = 0
        self._total_distance_m = 0.0
        self._last_pose = None

    def on_imu_sample(self, sample, now_s):
        """sample: dict with at least ax, ay, az (g). Feeds accelerometer
        magnitude into the internal StepCounter; increments step_count
        on a detected step."""
        accel_magnitude_g = math.sqrt(sample['ax'] ** 2 + sample['ay'] ** 2 + sample['az'] ** 2)
        if self._step_counter.update(accel_magnitude_g, now_s):
            self._step_count += 1

    def on_odom_pose(self, x_m, y_m):
        """Accumulates total_distance_m from the previous call's pose.
        The first call has no previous pose to diff against, so it only
        seeds the starting point and adds no distance."""
        if self._last_pose is not None:
            prev_x, prev_y = self._last_pose
            self._total_distance_m += math.hypot(x_m - prev_x, y_m - prev_y)
        self._last_pose = (x_m, y_m)

    @property
    def step_count(self):
        return self._step_count

    @property
    def total_distance_m(self):
        return self._total_distance_m

    @property
    def avg_step_length_m(self):
        if self._step_count == 0:
            return 0.0
        return self._total_distance_m / self._step_count
