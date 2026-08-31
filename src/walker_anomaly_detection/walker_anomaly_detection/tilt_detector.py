"""Pure tilt-from-vertical estimation and sustained-tilt detection for
walker_anomaly_detection. No ROS or hardware imports - shared between
anomaly_detection_node.py and the pytest suite. See
docs/superpowers/specs/2026-08-31-walker-anomaly-detection-design.md
Sec 2.5.
"""
import math


def tilt_from_accel_deg(ax, ay, az):
    """Tilt-from-vertical angle (degrees) from the accelerometer's
    gravity component - accurate only when the robot is roughly
    stationary (no significant non-gravity acceleration). 0 degrees is
    perfectly upright (gravity entirely along az)."""
    horizontal = math.sqrt(ax ** 2 + ay ** 2)
    return math.degrees(math.atan2(horizontal, az))


class TiltDetector:
    """Tracks tilt-from-vertical (degrees) across a stream of samples.
    update() is one-shot per confirmed sustained-tilt event: returns
    True exactly on the sample where tilt has been continuously above
    tilt_threshold_deg for at least tilt_sustained_duration_s, then
    False on every subsequent sample while still tilted (no repeated
    alerts for an ongoing condition) - re-arms once tilt_deg drops back
    below threshold."""

    def __init__(self, tilt_threshold_deg, tilt_sustained_duration_s):
        self._tilt_threshold_deg = tilt_threshold_deg
        self._tilt_sustained_duration_s = tilt_sustained_duration_s
        self._tilt_start_s = None
        self._triggered = False

    def update(self, tilt_deg, now_s):
        if tilt_deg < self._tilt_threshold_deg:
            self._tilt_start_s = None
            self._triggered = False
            return False

        if self._tilt_start_s is None:
            self._tilt_start_s = now_s

        # Small epsilon tolerance for floating-point precision in timestamp arithmetic
        epsilon = 1e-9
        if not self._triggered and \
                now_s - self._tilt_start_s >= self._tilt_sustained_duration_s - epsilon:
            self._triggered = True
            return True

        return False
