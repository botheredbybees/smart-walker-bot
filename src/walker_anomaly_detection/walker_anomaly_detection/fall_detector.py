"""Pure free-fall + impact fall detection for walker_anomaly_detection.
No ROS or hardware imports - shared between anomaly_detection_node.py
and the pytest suite. See
docs/superpowers/specs/2026-08-31-walker-anomaly-detection-design.md
Sec 2.4.
"""


class FallDetector:
    """Tracks accelerometer magnitude (g) across a stream of samples.
    Enters a "possible fall" state on a sustained drop below
    free_fall_threshold_g; while in that state, watches for a
    subsequent impact (magnitude above impact_threshold_g) within
    impact_window_s of the free-fall ending. update() is one-shot per
    confirmed fall - it resets its own state on the sample that
    triggers, so a later, independent fall can be detected too."""

    def __init__(self, free_fall_threshold_g, free_fall_min_duration_s,
                 impact_threshold_g, impact_window_s):
        self._free_fall_threshold_g = free_fall_threshold_g
        self._free_fall_min_duration_s = free_fall_min_duration_s
        self._impact_threshold_g = impact_threshold_g
        self._impact_window_s = impact_window_s
        self._reset()

    def update(self, accel_magnitude_g, now_s):
        """Call once per sample. Returns True exactly on the sample
        where a fall is confirmed, False otherwise."""
        if accel_magnitude_g < self._free_fall_threshold_g:
            if self._free_fall_start_s is None:
                self._free_fall_start_s = now_s
            if not self._free_fall_confirmed and \
                    now_s - self._free_fall_start_s >= self._free_fall_min_duration_s:
                self._free_fall_confirmed = True
            return False

        if self._free_fall_confirmed:
            if self._free_fall_end_s is None:
                self._free_fall_end_s = now_s
            if accel_magnitude_g >= self._impact_threshold_g and \
                    now_s - self._free_fall_end_s <= self._impact_window_s:
                self._reset()
                return True
            if now_s - self._free_fall_end_s > self._impact_window_s:
                self._reset()
            return False

        self._reset()
        return False

    def _reset(self):
        self._free_fall_start_s = None
        self._free_fall_confirmed = False
        self._free_fall_end_s = None
