"""Pure step-detection for walker_gait_metrics. No ROS or hardware
imports - shared between gait_metrics_node.py and the pytest suite. See
docs/superpowers/specs/2026-09-01-walker-gait-metrics-design.md Sec 2.4.
"""


class StepCounter:
    """Tracks accelerometer magnitude (g) across a stream of samples.
    Detects a step whenever magnitude crosses above step_threshold_g,
    debounced by min_step_interval_s so one footstep's impact-and-settle
    isn't counted twice."""

    def __init__(self, step_threshold_g, min_step_interval_s):
        self._step_threshold_g = step_threshold_g
        self._min_step_interval_s = min_step_interval_s
        self._last_step_s = None

    def update(self, accel_magnitude_g, now_s):
        """Call once per sample. Returns True exactly on the sample
        confirming a new step, False otherwise."""
        if accel_magnitude_g < self._step_threshold_g:
            return False
        # Small epsilon tolerance for floating-point precision in timestamp arithmetic
        epsilon = 1e-9
        if self._last_step_s is not None and now_s - self._last_step_s < self._min_step_interval_s - epsilon:
            return False
        self._last_step_s = now_s
        return True
