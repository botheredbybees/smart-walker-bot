"""Idealized kinematic motor simulator - the roadmap design's
"lightweight, not physics-realistic" simulator
(docs/superpowers/specs/2026-08-30-phase1-roadmap-design.md Sec 2.4),
applied to walker_motor_driver specifically.
"""
from walker_motor_driver.motor_backend import MotorBackend


class SimMotorBackend(MotorBackend):
    """Commanded wheel speed is achieved instantly, with no motor
    dynamics or slip. Time is passed in explicitly (now_s) rather than
    read from a wall clock, so tests are deterministic and don't need
    to sleep - matches the pattern watchdog_logic.py's Watchdog uses in
    walker_safety, for the same reason.
    """

    def __init__(self, now_s):
        self._left_rad_s = 0.0
        self._right_rad_s = 0.0
        self._last_read_s = now_s

    def apply_wheel_speeds(self, left_rad_s, right_rad_s):
        self._left_rad_s = left_rad_s
        self._right_rad_s = right_rad_s

    def read_wheel_deltas(self, now_s):
        dt_s = now_s - self._last_read_s
        if dt_s < 0:
            raise ValueError("now_s must not go backwards")
        left_delta_rad = self._left_rad_s * dt_s
        right_delta_rad = self._right_rad_s * dt_s
        self._last_read_s = now_s
        return left_delta_rad, right_delta_rad
