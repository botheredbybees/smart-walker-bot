"""Abstract interface separating walker_motor_driver's ROS2 node from how
wheel speeds actually get applied and measured - the sim/real boundary
described in docs/superpowers/specs/2026-08-30-walker-motor-driver-design.md
Sec 2.3. SimMotorBackend (sim_backend.py) is the only implementation
until hardware bring-up adds a GpioMotorBackend; motor_driver_node.py
doesn't change when that happens.
"""


class MotorBackend:
    def apply_wheel_speeds(self, left_rad_s, right_rad_s):
        """Command target wheel angular speeds, in radians/second."""
        raise NotImplementedError

    def read_wheel_deltas(self, now_s):
        """Return (left_rad, right_rad) wheel rotation since the last
        call, given the current time now_s (seconds, monotonic)."""
        raise NotImplementedError
