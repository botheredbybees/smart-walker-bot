"""Abstract interface separating walker_motor_driver's ROS2 node from how
wheel speeds actually get applied and measured - the sim/real boundary
described in docs/superpowers/specs/2026-08-30-walker-motor-driver-design.md
Sec 2.3. SimMotorBackend (sim_backend.py) is the only implementation
until hardware bring-up adds a GpioMotorBackend; motor_driver_node.py's
control logic doesn't change when that happens (see the design spec for
what "doesn't change" actually covers).
"""


class MotorBackend:
    def apply_wheel_speeds(self, left_rad_s, right_rad_s):
        """Command target wheel angular speeds, in radians/second."""
        raise NotImplementedError

    def read_wheel_deltas(self, now_s):
        """Return (left_rad, right_rad) wheel rotation since the last
        call, given the current time now_s (seconds, monotonic)."""
        raise NotImplementedError

    def stop(self):
        """De-energize the motors. Called on node shutdown (including a
        clean Ctrl-C) so a real backend doesn't leave motors driving after
        the process exits - SimMotorBackend's implementation is a no-op
        since there's nothing physical to de-energize."""
        raise NotImplementedError
