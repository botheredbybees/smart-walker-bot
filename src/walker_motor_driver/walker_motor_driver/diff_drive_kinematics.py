"""Pure differential-drive kinematics for walker_motor_driver.

No ROS or hardware imports here - this module is shared between the
rclpy node (motor_driver_node.py) and the desktop pytest suite, so the
same math that runs live is exactly what the tests exercise. See
docs/superpowers/specs/2026-08-30-walker-motor-driver-design.md Sec 2.3
for why the sim/real boundary lives one layer below this module, in
MotorBackend, not here.
"""
import math


def twist_to_wheel_speeds(linear_x_m_s, angular_z_rad_s, wheel_radius_m, wheel_separation_m):
    """Convert a body-frame Twist command into per-wheel angular speeds (rad/s)."""
    if wheel_radius_m <= 0:
        raise ValueError("wheel_radius_m must be positive")
    if wheel_separation_m <= 0:
        raise ValueError("wheel_separation_m must be positive")

    left_m_s = linear_x_m_s - (angular_z_rad_s * wheel_separation_m / 2.0)
    right_m_s = linear_x_m_s + (angular_z_rad_s * wheel_separation_m / 2.0)
    left_rad_s = left_m_s / wheel_radius_m
    right_rad_s = right_m_s / wheel_radius_m
    return left_rad_s, right_rad_s


class OdometryTracker:
    """Integrates wheel-rotation deltas into a 2D robot pose (x, y, theta).

    Pure differential-drive odometry: no ROS, no hardware, fully
    deterministic from its inputs - testable without rclpy or a backend.
    """

    def __init__(self, wheel_radius_m, wheel_separation_m):
        if wheel_radius_m <= 0:
            raise ValueError("wheel_radius_m must be positive")
        if wheel_separation_m <= 0:
            raise ValueError("wheel_separation_m must be positive")
        self._wheel_radius_m = wheel_radius_m
        self._wheel_separation_m = wheel_separation_m
        self.x_m = 0.0
        self.y_m = 0.0
        self.theta_rad = 0.0

    def update(self, left_wheel_delta_rad, right_wheel_delta_rad, dt_s):
        """Integrate one timestep of wheel rotation into the tracked pose.

        Returns (linear_x_m_s, angular_z_rad_s), the instantaneous
        body-frame velocity implied by the wheel motion this step -
        callers publish this directly as an Odometry message's twist.
        """
        if dt_s <= 0:
            raise ValueError("dt_s must be positive")

        left_dist_m = left_wheel_delta_rad * self._wheel_radius_m
        right_dist_m = right_wheel_delta_rad * self._wheel_radius_m
        center_dist_m = (left_dist_m + right_dist_m) / 2.0
        delta_theta_rad = (right_dist_m - left_dist_m) / self._wheel_separation_m

        # Integrate at the midpoint heading for better accuracy over a step.
        mid_theta_rad = self.theta_rad + delta_theta_rad / 2.0
        self.x_m += center_dist_m * math.cos(mid_theta_rad)
        self.y_m += center_dist_m * math.sin(mid_theta_rad)
        self.theta_rad += delta_theta_rad

        linear_x_m_s = center_dist_m / dt_s
        angular_z_rad_s = delta_theta_rad / dt_s
        return linear_x_m_s, angular_z_rad_s


def yaw_to_quaternion(yaw_rad):
    """Convert a 2D heading (radians) into an (x, y, z, w) quaternion,
    a rotation about the Z axis only - sufficient for a ground robot's
    Odometry/TF messages."""
    half = yaw_rad / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))
