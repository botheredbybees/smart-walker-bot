"""Pure 2D ray-casting against a fixed, hardcoded room - the simulated
environment walker_nav's fake LiDAR scans against. See
docs/superpowers/specs/2026-08-30-walker-nav-design.md Sec 2.2-2.3, 3
for why this is a lightweight hand-built room rather than Gazebo/a real
simulator, and why its origin coincides with the robot's odometry
origin (no offset math needed anywhere this module is used).
"""
import math

# Walls as (x1, y1, x2, y2) line segments, meters. Two connected
# rectangular rooms via a 1m doorway - enough geometry for slam_toolbox
# to build a real, if simple, map. The robot starts at (0, 0, 0), which
# is both this room's local origin and walker_motor_driver's odometry
# origin by construction.
ROOM_WALLS = (
    (-2.0, -1.5, 2.0, -1.5),   # room 1 bottom
    (-2.0, -1.5, -2.0, 1.5),   # room 1 left
    (2.0, -1.5, 2.0, 1.5),     # room 1 right
    (-2.0, 1.5, -0.5, 1.5),    # room 1 top, left of doorway
    (0.5, 1.5, 2.0, 1.5),      # room 1 top, right of doorway
    (-1.0, 1.5, -1.0, 3.5),    # room 2 left
    (1.0, 1.5, 1.0, 3.5),      # room 2 right
    (-1.0, 3.5, 1.0, 3.5),     # room 2 top
)


def cast_ray(x_m, y_m, angle_rad, max_range_m):
    """Return the distance to the nearest wall along one ray from
    (x_m, y_m) in direction angle_rad, or max_range_m if nothing is
    hit within range."""
    if max_range_m <= 0:
        raise ValueError("max_range_m must be positive")

    dx = math.cos(angle_rad)
    dy = math.sin(angle_rad)
    nearest = max_range_m
    for (x1, y1, x2, y2) in ROOM_WALLS:
        hit = _ray_segment_intersection(x_m, y_m, dx, dy, x1, y1, x2, y2)
        if hit is not None and hit < nearest:
            nearest = hit
    return nearest


def _ray_segment_intersection(px, py, dx, dy, x1, y1, x2, y2):
    """Distance from (px, py) along direction (dx, dy) - assumed a unit
    vector, so the result is a physical distance - to its intersection
    with segment (x1, y1)-(x2, y2), or None if the ray (t >= 0) doesn't
    hit the segment (0 <= u <= 1)."""
    sx = x2 - x1
    sy = y2 - y1
    denom = dx * sy - dy * sx
    if abs(denom) < 1e-12:
        return None  # parallel (including near-tangent floating point cases)
    t = ((x1 - px) * sy - (y1 - py) * sx) / denom
    u = ((x1 - px) * dy - (y1 - py) * dx) / denom
    if t >= 0 and 0.0 <= u <= 1.0:
        return t
    return None


def scan_room(x_m, y_m, theta_rad, angle_min_rad, angle_increment_rad, num_beams, max_range_m):
    """Return a list of num_beams range readings, one per beam, starting
    at theta_rad + angle_min_rad and stepping by angle_increment_rad -
    matches sensor_msgs/LaserScan's angle_min/angle_increment convention
    directly, so the ROS node can copy angle_min_rad/angle_increment_rad
    straight into the message."""
    if num_beams <= 0:
        raise ValueError("num_beams must be positive")

    ranges = []
    for i in range(num_beams):
        beam_angle = theta_rad + angle_min_rad + i * angle_increment_rad
        ranges.append(cast_ray(x_m, y_m, beam_angle, max_range_m))
    return ranges


def yaw_from_quaternion(qz, qw):
    """Recover a 2D heading (radians) from a Z-axis-only quaternion -
    the inverse of walker_motor_driver's yaw_to_quaternion. Valid only
    for a pure yaw rotation (qx=qy=0), which is all this project's
    ground robots ever produce. Lives here (not in fake_lidar_node.py)
    so it's testable with plain pytest - fake_lidar_node.py imports
    rclpy at module level, which isn't installed under this
    workstation's plain python3, so nothing meant to be pytest-tested
    can live there.
    """
    return 2.0 * math.atan2(qz, qw)
