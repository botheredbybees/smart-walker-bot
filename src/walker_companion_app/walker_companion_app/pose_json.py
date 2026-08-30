"""Pure pose extraction for walker_companion_app: converts a planar
robot's (x, y, quaternion z/w) into a JSON-serializable pose dict with a
2D heading. No ROS import - the node extracts these primitives from a
nav_msgs/Odometry message before calling this. See
docs/superpowers/specs/2026-08-30-walker-companion-app-design.md Sec 2.9.
"""
import math


def yaw_from_quaternion(qz, qw):
    """Recover a 2D heading (radians) from a Z-axis-only quaternion -
    valid only for a pure yaw rotation (qx=qy=0), which is all this
    project's ground robots ever produce. Same formula
    walker_nav/walker_nav/room_map.py's own yaw_from_quaternion uses,
    implemented independently here rather than imported across the
    package boundary (see design spec Sec 2.9)."""
    return 2.0 * math.atan2(qz, qw)


def pose_to_json(x, y, qz, qw):
    return {'x': x, 'y': y, 'theta': yaw_from_quaternion(qz, qw)}
