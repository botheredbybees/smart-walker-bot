"""Pure conversion of primitive occupancy-grid fields to a JSON-serializable
dict, for walker_companion_app's /api/map endpoint. No ROS import - the
node extracts these primitives from a nav_msgs/OccupancyGrid message
before calling this. See
docs/superpowers/specs/2026-08-30-walker-companion-app-design.md Sec 2.6.
"""


def grid_to_json(width, height, resolution, origin_x, origin_y, data):
    return {
        'width': width,
        'height': height,
        'resolution': resolution,
        'origin_x': origin_x,
        'origin_y': origin_y,
        'data': list(data),
    }
