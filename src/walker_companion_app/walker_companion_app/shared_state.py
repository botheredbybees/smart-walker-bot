"""Thread-safe shared state for walker_companion_app: written by rclpy
subscription callbacks (one thread), read by the HTTP server threads.
No ROS import - the node extracts primitives from messages before
calling these setters. This class is the sole thread-safety boundary
for all shared state, including the conversation log: ConversationLog
itself has no internal locking, and is only ever touched here, under
this class's one lock. See
docs/superpowers/specs/2026-08-30-walker-companion-app-design.md Sec
2.3, 2.10.
"""
import threading


class SharedState:
    def __init__(self, conversation_log):
        self._lock = threading.Lock()
        self._conversation_log = conversation_log
        self._pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self._nav_status = 'idle'
        self._gait = {'step_count': 0, 'total_distance_m': 0.0, 'avg_step_length_m': 0.0}
        self._map = {
            'width': 0, 'height': 0, 'resolution': 0.0,
            'origin_x': 0.0, 'origin_y': 0.0, 'data': [],
        }

    def set_pose(self, pose):
        with self._lock:
            self._pose = dict(pose)

    def set_nav_status(self, label):
        with self._lock:
            self._nav_status = label

    def set_gait_metrics(self, gait):
        with self._lock:
            self._gait = dict(gait)

    def set_map(self, grid):
        with self._lock:
            self._map = {**grid, 'data': list(grid['data'])}

    def add_conversation_entry(self, role, text, timestamp):
        with self._lock:
            self._conversation_log.append(role, text, timestamp)

    def status_snapshot(self, timestamp):
        with self._lock:
            return {
                'pose': dict(self._pose),
                'nav_status': self._nav_status,
                'gait': dict(self._gait),
                'timestamp': timestamp,
            }

    def map_snapshot(self):
        with self._lock:
            return {**self._map, 'data': list(self._map['data'])}

    def conversation_snapshot(self):
        with self._lock:
            return self._conversation_log.entries()
