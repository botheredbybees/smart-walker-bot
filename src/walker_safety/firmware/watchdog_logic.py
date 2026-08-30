"""Pure heartbeat-timeout logic for the walker_safety watchdog.

No hardware or MicroPython-specific imports here — this module is
shared between the Pico firmware (main.py) and the desktop pytest
suite, so the same logic that runs on real hardware is exactly what
the tests exercise.
"""


class Watchdog:
    """Tracks whether motors should be cut based on heartbeat recency.

    Fails safe: before the first heartbeat is received, is_tripped()
    always returns True (motors disabled). This matches README.md
    Sec 5.4's requirement that the watchdog halts motors on any loss
    of heartbeat signal, including "never started."
    """

    def __init__(self, timeout_s):
        if timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        self.timeout_s = timeout_s
        self._last_heartbeat_s = None

    def on_heartbeat(self, now_s):
        """Record a heartbeat received at now_s (seconds, monotonic)."""
        self._last_heartbeat_s = now_s

    def is_tripped(self, now_s):
        """Return True if motors should be cut given the current time."""
        if self._last_heartbeat_s is None:
            return True
        elapsed_s = now_s - self._last_heartbeat_s
        return elapsed_s >= self.timeout_s
