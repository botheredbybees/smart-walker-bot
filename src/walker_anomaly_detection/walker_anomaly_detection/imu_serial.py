"""Pure/near-pure serial-sample handling for walker_anomaly_detection.
parse_sample_line is pure - no ROS or hardware imports - shared between
anomaly_detection_node.py and the pytest suite. read_samples is a thin
blocking read loop, not itself pure, but takes any object with a
readline() method so it's testable with a fake in-memory serial double.
See docs/superpowers/specs/2026-08-31-walker-anomaly-detection-design.md
Sec 2.3.
"""
import json

REQUIRED_KEYS = ('ax', 'ay', 'az', 'gx', 'gy', 'gz', 'mx', 'my', 'mz', 't_ms')


def parse_sample_line(line):
    """Parse one JSON-line IMU sample. Returns a dict with keys ax, ay,
    az, gx, gy, gz, mx, my, mz, t_ms on success, or None on malformed
    JSON, a missing expected key, or a non-numeric value (e.g. null or
    a bool) for one of those keys - never raises."""
    try:
        data = json.loads(line)
    except (ValueError, TypeError):
        return None
    if not isinstance(data, dict):
        return None
    if not all(key in data for key in REQUIRED_KEYS):
        return None
    if not all(
        isinstance(data.get(key), (int, float)) and not isinstance(data.get(key), bool)
        for key in REQUIRED_KEYS
    ):
        return None
    return data


def read_samples(serial_conn, on_sample):
    """Blocking loop: reads lines from serial_conn (anything with a
    readline() -> bytes method), decodes and parses each one, and calls
    on_sample(sample_dict) for each successfully parsed sample.
    Malformed lines are silently skipped. Empty/timeout reads (b'') are
    silently skipped too — only a raised exception (e.g. SerialException
    on a genuinely disconnected port) stops the loop."""
    while True:
        raw_line = serial_conn.readline()
        if not raw_line:
            continue
        line = raw_line.decode('utf-8', errors='replace').strip()
        if not line:
            continue
        sample = parse_sample_line(line)
        if sample is not None:
            on_sample(sample)
