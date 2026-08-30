"""Wire-format helpers for the watchdog's heartbeat protocol.

Kept separate from main.py so the byte-level protocol can be unit
tested on the desktop without a Pico attached, same rationale as
watchdog_logic.py.
"""

HEARTBEAT_BYTE = b"\x01"


def is_heartbeat_byte(data):
    """Return True if data is exactly one heartbeat marker byte."""
    return data == HEARTBEAT_BYTE
