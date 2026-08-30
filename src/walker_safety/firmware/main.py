"""Pico watchdog firmware entry point.

Runs standalone on the Pico, independent of the Pi/ROS2 stack per
docs/superpowers/specs/2026-08-30-phase1-roadmap-design.md Sec 2.2.
Reads heartbeat bytes over USB serial (stdin) and drives ENABLE_PIN_NUM
high only while heartbeats are recent; on boot, on any gap longer than
TIMEOUT_S, or on any unhandled exception, the pin is driven low
(motors disabled).

Not unit-testable on the desktop (uses the `machine` module, which
only exists in MicroPython on-device) - verify manually per
../docs/e_stop_wiring.md's "Firmware bring-up" section.
"""
import sys
import time

import micropython

try:
    import uselect as select
except ImportError:
    import select

from machine import Pin

from heartbeat_framing import is_heartbeat_byte
from watchdog_logic import Watchdog

ENABLE_PIN_NUM = 15
TIMEOUT_S = 0.5
POLL_INTERVAL_S = 0.05


def main():
    # Treat stdin as raw binary: without this, MicroPython intercepts a
    # 0x03 byte on stdin as a Ctrl-C (KeyboardInterrupt) instead of
    # passing it through as data - a corrupted or adversarial heartbeat
    # byte must never be able to kill this loop.
    micropython.kbd_intr(-1)

    enable_pin = Pin(ENABLE_PIN_NUM, Pin.OUT)
    enable_pin.value(0)  # fail-safe: motors disabled until first heartbeat
    watchdog = Watchdog(timeout_s=TIMEOUT_S)
    poller = select.poll()
    poller.register(sys.stdin, select.POLLIN)
    start_ms = time.ticks_ms()

    try:
        while True:
            now_s = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
            # Drain every heartbeat byte buffered on stdin this tick, not
            # just one - otherwise a backlog of stale heartbeats can keep
            # feeding the watchdog long after the real sender is gone.
            while poller.poll(0):
                byte = sys.stdin.buffer.read(1)
                if is_heartbeat_byte(byte):
                    watchdog.on_heartbeat(now_s)
            enable_pin.value(0 if watchdog.is_tripped(now_s) else 1)
            time.sleep(POLL_INTERVAL_S)
    finally:
        # Any unhandled exception (or a normal exit, which never happens
        # in practice) must leave motors disabled, not at whatever value
        # the pin last held.
        enable_pin.value(0)


if __name__ == "__main__":
    main()
