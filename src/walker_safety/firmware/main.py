"""Pico watchdog firmware entry point.

Runs standalone on the Pico, independent of the Pi/ROS2 stack per
docs/superpowers/specs/2026-08-30-phase1-roadmap-design.md Sec 2.2.
Reads single-byte heartbeats over USB serial (stdin) and drives
ENABLE_PIN_NUM high only while heartbeats are recent; on boot, and on
any gap longer than TIMEOUT_S, the pin is driven low (motors disabled).

Not unit-testable on the desktop (uses the `machine` module, which
only exists in MicroPython on-device) - verify manually per
../docs/e_stop_wiring.md's "Firmware bring-up" section.
"""
import sys
import time

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
    enable_pin = Pin(ENABLE_PIN_NUM, Pin.OUT)
    enable_pin.value(0)  # fail-safe: motors disabled until first heartbeat
    watchdog = Watchdog(timeout_s=TIMEOUT_S)
    poller = select.poll()
    poller.register(sys.stdin, select.POLLIN)

    while True:
        now = time.time()
        if poller.poll(0):
            byte = sys.stdin.buffer.read(1)
            if is_heartbeat_byte(byte):
                watchdog.on_heartbeat(now)
        enable_pin.value(0 if watchdog.is_tripped(now) else 1)
        time.sleep(POLL_INTERVAL_S)


if __name__ == "__main__":
    main()
