"""Manual test helper: sends a fake heartbeat byte to a Pico running
walker_safety firmware, at a fixed interval, so a human can verify the
watchdog trips/recovers correctly (e.g. by watching an LED/multimeter
on ENABLE_PIN_NUM) without needing the full ROS2 stack running.

Usage: python3 send_fake_heartbeats.py /dev/ttyACM0 --interval 0.2
Stop sending (Ctrl+C) to observe the watchdog trip after TIMEOUT_S.
"""
import argparse
import time

import serial

HEARTBEAT_BYTE = b"\x01"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="Serial port, e.g. /dev/ttyACM0")
    parser.add_argument(
        "--interval", type=float, default=0.2, help="Seconds between heartbeats"
    )
    args = parser.parse_args()

    with serial.Serial(args.port, baudrate=115200, timeout=1) as ser:
        print(f"Sending heartbeats to {args.port} every {args.interval}s. Ctrl+C to stop.")
        try:
            while True:
                ser.write(HEARTBEAT_BYTE)
                time.sleep(args.interval)
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
