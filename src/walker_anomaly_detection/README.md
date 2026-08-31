# walker_anomaly_detection

Fall/anomaly detection for smart-walker-bot. See
`docs/superpowers/specs/2026-08-31-walker-anomaly-detection-design.md`
for the full design (this is a summary). This is a new addition beyond
the original Phase 1 roadmap's five packages — root `README.md` §5.2
originally assigned fall/anomaly detection to `walker_motor_driver`'s
scope, but that package's actual build never implemented it.

Real `ament_python` package — build it with
`colcon build --packages-select walker_anomaly_detection` from `src/`
(this repo's colcon workspace root).

**First package in this project developed against real hardware
rather than a simulation** — the user has physical MPU-9250/ICM-20948-
style 9-axis IMU units on hand. As of this package's initial build, the
ESP32 side isn't wired up yet ("a small project of its own") — see
`docs/bring_up.md`.

## Layout

- `walker_anomaly_detection/imu_serial.py` — pure Python:
  `parse_sample_line(line)` parses one JSON-line IMU sample, returning
  `None` on anything malformed rather than raising. `read_samples(conn,
  on_sample)` is the (not pure, but simple) blocking read loop that
  calls it per line.
- `walker_anomaly_detection/fall_detector.py` — pure Python:
  `FallDetector`, a two-stage free-fall-then-impact state machine.
  One-shot per confirmed fall.
- `walker_anomaly_detection/tilt_detector.py` — pure Python:
  `tilt_from_accel_deg(ax, ay, az)` and `TiltDetector`, a
  sustained-duration off-vertical state machine. Also one-shot per
  confirmed tilt event (re-arms once the robot returns upright).
- `walker_anomaly_detection/anomaly_detection_node.py` — the `rclpy`
  node: opens the configured serial port, reads samples on a background
  thread, feeds both detectors, publishes `/anomaly_detected`
  (`std_msgs/String`, JSON payload) on a detected event.
- `firmware/imu_reader.py` — MicroPython, on-device: reads the IMU over
  I2C, streams JSON-line samples over USB serial. No detection logic —
  untestable except on real hardware, mirrors
  `walker_safety/firmware/main.py`'s role.
- `docs/bring_up.md` — wiring notes and the manual real-hardware
  verification procedure (the one thing that can't be automated).
- `launch/anomaly_detection.launch.py` — launch file with a
  `serial_port` argument (default `/dev/ttyUSB0`).
- `tools/verify_anomaly_detection.py` — a scripted, **fully automated**
  end-to-end check using a virtual serial pair (`pty`) — no real
  hardware needed. See this file's own docstring for usage.

## Running the pure-module tests

```bash
cd src/walker_anomaly_detection
python3 -m pytest test/ -v
```

No ROS environment or colcon build needed for these.

## No coupling to walker_safety or walker_motor_driver

This package only publishes an observational alert — it never
subscribes to motor or safety topics, and never publishes anything
that could stop or control the robot. Only the hardware E-stop and
Pico watchdog are real stop mechanisms (root `README.md` §5.3/§5.4,
CLAUDE.md's safety invariants); this package must never become an
unintended third one.
