# walker_anomaly_detection Design

**Date:** 2026-08-31
**Status:** Approved by user; ready for implementation planning
**Scope:** First design pass for fall/anomaly detection (`README.md` §5.2's "lightweight monitor
on IMU tilt/deceleration data, triggering companion app alerts"): ESP32 firmware streaming a real
MPU-9250/ICM-20948-style 9-axis IMU over USB serial, plus a ROS2 package running the actual
detection algorithm against that stream. Developed against real IMU hardware on the bench — a
first departure from this project's sim-first pattern for prior packages, since the user has
physical IMU units on hand and no fake-IMU backend is being built this pass. Does not cover
wiring the resulting alerts into `walker_companion_app`'s dashboard (a separate follow-up).

## 1. Problem

`README.md` §5.2 originally assigned fall/anomaly detection to the "Motion controller" section
("Lightweight monitor on IMU tilt/deceleration data, triggering companion app alerts"), implying
it belonged inside what became `walker_motor_driver`. That package's actual build
(`docs/superpowers/specs/2026-08-30-walker-motor-driver-design.md`) never touched IMU or fall
logic at all — the assignment predates the sim-first roadmap redesign
(`docs/superpowers/specs/2026-08-30-phase1-roadmap-design.md`) and was never revisited. No
fall/anomaly subsystem exists anywhere in this codebase today. `walker_companion_app`'s alerts
panel is explicitly static placeholder text ("No anomaly detection configured yet") specifically
because of this gap
(`docs/superpowers/specs/2026-08-30-walker-companion-app-design.md` §2.7).

Unlike every package built so far, this one has real hardware available now — the user has
MPU-9250/ICM-20948-style 9-axis IMU units on hand — and wants this developed against that real
hardware on the bench rather than simulated, even though the rest of the robot (chassis, motors,
onboard compute board) remains simulated/undecided per the roadmap design.

## 2. Decisions

### 2.1 New package `walker_anomaly_detection`, not folded into `walker_motor_driver`

Real `ament_python` colcon package, matching `walker_motor_driver`/`walker_nav`/`walker_llm_bridge`/
`walker_companion_app`'s shape — unlike `walker_safety`, this package publishes into the ROS2
graph, so it isn't structured as an out-of-graph directory the way `walker_safety` is. Chosen as
its own package over reopening `walker_motor_driver` to add this (rejected: different hardware
concern entirely — I2C IMU + a second microcontroller vs. wheel motor control — and this
project's established pattern is one subsystem per package, not retrofitting an already-built and
already-reviewed package for an unrelated concern).

### 2.2 Firmware is a thin I2C-to-serial bridge; all detection logic lives in the ROS2-side Python package

The ESP32 firmware's only job is reading the IMU over I2C and streaming raw samples over USB
serial — no thresholds, no state machines, no detection logic on-device. Every part of the actual
algorithm lives in ordinary, desktop-pytest-testable Python on the ROS2 side. This goes further
than `walker_safety`'s own pure/hardware split (that package's trip *logic* is pure-tested, but it
still runs *on* the Pico) — here, nothing algorithmic runs on the microcontroller at all. Chosen
over implementing fall detection on the ESP32 itself (rejected: would mean porting/duplicating the
algorithm in MicroPython with no desktop pytest coverage for that copy, and this project has never
maintained parallel implementations of the same logic in two languages).

MicroPython for the firmware, not Arduino C++ — matches `walker_safety`'s own Pico firmware
language and the "Python-first approach and the rest of the project's tone" reasoning
`walker_motor_driver`'s design spec (§2.1) already established for this project generally.

### 2.3 Serial protocol: JSON-lines over USB serial

Firmware samples the IMU at a fixed rate and writes one JSON object per line:
`{"ax":.., "ay":.., "az":.., "gx":.., "gy":.., "gz":.., "mx":.., "my":.., "mz":.., "t_ms":..}\n`
— `t_ms` is milliseconds since the ESP32's own boot, used only by firmware-adjacent tooling to
detect dropped/gapped samples, not a wall-clock timestamp. Chosen over a packed/binary protocol
(rejected: the data rate here — tens of samples/second — doesn't need it, and JSON-lines can be
read directly off a serial monitor during bring-up/debugging, which a binary format can't).

`imu_serial.py`'s pure `parse_sample_line(line: str) -> dict | None` parses one line, returning a
dict with keys `ax, ay, az, gx, gy, gz, mx, my, mz, t_ms` on success, or `None` on malformed JSON
or a missing expected key — never raises. The node skips a `None` result rather than crashing
(§5's malformed-line test case). The actual `pyserial` read loop (blocking I/O against the real
port) is a separate, un-pure function/method in the same file, calling `parse_sample_line` per
line read.

### 2.4 Fall detection: two-stage free-fall → impact pattern

`fall_detector.py`'s pure `FallDetector` tracks accelerometer magnitude
(`sqrt(ax**2 + ay**2 + az**2)`) across a stream of samples. It enters a "possible fall" state when
magnitude drops below a low threshold (near-zero-g, i.e. free-fall) for at least a minimum
duration, then watches for a subsequent impact (magnitude spike above a high threshold) within a
bounded window after the free-fall ends — the classic phone/wearable fall-detection pattern.
Thresholds and windows are constructor parameters with placeholder defaults, not values calibrated
against real hardware — the exact accelerometer scale depends on the MPU-9250/ICM-20948's
full-scale-range configuration, which hasn't been chosen yet. Same "placeholder now, recalibrate
at bring-up" treatment `walker_motor_driver`'s physical constants (`wheel_radius_m` etc.) already
got.

`FallDetector(free_fall_threshold_g, free_fall_min_duration_s, impact_threshold_g,
impact_window_s)`; `.update(accel_magnitude_g: float, now_s: float) -> bool` — called once per
sample, returns `True` exactly on the sample where a fall is confirmed (free-fall min-duration
already satisfied, and this sample's magnitude crosses the impact threshold within the window),
`False` otherwise. Takes an already-computed magnitude (`sqrt(ax**2+ay**2+az**2)` in g units), not
raw axes — the node computes magnitude from a parsed sample and calls `update`; `FallDetector`
itself doesn't need to know about 3-axis decomposition, only a magnitude time series. `now_s` is
an explicit parameter (not read from a wall clock internally) so tests are deterministic and
don't need to sleep — same pattern `walker_safety`'s `Watchdog` and `walker_motor_driver`'s
`SimMotorBackend` already use for the same reason.

### 2.5 Tilt detection: sustained off-vertical duration, accelerometer-only estimate

`tilt_detector.py`'s pure `TiltDetector` estimates tilt-from-vertical using only the
accelerometer's gravity component (`atan2` on the accel vector), not a complementary/Kalman filter
fusing gyroscope data. This is only meaningful when the robot is roughly stationary — which is
exactly the case a "tipped over or stuck" check cares about — and normal driving-induced
accelerometer noise is tolerated by requiring the tilt to stay above threshold for a minimum
sustained duration before flagging, so a transient bump during normal driving doesn't trigger it.
Chosen over full sensor fusion (rejected as unnecessary complexity for this specific check; noted
as a limitation if finer-grained in-motion tilt tracking is ever needed later).

Pure `tilt_from_accel_deg(ax, ay, az) -> float` computes the tilt-from-vertical angle in degrees
from the accelerometer's gravity component. `TiltDetector(tilt_threshold_deg,
tilt_sustained_duration_s)`; `.update(tilt_deg: float, now_s: float) -> bool` — called once per
sample (fed `tilt_from_accel_deg`'s output), returns `True` exactly on the sample where the tilt
has been continuously above `tilt_threshold_deg` for at least `tilt_sustained_duration_s`,
`False` otherwise (including while still within the sustained-duration window). Same explicit
`now_s` pattern as `FallDetector.update`, for the same determinism reason.

### 2.6 No coupling to `walker_safety` or `walker_motor_driver` — alert-only

This package never subscribes to motor or safety topics, and never publishes anything that could
stop or control the robot — only an observational alert. Extends `walker_motor_driver`'s and
`walker_llm_bridge`'s existing "no coupling to `walker_safety`" sections
(`docs/superpowers/specs/2026-08-30-walker-motor-driver-design.md` §2.6,
`docs/superpowers/specs/2026-08-30-walker-llm-bridge-design.md` §2.3) to also explicitly rule out
coupling to `walker_motor_driver` — a naive design might auto-halt the motors on a detected fall,
which is deliberately rejected: `README.md` §5.3/§5.4 and CLAUDE.md's safety invariants name only
the hardware E-stop and Pico watchdog as real stop mechanisms, and this package must not become an
unintended third one.

### 2.7 Alert topic: `/anomaly_detected`, `std_msgs/String` with a JSON payload

Payload: `{"type": "fall"|"tilt", "timestamp": <float, seconds>}`. Reuses a standard message type
— this project has never defined a custom `.msg` type, and adding `rosidl` message-generation
build machinery for one topic isn't worth it. Mirrors `walker_llm_bridge`'s own JSON-in-`String`
pattern. `timestamp` is the ROS2 node's own detection time (`self.get_clock().now()`), not
firmware's `t_ms` (which isn't wall-clock and isn't meaningful outside the firmware's own
gap-detection use, per §2.3).

### 2.8 No simulated/fake IMU backend

Unlike every prior package's sim-first default, this one has no fake-serial-IMU fallback, per
explicit user preference — real IMU hardware exists now, and the pure detector modules already
give hardware-independent testability via synthetic pytest sequences, so a duplicate
simulated-serial-stream layer would be scope creep without a concrete need. A fake backend can be
added later (e.g. for a CI machine with no IMU attached) without restructuring anything here — the
serial-reading code is already isolated behind its own module boundary (§3).

### 2.9 `walker_companion_app` wiring is a separate follow-up

Per user preference. `/anomaly_detected`'s existence doesn't change `walker_companion_app`'s own
Task 5 decision (`docs/superpowers/specs/2026-08-30-walker-companion-app-design.md` §2.7) to keep
its alerts panel static this pass — that package needs its own follow-up task to subscribe and
replace the placeholder text.

## 3. Package structure

New `ament_python` package:

```
src/walker_anomaly_detection/
  package.xml, setup.py, setup.cfg, resource/walker_anomaly_detection
  walker_anomaly_detection/
    __init__.py
    fall_detector.py        (pure: FallDetector)
    tilt_detector.py         (pure: TiltDetector)
    imu_serial.py             (pure sample-parsing; pyserial I/O isolated separately)
    anomaly_detection_node.py (rclpy node: reads serial, runs both detectors, publishes alerts)
  firmware/
    imu_reader.py             (MicroPython, on-device: reads the IMU over I2C, writes JSON-lines
                                to USB serial - untestable except on real hardware, mirrors
                                walker_safety/firmware/main.py's role)
  docs/
    bring_up.md                (wiring notes: ESP32<->IMU I2C pins, USB serial setup, MicroPython
                                 flashing steps - mirrors walker_safety/docs/e_stop_wiring.md)
  launch/anomaly_detection.launch.py (serial_port, baud_rate arguments)
  test/
    conftest.py
    test_fall_detector.py
    test_tilt_detector.py
    test_imu_serial.py
  tools/
    verify_anomaly_detection.py (manual/scripted check requiring the real ESP32+IMU connected -
                                  cannot be a fully-automated E2E script the way sim-first
                                  packages' verify scripts are, since there's no fake IMU backend)
```

## 4. Interface

**Node:** `walker_anomaly_detection` (entry point `anomaly_detection_node`)

**Params:**
| Param | Default | Notes |
|---|---|---|
| `serial_port` | `/dev/ttyUSB0` | ESP32's USB-serial device path |
| `baud_rate` | `115200` | |
| `free_fall_threshold_g` | `0.3` | accel magnitude below this (in g) counts as free-fall |
| `free_fall_min_duration_s` | `0.05` | |
| `impact_threshold_g` | `2.0` | |
| `impact_window_s` | `0.5` | max time after free-fall ends to see the impact spike |
| `tilt_threshold_deg` | `45.0` | |
| `tilt_sustained_duration_s` | `3.0` | |

All seven threshold/duration params are placeholders per §2.4/§2.5 — recalibrate once real
full-scale-range configuration and bench data exist.

**Topics published:**
- `/anomaly_detected` (`std_msgs/String`, JSON payload `{"type": "fall"|"tilt", "timestamp": float}`)

## 5. Testing

`fall_detector.py`, `tilt_detector.py`, and `imu_serial.py`'s sample-parsing logic are pure
Python — unit-tested with pytest, no ROS sourcing or colcon build required, same `test/conftest.py`
pattern as every other package here. Test cases include: a constructed free-fall-then-impact
sequence triggers a fall event; a brief dip that recovers *before* the impact window elapses does
NOT trigger; a sustained-tilt sequence past the duration threshold triggers a tilt event; a
transient tilt that recovers before the duration threshold does NOT trigger; a malformed JSON line
is skipped by `imu_serial.py`'s parser rather than crashing the node.

`anomaly_detection_node.py` and `firmware/imu_reader.py` are not pytest-testable — they require a
real serial connection and real IMU hardware. `tools/verify_anomaly_detection.py` is a manual,
not fully automated, check: with the ESP32 flashed and connected, it subscribes to
`/anomaly_detected` and prompts the operator to perform a real drop/impact motion and a real
sustained-tilt motion with the IMU, confirming each produces the expected event type. This mirrors
`walker_safety`'s own hardware-bring-up-required treatment (`tools/send_fake_heartbeats.py` is
the closest existing precedent for a manual, hardware-attached verification tool in this project)
rather than the fully-automated E2E scripts the sim-first packages achieved.

## 6. Out of scope

- Simulated/fake IMU backend — explicitly rejected per §2.8, not merely deferred.
- Wiring `/anomaly_detected` into `walker_companion_app`'s dashboard — separate follow-up per §2.9.
- Any coupling to `walker_safety` or `walker_motor_driver` — explicitly rejected per §2.6.
- A custom ROS2 `.msg` type — explicitly rejected per §2.7.
- Calibrating the seven placeholder threshold/duration values against real hardware — deferred to
  bring-up, same treatment as `walker_motor_driver`'s physical constants.
- Onboard-compute board choice — still deferred per
  `docs/superpowers/specs/2026-08-30-phase1-roadmap-design.md`; this package runs on the dev
  workstation for now, like every other package.
- ESP32 wireless streaming — wired USB serial only this pass; the roadmap design's reservation of
  ESP32 for "a future non-safety-critical role" doesn't require using its wireless stack, and wired
  is simpler to bring up and debug first.
