# walker_anomaly_detection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce the `walker_anomaly_detection` ROS2 package: ESP32 firmware streaming a real 9-axis IMU over USB serial, and a pure, pytest-tested detection algorithm (free-fall + impact, sustained tilt) that publishes `/anomaly_detected` alerts — the first package in this project developed against real hardware rather than a simulation.

**Architecture:** Three small pure-Python modules (`imu_serial.py`'s sample parsing, `fall_detector.py`'s `FallDetector`, `tilt_detector.py`'s `tilt_from_accel_deg`/`TiltDetector`), each unit-tested with pytest using synthetic sample sequences — no hardware needed for any of them. A thin `rclpy` node reads a serial stream via a background thread, feeds parsed samples to both detectors, and publishes alerts as `std_msgs/String` JSON. End-to-end node verification uses a `pty`-backed virtual serial pair (no real ESP32/IMU required) — a genuine improvement over the original design sketch, since the node's own wiring logic doesn't actually depend on real hardware. MicroPython firmware for the ESP32 (a thin I2C-to-serial bridge, no detection logic) and a hardware bring-up doc are written but not executable this session — the user doesn't have the ESP32 wired up yet ("a small project of its own").

**Tech Stack:** Python 3 + `rclpy` (ROS2 Humble), pytest (pure-module unit tests), `pyserial` (already present in this environment), stdlib `pty`/`json`/`threading`, `std_msgs/String`. MicroPython for the ESP32 firmware.

**Spec:** `docs/superpowers/specs/2026-08-31-walker-anomaly-detection-design.md` (§2 for decisions, §3 for file structure, §4 for interface, §5 for testing approach).

## Global Constraints

- Real `ament_python` colcon package, buildable with `colcon build --packages-select walker_anomaly_detection` from `src/`. This is a NEW addition beyond the original 5-package Phase 1 roadmap (`README.md` §6) — fall/anomaly detection was originally assigned to `walker_motor_driver`'s scope but was never implemented there (spec §2.1).
- IMU sample units: `ax, ay, az` are in **g** (firmware converts from raw ADC counts before transmission); `gx, gy, gz` (gyro) and `mx, my, mz` (magnetometer) are captured but unused by either detector this pass. (spec §2.3)
- `parse_sample_line(line: str) -> dict | None` — returns a dict with keys `ax, ay, az, gx, gy, gz, mx, my, mz, t_ms` on success, `None` on malformed JSON or a missing key, never raises. (spec §2.3)
- `FallDetector(free_fall_threshold_g, free_fall_min_duration_s, impact_threshold_g, impact_window_s)`; `.update(accel_magnitude_g: float, now_s: float) -> bool` — **one-shot per event**: returns `True` exactly on the sample confirming a fall (free-fall min-duration satisfied, then an impact-threshold crossing within the window), resets its own state on that same call so a second, independent fall can be detected later. Explicit `now_s` param (no wall-clock reads internally), matching `walker_safety`'s `Watchdog` / `walker_motor_driver`'s `SimMotorBackend` determinism pattern. (spec §2.4)
- `tilt_from_accel_deg(ax, ay, az) -> float` — `degrees(atan2(sqrt(ax**2+ay**2), az))`, 0° = upright. `TiltDetector(tilt_threshold_deg, tilt_sustained_duration_s)`; `.update(tilt_deg: float, now_s: float) -> bool` — **also one-shot per event** (a refinement decided during planning, consistent with `FallDetector`'s semantics and the spec's own "exactly on the sample where..." wording): returns `True` once when the sustained-duration threshold is first crossed, then stays `False` on every subsequent sample while still tilted, re-arming only once `tilt_deg` drops back below threshold. This avoids re-publishing an identical alert every sample (tens of times/second) while the robot remains tilted. (spec §2.5, refined)
- Alert topic `/anomaly_detected` (`std_msgs/String`), JSON payload `{"type": "fall"|"tilt", "timestamp": float}` — `timestamp` is the node's own `self.get_clock().now()` at detection time, not firmware's `t_ms`. No custom `.msg` type. (spec §2.7)
- No coupling to `walker_safety` or `walker_motor_driver` anywhere in this package — publish-only, alert-only. (spec §2.6)
- No simulated/fake IMU backend as a first-class package feature — but see the next point, which is a *test-only* technique, not a persistent backend. (spec §2.8)
- `tools/verify_anomaly_detection.py` is fully automated via a `pty`-backed virtual serial pair — no real ESP32/IMU required, verifies the complete serial→parse→detect→publish pipeline. Real sensor validation (does an actual accelerometer produce sane values) is a separate manual step in `docs/bring_up.md`, not part of this plan's own task-completion gates, since the ESP32 isn't wired up yet. (spec §2.10)
- Pure modules (`imu_serial.py`'s `parse_sample_line`, `fall_detector.py`, `tilt_detector.py`) have zero `rclpy` imports — tests run with plain `python3 -m pytest`, no ROS sourcing or colcon build required, same `test/conftest.py` pattern as every other package here.
- `firmware/imu_reader.py` targets the MPU-9250 register map specifically (as a concrete, complete starting point) — if the actual chip turns out to be an ICM-20948, its different register map means this file needs adjustment at bring-up time; flagged in the file's own header comment, same "placeholder now, adjust at bring-up" treatment as `walker_motor_driver`'s physical constants.

---

## Task 1: Package Scaffold

**Files:**
- Create: `src/walker_anomaly_detection/package.xml`
- Create: `src/walker_anomaly_detection/setup.py`
- Create: `src/walker_anomaly_detection/setup.cfg`
- Create: `src/walker_anomaly_detection/resource/walker_anomaly_detection`
- Create: `src/walker_anomaly_detection/walker_anomaly_detection/__init__.py`
- Create: `src/walker_anomaly_detection/README.md`

**Interfaces:**
- Produces: an installable, buildable `ament_python` package shell. `console_scripts` entry point `anomaly_detection_node = walker_anomaly_detection.anomaly_detection_node:main` is declared now even though `anomaly_detection_node.py` doesn't exist until Task 5 — `colcon build` doesn't import entry-point targets at build time.

- [ ] **Step 1: Create the package directories**

```bash
mkdir -p /home/peter_sha/sourcecode/smart-walker-bot/src/walker_anomaly_detection/walker_anomaly_detection
mkdir -p /home/peter_sha/sourcecode/smart-walker-bot/src/walker_anomaly_detection/resource
mkdir -p /home/peter_sha/sourcecode/smart-walker-bot/src/walker_anomaly_detection/firmware
mkdir -p /home/peter_sha/sourcecode/smart-walker-bot/src/walker_anomaly_detection/docs
mkdir -p /home/peter_sha/sourcecode/smart-walker-bot/src/walker_anomaly_detection/launch
mkdir -p /home/peter_sha/sourcecode/smart-walker-bot/src/walker_anomaly_detection/test
mkdir -p /home/peter_sha/sourcecode/smart-walker-bot/src/walker_anomaly_detection/tools
```

- [ ] **Step 2: Write package.xml**

Create `src/walker_anomaly_detection/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>walker_anomaly_detection</name>
  <version>0.0.1</version>
  <description>Fall/anomaly detection for smart-walker-bot: ESP32-streamed IMU data, pure free-fall+impact and sustained-tilt detection, publishing alerts.</description>
  <maintainer email="botheredbybees@gmail.com">botheredbybees</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>python3-serial</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 3: Write setup.py**

Create `src/walker_anomaly_detection/setup.py`:

```python
from setuptools import find_packages, setup

package_name = 'walker_anomaly_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/anomaly_detection.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='botheredbybees',
    maintainer_email='botheredbybees@gmail.com',
    description='Fall/anomaly detection for smart-walker-bot: ESP32-streamed IMU data, pure free-fall+impact and sustained-tilt detection, publishing alerts.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'anomaly_detection_node = walker_anomaly_detection.anomaly_detection_node:main',
        ],
    },
)
```

Note: `launch/anomaly_detection.launch.py` is referenced here but doesn't exist until Task 5. If Step 6's build-verification fails because it's missing, create an empty placeholder first:

```python
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([])
```

Task 5 will overwrite this placeholder with the real launch file.

- [ ] **Step 4: Write setup.cfg**

Create `src/walker_anomaly_detection/setup.cfg`:

```ini
[develop]
script_dir=$base/lib/walker_anomaly_detection
[install]
install_scripts=$base/lib/walker_anomaly_detection
```

- [ ] **Step 5: Create the resource marker and package __init__**

```bash
touch /home/peter_sha/sourcecode/smart-walker-bot/src/walker_anomaly_detection/resource/walker_anomaly_detection
touch /home/peter_sha/sourcecode/smart-walker-bot/src/walker_anomaly_detection/walker_anomaly_detection/__init__.py
```

- [ ] **Step 6: Verify the package builds**

```bash
source /opt/ros/humble/setup.bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src
PYTHONNOUSERSITE=1 colcon build --packages-select walker_anomaly_detection --symlink-install
```

Expected: build succeeds (`Summary: 1 package finished`). If it fails because `launch/anomaly_detection.launch.py` is missing, create the placeholder from Step 3's note and retry.

- [ ] **Step 7: Write the package README**

Create `src/walker_anomaly_detection/README.md`:

```markdown
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

\`\`\`bash
cd src/walker_anomaly_detection
python3 -m pytest test/ -v
\`\`\`

No ROS environment or colcon build needed for these.

## No coupling to walker_safety or walker_motor_driver

This package only publishes an observational alert — it never
subscribes to motor or safety topics, and never publishes anything
that could stop or control the robot. Only the hardware E-stop and
Pico watchdog are real stop mechanisms (root `README.md` §5.3/§5.4,
CLAUDE.md's safety invariants); this package must never become an
unintended third one.
```

- [ ] **Step 8: Commit**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot
git add src/walker_anomaly_detection/
git commit -m "$(cat <<'EOF'
Add walker_anomaly_detection package scaffold

ament_python ROS2 package shell: package.xml, setup.py/cfg, resource
marker, and package README. colcon build verified working before any
node code exists. A new addition beyond the original 5-package Phase 1
roadmap - fall/anomaly detection was assigned to walker_motor_driver
originally but never implemented there.
EOF
)"
```

---

## Task 2: IMU Serial Sample Parsing (TDD)

**Files:**
- Create: `src/walker_anomaly_detection/walker_anomaly_detection/imu_serial.py`
- Create: `src/walker_anomaly_detection/test/conftest.py`
- Test: `src/walker_anomaly_detection/test/test_imu_serial.py`

**Interfaces:**
- Produces: `parse_sample_line(line) -> dict | None`; `read_samples(serial_conn, on_sample) -> None` (blocking loop; `serial_conn` needs only a `.readline() -> bytes` method — a real `pyserial.Serial` or a test double both satisfy this). Consumed by Task 5 (`anomaly_detection_node.py`).

- [ ] **Step 1: Confirm pytest and pyserial are available**

```bash
python3 -m pytest --version
python3 -c "import serial; print(serial.__version__)"
```

Both are already present in this environment (confirmed during planning). If either is missing: `python3 -m pip install --user pytest pyserial`.

- [ ] **Step 2: Write the test-path conftest**

Create `src/walker_anomaly_detection/test/conftest.py`:

```python
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
```

Same pattern as every other package here: inserts the *outer*
`src/walker_anomaly_detection/` directory onto `sys.path`, so tests use
the same fully-qualified import style
(`from walker_anomaly_detection.imu_serial import ...`) the real node
uses.

- [ ] **Step 3: Write the failing tests**

Create `src/walker_anomaly_detection/test/test_imu_serial.py`:

```python
from walker_anomaly_detection.imu_serial import parse_sample_line, read_samples

VALID_LINE = (
    '{"ax": 0.1, "ay": 0.2, "az": 9.8, "gx": 0.0, "gy": 0.0, "gz": 0.0, '
    '"mx": 10.0, "my": 20.0, "mz": 30.0, "t_ms": 1000}'
)


def test_valid_line_parses_all_keys():
    sample = parse_sample_line(VALID_LINE)
    assert sample == {
        'ax': 0.1, 'ay': 0.2, 'az': 9.8,
        'gx': 0.0, 'gy': 0.0, 'gz': 0.0,
        'mx': 10.0, 'my': 20.0, 'mz': 30.0,
        't_ms': 1000,
    }


def test_malformed_json_returns_none():
    assert parse_sample_line('{"ax": 0.1, "ay"') is None


def test_non_dict_json_returns_none():
    assert parse_sample_line('[1, 2, 3]') is None


def test_missing_key_returns_none():
    incomplete = '{"ax": 0.1, "ay": 0.2, "az": 9.8}'
    assert parse_sample_line(incomplete) is None


def test_empty_string_returns_none():
    assert parse_sample_line('') is None


class _FakeSerial:
    def __init__(self, lines):
        self._lines = iter(lines)

    def readline(self):
        try:
            return next(self._lines)
        except StopIteration:
            return b''


def test_read_samples_calls_callback_per_valid_line():
    lines = [
        (VALID_LINE + '\n').encode('utf-8'),
        b'garbage not json\n',
        (VALID_LINE + '\n').encode('utf-8'),
    ]
    fake = _FakeSerial(lines)
    received = []
    read_samples(fake, received.append)
    assert len(received) == 2


def test_read_samples_stops_on_empty_bytes():
    fake = _FakeSerial([b''])
    received = []
    read_samples(fake, received.append)
    assert received == []
```

- [ ] **Step 4: Run tests to verify they fail**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_anomaly_detection
python3 -m pytest test/test_imu_serial.py -v
```

Expected: `ModuleNotFoundError: No module named 'walker_anomaly_detection.imu_serial'`.

- [ ] **Step 5: Implement imu_serial.py**

Create `src/walker_anomaly_detection/walker_anomaly_detection/imu_serial.py`:

```python
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
    JSON or a missing expected key - never raises."""
    try:
        data = json.loads(line)
    except (ValueError, TypeError):
        return None
    if not isinstance(data, dict):
        return None
    if not all(key in data for key in REQUIRED_KEYS):
        return None
    return data


def read_samples(serial_conn, on_sample):
    """Blocking loop: reads lines from serial_conn (anything with a
    readline() -> bytes method), decodes and parses each one, and calls
    on_sample(sample_dict) for each successfully parsed sample.
    Malformed lines are silently skipped. Runs until
    serial_conn.readline() returns empty bytes (connection closed)."""
    while True:
        raw_line = serial_conn.readline()
        if not raw_line:
            break
        line = raw_line.decode('utf-8', errors='replace').strip()
        if not line:
            continue
        sample = parse_sample_line(line)
        if sample is not None:
            on_sample(sample)
```

- [ ] **Step 6: Run tests to verify they pass**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_anomaly_detection
python3 -m pytest test/test_imu_serial.py -v
```

Expected: 7 passed.

- [ ] **Step 7: Commit**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot
git add src/walker_anomaly_detection/walker_anomaly_detection/imu_serial.py \
        src/walker_anomaly_detection/test/conftest.py \
        src/walker_anomaly_detection/test/test_imu_serial.py
git commit -m "$(cat <<'EOF'
Add walker_anomaly_detection IMU serial sample parsing

parse_sample_line is pure (no ROS dependency); read_samples is a thin
blocking read loop tested against a fake in-memory serial double, no
real hardware needed. anomaly_detection_node.py (Task 5) wires this to
a real pyserial connection.
EOF
)"
```

---

## Task 3: Fall Detector (TDD)

**Files:**
- Create: `src/walker_anomaly_detection/walker_anomaly_detection/fall_detector.py`
- Test: `src/walker_anomaly_detection/test/test_fall_detector.py`

**Interfaces:**
- Produces: `FallDetector(free_fall_threshold_g, free_fall_min_duration_s, impact_threshold_g, impact_window_s)` with `.update(accel_magnitude_g, now_s) -> bool`. Consumed by Task 5 (`anomaly_detection_node.py`).

- [ ] **Step 1: Write the failing tests**

Create `src/walker_anomaly_detection/test/test_fall_detector.py`:

```python
from walker_anomaly_detection.fall_detector import FallDetector

FREE_FALL_THRESHOLD_G = 0.3
FREE_FALL_MIN_DURATION_S = 0.05
IMPACT_THRESHOLD_G = 2.0
IMPACT_WINDOW_S = 0.5


def _make_detector():
    return FallDetector(
        free_fall_threshold_g=FREE_FALL_THRESHOLD_G,
        free_fall_min_duration_s=FREE_FALL_MIN_DURATION_S,
        impact_threshold_g=IMPACT_THRESHOLD_G,
        impact_window_s=IMPACT_WINDOW_S,
    )


def test_normal_readings_never_trigger():
    detector = _make_detector()
    for t in (0.0, 0.1, 0.2, 0.3):
        assert detector.update(1.0, t) is False


def test_free_fall_then_immediate_impact_triggers():
    detector = _make_detector()
    assert detector.update(1.0, 0.00) is False
    assert detector.update(0.1, 0.01) is False
    assert detector.update(0.1, 0.02) is False
    assert detector.update(0.1, 0.07) is False  # min_duration reached, still below threshold
    assert detector.update(2.5, 0.15) is True   # impact spike right as free-fall ends


def test_free_fall_then_delayed_impact_within_window_triggers():
    detector = _make_detector()
    detector.update(1.0, 0.00)
    detector.update(0.1, 0.01)
    detector.update(0.1, 0.07)  # confirms free-fall
    assert detector.update(1.0, 0.15) is False   # recovers, but not an impact yet
    assert detector.update(3.0, 0.20) is True    # impact within window


def test_free_fall_with_no_impact_within_window_does_not_trigger():
    detector = _make_detector()
    detector.update(1.0, 0.00)
    detector.update(0.1, 0.01)
    detector.update(0.1, 0.07)  # confirms free-fall
    detector.update(1.0, 0.15)  # recovers, normal reading, window starts
    assert detector.update(1.0, 0.70) is False  # window (0.5s) has elapsed, no impact


def test_brief_dip_below_min_duration_never_confirms_and_does_not_trigger():
    detector = _make_detector()
    detector.update(1.0, 0.00)
    detector.update(0.1, 0.01)   # dips below threshold
    detector.update(0.1, 0.02)   # still below, but duration (0.01s) < min_duration (0.05s)
    detector.update(1.0, 0.03)   # recovers before free-fall confirmed
    assert detector.update(5.0, 0.04) is False  # even a huge spike right after doesn't trigger


def test_impact_spike_without_preceding_free_fall_does_not_trigger():
    detector = _make_detector()
    detector.update(1.0, 0.00)
    assert detector.update(5.0, 0.01) is False


def test_state_resets_after_a_confirmed_fall_so_a_second_fall_can_be_detected():
    detector = _make_detector()
    detector.update(1.0, 0.00)
    detector.update(0.1, 0.01)
    detector.update(0.1, 0.07)
    assert detector.update(2.5, 0.15) is True

    # second, independent fall sequence
    detector.update(1.0, 1.00)
    detector.update(0.1, 1.01)
    detector.update(0.1, 1.07)
    assert detector.update(2.5, 1.15) is True
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_anomaly_detection
python3 -m pytest test/test_fall_detector.py -v
```

Expected: `ModuleNotFoundError: No module named 'walker_anomaly_detection.fall_detector'`.

- [ ] **Step 3: Implement fall_detector.py**

Create `src/walker_anomaly_detection/walker_anomaly_detection/fall_detector.py`:

```python
"""Pure free-fall + impact fall detection for walker_anomaly_detection.
No ROS or hardware imports - shared between anomaly_detection_node.py
and the pytest suite. See
docs/superpowers/specs/2026-08-31-walker-anomaly-detection-design.md
Sec 2.4.
"""


class FallDetector:
    """Tracks accelerometer magnitude (g) across a stream of samples.
    Enters a "possible fall" state on a sustained drop below
    free_fall_threshold_g; while in that state, watches for a
    subsequent impact (magnitude above impact_threshold_g) within
    impact_window_s of the free-fall ending. update() is one-shot per
    confirmed fall - it resets its own state on the sample that
    triggers, so a later, independent fall can be detected too."""

    def __init__(self, free_fall_threshold_g, free_fall_min_duration_s,
                 impact_threshold_g, impact_window_s):
        self._free_fall_threshold_g = free_fall_threshold_g
        self._free_fall_min_duration_s = free_fall_min_duration_s
        self._impact_threshold_g = impact_threshold_g
        self._impact_window_s = impact_window_s
        self._reset()

    def update(self, accel_magnitude_g, now_s):
        """Call once per sample. Returns True exactly on the sample
        where a fall is confirmed, False otherwise."""
        if accel_magnitude_g < self._free_fall_threshold_g:
            if self._free_fall_start_s is None:
                self._free_fall_start_s = now_s
            if not self._free_fall_confirmed and \
                    now_s - self._free_fall_start_s >= self._free_fall_min_duration_s:
                self._free_fall_confirmed = True
            return False

        if self._free_fall_confirmed:
            if self._free_fall_end_s is None:
                self._free_fall_end_s = now_s
            if accel_magnitude_g >= self._impact_threshold_g and \
                    now_s - self._free_fall_end_s <= self._impact_window_s:
                self._reset()
                return True
            if now_s - self._free_fall_end_s > self._impact_window_s:
                self._reset()
            return False

        self._reset()
        return False

    def _reset(self):
        self._free_fall_start_s = None
        self._free_fall_confirmed = False
        self._free_fall_end_s = None
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_anomaly_detection
python3 -m pytest test/test_fall_detector.py -v
```

Expected: 7 passed.

- [ ] **Step 5: Commit**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot
git add src/walker_anomaly_detection/walker_anomaly_detection/fall_detector.py \
        src/walker_anomaly_detection/test/test_fall_detector.py
git commit -m "$(cat <<'EOF'
Add walker_anomaly_detection FallDetector

Pure two-stage free-fall-then-impact state machine, one-shot per
confirmed fall, unit-tested with synthetic accelerometer-magnitude
sequences - no hardware dependency. Covers the real-fall,
no-preceding-free-fall, no-following-impact, and
too-brief-to-confirm cases explicitly.
EOF
)"
```

---

## Task 4: Tilt Detector (TDD)

**Files:**
- Create: `src/walker_anomaly_detection/walker_anomaly_detection/tilt_detector.py`
- Test: `src/walker_anomaly_detection/test/test_tilt_detector.py`

**Interfaces:**
- Produces: `tilt_from_accel_deg(ax, ay, az) -> float`; `TiltDetector(tilt_threshold_deg, tilt_sustained_duration_s)` with `.update(tilt_deg, now_s) -> bool`. Consumed by Task 5 (`anomaly_detection_node.py`).

- [ ] **Step 1: Write the failing tests**

Create `src/walker_anomaly_detection/test/test_tilt_detector.py`:

```python
import math

import pytest

from walker_anomaly_detection.tilt_detector import TiltDetector, tilt_from_accel_deg

TILT_THRESHOLD_DEG = 45.0
TILT_SUSTAINED_DURATION_S = 3.0


def _make_detector():
    return TiltDetector(
        tilt_threshold_deg=TILT_THRESHOLD_DEG,
        tilt_sustained_duration_s=TILT_SUSTAINED_DURATION_S,
    )


def test_tilt_from_accel_deg_upright_is_zero():
    assert tilt_from_accel_deg(0.0, 0.0, 9.8) == pytest.approx(0.0, abs=1e-6)


def test_tilt_from_accel_deg_on_its_side_is_90():
    assert tilt_from_accel_deg(9.8, 0.0, 0.0) == pytest.approx(90.0, rel=1e-6)


def test_tilt_from_accel_deg_45_degrees():
    value = 9.8 / math.sqrt(2)
    assert tilt_from_accel_deg(value, 0.0, value) == pytest.approx(45.0, rel=1e-6)


def test_upright_never_triggers():
    detector = _make_detector()
    for t in range(0, 10):
        assert detector.update(0.0, float(t)) is False


def test_tilt_below_threshold_never_triggers():
    detector = _make_detector()
    for t in range(0, 10):
        assert detector.update(30.0, float(t)) is False


def test_sustained_tilt_triggers_once_after_duration():
    detector = _make_detector()
    assert detector.update(60.0, 0.0) is False
    assert detector.update(60.0, 1.0) is False
    assert detector.update(60.0, 3.0) is True
    assert detector.update(60.0, 4.0) is False  # doesn't re-trigger while still tilted


def test_transient_tilt_recovering_before_duration_does_not_trigger():
    detector = _make_detector()
    detector.update(60.0, 0.0)
    detector.update(60.0, 1.0)
    assert detector.update(20.0, 2.0) is False
    assert detector.update(60.0, 2.1) is False  # fresh start, not enough time elapsed yet


def test_retriggers_after_recovering_and_tilting_again():
    detector = _make_detector()
    detector.update(60.0, 0.0)
    detector.update(60.0, 1.0)
    assert detector.update(60.0, 3.0) is True
    detector.update(20.0, 3.5)  # recovers upright
    detector.update(60.0, 3.6)
    assert detector.update(60.0, 6.6) is True
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_anomaly_detection
python3 -m pytest test/test_tilt_detector.py -v
```

Expected: `ModuleNotFoundError: No module named 'walker_anomaly_detection.tilt_detector'`.

- [ ] **Step 3: Implement tilt_detector.py**

Create `src/walker_anomaly_detection/walker_anomaly_detection/tilt_detector.py`:

```python
"""Pure tilt-from-vertical estimation and sustained-tilt detection for
walker_anomaly_detection. No ROS or hardware imports - shared between
anomaly_detection_node.py and the pytest suite. See
docs/superpowers/specs/2026-08-31-walker-anomaly-detection-design.md
Sec 2.5.
"""
import math


def tilt_from_accel_deg(ax, ay, az):
    """Tilt-from-vertical angle (degrees) from the accelerometer's
    gravity component - accurate only when the robot is roughly
    stationary (no significant non-gravity acceleration). 0 degrees is
    perfectly upright (gravity entirely along az)."""
    horizontal = math.sqrt(ax ** 2 + ay ** 2)
    return math.degrees(math.atan2(horizontal, az))


class TiltDetector:
    """Tracks tilt-from-vertical (degrees) across a stream of samples.
    update() is one-shot per confirmed sustained-tilt event: returns
    True exactly on the sample where tilt has been continuously above
    tilt_threshold_deg for at least tilt_sustained_duration_s, then
    False on every subsequent sample while still tilted (no repeated
    alerts for an ongoing condition) - re-arms once tilt_deg drops back
    below threshold."""

    def __init__(self, tilt_threshold_deg, tilt_sustained_duration_s):
        self._tilt_threshold_deg = tilt_threshold_deg
        self._tilt_sustained_duration_s = tilt_sustained_duration_s
        self._tilt_start_s = None
        self._triggered = False

    def update(self, tilt_deg, now_s):
        if tilt_deg < self._tilt_threshold_deg:
            self._tilt_start_s = None
            self._triggered = False
            return False

        if self._tilt_start_s is None:
            self._tilt_start_s = now_s

        if not self._triggered and \
                now_s - self._tilt_start_s >= self._tilt_sustained_duration_s:
            self._triggered = True
            return True

        return False
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_anomaly_detection
python3 -m pytest test/test_tilt_detector.py -v
```

Expected: 8 passed.

- [ ] **Step 5: Run the full pure-module suite**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_anomaly_detection
python3 -m pytest test/ -v
```

Expected: 22 passed (7 imu_serial + 7 fall_detector + 8 tilt_detector).

- [ ] **Step 6: Commit**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot
git add src/walker_anomaly_detection/walker_anomaly_detection/tilt_detector.py \
        src/walker_anomaly_detection/test/test_tilt_detector.py
git commit -m "$(cat <<'EOF'
Add walker_anomaly_detection TiltDetector

tilt_from_accel_deg is pure trig on the accelerometer's gravity
component; TiltDetector is a one-shot-per-event sustained-duration
state machine, unit-tested including the re-arm-after-recovery case.
No hardware dependency.
EOF
)"
```

---

## Task 5: ROS2 Node, Launch File, and Automated (pty-based) End-to-End Verification

**Files:**
- Create: `src/walker_anomaly_detection/walker_anomaly_detection/anomaly_detection_node.py`
- Create: `src/walker_anomaly_detection/launch/anomaly_detection.launch.py` (overwrites Task 1's placeholder, if one was created)
- Create: `src/walker_anomaly_detection/tools/verify_anomaly_detection.py`

**Interfaces:**
- Consumes: `read_samples` from `imu_serial` (Task 2); `FallDetector` from `fall_detector` (Task 3); `tilt_from_accel_deg`, `TiltDetector` from `tilt_detector` (Task 4).
- Produces: the `/anomaly_detected` topic interface a future `walker_companion_app` follow-up will subscribe to. Nothing later in this plan consumes it as a Python interface — this is the last code-writing task (Task 6 is firmware/docs only).

- [ ] **Step 1: Write the ROS2 node**

Create `src/walker_anomaly_detection/walker_anomaly_detection/anomaly_detection_node.py`:

```python
"""walker_anomaly_detection's ROS2 node: reads IMU samples from a
serial-connected ESP32 on a background thread, runs FallDetector and
TiltDetector against the stream, and publishes /anomaly_detected on a
detected event. See
docs/superpowers/specs/2026-08-31-walker-anomaly-detection-design.md
for the full design.
"""
import json
import math
import threading

import rclpy
import serial
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from walker_anomaly_detection.fall_detector import FallDetector
from walker_anomaly_detection.imu_serial import read_samples
from walker_anomaly_detection.tilt_detector import TiltDetector, tilt_from_accel_deg


class AnomalyDetectionNode(Node):
    def __init__(self):
        super().__init__('walker_anomaly_detection')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('free_fall_threshold_g', 0.3)
        self.declare_parameter('free_fall_min_duration_s', 0.05)
        self.declare_parameter('impact_threshold_g', 2.0)
        self.declare_parameter('impact_window_s', 0.5)
        self.declare_parameter('tilt_threshold_deg', 45.0)
        self.declare_parameter('tilt_sustained_duration_s', 3.0)

        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value

        self._fall_detector = FallDetector(
            free_fall_threshold_g=self.get_parameter('free_fall_threshold_g').value,
            free_fall_min_duration_s=self.get_parameter('free_fall_min_duration_s').value,
            impact_threshold_g=self.get_parameter('impact_threshold_g').value,
            impact_window_s=self.get_parameter('impact_window_s').value,
        )
        self._tilt_detector = TiltDetector(
            tilt_threshold_deg=self.get_parameter('tilt_threshold_deg').value,
            tilt_sustained_duration_s=self.get_parameter('tilt_sustained_duration_s').value,
        )

        self._alert_pub = self.create_publisher(String, '/anomaly_detected', 10)

        self._serial_conn = serial.Serial(serial_port, baud_rate, timeout=1.0)
        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()

    def _read_loop(self):
        read_samples(self._serial_conn, self._on_sample)

    def _on_sample(self, sample):
        now_s = self.get_clock().now().nanoseconds / 1e9
        accel_magnitude_g = math.sqrt(
            sample['ax'] ** 2 + sample['ay'] ** 2 + sample['az'] ** 2
        )

        if self._fall_detector.update(accel_magnitude_g, now_s):
            self._publish_alert('fall')

        tilt_deg = tilt_from_accel_deg(sample['ax'], sample['ay'], sample['az'])
        if self._tilt_detector.update(tilt_deg, now_s):
            self._publish_alert('tilt')

    def _publish_alert(self, alert_type):
        payload = json.dumps({
            'type': alert_type,
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
        })
        self._alert_pub.publish(String(data=payload))
        self.get_logger().warning(f'Anomaly detected: {alert_type}')

    def stop(self):
        try:
            self._serial_conn.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AnomalyDetectionNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Syntax-check the node**

```bash
source /opt/ros/humble/setup.bash
python3 -m py_compile /home/peter_sha/sourcecode/smart-walker-bot/src/walker_anomaly_detection/walker_anomaly_detection/anomaly_detection_node.py
```

Expected: no output, exit code 0.

- [ ] **Step 3: Write the launch file**

Create (overwrite) `src/walker_anomaly_detection/launch/anomaly_detection.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='USB-serial device path for the ESP32 IMU bridge.',
    )

    anomaly_detection_node = Node(
        package='walker_anomaly_detection',
        executable='anomaly_detection_node',
        name='walker_anomaly_detection',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': 115200,
            'free_fall_threshold_g': 0.3,
            'free_fall_min_duration_s': 0.05,
            'impact_threshold_g': 2.0,
            'impact_window_s': 0.5,
            'tilt_threshold_deg': 45.0,
            'tilt_sustained_duration_s': 3.0,
        }],
    )

    return LaunchDescription([serial_port_arg, anomaly_detection_node])
```

- [ ] **Step 4: Write the automated end-to-end verification script**

Create `src/walker_anomaly_detection/tools/verify_anomaly_detection.py`:

```python
#!/usr/bin/env python3
"""Scripted end-to-end check for walker_anomaly_detection - not a
pytest test.

Fully automated: uses Python's stdlib `pty` module to create a virtual
serial device pair, points the node's serial_port param at one end,
and writes synthetic JSON-line IMU samples to the other end - no real
ESP32/IMU hardware required. See
docs/superpowers/specs/2026-08-31-walker-anomaly-detection-design.md
Sec 2.10 for why this is possible (the node's own wiring logic doesn't
depend on real hardware, only genuine sensor validation does - see
docs/bring_up.md for that separate manual step, not covered by this
script).

Usage (after `colcon build --packages-select walker_anomaly_detection`
and `source install/setup.bash` from src/):

    python3 tools/verify_anomaly_detection.py

Exits 0 and prints PASS on success, exits 1 and prints FAIL otherwise.
"""
import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VerifyNode(Node):
    def __init__(self):
        super().__init__('walker_anomaly_detection_verify')
        self.events = []
        self.create_subscription(String, '/anomaly_detected', self._on_event, 10)

    def _on_event(self, msg):
        self.events.append(json.loads(msg.data))


def _sample_line(ax, ay, az, t_ms):
    return json.dumps({
        'ax': ax, 'ay': ay, 'az': az,
        'gx': 0.0, 'gy': 0.0, 'gz': 0.0,
        'mx': 0.0, 'my': 0.0, 'mz': 0.0,
        't_ms': t_ms,
    }) + '\n'


def main():
    import pty
    controller_fd, device_fd = pty.openpty()
    device_path = os.ttyname(device_fd)

    node_process = subprocess.Popen(
        [
            'ros2', 'run', 'walker_anomaly_detection', 'anomaly_detection_node',
            '--ros-args', '-p', f'serial_port:={device_path}',
        ],
        start_new_session=True,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    os.close(device_fd)  # the node's own pyserial.Serial() opens device_path itself

    rclpy.init()
    node = VerifyNode()

    try:
        time.sleep(2.0)  # let the node declare parameters and open the serial port

        # --- Fall scenario: free-fall (low accel) then impact spike ---
        os.write(controller_fd, _sample_line(0.0, 0.0, 1.0, 0).encode())
        time.sleep(0.05)
        os.write(controller_fd, _sample_line(0.05, 0.0, 0.1, 50).encode())   # enters free-fall
        time.sleep(0.1)                                                      # exceeds 0.05s min duration
        os.write(controller_fd, _sample_line(0.05, 0.0, 0.1, 150).encode())  # confirms free-fall
        time.sleep(0.05)
        os.write(controller_fd, _sample_line(0.0, 0.0, 3.0, 200).encode())   # impact spike

        deadline = time.monotonic() + 10.0
        got_fall = False
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
            if any(e['type'] == 'fall' for e in node.events):
                got_fall = True
                break
        if not got_fall:
            print(f'FAIL: no fall event received within 10s (events so far: {node.events})')
            return 1
        print('Fall event received.')

        # --- Tilt scenario: sustained tilt past the 3.0s duration threshold ---
        # tilt_from_accel_deg(9.8, 0.0, 0.0) == 90 degrees, well past the 45-degree default.
        for i in range(40):  # ~4s at 0.1s spacing, past the 3.0s sustained-duration default
            os.write(controller_fd, _sample_line(9.8, 0.0, 0.0, 300 + i * 100).encode())
            time.sleep(0.1)
            rclpy.spin_once(node, timeout_sec=0.1)

        deadline = time.monotonic() + 10.0
        got_tilt = False
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
            if any(e['type'] == 'tilt' for e in node.events):
                got_tilt = True
                break
        if not got_tilt:
            print(f'FAIL: no tilt event received within 10s (events so far: {node.events})')
            return 1
        print('Tilt event received.')

        print('PASS: fall event and tilt event both verified via a virtual serial pair')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.close(controller_fd)
        try:
            os.killpg(os.getpgid(node_process.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            node_process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(node_process.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            node_process.wait()


if __name__ == '__main__':
    sys.exit(main())
```

- [ ] **Step 5: Syntax-check the verification script**

```bash
source /opt/ros/humble/setup.bash
python3 -m py_compile /home/peter_sha/sourcecode/smart-walker-bot/src/walker_anomaly_detection/tools/verify_anomaly_detection.py
```

Expected: no output, exit code 0.

- [ ] **Step 6: Build the full package**

```bash
source /opt/ros/humble/setup.bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src
PYTHONNOUSERSITE=1 colcon build --packages-select walker_anomaly_detection --symlink-install
source install/setup.bash
```

Expected: `Summary: 1 package finished`.

- [ ] **Step 7: Run the end-to-end verification**

```bash
source /opt/ros/humble/setup.bash
source /home/peter_sha/sourcecode/smart-walker-bot/src/install/setup.bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_anomaly_detection

python3 tools/verify_anomaly_detection.py
echo "verify_anomaly_detection.py exit code: $?"
```

Expected: prints `Fall event received.`, then `Tilt event received.`, then
`PASS: fall event and tilt event both verified via a virtual serial pair`, exit code `0`.

If the fall check fails, the timing sleeps between synthetic sample writes (Step 4's
`time.sleep(0.05)`/`time.sleep(0.1)` calls) may need widening — this is real wall-clock timing
against the node's actual `self.get_clock().now()`, not simulated time, so there's some
inherent (if generously margined) timing sensitivity. If it fails consistently rather than
flakily, check for a logic mismatch against `FallDetector`'s actual behavior first before
assuming it's a timing issue.

Check `ps aux | grep anomaly_detection_node` afterward to confirm no orphaned process (the
`start_new_session=True` + `killpg` pattern mirrors `walker_llm_bridge`'s own proven fix for
`ros2 run`'s internal fork-not-exec behavior).

- [ ] **Step 8: Verify the malformed-serial-data path doesn't crash the node**

Confirm the node tolerates a burst of garbage on the serial line without dying (a real ESP32
could send a partial/corrupted line during a USB hiccup):

```bash
source /opt/ros/humble/setup.bash
source /home/peter_sha/sourcecode/smart-walker-bot/src/install/setup.bash
python3 - <<'PYEOF'
import os
import pty
import subprocess
import time

controller_fd, device_fd = pty.openpty()
device_path = os.ttyname(device_fd)

node_process = subprocess.Popen(
    ['ros2', 'run', 'walker_anomaly_detection', 'anomaly_detection_node',
     '--ros-args', '-p', f'serial_port:={device_path}'],
    start_new_session=True,
)
os.close(device_fd)
time.sleep(2.0)

os.write(controller_fd, b'not json at all\n')
os.write(controller_fd, b'{"ax": 0.1\n')  # truncated
time.sleep(1.0)

still_running = node_process.poll() is None
print(f"Node still running after garbage input: {still_running}")

os.close(controller_fd)
node_process.terminate()
node_process.wait(timeout=5.0)
PYEOF
```

Expected: `Node still running after garbage input: True`. If the node crashed, `parse_sample_line`
isn't being called correctly or `read_samples` isn't skipping `None` results as designed —
revisit Task 2.

- [ ] **Step 9: Commit**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot
git add src/walker_anomaly_detection/walker_anomaly_detection/anomaly_detection_node.py \
        src/walker_anomaly_detection/launch/anomaly_detection.launch.py \
        src/walker_anomaly_detection/tools/verify_anomaly_detection.py
git commit -m "$(cat <<'EOF'
Add walker_anomaly_detection ROS2 node, launch file, and E2E verification

anomaly_detection_node.py wires read_samples + FallDetector +
TiltDetector together, publishing /anomaly_detected on a detected
event. Verified end-to-end via a pty-backed virtual serial pair - no
real ESP32/IMU hardware required, a genuine improvement over requiring
manual hardware verification for the node's own wiring logic. Also
confirmed the node tolerates malformed serial data without crashing.
EOF
)"
```

---

## Task 6: Firmware, Hardware Bring-Up Docs, and Repo-Level Doc Updates

**Files:**
- Create: `src/walker_anomaly_detection/firmware/imu_reader.py`
- Create: `src/walker_anomaly_detection/docs/bring_up.md`
- Modify: `src/README.md`
- Modify: `CLAUDE.md`

**Interfaces:**
- None — this task produces on-device firmware (not importable by anything in this plan) and documentation. It's the last task in this plan.

**Note on scope:** unlike every other task in this plan, this one has no automated verification —
`firmware/imu_reader.py` is genuinely untestable except on real hardware (a thin I2C register
read loop, per spec §2.2), and the ESP32 isn't wired up yet ("a small project of its own," per
the user). This task's acceptance criteria are "the code is written and self-consistent" and
"the bring-up doc accurately describes what's needed," not "it runs" — that verification happens
separately, later, when the hardware exists. Do not attempt to acquire or simulate real hardware
to satisfy this task.

- [ ] **Step 1: Write the firmware**

Create `src/walker_anomaly_detection/firmware/imu_reader.py`:

```python
"""ESP32 firmware (MicroPython): reads an MPU-9250 9-axis IMU over I2C
and streams JSON-line samples over USB serial (MicroPython's REPL/UART0
doubles as the USB-serial connection on typical ESP32 boards). No
detection logic here - untestable except on real hardware, mirrors
walker_safety/firmware/main.py's role as the hardware-facing entry
point with no pure logic of its own. See
docs/superpowers/specs/2026-08-31-walker-anomaly-detection-design.md
Sec 2.2, 2.3.

Targets the MPU-9250 register map specifically. If the actual chip in
hand turns out to be an ICM-20948 instead, this file's register
addresses and scaling need to be swapped for that chip's (different)
register map - the exact chip wasn't confirmed before this was
written (design spec Sec 1), same "placeholder now, adjust at bring-up"
treatment walker_motor_driver's physical constants got.
"""
import time

import ujson
from machine import I2C, Pin

_MPU9250_ADDR = 0x68
_PWR_MGMT_1 = 0x6B
_ACCEL_XOUT_H = 0x3B
_ACCEL_FS_SCALE_LSB_PER_G = 16384.0        # default +/-2g full-scale range
_GYRO_FS_SCALE_LSB_PER_DEG_S = 131.0       # default +/-250 deg/s full-scale range

_SAMPLE_INTERVAL_MS = 20  # ~50 Hz


def _read_word_signed(i2c, addr, reg):
    high, low = i2c.readfrom_mem(addr, reg, 2)
    value = (high << 8) | low
    if value >= 0x8000:
        value -= 0x10000
    return value


def _wake_up(i2c):
    i2c.writeto_mem(_MPU9250_ADDR, _PWR_MGMT_1, bytes([0x00]))


def _read_sample(i2c):
    ax_raw = _read_word_signed(i2c, _MPU9250_ADDR, _ACCEL_XOUT_H)
    ay_raw = _read_word_signed(i2c, _MPU9250_ADDR, _ACCEL_XOUT_H + 2)
    az_raw = _read_word_signed(i2c, _MPU9250_ADDR, _ACCEL_XOUT_H + 4)
    # ACCEL_XOUT_H + 6/7 is temperature (2 bytes) - skipped, not used.
    gx_raw = _read_word_signed(i2c, _MPU9250_ADDR, _ACCEL_XOUT_H + 8)
    gy_raw = _read_word_signed(i2c, _MPU9250_ADDR, _ACCEL_XOUT_H + 10)
    gz_raw = _read_word_signed(i2c, _MPU9250_ADDR, _ACCEL_XOUT_H + 12)

    return {
        'ax': ax_raw / _ACCEL_FS_SCALE_LSB_PER_G,
        'ay': ay_raw / _ACCEL_FS_SCALE_LSB_PER_G,
        'az': az_raw / _ACCEL_FS_SCALE_LSB_PER_G,
        'gx': gx_raw / _GYRO_FS_SCALE_LSB_PER_DEG_S,
        'gy': gy_raw / _GYRO_FS_SCALE_LSB_PER_DEG_S,
        'gz': gz_raw / _GYRO_FS_SCALE_LSB_PER_DEG_S,
        # Magnetometer (AK8963, behind the MPU-9250's I2C bypass) isn't
        # wired up this first pass - stream zeros so the JSON shape
        # always matches the design spec's protocol (Sec 2.3), even
        # though nothing consumes these fields yet (Sec 2.3's own note).
        'mx': 0.0,
        'my': 0.0,
        'mz': 0.0,
        't_ms': time.ticks_ms(),
    }


def main():
    i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
    _wake_up(i2c)
    time.sleep_ms(100)

    while True:
        sample = _read_sample(i2c)
        print(ujson.dumps(sample))
        time.sleep_ms(_SAMPLE_INTERVAL_MS)


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Write the bring-up doc**

Create `src/walker_anomaly_detection/docs/bring_up.md`:

```markdown
# walker_anomaly_detection Hardware Bring-Up

Not yet done as of this package's initial build — wiring the ESP32 is "a small project of its
own." This document records what's needed when that happens.

## Hardware

- An ESP32 dev board, flashed with MicroPython.
- An MPU-9250-style 9-axis IMU breakout (or ICM-20948 — see `firmware/imu_reader.py`'s header
  comment if so; the register map differs and the firmware needs adjusting).

## Wiring (I2C)

| IMU pin | ESP32 pin |
|---|---|
| VCC | 3.3V |
| GND | GND |
| SCL | GPIO22 (default I2C0 SCL) |
| SDA | GPIO21 (default I2C0 SDA) |

Adjust `firmware/imu_reader.py`'s `Pin(22)`/`Pin(21)` if using different GPIOs.

## Flashing MicroPython

1. Install `esptool` and flash the MicroPython firmware for ESP32 (see micropython.org's ESP32
   download page for the current `.bin`).
2. Copy `firmware/imu_reader.py` onto the device as `main.py` so it runs automatically on boot
   (e.g. `mpremote cp firmware/imu_reader.py :main.py`, or `ampy`/`rshell`).
3. Connect the ESP32 to this workstation via USB — it should enumerate as `/dev/ttyUSB0` or
   `/dev/ttyACM0` (check `dmesg` after plugging in). Update the `serial_port` launch argument if
   it's different from the default.

## Verifying the sensor itself works

Once wired and flashed:

```bash
# from this workstation, with nothing else using the serial port
python3 -c "
import serial
s = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)
for _ in range(10):
    print(s.readline().decode().strip())
"
```

Expected: ten lines of JSON, each with `ax`/`ay`/`az` roughly summing to ~1g in magnitude when the
board is stationary (e.g. `az` close to 1.0 if the IMU is lying flat, others close to 0.0). If
values look wildly wrong (all zeros, all identical, or magnitude far from 1g), check the I2C
wiring and the `_MPU9250_ADDR`/register constants in `firmware/imu_reader.py` against your
specific breakout board's datasheet.

Once real samples look sane:

```bash
ros2 launch walker_anomaly_detection anomaly_detection.launch.py
```

and, in another terminal, `ros2 topic echo /anomaly_detected` while manually performing a real
drop/catch motion and a real sustained-tilt motion with the IMU in hand, confirming each produces
the expected `fall`/`tilt` event. This manual step is the only thing
`tools/verify_anomaly_detection.py`'s automated `pty`-based check (design spec §2.10) can't
cover — it proves the node's wiring works, not that a real accelerometer produces sensible
values.
```

- [ ] **Step 3: Update src/README.md**

Read `src/README.md`, then in the "Planned packages" list, after the `walker_companion_app` entry,
add:

```markdown
- **Built (pure logic + node; hardware bring-up pending).** **`walker_anomaly_detection`** —
  fall/anomaly detection via a real ESP32-streamed 9-axis IMU: free-fall+impact and
  sustained-tilt detection, publishing `/anomaly_detected` alerts. A new addition beyond the
  original five-package roadmap — root `README.md` §5.2 originally assigned this to
  `walker_motor_driver`, but it was never implemented there. First package developed against
  real hardware rather than simulation; see the package's own README and `docs/bring_up.md`.
  Wiring `/anomaly_detected` into `walker_companion_app`'s dashboard is a separate follow-up.
```

And after the existing "Build/test `walker_companion_app`" bash block, add:

```markdown
Build/test `walker_anomaly_detection`:

\`\`\`bash
source /opt/ros/humble/setup.bash
cd src
PYTHONNOUSERSITE=1 colcon build --packages-select walker_anomaly_detection --symlink-install
source install/setup.bash

python3 -m pytest walker_anomaly_detection/test/ -v   # pure-module unit tests, no ROS sourcing needed
\`\`\`
```

- [ ] **Step 4: Update CLAUDE.md**

Read `CLAUDE.md`, then in the "Project status" section change:

```markdown
Five packages exist under `src/`: `walker_safety` (E-stop wiring docs + Pico watchdog
firmware - not a colcon package, see its own README), `walker_motor_driver` (a real
`ament_python` ROS2 node - differential-drive motor control backed by a simulator until real
hardware exists), `walker_nav` (a real `ament_python` ROS2 package - a simulated LiDAR
feeding `slam_toolbox` for mapping, backed by a fixed hardcoded room until real hardware
exists; Nav2 navigates autonomously against that live map, using `nav2_bringup`'s own
navigation stack), `walker_llm_bridge` (a real `ament_python` ROS2 package - a
text-based conversational bridge to an Ollama server; real STT/TTS and nav-goal
translation still deferred to hardware bring-up), and `walker_companion_app` (a real
`ament_python` ROS2 package - a local-network web dashboard over a stdlib HTTP server,
serving robot pose, Nav2 status, a live map, and the conversation log; fall/anomaly
alerts are a static placeholder, no IMU subsystem exists yet).
```

to:

```markdown
Six packages exist under `src/`: `walker_safety` (E-stop wiring docs + Pico watchdog
firmware - not a colcon package, see its own README), `walker_motor_driver` (a real
`ament_python` ROS2 node - differential-drive motor control backed by a simulator until real
hardware exists), `walker_nav` (a real `ament_python` ROS2 package - a simulated LiDAR
feeding `slam_toolbox` for mapping, backed by a fixed hardcoded room until real hardware
exists; Nav2 navigates autonomously against that live map, using `nav2_bringup`'s own
navigation stack), `walker_llm_bridge` (a real `ament_python` ROS2 package - a
text-based conversational bridge to an Ollama server; real STT/TTS and nav-goal
translation still deferred to hardware bring-up), `walker_companion_app` (a real
`ament_python` ROS2 package - a local-network web dashboard over a stdlib HTTP server,
serving robot pose, Nav2 status, a live map, and the conversation log), and
`walker_anomaly_detection` (a real `ament_python` ROS2 package - fall/anomaly detection via
a real ESP32-streamed 9-axis IMU, the first package developed against real hardware rather
than simulation; ESP32 wiring/bring-up is still pending, but the node's own logic is fully
verified via a pty-backed virtual serial pair - see the package's own README). Wiring
`walker_anomaly_detection`'s alerts into `walker_companion_app`'s dashboard is a separate,
not-yet-started follow-up.
```

And after the existing "Build/test `walker_companion_app`" bash block, add:

```markdown
Build/test `walker_anomaly_detection`:

\`\`\`bash
source /opt/ros/humble/setup.bash
cd src
PYTHONNOUSERSITE=1 colcon build --packages-select walker_anomaly_detection --symlink-install
python3 -m pytest walker_anomaly_detection/test/ -v   # pure-module unit tests, no ROS sourcing needed
\`\`\`
```

And change:

```markdown
All five planned Phase 1 packages now exist.
```

to:

```markdown
All five originally planned Phase 1 packages exist, plus `walker_anomaly_detection` — a sixth
package added beyond the original roadmap (see its own README for why).
```

- [ ] **Step 5: Commit**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot
git add src/walker_anomaly_detection/firmware/imu_reader.py \
        src/walker_anomaly_detection/docs/bring_up.md \
        src/README.md \
        CLAUDE.md
git commit -m "$(cat <<'EOF'
Add walker_anomaly_detection ESP32 firmware and bring-up docs

firmware/imu_reader.py targets the MPU-9250 register map (adjust for
ICM-20948 if that's the actual chip) - a thin I2C-to-serial bridge
with no detection logic, untestable except on real hardware.
docs/bring_up.md records the wiring/flashing/verification steps for
when the ESP32 is actually wired up. Updates src/README.md and
CLAUDE.md to reflect the new (sixth, beyond-roadmap) package.
EOF
)"
```

---

## Self-Review Notes

- **Spec coverage:** §2.1 (new package, not folded into motor_driver) — Task 1. §2.2 (firmware is a thin bridge, all logic in ROS2-side Python) — Task 6 (firmware) has zero detection logic; Tasks 2-4 hold all of it. §2.3 (JSON-lines protocol, units, parse_sample_line signature) — Task 2, and Task 6's firmware emits exactly this shape/units. §2.4 (FallDetector, one-shot semantics) — Task 3, consumed by Task 5's node. §2.5 (TiltDetector, accelerometer-only, one-shot semantics refined during planning) — Task 4, consumed by Task 5's node. §2.6 (no coupling to walker_safety/walker_motor_driver) — never referenced anywhere in this plan; stated explicitly in Task 1's README. §2.7 (alert topic, JSON-in-String, no custom .msg) — Task 5's node. §2.8 (no simulated IMU backend as a package feature) — Tasks 1-6 never build one; Task 5's pty-based verify script is explicitly a test-only technique per §2.10, not a package feature. §2.9 (companion_app wiring deferred) — never touched anywhere in this plan. §2.10 (automated pty-based node verification) — Task 5 Steps 4-8. §3 (file structure) — matches exactly. §4 (interface: params/topic) — Task 5's `declare_parameter` calls and publisher match the spec's table verbatim. §5 (testing approach) — Tasks 2-4 are pytest-TDD; Task 5's node gets the fully-automated pty-based E2E check per the corrected §2.10/§5; Task 6's firmware gets the genuinely-manual bring-up doc, explicitly not gated on this session's task completion.
- **Placeholder scan:** no TBD/TODO in any step. Task 1 Step 3's placeholder launch file (used only if Task 1 Step 6's build fails without it) is a real, valid, working `LaunchDescription([])`, and gets overwritten by Task 5's real launch file regardless. Task 6's explicit "no automated verification" note is a documented scope boundary, not an unwritten stub — the code and docs it produces are both complete as written.
- **Type/name consistency:** `parse_sample_line(line) -> dict | None` and `read_samples(serial_conn, on_sample)` used identically in Task 2's tests and Task 5's node (`read_samples(self._serial_conn, self._on_sample)`). `FallDetector(free_fall_threshold_g, free_fall_min_duration_s, impact_threshold_g, impact_window_s)` / `.update(accel_magnitude_g, now_s)` used identically in Task 3's tests and Task 5's node. `tilt_from_accel_deg(ax, ay, az)` and `TiltDetector(tilt_threshold_deg, tilt_sustained_duration_s)` / `.update(tilt_deg, now_s)` used identically in Task 4's tests and Task 5's node. Parameter names (`serial_port`, `baud_rate`, `free_fall_threshold_g`, `free_fall_min_duration_s`, `impact_threshold_g`, `impact_window_s`, `tilt_threshold_deg`, `tilt_sustained_duration_s`) match between Task 5's node's `declare_parameter` calls and its launch file's `parameters` dict. Sample dict keys (`ax, ay, az, gx, gy, gz, mx, my, mz, t_ms`) match identically across Task 2's `parse_sample_line`, Task 5's node's `_on_sample`, Task 5's verify script's `_sample_line`, and Task 6's firmware's `_read_sample` output. Topic name `/anomaly_detected` matches between Task 5's node and its own verify script's subscription.
