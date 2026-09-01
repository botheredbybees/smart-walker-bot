# walker_gait_metrics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Compute step count and step length from `walker_anomaly_detection`'s IMU stream and
`walker_motor_driver`'s odometry, publish them on a new topic via a new `walker_gait_metrics`
package, and surface them on `walker_companion_app`'s dashboard.

**Architecture:** `walker_anomaly_detection` gains a small, additive change: it republishes every
parsed IMU sample as JSON on a new `/imu/raw_sample` topic, alongside its existing
`/anomaly_detected` publisher. A new `walker_gait_metrics` package subscribes to that topic and to
`walker_motor_driver`'s existing `/odom`, feeds both into a pure `GaitTracker` (composing a pure
`StepCounter`), and publishes cumulative metrics as JSON on `/gait_metrics` on a 1 Hz timer.
`walker_companion_app` subscribes to `/gait_metrics` and folds it into its existing `/api/status`
endpoint and dashboard page, the same pattern pose/nav-status already use.

**Tech Stack:** Python 3, ROS2 Humble (`rclpy`), pytest, `ament_python`/`colcon`.

**Spec:** `docs/superpowers/specs/2026-09-01-walker-gait-metrics-design.md`

## Global Constraints

- New package `walker_gait_metrics` — not folded into `walker_anomaly_detection` (spec §2.1).
- `/imu/raw_sample`: `std_msgs/String`, JSON payload — the exact existing sample dict shape
  (`ax, ay, az, gx, gy, gz, mx, my, mz, t_ms`), not `sensor_msgs/Imu` (spec §2.2/§2.3).
- `/gait_metrics`: `std_msgs/String`, JSON payload `{"step_count": int, "total_distance_m":
  float, "avg_step_length_m": float, "timestamp": float}`, published on a periodic timer (not
  event-only) (spec §2.8).
- `avg_step_length_m` must be `0.0` — never a `ZeroDivisionError` — when `step_count == 0`
  (spec §2.6).
- No persistence or daily reset of cumulative metrics this pass (spec §2.7).
- No coupling to `walker_safety`; `/odom` is consumed read-only, the same relationship
  `walker_nav` already has to `walker_motor_driver` (spec §2.10).
- `walker_llm_bridge` conversational wiring, grip strength, and Kinect gait analysis are out of
  scope — the first is a named follow-up, the other two are tracked separately in
  `docs/ideas-backlog.md` (spec §6). Do not implement any of them in this plan.
- Step detection from the frame-mounted IMU is a genuine, unresolved real-world question (spec
  §2.5) — no task or test claims it's validated against real hardware. Pytest validates the
  `StepCounter`/`GaitTracker` algorithm against synthetic sequences only; real-world
  detectability is a bring-up-time finding (Task 5).

---

## Task 1: `step_counter.py` — pure step detection

**Files:**
- Create: `src/walker_gait_metrics/walker_gait_metrics/step_counter.py`
- Test: `src/walker_gait_metrics/test/test_step_counter.py`

**Interfaces:**
- Produces: `StepCounter(step_threshold_g, min_step_interval_s)`;
  `.update(accel_magnitude_g: float, now_s: float) -> bool` — Task 2's `GaitTracker` composes
  this directly.

This is the first file in a brand-new package. Package scaffolding (`package.xml`, `setup.py`,
etc.) is created in Task 3, once there's a node to register as an entry point — but the pure
modules and their tests don't need any of that machinery to run with plain `pytest`, so they come
first, same ordering `walker_anomaly_detection` used.

- [ ] **Step 1: Create the test directory and a minimal conftest**

Create `src/walker_gait_metrics/test/conftest.py`:

```python
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
```

Create empty `src/walker_gait_metrics/walker_gait_metrics/__init__.py` (just so the package
directory is importable as `walker_gait_metrics` once `step_counter.py` exists inside it).

- [ ] **Step 2: Write the failing tests**

Create `src/walker_gait_metrics/test/test_step_counter.py`:

```python
from walker_gait_metrics.step_counter import StepCounter

STEP_THRESHOLD_G = 1.2
MIN_STEP_INTERVAL_S = 0.3


def _make_counter():
    return StepCounter(step_threshold_g=STEP_THRESHOLD_G, min_step_interval_s=MIN_STEP_INTERVAL_S)


def test_values_below_threshold_never_trigger():
    counter = _make_counter()
    for t in (0.0, 0.1, 0.2, 0.3):
        assert counter.update(1.0, t) is False


def test_first_crossing_triggers_immediately():
    counter = _make_counter()
    assert counter.update(1.5, 0.0) is True


def test_crossings_spaced_past_min_interval_each_count():
    counter = _make_counter()
    assert counter.update(1.5, 0.0) is True
    assert counter.update(1.5, 0.3) is True
    assert counter.update(1.5, 0.6) is True


def test_crossings_closer_than_min_interval_count_once():
    counter = _make_counter()
    assert counter.update(1.5, 0.0) is True
    assert counter.update(1.5, 0.1) is False   # only 0.1s since last step, debounced
    assert counter.update(1.5, 0.29) is False  # still within debounce window


def test_crossing_exactly_at_min_interval_boundary_counts():
    counter = _make_counter()
    counter.update(1.5, 0.0)
    assert counter.update(1.5, 0.3) is True  # exactly min_step_interval_s later


def test_a_debounced_sample_does_not_reset_the_debounce_window():
    counter = _make_counter()
    counter.update(1.5, 0.0)                   # step 1
    counter.update(1.5, 0.1)                   # debounced, must not move the "last step" time
    assert counter.update(1.5, 0.35) is True   # 0.35s since step 1 (not since the debounced 0.1)
```

- [ ] **Step 3: Run the tests to verify they fail**

```bash
cd src/walker_gait_metrics
python3 -m pytest test/test_step_counter.py -v
```

Expected: FAIL / ERROR — `ModuleNotFoundError: No module named 'walker_gait_metrics.step_counter'`.

- [ ] **Step 4: Implement `StepCounter`**

Create `src/walker_gait_metrics/walker_gait_metrics/step_counter.py`:

```python
"""Pure step-detection for walker_gait_metrics. No ROS or hardware
imports - shared between gait_metrics_node.py and the pytest suite. See
docs/superpowers/specs/2026-09-01-walker-gait-metrics-design.md Sec 2.4.
"""


class StepCounter:
    """Tracks accelerometer magnitude (g) across a stream of samples.
    Detects a step whenever magnitude crosses above step_threshold_g,
    debounced by min_step_interval_s so one footstep's impact-and-settle
    isn't counted twice."""

    def __init__(self, step_threshold_g, min_step_interval_s):
        self._step_threshold_g = step_threshold_g
        self._min_step_interval_s = min_step_interval_s
        self._last_step_s = None

    def update(self, accel_magnitude_g, now_s):
        """Call once per sample. Returns True exactly on the sample
        confirming a new step, False otherwise."""
        if accel_magnitude_g < self._step_threshold_g:
            return False
        if self._last_step_s is not None and now_s - self._last_step_s < self._min_step_interval_s:
            return False
        self._last_step_s = now_s
        return True
```

- [ ] **Step 5: Run the tests to verify they pass**

```bash
cd src/walker_gait_metrics
python3 -m pytest test/ -v
```

Expected: PASS — all 6 tests.

- [ ] **Step 6: Commit**

```bash
git add src/walker_gait_metrics/walker_gait_metrics/__init__.py \
        src/walker_gait_metrics/walker_gait_metrics/step_counter.py \
        src/walker_gait_metrics/test/conftest.py \
        src/walker_gait_metrics/test/test_step_counter.py
git commit -m "walker_gait_metrics: add pure StepCounter"
```

---

## Task 2: `gait_tracker.py` — pure cumulative gait metrics

**Files:**
- Create: `src/walker_gait_metrics/walker_gait_metrics/gait_tracker.py`
- Test: `src/walker_gait_metrics/test/test_gait_tracker.py`

**Interfaces:**
- Consumes: `StepCounter(step_threshold_g, min_step_interval_s)` /
  `.update(accel_magnitude_g, now_s) -> bool` from Task 1.
- Produces: `GaitTracker(step_threshold_g, min_step_interval_s)`;
  `.on_imu_sample(sample: dict, now_s: float)`, `.on_odom_pose(x_m: float, y_m: float)`, and
  read-only properties `step_count: int`, `total_distance_m: float`, `avg_step_length_m: float`
  — Task 3's `gait_metrics_node.py` wires this to the two ROS subscriptions and a publish timer.

- [ ] **Step 1: Write the failing tests**

Create `src/walker_gait_metrics/test/test_gait_tracker.py`:

```python
from walker_gait_metrics.gait_tracker import GaitTracker

STEP_THRESHOLD_G = 1.2
MIN_STEP_INTERVAL_S = 0.3


def _make_tracker():
    return GaitTracker(step_threshold_g=STEP_THRESHOLD_G, min_step_interval_s=MIN_STEP_INTERVAL_S)


def _sample(ax, ay, az):
    return {'ax': ax, 'ay': ay, 'az': az}


def test_step_count_starts_at_zero():
    tracker = _make_tracker()
    assert tracker.step_count == 0


def test_on_imu_sample_below_threshold_does_not_increment_step_count():
    tracker = _make_tracker()
    tracker.on_imu_sample(_sample(0.0, 0.0, 1.0), 0.0)
    assert tracker.step_count == 0


def test_on_imu_sample_above_threshold_increments_step_count():
    tracker = _make_tracker()
    tracker.on_imu_sample(_sample(0.0, 0.0, 1.5), 0.0)
    assert tracker.step_count == 1


def test_multiple_debounced_steps_increment_step_count_correctly():
    tracker = _make_tracker()
    tracker.on_imu_sample(_sample(0.0, 0.0, 1.5), 0.0)
    tracker.on_imu_sample(_sample(0.0, 0.0, 1.5), 0.1)  # debounced
    tracker.on_imu_sample(_sample(0.0, 0.0, 1.5), 0.3)  # counts
    assert tracker.step_count == 2


def test_first_odom_pose_adds_no_distance():
    tracker = _make_tracker()
    tracker.on_odom_pose(1.0, 2.0)
    assert tracker.total_distance_m == 0.0


def test_odom_poses_accumulate_distance():
    tracker = _make_tracker()
    tracker.on_odom_pose(0.0, 0.0)
    tracker.on_odom_pose(3.0, 4.0)  # 3-4-5 triangle: 5.0m
    assert tracker.total_distance_m == 5.0
    tracker.on_odom_pose(3.0, 4.0)  # no movement
    assert tracker.total_distance_m == 5.0
    tracker.on_odom_pose(6.0, 8.0)  # another 5.0m
    assert tracker.total_distance_m == 10.0


def test_avg_step_length_is_zero_when_no_steps_taken():
    tracker = _make_tracker()
    tracker.on_odom_pose(0.0, 0.0)
    tracker.on_odom_pose(3.0, 4.0)
    assert tracker.avg_step_length_m == 0.0


def test_avg_step_length_computes_distance_over_steps():
    tracker = _make_tracker()
    tracker.on_odom_pose(0.0, 0.0)
    tracker.on_odom_pose(10.0, 0.0)  # 10m traveled
    tracker.on_imu_sample(_sample(0.0, 0.0, 1.5), 0.0)
    tracker.on_imu_sample(_sample(0.0, 0.0, 1.5), 0.3)
    tracker.on_imu_sample(_sample(0.0, 0.0, 1.5), 0.6)
    tracker.on_imu_sample(_sample(0.0, 0.0, 1.5), 0.9)
    tracker.on_imu_sample(_sample(0.0, 0.0, 1.5), 1.2)  # 5 steps
    assert tracker.avg_step_length_m == 2.0
```

- [ ] **Step 2: Run the tests to verify they fail**

```bash
cd src/walker_gait_metrics
python3 -m pytest test/test_gait_tracker.py -v
```

Expected: FAIL / ERROR — `ModuleNotFoundError: No module named 'walker_gait_metrics.gait_tracker'`.

- [ ] **Step 3: Implement `GaitTracker`**

Create `src/walker_gait_metrics/walker_gait_metrics/gait_tracker.py`:

```python
"""Pure cumulative gait-metrics tracking for walker_gait_metrics. No ROS
or hardware imports - shared between gait_metrics_node.py and the
pytest suite. See
docs/superpowers/specs/2026-09-01-walker-gait-metrics-design.md Sec 2.6.
"""
import math

from walker_gait_metrics.step_counter import StepCounter


class GaitTracker:
    """Combines step counting (from IMU samples) with distance
    accumulation (from odometry poses) into cumulative gait metrics:
    step_count, total_distance_m, and avg_step_length_m =
    total_distance_m / step_count (0.0 while step_count is 0, never a
    ZeroDivisionError)."""

    def __init__(self, step_threshold_g, min_step_interval_s):
        self._step_counter = StepCounter(step_threshold_g, min_step_interval_s)
        self._step_count = 0
        self._total_distance_m = 0.0
        self._last_pose = None

    def on_imu_sample(self, sample, now_s):
        """sample: dict with at least ax, ay, az (g). Feeds accelerometer
        magnitude into the internal StepCounter; increments step_count
        on a detected step."""
        accel_magnitude_g = math.sqrt(sample['ax'] ** 2 + sample['ay'] ** 2 + sample['az'] ** 2)
        if self._step_counter.update(accel_magnitude_g, now_s):
            self._step_count += 1

    def on_odom_pose(self, x_m, y_m):
        """Accumulates total_distance_m from the previous call's pose.
        The first call has no previous pose to diff against, so it only
        seeds the starting point and adds no distance."""
        if self._last_pose is not None:
            prev_x, prev_y = self._last_pose
            self._total_distance_m += math.hypot(x_m - prev_x, y_m - prev_y)
        self._last_pose = (x_m, y_m)

    @property
    def step_count(self):
        return self._step_count

    @property
    def total_distance_m(self):
        return self._total_distance_m

    @property
    def avg_step_length_m(self):
        if self._step_count == 0:
            return 0.0
        return self._total_distance_m / self._step_count
```

- [ ] **Step 4: Run the tests to verify they pass**

```bash
cd src/walker_gait_metrics
python3 -m pytest test/ -v
```

Expected: PASS — all tests (Task 1's 6 plus this task's 8, 14 total).

- [ ] **Step 5: Commit**

```bash
git add src/walker_gait_metrics/walker_gait_metrics/gait_tracker.py \
        src/walker_gait_metrics/test/test_gait_tracker.py
git commit -m "walker_gait_metrics: add pure GaitTracker"
```

---

## Task 3: `walker_gait_metrics` package scaffold + `gait_metrics_node.py`

**Files:**
- Create: `src/walker_gait_metrics/package.xml`
- Create: `src/walker_gait_metrics/setup.py`
- Create: `src/walker_gait_metrics/setup.cfg`
- Create: `src/walker_gait_metrics/resource/walker_gait_metrics` (empty)
- Create: `src/walker_gait_metrics/walker_gait_metrics/gait_metrics_node.py`
- Create: `src/walker_gait_metrics/launch/gait_metrics.launch.py`
- Create: `src/walker_gait_metrics/README.md`

**Interfaces:**
- Consumes: `GaitTracker(step_threshold_g, min_step_interval_s)` from Task 2.
- Produces: the `walker_gait_metrics` `ament_python` package itself, buildable via
  `colcon build --packages-select walker_gait_metrics`; the `/imu/raw_sample` subscription
  (Task 5 makes `walker_anomaly_detection` actually publish it) and `/odom` subscription
  (already published by `walker_motor_driver`); the `/gait_metrics` topic Task 4 and Task 6
  both consume.

This node isn't pytest-tested (it imports `rclpy` at module level, matching every other `rclpy`
node in this project). Its own check is a build + a timed smoke run; full behavioral
verification is Task 4's job.

- [ ] **Step 1: Create the package manifest and build files**

Create `src/walker_gait_metrics/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>walker_gait_metrics</name>
  <version>0.0.1</version>
  <description>Wellness gait metrics (step count, step length) for smart-walker-bot, derived from walker_anomaly_detection's IMU stream and walker_motor_driver's odometry.</description>
  <maintainer email="botheredbybees@gmail.com">botheredbybees</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>nav_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

Create `src/walker_gait_metrics/setup.py`:

```python
from setuptools import find_packages, setup

package_name = 'walker_gait_metrics'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gait_metrics.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='botheredbybees',
    maintainer_email='botheredbybees@gmail.com',
    description=(
        "Wellness gait metrics (step count, step length) for smart-walker-bot, derived from "
        "walker_anomaly_detection's IMU stream and walker_motor_driver's odometry."
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gait_metrics_node = walker_gait_metrics.gait_metrics_node:main',
        ],
    },
)
```

Create `src/walker_gait_metrics/setup.cfg`:

```
[develop]
script_dir=$base/lib/walker_gait_metrics
[install]
install_scripts=$base/lib/walker_gait_metrics
```

Create an empty file at `src/walker_gait_metrics/resource/walker_gait_metrics` (zero bytes —
`touch` it; this is `ament`'s package-marker file, matching every other package's identical
empty marker).

- [ ] **Step 2: Implement `gait_metrics_node.py`**

Create `src/walker_gait_metrics/walker_gait_metrics/gait_metrics_node.py`:

```python
"""walker_gait_metrics's ROS2 node: subscribes to walker_anomaly_detection's
/imu/raw_sample and walker_motor_driver's /odom, feeds both into a
GaitTracker, and publishes cumulative gait metrics on /gait_metrics on a
timer. See
docs/superpowers/specs/2026-09-01-walker-gait-metrics-design.md for the
full design.
"""
import json

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from walker_gait_metrics.gait_tracker import GaitTracker

REQUIRED_IMU_KEYS = ('ax', 'ay', 'az')


def _parse_imu_sample(data_str):
    """Parse one /imu/raw_sample JSON payload. Returns a dict with at
    least ax, ay, az on success, or None on malformed JSON, a missing
    key, or a non-numeric value - never raises. A small, deliberate
    duplicate of walker_anomaly_detection.imu_serial.parse_sample_line's
    validation, not a cross-package import - see design spec Sec 2.2."""
    try:
        data = json.loads(data_str)
    except (ValueError, TypeError):
        return None
    if not isinstance(data, dict):
        return None
    if not all(key in data for key in REQUIRED_IMU_KEYS):
        return None
    if not all(
        isinstance(data.get(key), (int, float)) and not isinstance(data.get(key), bool)
        for key in REQUIRED_IMU_KEYS
    ):
        return None
    return data


class GaitMetricsNode(Node):
    def __init__(self):
        super().__init__('walker_gait_metrics')

        self.declare_parameter('step_threshold_g', 1.2)
        self.declare_parameter('min_step_interval_s', 0.3)
        self.declare_parameter('publish_rate_hz', 1.0)

        step_threshold_g = self.get_parameter('step_threshold_g').value
        min_step_interval_s = self.get_parameter('min_step_interval_s').value
        publish_rate_hz = self.get_parameter('publish_rate_hz').value

        if step_threshold_g <= 0:
            raise ValueError("step_threshold_g must be positive")
        if min_step_interval_s <= 0:
            raise ValueError("min_step_interval_s must be positive")
        if publish_rate_hz <= 0:
            raise ValueError("publish_rate_hz must be positive")

        self._tracker = GaitTracker(step_threshold_g, min_step_interval_s)

        self.create_subscription(String, '/imu/raw_sample', self._on_imu_sample, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self._metrics_pub = self.create_publisher(String, '/gait_metrics', 10)
        self._timer = self.create_timer(1.0 / publish_rate_hz, self._on_timer)

    def _on_imu_sample(self, msg):
        sample = _parse_imu_sample(msg.data)
        if sample is None:
            self.get_logger().warn(
                'Ignoring malformed /imu/raw_sample payload.', throttle_duration_sec=5.0,
            )
            return
        now_s = self.get_clock().now().nanoseconds / 1e9
        self._tracker.on_imu_sample(sample, now_s)

    def _on_odom(self, msg):
        x_m = msg.pose.pose.position.x
        y_m = msg.pose.pose.position.y
        self._tracker.on_odom_pose(x_m, y_m)

    def _on_timer(self):
        payload = json.dumps({
            'step_count': self._tracker.step_count,
            'total_distance_m': self._tracker.total_distance_m,
            'avg_step_length_m': self._tracker.avg_step_length_m,
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
        })
        self._metrics_pub.publish(String(data=payload))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GaitMetricsNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 3: Create the launch file**

Create `src/walker_gait_metrics/launch/gait_metrics.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

GAIT_STEP_THRESHOLD_G = 1.2
GAIT_MIN_STEP_INTERVAL_S = 0.3
GAIT_PUBLISH_RATE_HZ = 1.0


def generate_launch_description():
    gait_metrics_node = Node(
        package='walker_gait_metrics',
        executable='gait_metrics_node',
        name='walker_gait_metrics',
        output='screen',
        parameters=[{
            'step_threshold_g': GAIT_STEP_THRESHOLD_G,
            'min_step_interval_s': GAIT_MIN_STEP_INTERVAL_S,
            'publish_rate_hz': GAIT_PUBLISH_RATE_HZ,
        }],
    )

    return LaunchDescription([gait_metrics_node])
```

- [ ] **Step 4: Build and run the full pure-module regression suite**

```bash
source /opt/ros/humble/setup.bash
cd src
PYTHONNOUSERSITE=1 colcon build --packages-select walker_gait_metrics --symlink-install
```

Expected: builds cleanly, no errors.

In a separate, non-ROS-sourced shell (a prior package's implementation found that running
`python3 -m pytest` in the same shell where ROS's `setup.bash` was sourced can hit an unrelated
pre-existing workstation issue — a `lark`-module conflict in `launch_testing`'s pytest11 entry
point):

```bash
cd src/walker_gait_metrics
python3 -m pytest test/ -v
```

Expected: PASS — all 14 tests from Tasks 1-2, unaffected by this task's changes.

- [ ] **Step 5: Smoke-test the node starts cleanly**

```bash
source /opt/ros/humble/setup.bash
cd src
source install/setup.bash
timeout 3 ros2 run walker_gait_metrics gait_metrics_node
```

Expected: the node starts and runs for ~3 seconds with no traceback (no `/imu/raw_sample` or
`/odom` publisher running yet is fine — the node just idles with zero-valued metrics), then
`timeout` kills it. A traceback means the wiring is wrong — re-check Step 2.

- [ ] **Step 6: Write the package README**

Create `src/walker_gait_metrics/README.md`:

```markdown
# walker_gait_metrics

Wellness gait metrics (step count, step length) for smart-walker-bot. See
`docs/superpowers/specs/2026-09-01-walker-gait-metrics-design.md` for the full design (this is a
summary).

Real `ament_python` package — build it with
`colcon build --packages-select walker_gait_metrics` from `src/` (this repo's colcon workspace
root).

## Layout

- `walker_gait_metrics/step_counter.py` — pure Python: `StepCounter`, a threshold-crossing step
  detector with a minimum-interval debounce. No ROS import; unit-tested with pytest.
- `walker_gait_metrics/gait_tracker.py` — pure Python: `GaitTracker`, composing a `StepCounter`
  with odometry-based distance accumulation into cumulative `step_count`, `total_distance_m`, and
  `avg_step_length_m`.
- `walker_gait_metrics/gait_metrics_node.py` — the `rclpy` node: subscribes
  `walker_anomaly_detection`'s `/imu/raw_sample` and `walker_motor_driver`'s `/odom`, publishes
  `/gait_metrics` (`std_msgs/String`, JSON) on a 1 Hz timer. Parameters: `step_threshold_g`
  (default 1.2), `min_step_interval_s` (default 0.3), `publish_rate_hz` (default 1.0) — all
  placeholders pending real-hardware calibration.
- `launch/gait_metrics.launch.py` — starts `gait_metrics_node` with the above defaults.
- `tools/verify_gait_metrics.py` — a scripted (not pytest) end-to-end check: publishes synthetic
  `/imu/raw_sample` and `/odom` messages directly (no real IMU/serial hardware needed, unlike
  `walker_anomaly_detection`'s own verification script) and confirms `/gait_metrics` reflects the
  expected values.

## Running the pure-module tests

\`\`\`bash
cd src/walker_gait_metrics
python3 -m pytest test/ -v
\`\`\`

No ROS environment or colcon build needed for these.

## Step detection from a frame-mounted IMU is an open real-world question

`walker_anomaly_detection`'s IMU monitors the walker frame's own motion, not something worn by
the person. Whether a person's footsteps transmit a detectable jolt through a wheeled, motorized
frame is a genuine bring-up-time question this package's pytest suite cannot answer (it validates
`StepCounter`/`GaitTracker`'s logic against synthetic sequences only) — see
`walker_anomaly_detection/docs/bring_up.md` for the real-hardware finding once bring-up happens,
and this package's own design spec §2.5 for the reasoning.

## No coupling to walker_safety

This package only publishes an observational metric and read-only consumes `/odom` — it never
subscribes to safety topics and never publishes anything that could stop or control the robot.
```

- [ ] **Step 7: Commit**

```bash
git add src/walker_gait_metrics/package.xml \
        src/walker_gait_metrics/setup.py \
        src/walker_gait_metrics/setup.cfg \
        src/walker_gait_metrics/resource/walker_gait_metrics \
        src/walker_gait_metrics/walker_gait_metrics/gait_metrics_node.py \
        src/walker_gait_metrics/launch/gait_metrics.launch.py \
        src/walker_gait_metrics/README.md
git commit -m "walker_gait_metrics: add package scaffold and gait_metrics_node"
```

---

## Task 4: `tools/verify_gait_metrics.py` — automated end-to-end check

**Files:**
- Create: `src/walker_gait_metrics/tools/verify_gait_metrics.py`

**Interfaces:**
- Consumes: `walker_gait_metrics`'s built `gait_metrics_node` executable (Task 3); the
  `/imu/raw_sample` and `/odom` topic shapes (spec §2.3/§4); the `/gait_metrics` topic shape
  (spec §2.8).

Unlike `walker_anomaly_detection`'s verify script, this node has no serial/hardware dependency of
its own — it only consumes ROS topics — so this script publishes synthetic messages directly via
`rclpy` publishers, no `pty` trick needed.

- [ ] **Step 1: Write the script**

Create `src/walker_gait_metrics/tools/verify_gait_metrics.py`:

```python
#!/usr/bin/env python3
"""Scripted end-to-end check for walker_gait_metrics - not a pytest
test.

Fully automated: publishes synthetic /imu/raw_sample and /odom messages
directly - this node has no serial/hardware dependency of its own,
unlike walker_anomaly_detection's node, so no pty trick is needed. See
docs/superpowers/specs/2026-09-01-walker-gait-metrics-design.md Sec 2.11.

Usage (after `colcon build --packages-select walker_gait_metrics` and
`source install/setup.bash` from src/):

    python3 tools/verify_gait_metrics.py

Exits 0 and prints PASS on success, exits 1 and prints FAIL otherwise.
"""
import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

# Must match launch/gait_metrics.launch.py's defaults (this script relies
# on the node's default parameters, launched via plain `ros2 run` below).
STEP_THRESHOLD_G = 1.2
MIN_STEP_INTERVAL_S = 0.3

NUM_STEPS = 5
TOTAL_DISTANCE_M = 10.0
EXPECTED_AVG_STEP_LENGTH_M = TOTAL_DISTANCE_M / NUM_STEPS


class VerifyNode(Node):
    def __init__(self):
        super().__init__('walker_gait_metrics_verify')
        self.latest_metrics = None
        self.imu_pub = self.create_publisher(String, '/imu/raw_sample', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(String, '/gait_metrics', self._on_metrics, 10)

    def _on_metrics(self, msg):
        self.latest_metrics = json.loads(msg.data)

    def publish_imu_sample(self, ax, ay, az):
        payload = json.dumps({
            'ax': ax, 'ay': ay, 'az': az,
            'gx': 0.0, 'gy': 0.0, 'gz': 0.0,
            'mx': 0.0, 'my': 0.0, 'mz': 0.0,
            't_ms': 0,
        })
        self.imu_pub.publish(String(data=payload))

    def publish_odom_pose(self, x_m, y_m):
        msg = Odometry()
        msg.pose.pose.position.x = x_m
        msg.pose.pose.position.y = y_m
        msg.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(msg)


def main():
    node_process = subprocess.Popen(
        ['ros2', 'run', 'walker_gait_metrics', 'gait_metrics_node'],
        start_new_session=True,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )

    rclpy.init()
    node = VerifyNode()

    try:
        time.sleep(2.0)  # let the node declare parameters and subscribe

        # --- Odometry: a single 10m displacement ---
        node.publish_odom_pose(0.0, 0.0)
        time.sleep(0.1)
        node.publish_odom_pose(TOTAL_DISTANCE_M, 0.0)
        time.sleep(0.1)

        # --- IMU: five steps, each spaced past the debounce interval ---
        for _ in range(NUM_STEPS):
            node.publish_imu_sample(0.0, 0.0, 1.5)  # magnitude 1.5g, above the 1.2g threshold
            time.sleep(MIN_STEP_INTERVAL_S + 0.05)

        # publish_rate_hz defaults to 1.0 - give it time to publish at least
        # once after all the synthetic input above has been processed.
        deadline = time.monotonic() + 10.0
        while node.latest_metrics is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)

        if node.latest_metrics is None:
            print('FAIL: no /gait_metrics message received within 10s')
            return 1

        metrics = node.latest_metrics
        if metrics['step_count'] != NUM_STEPS:
            print(f"FAIL: step_count={metrics['step_count']}, expected {NUM_STEPS}")
            return 1
        if metrics['total_distance_m'] != TOTAL_DISTANCE_M:
            print(f"FAIL: total_distance_m={metrics['total_distance_m']}, expected {TOTAL_DISTANCE_M}")
            return 1
        if metrics['avg_step_length_m'] != EXPECTED_AVG_STEP_LENGTH_M:
            print(
                f"FAIL: avg_step_length_m={metrics['avg_step_length_m']}, "
                f"expected {EXPECTED_AVG_STEP_LENGTH_M}"
            )
            return 1

        print(
            f"PASS: step_count={metrics['step_count']}, "
            f"total_distance_m={metrics['total_distance_m']}, "
            f"avg_step_length_m={metrics['avg_step_length_m']}"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
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

- [ ] **Step 2: Build and run the script**

```bash
source /opt/ros/humble/setup.bash
cd src
PYTHONNOUSERSITE=1 colcon build --packages-select walker_gait_metrics --symlink-install
source install/setup.bash
python3 walker_gait_metrics/tools/verify_gait_metrics.py
```

Expected: `PASS: step_count=5, total_distance_m=10.0, avg_step_length_m=2.0`. If it fails, check
`ps aux` for a leftover `gait_metrics_node` process from a previous attempt and kill it before
retrying.

- [ ] **Step 3: Commit**

```bash
git add src/walker_gait_metrics/tools/verify_gait_metrics.py
git commit -m "walker_gait_metrics: add automated end-to-end verification script"
```

---

## Task 5: `walker_anomaly_detection` republishes raw IMU samples

**Files:**
- Modify: `src/walker_anomaly_detection/walker_anomaly_detection/anomaly_detection_node.py`
- Modify: `src/walker_anomaly_detection/tools/verify_anomaly_detection.py`
- Modify: `src/walker_anomaly_detection/docs/bring_up.md`
- Modify: `src/walker_anomaly_detection/README.md`

**Interfaces:**
- Produces: a new `/imu/raw_sample` topic (`std_msgs/String`, JSON — the same dict
  `imu_serial.parse_sample_line` already returns) on `walker_anomaly_detection`'s existing node —
  this is the real-world producer `walker_gait_metrics`'s node subscribes to (Task 3 already
  built the subscriber against this exact shape).

This task is independent of Tasks 1-4's code (it doesn't import anything from
`walker_gait_metrics`), but comes after them here so the plan reads as "build the consumer, then
wire up the real producer."

- [ ] **Step 1: Add the publisher**

In `src/walker_anomaly_detection/walker_anomaly_detection/anomaly_detection_node.py`, add a new
publisher right after the existing one in `__init__`:

```python
        self._alert_pub = self.create_publisher(String, '/anomaly_detected', 10)
        self._raw_sample_pub = self.create_publisher(String, '/imu/raw_sample', 10)
```

And republish every sample at the top of `_on_sample`:

```python
    def _on_sample(self, sample):
        self._raw_sample_pub.publish(String(data=json.dumps(sample)))

        now_s = self.get_clock().now().nanoseconds / 1e9
        accel_magnitude_g = math.sqrt(
            sample['ax'] ** 2 + sample['ay'] ** 2 + sample['az'] ** 2
        )

        if self._fall_detector.update(accel_magnitude_g, now_s):
            self._publish_alert('fall')

        tilt_deg = tilt_from_accel_deg(sample['ax'], sample['ay'], sample['az'])
        if self._tilt_detector.update(tilt_deg, now_s):
            self._publish_alert('tilt')
```

(`json` is already imported at the top of this file for `_publish_alert`'s `json.dumps` call — no
new import needed.)

- [ ] **Step 2: Extend the verification script to check the new topic**

In `src/walker_anomaly_detection/tools/verify_anomaly_detection.py`, extend `VerifyNode` to also
track raw samples:

```python
class VerifyNode(Node):
    def __init__(self):
        super().__init__('walker_anomaly_detection_verify')
        self.events = []
        self.raw_samples = []
        self.create_subscription(String, '/anomaly_detected', self._on_event, 10)
        self.create_subscription(String, '/imu/raw_sample', self._on_raw_sample, 10)

    def _on_event(self, msg):
        self.events.append(json.loads(msg.data))

    def _on_raw_sample(self, msg):
        self.raw_samples.append(json.loads(msg.data))
```

Then, right after the existing `print('Fall event received.')` line (after the fall scenario
completes and before the tilt scenario starts), add:

```python
        if not node.raw_samples:
            print(
                'FAIL: no /imu/raw_sample messages received - anomaly_detection_node should '
                'republish every parsed sample'
            )
            return 1
        first = node.raw_samples[0]
        if not (first['ax'] == 0.0 and first['ay'] == 0.0 and first['az'] == 1.0):
            print(f'FAIL: first /imu/raw_sample payload {first} does not match the first sample sent')
            return 1
        print(f'/imu/raw_sample verified ({len(node.raw_samples)} samples received).')
```

(This checks against the fall scenario's first `_sample_line(0.0, 0.0, 1.0, 0)` call, already
sent earlier in `main()`.)

- [ ] **Step 3: Build and run the extended verification script**

```bash
source /opt/ros/humble/setup.bash
cd src
PYTHONNOUSERSITE=1 colcon build --packages-select walker_anomaly_detection --symlink-install
source install/setup.bash
python3 walker_anomaly_detection/tools/verify_anomaly_detection.py
```

Expected: `Fall event received.` → `/imu/raw_sample verified (N samples received).` → `Tilt event
received.` → `PASS: fall event and tilt event both verified via a virtual serial pair`.

- [ ] **Step 4: Add a bring-up note about real-world step-detectability**

In `src/walker_anomaly_detection/docs/bring_up.md`, add a new section at the end (after
"Verifying the sensor itself works"):

```markdown
## Open question for walker_gait_metrics: does this IMU see footsteps at all?

This IMU monitors the walker frame's own motion (fall/tilt of the robot), not something worn by
the person. `walker_gait_metrics` (a separate package) assumes a person's footsteps produce a
detectable jolt through the frame — this has never been tested on real hardware. Once the sensor
itself is verified working (above), also check: with `walker_gait_metrics` running
(`ros2 launch walker_gait_metrics gait_metrics.launch.py`) alongside this package and
`walker_motor_driver`, `echo /gait_metrics` while a person actually walks with the assembled
walker and see whether `step_count` increments at a plausible rate. If it doesn't, that's a real
finding — see `walker_gait_metrics`'s own design spec §2.5 for what to consider next (a
wheel-odometry-based step signal, or a person/handle-mounted sensor instead).
```

- [ ] **Step 5: Update the package README**

In `src/walker_anomaly_detection/README.md`'s "Layout" section, extend the existing
`anomaly_detection_node.py` bullet to mention the new publisher. Change:

```
- `walker_anomaly_detection/anomaly_detection_node.py` — the `rclpy` node: opens the configured
  serial port, reads samples on a background thread, feeds both detectors, publishes
  `/anomaly_detected` (`std_msgs/String`, JSON payload) on a detected event.
```

to:

```
- `walker_anomaly_detection/anomaly_detection_node.py` — the `rclpy` node: opens the configured
  serial port, reads samples on a background thread, feeds both detectors, publishes
  `/anomaly_detected` (`std_msgs/String`, JSON payload) on a detected event, and republishes
  every parsed sample as JSON on `/imu/raw_sample` (consumed by `walker_gait_metrics`).
```

- [ ] **Step 6: Commit**

```bash
git add src/walker_anomaly_detection/walker_anomaly_detection/anomaly_detection_node.py \
        src/walker_anomaly_detection/tools/verify_anomaly_detection.py \
        src/walker_anomaly_detection/docs/bring_up.md \
        src/walker_anomaly_detection/README.md
git commit -m "walker_anomaly_detection: republish raw IMU samples on /imu/raw_sample"
```

---

## Task 6: `walker_companion_app` dashboard wiring

**Files:**
- Modify: `src/walker_companion_app/walker_companion_app/shared_state.py`
- Modify: `src/walker_companion_app/test/test_shared_state.py`
- Modify: `src/walker_companion_app/walker_companion_app/dashboard_app_node.py`
- Modify: `src/walker_companion_app/web/index.html`
- Modify: `src/walker_companion_app/tools/verify_dashboard_app.py`
- Modify: `src/walker_companion_app/README.md`

**Interfaces:**
- Consumes: `/gait_metrics`'s JSON shape (spec §2.8, Task 3) — `{"step_count": int,
  "total_distance_m": float, "avg_step_length_m": float, "timestamp": float}`.

- [ ] **Step 1: Extend `SharedState` with gait metrics**

In `src/walker_companion_app/walker_companion_app/shared_state.py`, add a default and a setter,
and extend `status_snapshot`:

```python
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
```

(Only `set_gait_metrics`, the `self._gait` default, and `status_snapshot`'s `'gait'` key are new
— everything else shown is existing code, included for exact placement.)

- [ ] **Step 2: Update and extend `test_shared_state.py`**

`status_snapshot`'s shape changed, so the existing default-snapshot test needs updating, not just
a new test alongside it. In `src/walker_companion_app/test/test_shared_state.py`, change:

```python
def test_default_status_snapshot(tmp_path):
    state = _make_state(tmp_path)
    snapshot = state.status_snapshot(timestamp=123.0)
    assert snapshot == {'pose': {'x': 0.0, 'y': 0.0, 'theta': 0.0}, 'nav_status': 'idle', 'timestamp': 123.0}
```

to:

```python
def test_default_status_snapshot(tmp_path):
    state = _make_state(tmp_path)
    snapshot = state.status_snapshot(timestamp=123.0)
    assert snapshot == {
        'pose': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
        'nav_status': 'idle',
        'gait': {'step_count': 0, 'total_distance_m': 0.0, 'avg_step_length_m': 0.0},
        'timestamp': 123.0,
    }
```

Then add two new tests, matching the existing `set_pose`/`set_nav_status` test style:

```python
def test_set_gait_metrics_reflected_in_status_snapshot(tmp_path):
    state = _make_state(tmp_path)
    state.set_gait_metrics({'step_count': 42, 'total_distance_m': 84.0, 'avg_step_length_m': 2.0})
    snapshot = state.status_snapshot(timestamp=1.0)
    assert snapshot['gait'] == {'step_count': 42, 'total_distance_m': 84.0, 'avg_step_length_m': 2.0}


def test_status_snapshot_gait_returns_a_copy(tmp_path):
    state = _make_state(tmp_path)
    state.set_gait_metrics({'step_count': 1, 'total_distance_m': 1.0, 'avg_step_length_m': 1.0})
    snapshot = state.status_snapshot(timestamp=1.0)
    snapshot['gait']['step_count'] = 999
    assert state.status_snapshot(timestamp=1.0)['gait']['step_count'] == 1
```

- [ ] **Step 3: Run the pure-module tests**

```bash
cd src/walker_companion_app
python3 -m pytest test/ -v
```

Expected: PASS — all tests, including the two new ones and the updated default-snapshot test.

- [ ] **Step 4: Subscribe `dashboard_app_node.py` to `/gait_metrics`**

In `src/walker_companion_app/walker_companion_app/dashboard_app_node.py`, add `import json` to
the top-level imports (not currently imported in this file):

```python
import json
import os
import threading
from http.server import ThreadingHTTPServer
```

Add the subscription in `__init__`, alongside the existing ones:

```python
        self.create_subscription(String, '/llm_bridge/text_in', self._on_text_in, 10)
        self.create_subscription(String, '/llm_bridge/text_out', self._on_text_out, 10)
        self.create_subscription(String, '/gait_metrics', self._on_gait_metrics, 10)
```

Add the callback, alongside the existing `_on_*` methods:

```python
    def _on_gait_metrics(self, msg):
        try:
            gait = json.loads(msg.data)
        except (ValueError, TypeError):
            self.get_logger().warn(
                'Ignoring malformed /gait_metrics payload.', throttle_duration_sec=5.0,
            )
            return
        self._state.set_gait_metrics(gait)
```

- [ ] **Step 5: Add a Gait section to the dashboard page**

In `src/walker_companion_app/web/index.html`, add a new section right after `status-section` and
before `map-section`:

```html
<section id="gait-section">
  <h2>Gait</h2>
  <p>Steps: <span id="gait-step-count">-</span></p>
  <p>Distance: <span id="gait-total-distance">-</span> m</p>
  <p>Avg step length: <span id="gait-avg-step-length">-</span> m</p>
</section>
```

Extend the existing `pollStatus()` function to populate it:

```javascript
async function pollStatus() {
  try {
    const res = await fetch('/api/status');
    const data = await res.json();
    document.getElementById('pose-x').textContent = data.pose.x.toFixed(2);
    document.getElementById('pose-y').textContent = data.pose.y.toFixed(2);
    document.getElementById('pose-theta').textContent = data.pose.theta.toFixed(2);
    document.getElementById('nav-status').textContent = data.nav_status;
    document.getElementById('gait-step-count').textContent = data.gait.step_count;
    document.getElementById('gait-total-distance').textContent = data.gait.total_distance_m.toFixed(2);
    document.getElementById('gait-avg-step-length').textContent = data.gait.avg_step_length_m.toFixed(2);
  } catch (e) {
    console.error('status poll failed', e);
  }
}
```

- [ ] **Step 6: Extend the end-to-end verification script**

In `src/walker_companion_app/tools/verify_dashboard_app.py`, add `String` to the existing
`std_msgs.msg` usage — this file doesn't import it yet, so add:

```python
from std_msgs.msg import String
```

alongside the existing `geometry_msgs`/`nav2_msgs`/`rclpy` imports. In `VerifyDriverNode.__init__`,
add a publisher:

```python
class VerifyDriverNode(Node):
    def __init__(self):
        super().__init__('walker_companion_app_verify')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gait_pub = self.create_publisher(String, '/gait_metrics', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
```

In `main()`, right after the existing "Pose changes after a `/cmd_vel` command" block (after its
`if not (after['pose']['x'] > before['pose']['x'])` check), add:

```python
        # --- Gait metrics appear in /api/status ---
        gait_payload = json.dumps({'step_count': 42, 'total_distance_m': 84.0, 'avg_step_length_m': 2.0})
        node.gait_pub.publish(String(data=gait_payload))
        time.sleep(1.0)
        status = _get_json('/api/status')
        if status.get('gait', {}).get('step_count') != 42:
            print(
                f"FAIL: /api/status gait.step_count did not reflect published /gait_metrics "
                f"(got {status.get('gait')})"
            )
            return 1
        print(f"Gait metrics verified: {status['gait']}")
```

(`json` is already imported at the top of this file.) This check publishes the synthetic
`/gait_metrics` message directly from the verify script — it does not require
`walker_gait_metrics` or `walker_anomaly_detection` to be running, since `dashboard_app_node.py`
only cares that *something* published on that topic.

- [ ] **Step 7: Build and run the extended end-to-end check**

```bash
source /opt/ros/humble/setup.bash
cd src
PYTHONNOUSERSITE=1 colcon build --packages-select walker_motor_driver walker_nav walker_llm_bridge walker_companion_app --symlink-install
source install/setup.bash

ros2 launch walker_motor_driver motor_driver.launch.py &
ros2 launch walker_nav walker_nav.launch.py &
sleep 3
ros2 launch walker_nav nav2.launch.py &
sleep 10
ros2 launch walker_companion_app dashboard_app.launch.py &
sleep 2

python3 walker_companion_app/tools/verify_dashboard_app.py
```

(Substitute the `http_port:=8081`/`WALKER_DASHBOARD_URL` override from this package's README if
port 8080 is unavailable on this machine, per that README's existing note.)

Expected: `Gait metrics verified: {'step_count': 42, 'total_distance_m': 84.0, 'avg_step_length_m':
2.0}` appears among the script's output, followed by the existing `PASS: pose update, Nav2 status
transition, live map, and conversation log all verified`. Kill all four launched processes when
done, checking `ps aux` per this package's README's existing note.

- [ ] **Step 8: Update the package README**

In `src/walker_companion_app/README.md`'s "Layout" section, extend the existing
`dashboard_app_node.py` bullet:

```
- `walker_companion_app/dashboard_app_node.py` — the `rclpy` node:
  subscribes `/odom`, `/map`, `/navigate_to_pose/_action/status`,
  `/llm_bridge/text_in`, `/llm_bridge/text_out`, `/gait_metrics`; runs the HTTP server in
  a background thread.
```

And the `web/index.html` bullet:

```
- `web/index.html` — the dashboard page: polls `/api/status`,
  `/api/map`, `/api/conversation` on an interval, renders the map on a
  `<canvas>`, shows gait metrics (step count, distance, average step length), and shows a static
  (unwired) alerts placeholder.
```

- [ ] **Step 9: Commit**

```bash
git add src/walker_companion_app/walker_companion_app/shared_state.py \
        src/walker_companion_app/test/test_shared_state.py \
        src/walker_companion_app/walker_companion_app/dashboard_app_node.py \
        src/walker_companion_app/web/index.html \
        src/walker_companion_app/tools/verify_dashboard_app.py \
        src/walker_companion_app/README.md
git commit -m "walker_companion_app: surface gait metrics on the dashboard"
```

---

## Out of scope for this plan (already documented in the spec)

`walker_llm_bridge` conversational exposure ("how many steps have I taken"), grip strength (needs
handle hardware that doesn't exist yet), and Kinect-based gait/fitness analysis (its own larger
idea with an open mounting-geometry question) are deliberately not implemented here — see the
design spec §6 and `docs/ideas-backlog.md`. Plan these separately once their own open questions
are resolved.
