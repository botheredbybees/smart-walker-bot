# walker_motor_driver Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce the `walker_motor_driver` ROS2 package: a real `ament_python` node that takes `Twist` velocity commands and publishes `Odometry`/TF, backed entirely by a lightweight kinematic simulator (no motor hardware exists yet).

**Architecture:** Split into a pure-Python differential-drive kinematics core (`diff_drive_kinematics.py`) unit-tested with pytest, a `MotorBackend` interface with a `SimMotorBackend` implementation (also pure, also pytest-tested) that stands in for real GPIO until hardware bring-up, and a thin `rclpy` node (`motor_driver_node.py`) that wires the two together. Unlike `walker_safety`'s Pico firmware, this node *can* actually run in this environment — the full ROS2 Humble install is here — so instead of a human-required hardware bring-up, verification is a real, scripted `rclpy` check (`tools/verify_motor_driver.py`) that publishes a command and asserts the resulting odometry, fully automatable without any physical hardware.

**Tech Stack:** Python 3 + `rclpy` (ROS2 Humble), pytest (pure-module unit tests), standard ROS2 messages (`geometry_msgs/Twist`, `nav_msgs/Odometry`, `tf2_ros`).

**Spec:** `docs/superpowers/specs/2026-08-30-walker-motor-driver-design.md` (§2 for decisions, §3 for file structure, §4 for testing approach).

## Global Constraints

- Command in: `geometry_msgs/Twist` on `/cmd_vel`. Feedback out: `nav_msgs/Odometry` on `/odom`, plus an `odom`→`base_link` TF broadcast via `tf2_ros`. (spec §2.4)
- `MotorBackend` interface: `apply_wheel_speeds(left_rad_s, right_rad_s) -> None`, `read_wheel_deltas(now_s) -> (left_rad, right_rad)`. Note: `now_s` is an explicit parameter added beyond the spec §2.3 sketch signature — this plan refines it that way for the same determinism reason `walker_safety`'s `Watchdog` takes explicit `now_s` instead of reading a wall clock internally: it keeps `SimMotorBackend` fast and deterministic under pytest with no sleeping or clock-mocking.
- Physical parameters (`wheel_radius_m=0.03`, `wheel_separation_m=0.2`, `max_wheel_speed_rad_s=10.0`) are ROS2 node parameters with placeholder defaults, not measured values — recalibration is explicitly deferred to hardware bring-up. (spec §2.5)
- No coupling to `walker_safety`'s watchdog: this package never checks watchdog state and never publishes a heartbeat. (spec §2.6)
- Real `ament_python` colcon package (unlike `walker_safety`, which deliberately isn't one) — buildable with `colcon build --packages-select walker_motor_driver` from the `src/` workspace root (matches this repo's existing convention: `src/` itself is the colcon workspace root, per `src/README.md` and its `.gitignore` entries `src/build/`, `src/install/`, `src/log/`). (spec §2.2)
- `GpioMotorBackend` and real hardware wiring are out of scope for this plan — deferred to the hardware bring-up checkpoint. (spec §5)
- Pure modules (`diff_drive_kinematics.py`, `sim_backend.py`) have zero `rclpy` imports, so their tests run with plain `python3 -m pytest` — no ROS environment sourcing or colcon build required. Tests import via a `test/conftest.py` that inserts the package's inner `walker_motor_driver/` directory onto `sys.path` (bare module imports, e.g. `from diff_drive_kinematics import ...`), the same pattern `walker_safety/firmware/tests/conftest.py` already established in this repo — chosen over the standard ament_python "import the installed package" convention specifically so these fast unit tests never depend on a successful `colcon build` first.

---

## Task 1: Package Scaffold

**Files:**
- Create: `src/walker_motor_driver/package.xml`
- Create: `src/walker_motor_driver/setup.py`
- Create: `src/walker_motor_driver/setup.cfg`
- Create: `src/walker_motor_driver/resource/walker_motor_driver`
- Create: `src/walker_motor_driver/walker_motor_driver/__init__.py`
- Create: `src/walker_motor_driver/README.md`

**Interfaces:**
- Produces: an installable, buildable `ament_python` package shell. `console_scripts` entry point `motor_driver_node = walker_motor_driver.motor_driver_node:main` is declared now even though `motor_driver_node.py` doesn't exist until Task 4 — `colcon build` doesn't import entry-point targets at build time, only when actually run, so this is safe to declare early and matches standard ROS2 package scaffolding order.

- [ ] **Step 1: Create the package directories**

```bash
mkdir -p /home/peter_sha/sourcecode/smart-walker-bot/src/walker_motor_driver/walker_motor_driver
mkdir -p /home/peter_sha/sourcecode/smart-walker-bot/src/walker_motor_driver/resource
mkdir -p /home/peter_sha/sourcecode/smart-walker-bot/src/walker_motor_driver/launch
mkdir -p /home/peter_sha/sourcecode/smart-walker-bot/src/walker_motor_driver/test
mkdir -p /home/peter_sha/sourcecode/smart-walker-bot/src/walker_motor_driver/tools
```

- [ ] **Step 2: Write package.xml**

Create `src/walker_motor_driver/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>walker_motor_driver</name>
  <version>0.0.1</version>
  <description>Differential-drive motor driver node for smart-walker-bot, backed by a sim or (later) real GPIO backend.</description>
  <maintainer email="botheredbybees@gmail.com">botheredbybees</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2_ros</depend>

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

Create `src/walker_motor_driver/setup.py`:

```python
from setuptools import find_packages, setup

package_name = 'walker_motor_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/motor_driver.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='botheredbybees',
    maintainer_email='botheredbybees@gmail.com',
    description='Differential-drive motor driver node for smart-walker-bot, backed by a sim or (later) real GPIO backend.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_driver_node = walker_motor_driver.motor_driver_node:main',
        ],
    },
)
```

Note: `launch/motor_driver.launch.py` is referenced here but doesn't exist until Task 4. This is fine for `colcon build` (it packages `data_files` at install time, and Task 4 will have created the file by the time anyone actually builds *and* launches) — but if this task's own build-verification step (Step 6) fails because the referenced launch file is missing, create an empty placeholder `launch/motor_driver.launch.py` first with just a valid empty `LaunchDescription`:

```python
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([])
```

Task 4 will overwrite this placeholder with the real launch file.

- [ ] **Step 4: Write setup.cfg**

Create `src/walker_motor_driver/setup.cfg`:

```ini
[develop]
script_dir=$base/lib/walker_motor_driver
[install]
install_scripts=$base/lib/walker_motor_driver
```

- [ ] **Step 5: Create the resource marker and package __init__**

Create `src/walker_motor_driver/resource/walker_motor_driver` as an empty file (this is how `ament_index` discovers the package — it just needs to exist, content is irrelevant):

```bash
touch /home/peter_sha/sourcecode/smart-walker-bot/src/walker_motor_driver/resource/walker_motor_driver
touch /home/peter_sha/sourcecode/smart-walker-bot/src/walker_motor_driver/walker_motor_driver/__init__.py
```

- [ ] **Step 6: Verify the package builds**

```bash
source /opt/ros/humble/setup.bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src
colcon build --packages-select walker_motor_driver --symlink-install
```

Expected: build succeeds (`Summary: 1 package finished`). If it fails because `launch/motor_driver.launch.py` is missing, create the placeholder from Step 3's note and retry.

- [ ] **Step 7: Write the package README**

Create `src/walker_motor_driver/README.md`:

```markdown
# walker_motor_driver

Differential-drive motor driver node for smart-walker-bot. See
`docs/superpowers/specs/2026-08-30-walker-motor-driver-design.md` for the
full design (this is a summary).

Unlike `walker_safety`, this is a real ROS2 `ament_python` package —
build it with `colcon build --packages-select walker_motor_driver` from
`src/` (this repo's colcon workspace root).

## Layout

- `walker_motor_driver/diff_drive_kinematics.py` — pure Python: converts
  `Twist` commands to per-wheel speeds, and integrates wheel-rotation
  deltas into a tracked `(x, y, theta)` pose. No ROS or hardware imports;
  unit-tested with pytest.
- `walker_motor_driver/motor_backend.py` — the `MotorBackend` interface
  that separates the ROS2 node from how wheel speeds actually get
  applied and measured. This is the sim/real boundary: the node's code
  never changes when a real backend replaces the sim one.
- `walker_motor_driver/sim_backend.py` — `SimMotorBackend`, an idealized
  kinematic simulator (commanded speed achieved instantly, no motor
  dynamics or slip). The only backend that exists until hardware
  bring-up adds a `GpioMotorBackend`.
- `walker_motor_driver/motor_driver_node.py` — the `rclpy` node wiring
  the above together: subscribes `/cmd_vel` (`geometry_msgs/Twist`),
  publishes `/odom` (`nav_msgs/Odometry`) and an `odom`→`base_link` TF.
- `launch/motor_driver.launch.py` — launch file with a `backend`
  argument (default `sim`; any other value raises a clear error until a
  real backend is implemented).
- `tools/verify_motor_driver.py` — a scripted (not pytest) end-to-end
  check: launch the node, publish a `/cmd_vel` command, confirm `/odom`
  moves as expected. Doesn't need any physical hardware — the sim
  backend is enough. See this file's own docstring for usage.

## Running the pure-module tests

```bash
cd src/walker_motor_driver
python3 -m pytest test/ -v
```

No ROS environment or colcon build needed for these — see this repo's
plan document for why (`docs/superpowers/plans/2026-08-30-walker-motor-driver.md`
Global Constraints).

## Physical parameters are placeholders

`wheel_radius_m` (0.03), `wheel_separation_m` (0.2), and
`max_wheel_speed_rad_s` (10.0) are typical small-robot-vacuum-sized
defaults, not measurements — the real salvaged-vacuum dimensions aren't
known until vacuums are stripped (root `README.md` §6 step 1).
Recalibrate these at hardware bring-up.

## No coupling to walker_safety

The hardware E-stop and Pico watchdog cut motor power physically, in
series with the driver board's power rail, independent of this
package's software. `walker_motor_driver` doesn't check watchdog state
or publish a heartbeat — adding that here would be redundant with (and
could create false confidence alongside) the physical cutoff that's
deliberately independent of software like this.
```

- [ ] **Step 8: Commit**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot
git add src/walker_motor_driver/
git commit -m "$(cat <<'EOF'
Add walker_motor_driver package scaffold

ament_python ROS2 package shell: package.xml, setup.py/cfg, resource
marker, and package README. colcon build verified working before any
node code exists.
EOF
)"
```

---

## Task 2: Differential-Drive Kinematics Core (TDD)

**Files:**
- Create: `src/walker_motor_driver/walker_motor_driver/diff_drive_kinematics.py`
- Create: `src/walker_motor_driver/test/conftest.py`
- Test: `src/walker_motor_driver/test/test_diff_drive_kinematics.py`

**Interfaces:**
- Produces: `twist_to_wheel_speeds(linear_x_m_s, angular_z_rad_s, wheel_radius_m, wheel_separation_m) -> (left_rad_s, right_rad_s)`; `OdometryTracker(wheel_radius_m, wheel_separation_m)` with `.x_m`, `.y_m`, `.theta_rad` (public, read by the node) and `.update(left_wheel_delta_rad, right_wheel_delta_rad, dt_s) -> (linear_x_m_s, angular_z_rad_s)`; `yaw_to_quaternion(yaw_rad) -> (x, y, z, w)`. All consumed by Task 4 (`motor_driver_node.py`).

- [ ] **Step 1: Confirm pytest is available**

```bash
python3 -m pytest --version
```

If missing: `python3 -m pip install --user pytest`.

- [ ] **Step 2: Write the test-path conftest**

Create `src/walker_motor_driver/test/conftest.py`:

```python
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / 'walker_motor_driver'))
```

- [ ] **Step 3: Write the failing tests**

Create `src/walker_motor_driver/test/test_diff_drive_kinematics.py`:

```python
import math

import pytest

from diff_drive_kinematics import OdometryTracker, twist_to_wheel_speeds, yaw_to_quaternion

WHEEL_RADIUS_M = 0.03
WHEEL_SEPARATION_M = 0.2


def test_straight_line_gives_equal_wheel_speeds():
    left, right = twist_to_wheel_speeds(1.0, 0.0, WHEEL_RADIUS_M, WHEEL_SEPARATION_M)
    assert left == pytest.approx(33.3333333, rel=1e-6)
    assert right == pytest.approx(33.3333333, rel=1e-6)


def test_pure_rotation_gives_opposite_wheel_speeds():
    left, right = twist_to_wheel_speeds(0.0, 1.0, WHEEL_RADIUS_M, WHEEL_SEPARATION_M)
    assert left == pytest.approx(-3.3333333, rel=1e-6)
    assert right == pytest.approx(3.3333333, rel=1e-6)


def test_combined_motion_gives_asymmetric_wheel_speeds():
    left, right = twist_to_wheel_speeds(1.0, 1.0, WHEEL_RADIUS_M, WHEEL_SEPARATION_M)
    assert left == pytest.approx(30.0, rel=1e-6)
    assert right == pytest.approx(36.6666667, rel=1e-6)


def test_twist_zero_wheel_radius_rejected():
    with pytest.raises(ValueError):
        twist_to_wheel_speeds(1.0, 0.0, 0.0, WHEEL_SEPARATION_M)


def test_twist_negative_wheel_separation_rejected():
    with pytest.raises(ValueError):
        twist_to_wheel_speeds(1.0, 0.0, WHEEL_RADIUS_M, -0.1)


def test_straight_line_update_moves_forward():
    tracker = OdometryTracker(WHEEL_RADIUS_M, WHEEL_SEPARATION_M)
    linear_x, angular_z = tracker.update(10.0, 10.0, 1.0)
    assert tracker.x_m == pytest.approx(0.3, rel=1e-6)
    assert tracker.y_m == pytest.approx(0.0, abs=1e-9)
    assert tracker.theta_rad == pytest.approx(0.0, abs=1e-9)
    assert linear_x == pytest.approx(0.3, rel=1e-6)
    assert angular_z == pytest.approx(0.0, abs=1e-9)


def test_pure_rotation_update_changes_heading_only():
    tracker = OdometryTracker(WHEEL_RADIUS_M, WHEEL_SEPARATION_M)
    linear_x, angular_z = tracker.update(-5.0, 5.0, 1.0)
    assert tracker.x_m == pytest.approx(0.0, abs=1e-9)
    assert tracker.y_m == pytest.approx(0.0, abs=1e-9)
    assert tracker.theta_rad == pytest.approx(1.5, rel=1e-6)
    assert linear_x == pytest.approx(0.0, abs=1e-9)
    assert angular_z == pytest.approx(1.5, rel=1e-6)


def test_multiple_updates_accumulate_pose():
    tracker = OdometryTracker(WHEEL_RADIUS_M, WHEEL_SEPARATION_M)
    tracker.update(10.0, 10.0, 1.0)
    tracker.update(10.0, 10.0, 1.0)
    assert tracker.x_m == pytest.approx(0.6, rel=1e-6)
    assert tracker.y_m == pytest.approx(0.0, abs=1e-9)
    assert tracker.theta_rad == pytest.approx(0.0, abs=1e-9)


def test_odometry_non_positive_dt_rejected():
    tracker = OdometryTracker(WHEEL_RADIUS_M, WHEEL_SEPARATION_M)
    with pytest.raises(ValueError):
        tracker.update(1.0, 1.0, 0.0)


def test_odometry_constructor_rejects_non_positive_wheel_radius():
    with pytest.raises(ValueError):
        OdometryTracker(0.0, WHEEL_SEPARATION_M)


def test_odometry_constructor_rejects_non_positive_wheel_separation():
    with pytest.raises(ValueError):
        OdometryTracker(WHEEL_RADIUS_M, -0.1)


def test_zero_yaw_gives_identity_quaternion():
    x, y, z, w = yaw_to_quaternion(0.0)
    assert (x, y, z, w) == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1e-9)


def test_half_turn_yaw_gives_expected_quaternion():
    x, y, z, w = yaw_to_quaternion(math.pi)
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(0.0, abs=1e-9)
    assert z == pytest.approx(1.0, rel=1e-6)
    assert w == pytest.approx(0.0, abs=1e-9)


def test_quarter_turn_yaw_gives_expected_quaternion():
    x, y, z, w = yaw_to_quaternion(math.pi / 2)
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(0.0, abs=1e-9)
    assert z == pytest.approx(math.sqrt(2) / 2, rel=1e-6)
    assert w == pytest.approx(math.sqrt(2) / 2, rel=1e-6)
```

- [ ] **Step 4: Run tests to verify they fail**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_motor_driver
python3 -m pytest test/test_diff_drive_kinematics.py -v
```

Expected: `ModuleNotFoundError: No module named 'diff_drive_kinematics'`.

- [ ] **Step 5: Implement the kinematics module**

Create `src/walker_motor_driver/walker_motor_driver/diff_drive_kinematics.py`:

```python
"""Pure differential-drive kinematics for walker_motor_driver.

No ROS or hardware imports here - this module is shared between the
rclpy node (motor_driver_node.py) and the desktop pytest suite, so the
same math that runs live is exactly what the tests exercise. See
docs/superpowers/specs/2026-08-30-walker-motor-driver-design.md Sec 2.3
for why the sim/real boundary lives one layer below this module, in
MotorBackend, not here.
"""
import math


def twist_to_wheel_speeds(linear_x_m_s, angular_z_rad_s, wheel_radius_m, wheel_separation_m):
    """Convert a body-frame Twist command into per-wheel angular speeds (rad/s)."""
    if wheel_radius_m <= 0:
        raise ValueError("wheel_radius_m must be positive")
    if wheel_separation_m <= 0:
        raise ValueError("wheel_separation_m must be positive")

    left_m_s = linear_x_m_s - (angular_z_rad_s * wheel_separation_m / 2.0)
    right_m_s = linear_x_m_s + (angular_z_rad_s * wheel_separation_m / 2.0)
    left_rad_s = left_m_s / wheel_radius_m
    right_rad_s = right_m_s / wheel_radius_m
    return left_rad_s, right_rad_s


class OdometryTracker:
    """Integrates wheel-rotation deltas into a 2D robot pose (x, y, theta).

    Pure differential-drive odometry: no ROS, no hardware, fully
    deterministic from its inputs - testable without rclpy or a backend.
    """

    def __init__(self, wheel_radius_m, wheel_separation_m):
        if wheel_radius_m <= 0:
            raise ValueError("wheel_radius_m must be positive")
        if wheel_separation_m <= 0:
            raise ValueError("wheel_separation_m must be positive")
        self._wheel_radius_m = wheel_radius_m
        self._wheel_separation_m = wheel_separation_m
        self.x_m = 0.0
        self.y_m = 0.0
        self.theta_rad = 0.0

    def update(self, left_wheel_delta_rad, right_wheel_delta_rad, dt_s):
        """Integrate one timestep of wheel rotation into the tracked pose.

        Returns (linear_x_m_s, angular_z_rad_s), the instantaneous
        body-frame velocity implied by the wheel motion this step -
        callers publish this directly as an Odometry message's twist.
        """
        if dt_s <= 0:
            raise ValueError("dt_s must be positive")

        left_dist_m = left_wheel_delta_rad * self._wheel_radius_m
        right_dist_m = right_wheel_delta_rad * self._wheel_radius_m
        center_dist_m = (left_dist_m + right_dist_m) / 2.0
        delta_theta_rad = (right_dist_m - left_dist_m) / self._wheel_separation_m

        # Integrate at the midpoint heading for better accuracy over a step.
        mid_theta_rad = self.theta_rad + delta_theta_rad / 2.0
        self.x_m += center_dist_m * math.cos(mid_theta_rad)
        self.y_m += center_dist_m * math.sin(mid_theta_rad)
        self.theta_rad += delta_theta_rad

        linear_x_m_s = center_dist_m / dt_s
        angular_z_rad_s = delta_theta_rad / dt_s
        return linear_x_m_s, angular_z_rad_s


def yaw_to_quaternion(yaw_rad):
    """Convert a 2D heading (radians) into an (x, y, z, w) quaternion,
    a rotation about the Z axis only - sufficient for a ground robot's
    Odometry/TF messages."""
    half = yaw_rad / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))
```

- [ ] **Step 6: Run tests to verify they pass**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_motor_driver
python3 -m pytest test/test_diff_drive_kinematics.py -v
```

Expected: 14 passed.

- [ ] **Step 7: Commit**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot
git add src/walker_motor_driver/walker_motor_driver/diff_drive_kinematics.py \
        src/walker_motor_driver/test/conftest.py \
        src/walker_motor_driver/test/test_diff_drive_kinematics.py
git commit -m "$(cat <<'EOF'
Add walker_motor_driver differential-drive kinematics core

Pure-Python twist->wheel-speed conversion, wheel-delta odometry
integration, and yaw->quaternion helper, unit-tested with no ROS
dependency. motor_driver_node.py (Task 4) wires this to real topics.
EOF
)"
```

---

## Task 3: MotorBackend Interface + Sim Backend (TDD)

**Files:**
- Create: `src/walker_motor_driver/walker_motor_driver/motor_backend.py`
- Create: `src/walker_motor_driver/walker_motor_driver/sim_backend.py`
- Test: `src/walker_motor_driver/test/test_sim_backend.py`

**Interfaces:**
- Produces: `MotorBackend` (interface, `apply_wheel_speeds`/`read_wheel_deltas`, both `NotImplementedError` stubs); `SimMotorBackend(now_s)` implementing it. Consumed by Task 4 (`motor_driver_node.py`).

- [ ] **Step 1: Write the failing tests**

Create `src/walker_motor_driver/test/test_sim_backend.py`:

```python
import pytest

from sim_backend import SimMotorBackend


def test_zero_speed_gives_zero_delta():
    backend = SimMotorBackend(now_s=0.0)
    backend.apply_wheel_speeds(0.0, 0.0)
    left, right = backend.read_wheel_deltas(now_s=1.0)
    assert left == pytest.approx(0.0, abs=1e-9)
    assert right == pytest.approx(0.0, abs=1e-9)


def test_constant_speed_gives_proportional_delta():
    backend = SimMotorBackend(now_s=0.0)
    backend.apply_wheel_speeds(2.0, 3.0)
    left, right = backend.read_wheel_deltas(now_s=1.0)
    assert left == pytest.approx(2.0, rel=1e-6)
    assert right == pytest.approx(3.0, rel=1e-6)


def test_successive_reads_only_count_new_elapsed_time():
    backend = SimMotorBackend(now_s=0.0)
    backend.apply_wheel_speeds(1.0, 1.0)
    first_left, first_right = backend.read_wheel_deltas(now_s=1.0)
    second_left, second_right = backend.read_wheel_deltas(now_s=2.0)
    assert first_left == pytest.approx(1.0, rel=1e-6)
    assert first_right == pytest.approx(1.0, rel=1e-6)
    assert second_left == pytest.approx(1.0, rel=1e-6)
    assert second_right == pytest.approx(1.0, rel=1e-6)


def test_speed_change_only_affects_subsequent_interval():
    backend = SimMotorBackend(now_s=0.0)
    backend.apply_wheel_speeds(1.0, 1.0)
    backend.read_wheel_deltas(now_s=1.0)
    backend.apply_wheel_speeds(5.0, 5.0)
    left, right = backend.read_wheel_deltas(now_s=2.0)
    assert left == pytest.approx(5.0, rel=1e-6)
    assert right == pytest.approx(5.0, rel=1e-6)


def test_asymmetric_speeds_give_independent_deltas():
    backend = SimMotorBackend(now_s=0.0)
    backend.apply_wheel_speeds(1.0, -2.0)
    left, right = backend.read_wheel_deltas(now_s=1.0)
    assert left == pytest.approx(1.0, rel=1e-6)
    assert right == pytest.approx(-2.0, rel=1e-6)


def test_backwards_time_rejected():
    backend = SimMotorBackend(now_s=10.0)
    with pytest.raises(ValueError):
        backend.read_wheel_deltas(now_s=5.0)
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_motor_driver
python3 -m pytest test/test_sim_backend.py -v
```

Expected: `ModuleNotFoundError: No module named 'sim_backend'`.

- [ ] **Step 3: Implement the backend interface**

Create `src/walker_motor_driver/walker_motor_driver/motor_backend.py`:

```python
"""Abstract interface separating walker_motor_driver's ROS2 node from how
wheel speeds actually get applied and measured - the sim/real boundary
described in docs/superpowers/specs/2026-08-30-walker-motor-driver-design.md
Sec 2.3. SimMotorBackend (sim_backend.py) is the only implementation
until hardware bring-up adds a GpioMotorBackend; motor_driver_node.py
doesn't change when that happens.
"""


class MotorBackend:
    def apply_wheel_speeds(self, left_rad_s, right_rad_s):
        """Command target wheel angular speeds, in radians/second."""
        raise NotImplementedError

    def read_wheel_deltas(self, now_s):
        """Return (left_rad, right_rad) wheel rotation since the last
        call, given the current time now_s (seconds, monotonic)."""
        raise NotImplementedError
```

- [ ] **Step 4: Implement the sim backend**

Create `src/walker_motor_driver/walker_motor_driver/sim_backend.py`:

```python
"""Idealized kinematic motor simulator - the roadmap design's
"lightweight, not physics-realistic" simulator
(docs/superpowers/specs/2026-08-30-phase1-roadmap-design.md Sec 2.4),
applied to walker_motor_driver specifically.
"""
from motor_backend import MotorBackend


class SimMotorBackend(MotorBackend):
    """Commanded wheel speed is achieved instantly, with no motor
    dynamics or slip. Time is passed in explicitly (now_s) rather than
    read from a wall clock, so tests are deterministic and don't need
    to sleep - matches the pattern watchdog_logic.py's Watchdog uses in
    walker_safety, for the same reason.
    """

    def __init__(self, now_s):
        self._left_rad_s = 0.0
        self._right_rad_s = 0.0
        self._last_read_s = now_s

    def apply_wheel_speeds(self, left_rad_s, right_rad_s):
        self._left_rad_s = left_rad_s
        self._right_rad_s = right_rad_s

    def read_wheel_deltas(self, now_s):
        dt_s = now_s - self._last_read_s
        if dt_s < 0:
            raise ValueError("now_s must not go backwards")
        left_delta_rad = self._left_rad_s * dt_s
        right_delta_rad = self._right_rad_s * dt_s
        self._last_read_s = now_s
        return left_delta_rad, right_delta_rad
```

Note: this file imports `from motor_backend import MotorBackend` (bare
module import, not `from walker_motor_driver.motor_backend import
MotorBackend`) — consistent with the bare-import style
`test/conftest.py` sets up for tests, and correct at runtime too,
since both files live in the same `walker_motor_driver/` package
directory and Python resolves same-directory imports this way whether
or not the package itself has been `pip install`-ed.

- [ ] **Step 5: Run tests to verify they pass**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_motor_driver
python3 -m pytest test/ -v
```

Expected: 20 passed (14 from Task 2 + 6 new).

- [ ] **Step 6: Commit**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot
git add src/walker_motor_driver/walker_motor_driver/motor_backend.py \
        src/walker_motor_driver/walker_motor_driver/sim_backend.py \
        src/walker_motor_driver/test/test_sim_backend.py
git commit -m "$(cat <<'EOF'
Add walker_motor_driver MotorBackend interface and sim backend

SimMotorBackend is an idealized, deterministic kinematic simulator -
the only backend until hardware bring-up adds a GpioMotorBackend
behind the same MotorBackend interface. Unit-tested with fake
timestamps, no rclpy or hardware dependency.
EOF
)"
```

---

## Task 4: ROS2 Node + Launch File + End-to-End Verification

**Files:**
- Create: `src/walker_motor_driver/walker_motor_driver/motor_driver_node.py`
- Create: `src/walker_motor_driver/launch/motor_driver.launch.py` (overwrites Task 1's placeholder, if one was created)
- Create: `src/walker_motor_driver/tools/verify_motor_driver.py`

**Interfaces:**
- Consumes: `twist_to_wheel_speeds`, `OdometryTracker`, `yaw_to_quaternion` from `diff_drive_kinematics` (Task 2); `SimMotorBackend` from `sim_backend` (Task 3).
- Produces: the `/cmd_vel` / `/odom` / TF topic interface that `walker_nav` (the next roadmap step) will configure `nav2`/`slam_toolbox` against. Nothing later in this plan consumes it as a Python interface.

- [ ] **Step 1: Write the ROS2 node**

Create `src/walker_motor_driver/walker_motor_driver/motor_driver_node.py`:

```python
"""walker_motor_driver's ROS2 node: subscribes /cmd_vel, publishes /odom
and an odom->base_link TF, driving a MotorBackend (sim_backend.py's
SimMotorBackend for now) in between. See
docs/superpowers/specs/2026-08-30-walker-motor-driver-design.md for the
full design.

Unlike walker_safety's main.py, this node runs on ordinary desktop
Python + rclpy - there's no missing-hardware-module problem here, so
it's verified by actually running it (see tools/verify_motor_driver.py)
rather than requiring physical hardware.
"""
import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from walker_motor_driver.diff_drive_kinematics import (
    OdometryTracker,
    twist_to_wheel_speeds,
    yaw_to_quaternion,
)
from walker_motor_driver.sim_backend import SimMotorBackend


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('walker_motor_driver')

        self.declare_parameter('wheel_radius_m', 0.03)
        self.declare_parameter('wheel_separation_m', 0.2)
        self.declare_parameter('max_wheel_speed_rad_s', 10.0)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('backend', 'sim')

        self._wheel_radius_m = self.get_parameter('wheel_radius_m').value
        self._wheel_separation_m = self.get_parameter('wheel_separation_m').value
        self._max_wheel_speed_rad_s = self.get_parameter('max_wheel_speed_rad_s').value
        publish_rate_hz = self.get_parameter('publish_rate_hz').value
        backend_name = self.get_parameter('backend').value

        now_s = self.get_clock().now().nanoseconds / 1e9

        if backend_name == 'sim':
            self._backend = SimMotorBackend(now_s=now_s)
        else:
            raise ValueError(
                f"Unknown backend '{backend_name}' - only 'sim' is implemented; "
                "a 'real' GPIO backend is added at the hardware bring-up checkpoint."
            )

        self._odometry = OdometryTracker(self._wheel_radius_m, self._wheel_separation_m)

        self._cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._last_update_s = now_s
        self._timer = self.create_timer(1.0 / publish_rate_hz, self._on_timer)

    def _on_cmd_vel(self, msg):
        left_rad_s, right_rad_s = twist_to_wheel_speeds(
            msg.linear.x, msg.angular.z, self._wheel_radius_m, self._wheel_separation_m
        )
        left_rad_s = max(-self._max_wheel_speed_rad_s, min(self._max_wheel_speed_rad_s, left_rad_s))
        right_rad_s = max(-self._max_wheel_speed_rad_s, min(self._max_wheel_speed_rad_s, right_rad_s))
        self._backend.apply_wheel_speeds(left_rad_s, right_rad_s)

    def _on_timer(self):
        now_s = self.get_clock().now().nanoseconds / 1e9
        left_delta_rad, right_delta_rad = self._backend.read_wheel_deltas(now_s)
        dt_s = now_s - self._last_update_s
        self._last_update_s = now_s
        if dt_s <= 0:
            return
        linear_x_m_s, angular_z_rad_s = self._odometry.update(left_delta_rad, right_delta_rad, dt_s)
        self._publish_odometry(linear_x_m_s, angular_z_rad_s)

    def _publish_odometry(self, linear_x_m_s, angular_z_rad_s):
        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quaternion(self._odometry.theta_rad)

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self._odometry.x_m
        odom_msg.pose.pose.position.y = self._odometry.y_m
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        odom_msg.twist.twist.linear.x = linear_x_m_s
        odom_msg.twist.twist.angular.z = angular_z_rad_s
        self._odom_pub.publish(odom_msg)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = self._odometry.x_m
        tf_msg.transform.translation.y = self._odometry.y_m
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Syntax-check the node**

```bash
source /opt/ros/humble/setup.bash
python3 -m py_compile /home/peter_sha/sourcecode/smart-walker-bot/src/walker_motor_driver/walker_motor_driver/motor_driver_node.py
```

Expected: no output, exit code 0.

- [ ] **Step 3: Write the launch file**

Create (overwrite) `src/walker_motor_driver/launch/motor_driver.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    backend_arg = DeclareLaunchArgument(
        'backend',
        default_value='sim',
        description="Motor backend to use: 'sim' (default) or 'real' (not yet implemented - added at hardware bring-up).",
    )

    motor_driver_node = Node(
        package='walker_motor_driver',
        executable='motor_driver_node',
        name='walker_motor_driver',
        output='screen',
        parameters=[{
            'wheel_radius_m': 0.03,
            'wheel_separation_m': 0.2,
            'max_wheel_speed_rad_s': 10.0,
            'publish_rate_hz': 20.0,
            'backend': LaunchConfiguration('backend'),
        }],
    )

    return LaunchDescription([backend_arg, motor_driver_node])
```

- [ ] **Step 4: Write the end-to-end verification script**

Create `src/walker_motor_driver/tools/verify_motor_driver.py`:

```python
#!/usr/bin/env python3
"""Scripted end-to-end check for walker_motor_driver - not a pytest test.

motor_driver_node.py needs a live rclpy context and the sim backend's
real-time clock, which doesn't fit the fast, deterministic pytest suite
the rest of this package uses (see this package's README). Unlike
walker_safety's hardware bring-up, this doesn't need any physical
hardware - just the node running with the sim backend.

Usage (after `colcon build --packages-select walker_motor_driver` and
`source install/setup.bash` from src/, with the node already launched
via `ros2 launch walker_motor_driver motor_driver.launch.py`):

    python3 tools/verify_motor_driver.py

Exits 0 and prints PASS on success, exits 1 and prints FAIL otherwise.
"""
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class VerifyNode(Node):
    def __init__(self):
        super().__init__('walker_motor_driver_verify')
        self.latest_odom = None
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)

    def _on_odom(self, msg):
        self.latest_odom = msg


def main():
    rclpy.init()
    node = VerifyNode()

    try:
        twist = Twist()
        twist.linear.x = 1.0
        node.cmd_pub.publish(twist)

        deadline = time.monotonic() + 5.0
        while node.latest_odom is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)

        if node.latest_odom is None:
            print('FAIL: no /odom message received within 5s')
            return 1

        first_x = node.latest_odom.pose.pose.position.x
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
        second_x = node.latest_odom.pose.pose.position.x

        if not (second_x > first_x):
            print(f'FAIL: pose.position.x did not increase ({first_x} -> {second_x})')
            return 1

        twist_x = node.latest_odom.twist.twist.linear.x
        if not (twist_x > 0.0):
            print(f'FAIL: twist.twist.linear.x should be positive, got {twist_x}')
            return 1

        print(f'PASS: odom.pose.position.x increased ({first_x:.4f} -> {second_x:.4f}), '
              f'twist.linear.x={twist_x:.4f}')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
```

- [ ] **Step 5: Syntax-check the verification script**

```bash
source /opt/ros/humble/setup.bash
python3 -m py_compile /home/peter_sha/sourcecode/smart-walker-bot/src/walker_motor_driver/tools/verify_motor_driver.py
```

Expected: no output, exit code 0.

- [ ] **Step 6: Build the full package**

```bash
source /opt/ros/humble/setup.bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src
colcon build --packages-select walker_motor_driver --symlink-install
source install/setup.bash
```

Expected: `Summary: 1 package finished`.

- [ ] **Step 7: Run the end-to-end verification against the sim backend**

```bash
source /opt/ros/humble/setup.bash
source /home/peter_sha/sourcecode/smart-walker-bot/src/install/setup.bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_motor_driver

ros2 launch walker_motor_driver motor_driver.launch.py > /tmp/motor_driver_node.log 2>&1 &
NODE_PID=$!
sleep 2

python3 tools/verify_motor_driver.py
VERIFY_EXIT=$?

kill $NODE_PID 2>/dev/null
wait $NODE_PID 2>/dev/null

echo "verify_motor_driver.py exit code: $VERIFY_EXIT"
cat /tmp/motor_driver_node.log
```

Expected: `verify_motor_driver.py` prints `PASS: ...` and `VERIFY_EXIT` is `0`. If it fails, `/tmp/motor_driver_node.log` has the node's stderr/stdout for debugging — check for parameter errors, import errors, or exceptions in `_on_timer`/`_on_cmd_vel`.

- [ ] **Step 8: Verify the backend-parameter error path**

Confirm an unknown `backend` value fails clearly rather than silently running with no backend:

```bash
source /opt/ros/humble/setup.bash
source /home/peter_sha/sourcecode/smart-walker-bot/src/install/setup.bash
timeout 5 ros2 run walker_motor_driver motor_driver_node --ros-args -p backend:=bogus
echo "exit code: $?"
```

Expected: the node raises `ValueError: Unknown backend 'bogus' - ...` and exits with a non-zero code (rclpy typically reports this as a Python traceback followed by a non-zero exit, not a hang) — confirm the process actually exits (doesn't need the 5s `timeout` to kill it) and the error message appears in the output.

- [ ] **Step 9: Commit**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot
git add src/walker_motor_driver/walker_motor_driver/motor_driver_node.py \
        src/walker_motor_driver/launch/motor_driver.launch.py \
        src/walker_motor_driver/tools/verify_motor_driver.py
git commit -m "$(cat <<'EOF'
Add walker_motor_driver ROS2 node, launch file, and E2E verification

motor_driver_node.py wires diff_drive_kinematics + SimMotorBackend to
/cmd_vel in, /odom + TF out - the interface walker_nav will configure
nav2/slam_toolbox against next. Verified end-to-end against the sim
backend with tools/verify_motor_driver.py; no physical hardware needed
for this verification, unlike walker_safety's Pico bring-up.
EOF
)"
```

---

## Self-Review Notes

- **Spec coverage:** §2.1 (Python/rclpy) — used throughout Task 4. §2.2 (real ament_python package) — Task 1. §2.3 (backend abstraction) — Task 3, with the `now_s` parameter refinement called out explicitly in Global Constraints rather than silently diverging from the spec's sketch signature. §2.4 (topic/message conventions) — Task 4's node. §2.5 (placeholder physical parameters) — Task 4's `declare_parameter` defaults, documented in Task 1's README. §2.6 (no coupling to walker_safety) — never referenced anywhere in the implementation; stated explicitly in Task 1's README and Global Constraints so it isn't "fixed" by someone adding it later. §3 (file structure) — matches exactly. §4 (testing approach) — Tasks 2-3 are pytest-TDD; Task 4's node gets the scripted `verify_motor_driver.py` check instead of pytest, as the spec anticipated, and — going further than the spec required — that check is fully automatable in this environment (no physical hardware needed, unlike `walker_safety`), so Task 4 runs it as a real pass/fail gate rather than a human-only manual step. §5 (out of scope) — no `GpioMotorBackend`, no hardware wiring, no `walker_nav` work, and no watchdog coupling appears anywhere in this plan.
- **Placeholder scan:** no TBD/TODO in any step. Task 1 Step 3's placeholder launch file is a real, valid, working `LaunchDescription([])`, not an unwritten stub — and it gets overwritten by Task 4's real launch file regardless of whether it was needed.
- **Type/name consistency:** `twist_to_wheel_speeds(linear_x_m_s, angular_z_rad_s, wheel_radius_m, wheel_separation_m)`, `OdometryTracker(wheel_radius_m, wheel_separation_m)` with `.x_m`/`.y_m`/`.theta_rad`/`.update(left_wheel_delta_rad, right_wheel_delta_rad, dt_s)`, and `yaw_to_quaternion(yaw_rad)` are used identically in Task 2's tests and Task 4's node. `MotorBackend.apply_wheel_speeds(left_rad_s, right_rad_s)` / `.read_wheel_deltas(now_s)` and `SimMotorBackend(now_s)` are used identically in Task 3's tests and Task 4's node. Parameter names (`wheel_radius_m`, `wheel_separation_m`, `max_wheel_speed_rad_s`, `publish_rate_hz`, `backend`) match between Task 4's node's `declare_parameter` calls and its launch file.
