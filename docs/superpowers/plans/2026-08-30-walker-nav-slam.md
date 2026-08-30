# walker_nav (SLAM pass) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce the `walker_nav` ROS2 package's SLAM pass: a simulated LiDAR (ray-cast against a fixed room, from `walker_motor_driver`'s real tracked pose) feeding `slam_toolbox`, verified by actually driving the simulated robot through a doorway into a second room and confirming a real map gets built.

**Architecture:** A pure-Python room-map/ray-casting core (`room_map.py`, no ROS or hardware imports, pytest-tested) wrapped by a thin `rclpy` node (`fake_lidar_node.py`) that subscribes `walker_motor_driver`'s `/odom` and publishes `sensor_msgs/LaserScan` on `/scan`. A `slam_toolbox` params file binds just the frames/topics that matter; a launch file starts both together. Nav2 is explicitly out of scope for this plan (deferred to a separate follow-up spec/plan).

**Tech Stack:** Python 3 + `rclpy` (ROS2 Humble), pytest (pure-module unit tests), `slam_toolbox` (already installed on this workstation via `ros-humble-slam-toolbox`), standard ROS2 messages (`nav_msgs/Odometry`, `sensor_msgs/LaserScan`, `nav_msgs/OccupancyGrid`, `tf2_msgs/TFMessage`).

**Spec:** `docs/superpowers/specs/2026-08-30-walker-nav-design.md` (§2 for decisions, §3 for room geometry, §4 for file structure, §5 for testing).

## Global Constraints

- Room geometry (spec §3.1) — exact wall segments, meters, as `(x1, y1, x2, y2)` line segments:
  ```
  Room 1: x in [-2.0, 2.0], y in [-1.5, 1.5]  (robot starts at (0, 0, 0), facing +x)
  Doorway: gap in Room 1's y=1.5 wall, from x=-0.5 to x=0.5
  Room 2: x in [-1.0, 1.0], y in [1.5, 3.5]
  ```
  The robot's start pose is the room's local origin — this is also `walker_motor_driver`'s odometry origin by construction, so no coordinate offset is ever applied anywhere in this package. (spec §2.3)
- `fake_lidar_node.py` publishes `sensor_msgs/LaserScan` with `frame_id='base_link'` directly — no separate `laser` frame, no static transform publisher needed. (spec §2.4)
- `slam_toolbox` params bind only `odom_frame`, `map_frame`, `base_frame`, `scan_topic`, `resolution`, `max_laser_range` — everything else stays at `slam_toolbox`'s own defaults. (spec §2.5)
- Pure modules (`room_map.py`) have zero `rclpy` imports, so their tests run with plain `python3 -m pytest` — no ROS environment sourcing or colcon build required. **This means any function meant to be pytest-tested must live in `room_map.py`, not in a file that imports `rclpy` at module level** (a file importing `rclpy` fails to import at all under this workstation's plain `python3`, which lacks `rclpy` — see `docs/superpowers/plans/2026-08-30-walker-motor-driver.md`'s Global Constraints for why). `test/conftest.py` inserts the package's *outer* directory onto `sys.path`, so tests use the fully-qualified `from walker_nav.room_map import ...` form, matching what `fake_lidar_node.py` uses — same convention `walker_motor_driver` established (and the same convention that plan's pre-flight review had to fix after an initial bare-import mistake — this plan uses the correct form from the start).
- `walker_motor_driver`'s `/cmd_vel` interface has a `cmd_vel_timeout_s` parameter (default 0.5s, added in that package's final review) — any script driving the simulated robot **must republish its command periodically** (at least every ~0.1-0.2s), not publish once and wait, or the robot will stop mid-maneuver. (`docs/superpowers/specs/2026-08-30-walker-motor-driver-design.md` §2.6, as amended)
- This is a real `ament_python` colcon package (like `walker_motor_driver`, unlike `walker_safety`) — buildable with `colcon build --packages-select walker_nav` from the `src/` workspace root. This workstation needs `PYTHONNOUSERSITE=1` set for any `colcon build` to succeed (a pre-existing, unrelated environment issue — see `walker_motor_driver`'s plan/ledger).
- Nav2 (path planning, costmaps, behavior tree, recovery behaviors) and map persistence (saving `/map` to disk) are out of scope for this plan. (spec §6)

---

## Task 1: Package Scaffold

**Files:**
- Create: `src/walker_nav/package.xml`
- Create: `src/walker_nav/setup.py`
- Create: `src/walker_nav/setup.cfg`
- Create: `src/walker_nav/resource/walker_nav`
- Create: `src/walker_nav/walker_nav/__init__.py`
- Create: `src/walker_nav/README.md`

**Interfaces:**
- Produces: an installable, buildable `ament_python` package shell. `console_scripts` entry point `fake_lidar_node = walker_nav.fake_lidar_node:main` is declared now even though `fake_lidar_node.py` doesn't exist until Task 3 — `colcon build` doesn't import entry-point targets at build time, only when actually run, matching the pattern `walker_motor_driver`'s Task 1 already used successfully in this project.

- [ ] **Step 1: Create the package directories**

```bash
mkdir -p /home/peter_sha/sourcecode/smart-walker-bot/src/walker_nav/walker_nav
mkdir -p /home/peter_sha/sourcecode/smart-walker-bot/src/walker_nav/resource
mkdir -p /home/peter_sha/sourcecode/smart-walker-bot/src/walker_nav/config
mkdir -p /home/peter_sha/sourcecode/smart-walker-bot/src/walker_nav/launch
mkdir -p /home/peter_sha/sourcecode/smart-walker-bot/src/walker_nav/test
mkdir -p /home/peter_sha/sourcecode/smart-walker-bot/src/walker_nav/tools
```

- [ ] **Step 2: Write package.xml**

Create `src/walker_nav/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>walker_nav</name>
  <version>0.0.1</version>
  <description>SLAM integration layer for smart-walker-bot: a simulated LiDAR (ray-cast against a fixed room) feeding slam_toolbox, until real hardware exists.</description>
  <maintainer email="botheredbybees@gmail.com">botheredbybees</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>nav_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>tf2_msgs</depend>
  <exec_depend>slam_toolbox</exec_depend>

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

Create `src/walker_nav/setup.py`. `launch/walker_nav.launch.py` and `config/slam_toolbox_params.yaml` are referenced here but don't exist until Task 3 — if this task's own build-verification step (Step 6) fails because either file is missing, create the placeholders shown in the note below first:

```python
from setuptools import find_packages, setup

package_name = 'walker_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/walker_nav.launch.py']),
        ('share/' + package_name + '/config', ['config/slam_toolbox_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='botheredbybees',
    maintainer_email='botheredbybees@gmail.com',
    description='SLAM integration layer for smart-walker-bot: a simulated LiDAR (ray-cast against a fixed room) feeding slam_toolbox, until real hardware exists.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_lidar_node = walker_nav.fake_lidar_node:main',
        ],
    },
)
```

If Step 6's build fails on the missing files, create these placeholders first, then retry (Task 3 overwrites both with the real content):

`launch/walker_nav.launch.py`:
```python
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([])
```

`config/slam_toolbox_params.yaml`:
```yaml
slam_toolbox:
  ros__parameters: {}
```

- [ ] **Step 4: Write setup.cfg**

Create `src/walker_nav/setup.cfg`:

```ini
[develop]
script_dir=$base/lib/walker_nav
[install]
install_scripts=$base/lib/walker_nav
```

- [ ] **Step 5: Create the resource marker and package __init__**

```bash
touch /home/peter_sha/sourcecode/smart-walker-bot/src/walker_nav/resource/walker_nav
touch /home/peter_sha/sourcecode/smart-walker-bot/src/walker_nav/walker_nav/__init__.py
```

- [ ] **Step 6: Verify the package builds**

```bash
source /opt/ros/humble/setup.bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src
PYTHONNOUSERSITE=1 colcon build --packages-select walker_nav --symlink-install
```

Expected: `Summary: 1 package finished`. If it fails on a missing `launch/`/`config/` file, create the placeholders from Step 3's note and retry.

- [ ] **Step 7: Write the package README**

Create `src/walker_nav/README.md`:

```markdown
# walker_nav (SLAM pass)

Simulated-LiDAR + `slam_toolbox` integration layer for smart-walker-bot.
See `docs/superpowers/specs/2026-08-30-walker-nav-design.md` for the
full design (this is a summary). Nav2 (path planning) is a separate,
later package — this one only covers building a map.

Real `ament_python` package — build with
`colcon build --packages-select walker_nav` from `src/` (this repo's
colcon workspace root).

## Layout

- `walker_nav/room_map.py` — pure Python: a fixed two-room floor plan
  (a 4m x 3m room connected to a 2m x 2m room via a 1m doorway) as wall
  line segments, plus ray-casting against it. Also has
  `yaw_from_quaternion`, decoding a heading from an odometry message's
  orientation. No ROS or hardware imports; unit-tested with pytest.
- `walker_nav/fake_lidar_node.py` — the `rclpy` node: subscribes
  `walker_motor_driver`'s `/odom`, publishes `sensor_msgs/LaserScan` on
  `/scan` (`frame_id='base_link'`, no separate laser frame) built from
  the room via `room_map.py`.
- `config/slam_toolbox_params.yaml` — binds `odom_frame`/`base_frame`/
  `map_frame`/`scan_topic`/`resolution`/`max_laser_range`; everything
  else is `slam_toolbox`'s own default.
- `launch/walker_nav.launch.py` — starts `fake_lidar_node` and
  `slam_toolbox`'s `online_async` node together.
- `tools/verify_slam.py` — scripted (not pytest) end-to-end check: with
  `walker_motor_driver` and this package both launched, drives the
  simulated robot through the doorway into Room 2 and confirms `/map`
  actually accumulates known cells and `slam_toolbox` publishes
  `map`→`odom` on `/tf`.

## Running the pure-module tests

```bash
cd src/walker_nav
python3 -m pytest test/ -v
```

No ROS environment or colcon build needed for these.

## The room's origin is the robot's start pose

`room_map.py`'s walls are defined so `(0, 0)` is both the room's local
origin and `walker_motor_driver`'s odometry origin (which always starts
at `(0, 0, 0)` by construction) — so `fake_lidar_node.py` reads `/odom`
directly as room coordinates, no offset math anywhere.
```

- [ ] **Step 8: Commit**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot
git add src/walker_nav/
git commit -m "$(cat <<'EOF'
Add walker_nav package scaffold

ament_python ROS2 package shell for the SLAM pass: package.xml,
setup.py/cfg, resource marker, and package README. colcon build
verified working before any node code exists.
EOF
)"
```

---

## Task 2: Room Map + Ray-Casting Core (TDD)

**Files:**
- Create: `src/walker_nav/walker_nav/room_map.py`
- Create: `src/walker_nav/test/conftest.py`
- Test: `src/walker_nav/test/test_room_map.py`

**Interfaces:**
- Produces: `ROOM_WALLS` (tuple of `(x1, y1, x2, y2)` wall segments); `cast_ray(x_m, y_m, angle_rad, max_range_m) -> float`; `scan_room(x_m, y_m, theta_rad, angle_min_rad, angle_increment_rad, num_beams, max_range_m) -> list[float]`; `yaw_from_quaternion(qz, qw) -> float`. All consumed by Task 3 (`fake_lidar_node.py`).

- [ ] **Step 1: Confirm pytest is available**

```bash
python3 -m pytest --version
```

If missing: `python3 -m pip install --user pytest`.

- [ ] **Step 2: Write the test-path conftest**

Create `src/walker_nav/test/conftest.py`:

```python
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
```

This inserts the *outer* `src/walker_nav/` directory (containing
`setup.py`) onto `sys.path`, so tests import as
`from walker_nav.room_map import ...` — the same fully-qualified form
`fake_lidar_node.py` (Task 3) will use. Do not insert the inner
`walker_nav/walker_nav/` directory directly — that produces bare
same-directory imports that only work by accident under pytest and
break at real runtime (this is exactly the defect
`walker_motor_driver`'s plan had to fix during its own pre-flight
review; this plan uses the correct form from the start).

- [ ] **Step 3: Write the failing tests**

Create `src/walker_nav/test/test_room_map.py`:

```python
import math

import pytest

from walker_nav.room_map import cast_ray, scan_room, yaw_from_quaternion


def test_cast_ray_hits_right_wall_at_known_distance():
    distance = cast_ray(0.0, 0.0, 0.0, max_range_m=8.0)
    assert distance == pytest.approx(2.0, rel=1e-6)


def test_cast_ray_facing_away_hits_left_wall():
    distance = cast_ray(0.0, 0.0, math.pi, max_range_m=8.0)
    assert distance == pytest.approx(2.0, rel=1e-6)


def test_cast_ray_through_doorway_hits_room2_far_wall():
    distance = cast_ray(0.0, 0.0, math.pi / 2, max_range_m=8.0)
    assert distance == pytest.approx(3.5, rel=1e-6)


def test_cast_ray_max_range_when_nothing_within_range():
    distance = cast_ray(0.0, 0.0, 0.0, max_range_m=1.0)
    assert distance == pytest.approx(1.0, rel=1e-9)


def test_cast_ray_rejects_non_positive_max_range():
    with pytest.raises(ValueError):
        cast_ray(0.0, 0.0, 0.0, max_range_m=0.0)


def test_scan_room_returns_one_reading_per_beam():
    ranges = scan_room(
        0.0, 0.0, 0.0,
        angle_min_rad=-math.pi, angle_increment_rad=(2 * math.pi) / 8,
        num_beams=8, max_range_m=8.0,
    )
    assert len(ranges) == 8


def test_scan_room_first_beam_matches_direct_cast_ray():
    angle_min_rad = -math.pi
    angle_increment_rad = (2 * math.pi) / 8
    ranges = scan_room(
        0.0, 0.0, 0.0,
        angle_min_rad=angle_min_rad, angle_increment_rad=angle_increment_rad,
        num_beams=8, max_range_m=8.0,
    )
    expected_first = cast_ray(0.0, 0.0, angle_min_rad, max_range_m=8.0)
    assert ranges[0] == pytest.approx(expected_first, rel=1e-9)


def test_scan_room_rejects_non_positive_num_beams():
    with pytest.raises(ValueError):
        scan_room(0.0, 0.0, 0.0, angle_min_rad=-math.pi, angle_increment_rad=0.1,
                  num_beams=0, max_range_m=8.0)


def test_yaw_from_quaternion_identity_gives_zero():
    assert yaw_from_quaternion(0.0, 1.0) == pytest.approx(0.0, abs=1e-9)


def test_yaw_from_quaternion_half_turn():
    assert yaw_from_quaternion(1.0, 0.0) == pytest.approx(math.pi, rel=1e-6)


def test_yaw_from_quaternion_quarter_turn():
    half = math.pi / 4
    assert yaw_from_quaternion(math.sin(half), math.cos(half)) == pytest.approx(math.pi / 2, rel=1e-6)


def test_yaw_from_quaternion_round_trips_for_several_angles():
    for yaw in (0.3, -1.2, 2.5):
        half = yaw / 2.0
        recovered = yaw_from_quaternion(math.sin(half), math.cos(half))
        assert recovered == pytest.approx(yaw, rel=1e-6)
```

- [ ] **Step 4: Run tests to verify they fail**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_nav
python3 -m pytest test/test_room_map.py -v
```

Expected: `ModuleNotFoundError: No module named 'walker_nav.room_map'` (or `No module named 'walker_nav'` if `walker_nav/__init__.py` from Task 1 is somehow missing).

- [ ] **Step 5: Implement the room map module**

Create `src/walker_nav/walker_nav/room_map.py`:

```python
"""Pure 2D ray-casting against a fixed, hardcoded room - the simulated
environment walker_nav's fake LiDAR scans against. See
docs/superpowers/specs/2026-08-30-walker-nav-design.md Sec 2.2-2.3, 3
for why this is a lightweight hand-built room rather than Gazebo/a real
simulator, and why its origin coincides with the robot's odometry
origin (no offset math needed anywhere this module is used).
"""
import math

# Walls as (x1, y1, x2, y2) line segments, meters. Two connected
# rectangular rooms via a 1m doorway - enough geometry for slam_toolbox
# to build a real, if simple, map. The robot starts at (0, 0, 0), which
# is both this room's local origin and walker_motor_driver's odometry
# origin by construction.
ROOM_WALLS = (
    (-2.0, -1.5, 2.0, -1.5),   # room 1 bottom
    (-2.0, -1.5, -2.0, 1.5),   # room 1 left
    (2.0, -1.5, 2.0, 1.5),     # room 1 right
    (-2.0, 1.5, -0.5, 1.5),    # room 1 top, left of doorway
    (0.5, 1.5, 2.0, 1.5),      # room 1 top, right of doorway
    (-1.0, 1.5, -1.0, 3.5),    # room 2 left
    (1.0, 1.5, 1.0, 3.5),      # room 2 right
    (-1.0, 3.5, 1.0, 3.5),     # room 2 top
)


def cast_ray(x_m, y_m, angle_rad, max_range_m):
    """Return the distance to the nearest wall along one ray from
    (x_m, y_m) in direction angle_rad, or max_range_m if nothing is
    hit within range."""
    if max_range_m <= 0:
        raise ValueError("max_range_m must be positive")

    dx = math.cos(angle_rad)
    dy = math.sin(angle_rad)
    nearest = max_range_m
    for (x1, y1, x2, y2) in ROOM_WALLS:
        hit = _ray_segment_intersection(x_m, y_m, dx, dy, x1, y1, x2, y2)
        if hit is not None and hit < nearest:
            nearest = hit
    return nearest


def _ray_segment_intersection(px, py, dx, dy, x1, y1, x2, y2):
    """Distance from (px, py) along direction (dx, dy) - assumed a unit
    vector, so the result is a physical distance - to its intersection
    with segment (x1, y1)-(x2, y2), or None if the ray (t >= 0) doesn't
    hit the segment (0 <= u <= 1)."""
    sx = x2 - x1
    sy = y2 - y1
    denom = dx * sy - dy * sx
    if abs(denom) < 1e-12:
        return None  # parallel (including near-tangent floating point cases)
    t = ((x1 - px) * sy - (y1 - py) * sx) / denom
    u = ((x1 - px) * dy - (y1 - py) * dx) / denom
    if t >= 0 and 0.0 <= u <= 1.0:
        return t
    return None


def scan_room(x_m, y_m, theta_rad, angle_min_rad, angle_increment_rad, num_beams, max_range_m):
    """Return a list of num_beams range readings, one per beam, starting
    at theta_rad + angle_min_rad and stepping by angle_increment_rad -
    matches sensor_msgs/LaserScan's angle_min/angle_increment convention
    directly, so the ROS node can copy angle_min_rad/angle_increment_rad
    straight into the message."""
    if num_beams <= 0:
        raise ValueError("num_beams must be positive")

    ranges = []
    for i in range(num_beams):
        beam_angle = theta_rad + angle_min_rad + i * angle_increment_rad
        ranges.append(cast_ray(x_m, y_m, beam_angle, max_range_m))
    return ranges


def yaw_from_quaternion(qz, qw):
    """Recover a 2D heading (radians) from a Z-axis-only quaternion -
    the inverse of walker_motor_driver's yaw_to_quaternion. Valid only
    for a pure yaw rotation (qx=qy=0), which is all this project's
    ground robots ever produce. Lives here (not in fake_lidar_node.py)
    so it's testable with plain pytest - fake_lidar_node.py imports
    rclpy at module level, which isn't installed under this
    workstation's plain python3, so nothing meant to be pytest-tested
    can live there.
    """
    return 2.0 * math.atan2(qz, qw)
```

- [ ] **Step 6: Run tests to verify they pass**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_nav
python3 -m pytest test/test_room_map.py -v
```

Expected: 13 passed.

- [ ] **Step 7: Commit**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot
git add src/walker_nav/walker_nav/room_map.py \
        src/walker_nav/test/conftest.py \
        src/walker_nav/test/test_room_map.py
git commit -m "$(cat <<'EOF'
Add walker_nav room map and ray-casting core

Pure-Python fixed two-room floor plan and ray-casting, plus a
quaternion-to-yaw helper, unit-tested with no ROS dependency.
fake_lidar_node.py (Task 3) wires this to real /odom and /scan topics.
EOF
)"
```

---

## Task 3: Fake LiDAR Node + slam_toolbox Config + Launch File

**Files:**
- Create: `src/walker_nav/walker_nav/fake_lidar_node.py`
- Create: `src/walker_nav/config/slam_toolbox_params.yaml` (overwrites Task 1's placeholder, if one was created)
- Create: `src/walker_nav/launch/walker_nav.launch.py` (overwrites Task 1's placeholder, if one was created)

**Interfaces:**
- Consumes: `scan_room`, `yaw_from_quaternion` from `room_map` (Task 2).
- Produces: the `/scan` topic and running `slam_toolbox` node that Task 4's verification (and, later, Nav2) depend on. Nothing later in this plan consumes it as a Python interface.

- [ ] **Step 1: Write the fake LiDAR node**

Create `src/walker_nav/walker_nav/fake_lidar_node.py`:

```python
"""walker_nav's fake LiDAR node: subscribes walker_motor_driver's
/odom, publishes a sensor_msgs/LaserScan on /scan built from the
robot's real, live-tracked pose against the fixed room in room_map.py.
See docs/superpowers/specs/2026-08-30-walker-nav-design.md Sec 2.4.
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from walker_nav.room_map import scan_room, yaw_from_quaternion


class FakeLidarNode(Node):
    def __init__(self):
        super().__init__('walker_nav_fake_lidar')

        self.declare_parameter('num_beams', 360)
        self.declare_parameter('max_range_m', 8.0)
        self.declare_parameter('scan_rate_hz', 5.0)

        self._num_beams = self.get_parameter('num_beams').value
        self._max_range_m = self.get_parameter('max_range_m').value
        scan_rate_hz = self.get_parameter('scan_rate_hz').value

        if self._num_beams <= 0:
            raise ValueError("num_beams must be positive")
        if self._max_range_m <= 0:
            raise ValueError("max_range_m must be positive")
        if scan_rate_hz <= 0:
            raise ValueError("scan_rate_hz must be positive")

        self._angle_min_rad = -math.pi
        self._angle_increment_rad = (2.0 * math.pi) / self._num_beams

        # Defaults match the room's origin (spec Sec 2.3) - if /odom
        # never arrives, the node still publishes a valid, if
        # stationary-at-the-origin, scan rather than erroring.
        self._x_m = 0.0
        self._y_m = 0.0
        self._theta_rad = 0.0

        self._odom_sub = self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self._scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self._timer = self.create_timer(1.0 / scan_rate_hz, self._on_timer)

    def _on_odom(self, msg):
        self._x_m = msg.pose.pose.position.x
        self._y_m = msg.pose.pose.position.y
        self._theta_rad = yaw_from_quaternion(
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        )

    def _on_timer(self):
        ranges = scan_room(
            self._x_m, self._y_m, self._theta_rad,
            self._angle_min_rad, self._angle_increment_rad,
            self._num_beams, self._max_range_m,
        )
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.angle_min = self._angle_min_rad
        msg.angle_max = self._angle_min_rad + (self._num_beams - 1) * self._angle_increment_rad
        msg.angle_increment = self._angle_increment_rad
        msg.range_min = 0.05
        msg.range_max = self._max_range_m
        msg.ranges = ranges
        self._scan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FakeLidarNode()
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

- [ ] **Step 2: Syntax-check the node**

```bash
source /opt/ros/humble/setup.bash
python3 -m py_compile /home/peter_sha/sourcecode/smart-walker-bot/src/walker_nav/walker_nav/fake_lidar_node.py
```

Expected: no output, exit code 0.

- [ ] **Step 3: Write the slam_toolbox params file**

Create (overwrite) `src/walker_nav/config/slam_toolbox_params.yaml`:

```yaml
slam_toolbox:
  ros__parameters:
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    mode: mapping
    resolution: 0.05
    max_laser_range: 8.0
```

- [ ] **Step 4: Write the launch file**

Create (overwrite) `src/walker_nav/launch/walker_nav.launch.py`:

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    walker_nav_share = get_package_share_directory('walker_nav')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')
    params_file = os.path.join(walker_nav_share, 'config', 'slam_toolbox_params.yaml')

    fake_lidar_node = Node(
        package='walker_nav',
        executable='fake_lidar_node',
        name='walker_nav_fake_lidar',
        output='screen',
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'slam_params_file': params_file}.items(),
    )

    return LaunchDescription([fake_lidar_node, slam_toolbox_launch])
```

- [ ] **Step 5: Build the package**

```bash
source /opt/ros/humble/setup.bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src
PYTHONNOUSERSITE=1 colcon build --packages-select walker_nav --symlink-install
source install/setup.bash
```

Expected: `Summary: 1 package finished`.

- [ ] **Step 6: Smoke-test the launch (no walker_motor_driver yet — this only confirms walker_nav itself doesn't crash)**

`walker_motor_driver` isn't running in this step, so there's no real `/odom`/`odom->base_link` TF — `slam_toolbox` is expected to log warnings about a missing transform and simply wait, not crash. That's the correct, graceful behavior this step confirms; full mapping verification (with `walker_motor_driver` running) is Task 4's job.

```bash
source /opt/ros/humble/setup.bash
source /home/peter_sha/sourcecode/smart-walker-bot/src/install/setup.bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_nav

ros2 launch walker_nav walker_nav.launch.py > /tmp/walker_nav_smoke.log 2>&1 &
LAUNCH_PID=$!
sleep 5

if kill -0 $LAUNCH_PID 2>/dev/null; then
    echo "walker_nav launch is still running after 5s (expected)"
else
    echo "walker_nav launch exited early (unexpected) - check /tmp/walker_nav_smoke.log"
fi

timeout 3 ros2 topic echo /scan --once > /tmp/walker_nav_scan_check.log 2>&1
if [ -s /tmp/walker_nav_scan_check.log ]; then
    echo "/scan is publishing"
else
    echo "/scan produced no output - check /tmp/walker_nav_scan_check.log"
fi

kill $LAUNCH_PID 2>/dev/null
wait $LAUNCH_PID 2>/dev/null
cat /tmp/walker_nav_smoke.log
```

Expected: the launch process is still alive after 5s, `/scan` produces at least one message (the node publishes from its default zero pose even with no `/odom` input), and the log shows no Python traceback (warnings about a missing `odom`→`base_link` transform from `slam_toolbox` are expected and fine).

- [ ] **Step 7: Commit**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot
git add src/walker_nav/walker_nav/fake_lidar_node.py \
        src/walker_nav/config/slam_toolbox_params.yaml \
        src/walker_nav/launch/walker_nav.launch.py
git commit -m "$(cat <<'EOF'
Add walker_nav fake LiDAR node, slam_toolbox config, and launch file

fake_lidar_node.py wires room_map.py's ray-casting to real /odom in,
/scan out. slam_toolbox_params.yaml binds only the frames/topics that
matter, defaults for everything else. Smoke-tested standalone (no
walker_motor_driver yet, so slam_toolbox gracefully waits on the
missing odom->base_link transform rather than crashing) - full mapping
verification is Task 4.
EOF
)"
```

---

## Task 4: End-to-End SLAM Verification

**Files:**
- Create: `src/walker_nav/tools/verify_slam.py`

**Interfaces:**
- Consumes: `walker_motor_driver`'s `/cmd_vel` (in) and `/odom` (out) topics, and `walker_nav`'s `/scan`, `/map`, `/tf` topics — all via the ROS graph, not Python imports.
- Produces: nothing consumed by a later task — this is the last task in this plan. The `/map` topic this verifies is what a future Nav2 follow-up plan will consume.

- [ ] **Step 1: Write the end-to-end verification script**

Create `src/walker_nav/tools/verify_slam.py`:

```python
#!/usr/bin/env python3
"""Scripted end-to-end check for walker_nav's SLAM pipeline - not a
pytest test. Assumes walker_motor_driver's node (backend:=sim) and
walker_nav's fake_lidar_node + slam_toolbox are already running (see
this package's README for the launch commands). Drives the simulated
robot from its start pose, through the doorway, into Room 2, then
checks that /map has actually accumulated known cells and that
slam_toolbox is publishing map->odom on /tf - confirming SLAM is
genuinely running and building a map, not just that topics are wired
up.

Usage: python3 tools/verify_slam.py
(On this project's dev workstation, use /usr/bin/python3 if plain
python3 can't import rclpy - see walker_motor_driver's
verify_motor_driver.py docstring for why.)

Exits 0 and prints PASS on success, exits 1 and prints FAIL otherwise.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

# Robot starts at (0,0,0) facing +x; the doorway is reached by heading
# +y (spec Sec 3.1). Turn ~90 degrees at pi/2 rad/s for 1s (an exact
# 90-degree turn if timing were perfect; a few degrees of loop-timing
# jitter is fine - the 1m-wide doorway comfortably absorbs errors up to
# about 18 degrees given the ~1.5m distance to it, worked out from the
# room geometry in room_map.py).
TURN_ANGULAR_Z_RAD_S = math.pi / 2
TURN_DURATION_S = 1.0

# linear.x=1.0 is clamped by walker_motor_driver's placeholder
# max_wheel_speed_rad_s=10.0/wheel_radius_m=0.03 to an actual ~0.3 m/s
# (see walker_motor_driver's verify_motor_driver.py for the same
# arithmetic). ~2.7m at 0.3 m/s takes 9s, landing well inside Room 2
# (which spans y in [1.5, 3.5]) without reaching its far wall.
DRIVE_LINEAR_X = 1.0
DRIVE_DURATION_S = 9.0

SETTLE_DURATION_S = 3.0

MINIMUM_KNOWN_CELLS = 50  # /map cells that are free (0) or occupied (100), not unknown (-1)

# Republish every command at this interval - walker_motor_driver has a
# cmd_vel_timeout_s (default 0.5s) that zeroes wheel speeds if no
# command arrives in time, so a single publish-and-wait would stop the
# robot mid-maneuver.
REPUBLISH_INTERVAL_S = 0.1


class VerifySlamNode(Node):
    def __init__(self):
        super().__init__('walker_nav_verify_slam')
        self.latest_map = None
        self.saw_map_to_odom_tf = False
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, 10)
        self.create_subscription(TFMessage, '/tf', self._on_tf, 10)

    def _on_map(self, msg):
        self.latest_map = msg

    def _on_tf(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == 'map' and transform.child_frame_id == 'odom':
                self.saw_map_to_odom_tf = True

    def publish_cmd(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)


def _drive_phase(node, linear_x, angular_z, duration_s):
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        node.publish_cmd(linear_x, angular_z)
        rclpy.spin_once(node, timeout_sec=REPUBLISH_INTERVAL_S)


def main():
    rclpy.init()
    node = VerifySlamNode()

    try:
        _drive_phase(node, linear_x=0.0, angular_z=TURN_ANGULAR_Z_RAD_S, duration_s=TURN_DURATION_S)
        _drive_phase(node, linear_x=DRIVE_LINEAR_X, angular_z=0.0, duration_s=DRIVE_DURATION_S)
        _drive_phase(node, linear_x=0.0, angular_z=0.0, duration_s=SETTLE_DURATION_S)

        if node.latest_map is None:
            print('FAIL: no /map message received')
            return 1

        known_cells = sum(1 for cell in node.latest_map.data if cell != -1)
        if known_cells < MINIMUM_KNOWN_CELLS:
            print(f'FAIL: /map has only {known_cells} known cells, expected at least '
                  f'{MINIMUM_KNOWN_CELLS} - slam_toolbox may not be receiving /scan data')
            return 1

        if not node.saw_map_to_odom_tf:
            print('FAIL: no map->odom transform seen on /tf - slam_toolbox may not be running')
            return 1

        print(f'PASS: /map has {known_cells} known cells, map->odom TF observed')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
```

- [ ] **Step 2: Syntax-check the script**

```bash
source /opt/ros/humble/setup.bash
python3 -m py_compile /home/peter_sha/sourcecode/smart-walker-bot/src/walker_nav/tools/verify_slam.py
```

Expected: no output, exit code 0.

- [ ] **Step 3: Run the full end-to-end verification**

Both `walker_motor_driver` and `walker_nav` must already be built (`colcon build --packages-select walker_motor_driver walker_nav --symlink-install` with `PYTHONNOUSERSITE=1`, from a prior task or session — `walker_motor_driver` was built in its own plan; rebuild it here too if `src/install/` was cleaned since then).

```bash
source /opt/ros/humble/setup.bash
source /home/peter_sha/sourcecode/smart-walker-bot/src/install/setup.bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src

ros2 launch walker_motor_driver motor_driver.launch.py > /tmp/motor_driver_node.log 2>&1 &
MOTOR_PID=$!
ros2 launch walker_nav walker_nav.launch.py > /tmp/walker_nav.log 2>&1 &
NAV_PID=$!
sleep 3

python3 walker_nav/tools/verify_slam.py
VERIFY_EXIT=$?

kill $MOTOR_PID $NAV_PID 2>/dev/null
wait $MOTOR_PID $NAV_PID 2>/dev/null

echo "verify_slam.py exit code: $VERIFY_EXIT"
echo "--- motor_driver_node.log ---"
cat /tmp/motor_driver_node.log
echo "--- walker_nav.log ---"
cat /tmp/walker_nav.log
```

Expected: `verify_slam.py` prints `PASS: /map has N known cells, map->odom TF observed` (N well over 50) and `VERIFY_EXIT` is `0`. The whole script takes about 15-20 seconds (turn + drive + settle phases) plus the two `ros2 launch` startup times — this is expected, not a hang. If it fails, use the two log files: `motor_driver_node.log` for anything wrong with the sim/odometry side, `walker_nav.log` for the fake LiDAR or `slam_toolbox` side (look for repeated "missing transform" warnings past the first few seconds, which would mean `walker_motor_driver`'s `odom->base_link` TF never arrived).

- [ ] **Step 4: Commit**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot
git add src/walker_nav/tools/verify_slam.py
git commit -m "$(cat <<'EOF'
Add walker_nav end-to-end SLAM verification

Drives the simulated robot through the doorway into Room 2 with both
walker_motor_driver and walker_nav running, confirms /map accumulates
real known cells and slam_toolbox publishes map->odom on /tf. No
physical hardware needed - the room_map.py simulation is enough.
EOF
)"
```

---

## Self-Review Notes

- **Spec coverage:** §2.1 (SLAM-only, Nav2 deferred) — nothing in this plan touches Nav2/costmaps/behavior trees, called out explicitly in Global Constraints. §2.2-2.3 (fixed room, pose-consistent, no offset math) — Task 2's `room_map.py` and its exact wall coordinates, Task 3's `fake_lidar_node.py` reading `/odom` directly with zero offset arithmetic. §2.4 (fake LiDAR: `/odom` in, `/scan` out, `frame_id='base_link'`) — Task 3. §2.5 (slam_toolbox params, minimal) — Task 3's `slam_toolbox_params.yaml` binds exactly the six params the spec names, nothing more. §3 (room geometry, ray-casting) — Task 2, coordinates copied verbatim from spec §3.1 into Global Constraints and then into the implementation. §4 (file structure) — matches exactly. §5 (testing: pytest for the pure core, scripted E2E for the rest, checking both `/map` content and `/tf`) — Tasks 2-4. §6 (out of scope: Nav2, map persistence, real LiDAR/GPIO) — none of these appear anywhere in this plan.
- **Placeholder scan:** no TBD/TODO in any step. Task 1's placeholder launch file and params YAML are real, minimal, valid content (an empty `LaunchDescription` and an empty `ros__parameters` block), not unwritten stubs, and both get overwritten by Task 3's real versions regardless of whether they were needed.
- **Type/name consistency:** `cast_ray(x_m, y_m, angle_rad, max_range_m)`, `scan_room(x_m, y_m, theta_rad, angle_min_rad, angle_increment_rad, num_beams, max_range_m)`, and `yaw_from_quaternion(qz, qw)` are used identically in Task 2's tests and Task 3's node. `ROOM_WALLS` coordinates in Task 2's implementation match the Global Constraints block and the spec exactly (checked each of the 8 segments against spec §3.1's room bounds). Task 4's `verify_slam.py` topic names (`/cmd_vel`, `/map`, `/tf`) and message types (`geometry_msgs/Twist`, `nav_msgs/OccupancyGrid`, `tf2_msgs/TFMessage`) match what `walker_motor_driver` and `walker_nav`'s own code actually publish/subscribe.
- **Cross-package interface risk (learned from `walker_motor_driver`'s final review):** `walker_motor_driver`'s `cmd_vel_timeout_s` (added in that package's post-review fix, not its original plan) means any script driving the robot must republish continuously — Task 4's `verify_slam.py` does this via `_drive_phase`'s spin-and-republish loop, called out explicitly in Global Constraints so this isn't rediscovered the hard way during Task 4's own review.
