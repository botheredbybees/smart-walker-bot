# walker_nav Kinect Sensing Redesign Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give `walker_nav`'s simulated LiDAR a backward-compatible narrow-field-of-view mode,
and use it to genuinely re-verify the existing SLAM/Nav2 test suite against a Kinect-realistic
sensor profile (57° FOV, 4.0m range).

**Architecture:** Extract the angle-parameter math that today's full-circle `fake_lidar_node.py`
computes inline into a new pure function in `room_map.py` (`fov_to_scan_params`), pytest-covered
directly. Wire that function into `fake_lidar_node.py` behind a new `fov_deg` parameter
(default `360`, exactly reproducing today's behavior). Expose `fov_deg`/`max_range_m` as launch
arguments on the existing `walker_nav.launch.py` (default-preserving, mirroring
`walker_motor_driver.launch.py`'s `backend` argument pattern) so the Kinect profile can be
launched with the same launch file, no new file. Then run the existing `verify_slam.py`/
`verify_nav2.py` scripts unmodified against both profiles and document what actually happens —
this is a real experiment, not an assumed pass.

**Tech Stack:** Python 3, ROS2 Humble (`rclpy`, `slam_toolbox`, `nav2_bringup`), pytest,
`ament_python`/`colcon`.

**Spec:** `docs/superpowers/specs/2026-09-01-walker-nav-kinect-design.md`

## Global Constraints

- `fov_deg >= 360` must reproduce today's exact formula (`angle_min_rad = -pi`,
  `angle_increment_rad = 2*pi / num_beams`) — zero regression on the existing full-circle path.
- `fov_deg < 360` uses a non-wrapping arc: `angle_min_rad = -fov_rad / 2`,
  `angle_increment_rad = fov_rad / (num_beams - 1)`, requiring `num_beams >= 2`.
- `room_map.py`'s `scan_room`/`cast_ray` get **zero changes** — they already take
  `angle_min_rad`/`angle_increment_rad` as plain parameters.
- This pass implements **Part 1 only** (spec §3): the simulation update and its re-verification.
  Part 2 (the real Kinect-backed sensing backend: `lidar_backend:=sim|kinect` launch argument,
  `kinect_depth_bridge_node.py`, `docs/kinect_bring_up.md`) stays documented-only in the spec —
  it has no automated verification available until hardware bring-up confirms which depth-camera
  driver actually works (spec §2.5), so there is nothing buildable/testable for it yet. Do not
  implement any Part 2 file in this plan.
- No changes to `slam_toolbox`/`nav2` configuration (`config/slam_toolbox_params.yaml`,
  `config/nav2_params.yaml`) — the whole point of the `/scan`-boundary design is that neither
  needs to change (spec §2.4, §5).
- If the Kinect-profile experiment (Task 4) fails, do not attempt to fix nav2 tuning, the room
  layout, or the doorway maneuver — deciding what to do about a real failure is explicitly future
  work, not part of this design (spec §5). Record the failure and stop.

---

## Task 1: `fov_to_scan_params` in `room_map.py`

**Files:**
- Modify: `src/walker_nav/walker_nav/room_map.py`
- Test: `src/walker_nav/test/test_room_map.py`

**Interfaces:**
- Produces: `fov_to_scan_params(fov_deg, num_beams) -> (angle_min_rad, angle_increment_rad)` in
  `walker_nav.room_map` — a pure function, no ROS imports. Task 2 imports and calls this directly.

- [ ] **Step 1: Write the failing tests**

Add to `src/walker_nav/test/test_room_map.py` (extend the existing `from walker_nav.room_map
import ...` line to include `fov_to_scan_params`):

```python
from walker_nav.room_map import cast_ray, fov_to_scan_params, scan_room, yaw_from_quaternion
```

```python
def test_fov_to_scan_params_full_circle_matches_existing_formula():
    angle_min_rad, angle_increment_rad = fov_to_scan_params(fov_deg=360, num_beams=360)
    assert angle_min_rad == pytest.approx(-math.pi, rel=1e-9)
    assert angle_increment_rad == pytest.approx((2.0 * math.pi) / 360, rel=1e-9)


def test_fov_to_scan_params_above_360_still_treated_as_full_circle():
    angle_min_rad, angle_increment_rad = fov_to_scan_params(fov_deg=400, num_beams=360)
    assert angle_min_rad == pytest.approx(-math.pi, rel=1e-9)
    assert angle_increment_rad == pytest.approx((2.0 * math.pi) / 360, rel=1e-9)


def test_fov_to_scan_params_narrow_arc_edges_land_on_fov_boundary():
    fov_deg = 57
    num_beams = 57
    angle_min_rad, angle_increment_rad = fov_to_scan_params(fov_deg=fov_deg, num_beams=num_beams)
    fov_rad = math.radians(fov_deg)
    assert angle_min_rad == pytest.approx(-fov_rad / 2.0, rel=1e-9)
    last_beam_angle = angle_min_rad + (num_beams - 1) * angle_increment_rad
    assert last_beam_angle == pytest.approx(fov_rad / 2.0, rel=1e-9)


def test_fov_to_scan_params_narrow_arc_rejects_single_beam():
    with pytest.raises(ValueError):
        fov_to_scan_params(fov_deg=57, num_beams=1)


def test_fov_to_scan_params_rejects_non_positive_fov_deg():
    with pytest.raises(ValueError):
        fov_to_scan_params(fov_deg=0, num_beams=57)


def test_fov_to_scan_params_rejects_non_positive_num_beams():
    with pytest.raises(ValueError):
        fov_to_scan_params(fov_deg=360, num_beams=0)
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd src/walker_nav
python3 -m pytest test/test_room_map.py -v -k fov_to_scan_params
```

Expected: FAIL / ERROR — `ImportError: cannot import name 'fov_to_scan_params'`.

- [ ] **Step 3: Implement `fov_to_scan_params`**

Add to `src/walker_nav/walker_nav/room_map.py`, after `cast_ray`/`_ray_segment_intersection` and
before `scan_room` (it's the natural producer of `scan_room`'s angle arguments):

```python
def fov_to_scan_params(fov_deg, num_beams):
    """Compute (angle_min_rad, angle_increment_rad) for a LaserScan-style
    beam fan, given a horizontal field of view in degrees and a beam
    count - the sensor_msgs/LaserScan angle_min/angle_increment
    convention scan_room and fake_lidar_node.py both use directly.

    fov_deg >= 360 reproduces the original full-circle formula exactly:
    a full circle deliberately does not place a beam at both -180 and
    +180 degrees (the same physical direction), hence dividing by
    num_beams rather than num_beams - 1.

    fov_deg < 360 produces a non-wrapping arc whose first and last beams
    land exactly on the FOV's two edges (-fov_rad/2 and +fov_rad/2),
    which requires num_beams >= 2 - a single beam can't have two
    distinct edges.
    """
    if fov_deg <= 0:
        raise ValueError("fov_deg must be positive")
    if num_beams <= 0:
        raise ValueError("num_beams must be positive")

    if fov_deg >= 360:
        return -math.pi, (2.0 * math.pi) / num_beams

    if num_beams < 2:
        raise ValueError("num_beams must be at least 2 when fov_deg < 360")
    fov_rad = math.radians(fov_deg)
    return -fov_rad / 2.0, fov_rad / (num_beams - 1)
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd src/walker_nav
python3 -m pytest test/test_room_map.py -v
```

Expected: PASS — all tests, including the pre-existing ones (no regression).

- [ ] **Step 5: Commit**

```bash
git add src/walker_nav/walker_nav/room_map.py src/walker_nav/test/test_room_map.py
git commit -m "walker_nav: add fov_to_scan_params for narrow-FOV LiDAR sim"
```

---

## Task 2: Wire `fov_deg` into `fake_lidar_node.py`

**Files:**
- Modify: `src/walker_nav/walker_nav/fake_lidar_node.py`

**Interfaces:**
- Consumes: `fov_to_scan_params(fov_deg, num_beams) -> (angle_min_rad, angle_increment_rad)`
  from Task 1 (`walker_nav.room_map`).
- Produces: a new `fov_deg` ROS parameter on `walker_nav_fake_lidar`, default `360.0` — Task 3's
  launch file reads/overrides it via `LaunchConfiguration`.

This node isn't pytest-tested (it imports `rclpy` at module level — matches every other `rclpy`
node in this project, per `room_map.py`'s own `yaw_from_quaternion` docstring). Its behavior is
exercised at runtime in Task 4 via `verify_slam.py`/`verify_nav2.py`. This task's own check is a
build + import sanity check plus the full pure-module regression suite from Task 1.

- [ ] **Step 1: Add the parameter and wire the extracted function**

In `src/walker_nav/walker_nav/fake_lidar_node.py`, change the import line:

```python
from walker_nav.room_map import fov_to_scan_params, scan_room, yaw_from_quaternion
```

Replace the existing parameter-declaration/validation block in `__init__` (currently lines
21-37) with:

```python
        self.declare_parameter('num_beams', 360)
        self.declare_parameter('max_range_m', 8.0)
        self.declare_parameter('scan_rate_hz', 5.0)
        self.declare_parameter('fov_deg', 360.0)

        self._num_beams = self.get_parameter('num_beams').value
        self._max_range_m = self.get_parameter('max_range_m').value
        scan_rate_hz = self.get_parameter('scan_rate_hz').value
        fov_deg = self.get_parameter('fov_deg').value

        if self._num_beams <= 0:
            raise ValueError("num_beams must be positive")
        if self._max_range_m <= 0:
            raise ValueError("max_range_m must be positive")
        if scan_rate_hz <= 0:
            raise ValueError("scan_rate_hz must be positive")

        # fov_deg's own >0 / num_beams>=2-for-narrow-arc validation lives in
        # fov_to_scan_params itself (walker_nav.room_map) - see its docstring.
        self._angle_min_rad, self._angle_increment_rad = fov_to_scan_params(fov_deg, self._num_beams)
```

This removes the old two inline lines:
```python
        self._angle_min_rad = -math.pi
        self._angle_increment_rad = (2.0 * math.pi) / self._num_beams
```

- [ ] **Step 2: Build and run the pure-module regression suite**

```bash
source /opt/ros/humble/setup.bash
cd src
PYTHONNOUSERSITE=1 colcon build --packages-select walker_nav --symlink-install
python3 -m pytest walker_nav/test/ -v
```

Expected: `colcon build` succeeds with no errors; all pytest tests PASS (Task 1's tests are
unaffected by this node-only change, but re-running confirms nothing else broke).

- [ ] **Step 3: Smoke-test the node imports and starts cleanly with defaults**

```bash
source /opt/ros/humble/setup.bash
cd src
source install/setup.bash
timeout 3 ros2 run walker_nav fake_lidar_node
```

Expected: the node starts and logs normally for ~3 seconds (no `/odom` publisher running yet is
fine — it publishes from the room origin per its own doc comment), then `timeout` kills it with
no Python traceback printed before that point. A traceback (e.g. `ValueError`, `ImportError`)
means the wiring is wrong — re-check Step 1.

- [ ] **Step 4: Commit**

```bash
git add src/walker_nav/walker_nav/fake_lidar_node.py
git commit -m "walker_nav: wire fov_deg parameter into fake_lidar_node"
```

---

## Task 3: Expose `fov_deg`/`max_range_m` as launch arguments

**Files:**
- Modify: `src/walker_nav/launch/walker_nav.launch.py`

**Interfaces:**
- Consumes: the `fov_deg` parameter added to `fake_lidar_node` in Task 2 (plus the pre-existing
  `max_range_m` parameter).
- Produces: two new `ros2 launch` arguments, `fov_deg` (default `"360.0"`) and `max_range_m`
  (default `"8.0"`), on `walker_nav.launch.py` — Task 4 launches with these overridden to run the
  Kinect profile, with no new launch file (spec §2.3: "no new launch file needed - same
  `walker_nav.launch.py`, different parameter values").

- [ ] **Step 1: Add the launch arguments**

Replace the full contents of `src/walker_nav/launch/walker_nav.launch.py` with:

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Must match config/slam_toolbox_params.yaml's max_laser_range - nothing
# else keeps these two in sync (see walker_nav's README "Known limitations").
# max_range_m below is overridable at launch (see max_range_m_arg); these
# constants are only its default.
FAKE_LIDAR_MAX_RANGE_M = 8.0
FAKE_LIDAR_NUM_BEAMS = 360
FAKE_LIDAR_SCAN_RATE_HZ = 5.0
FAKE_LIDAR_FOV_DEG = 360.0


def generate_launch_description():
    walker_nav_share = get_package_share_directory('walker_nav')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')
    params_file = os.path.join(walker_nav_share, 'config', 'slam_toolbox_params.yaml')

    fov_deg_arg = DeclareLaunchArgument(
        'fov_deg',
        default_value=str(FAKE_LIDAR_FOV_DEG),
        description=(
            "fake_lidar_node's horizontal field of view in degrees. 360 (default) "
            "reproduces the full-circle sim; 57 is the documented Kinect-realistic "
            "profile (docs/superpowers/specs/2026-09-01-walker-nav-kinect-design.md Sec 2.3)."
        ),
    )
    max_range_m_arg = DeclareLaunchArgument(
        'max_range_m',
        default_value=str(FAKE_LIDAR_MAX_RANGE_M),
        description=(
            "fake_lidar_node's max sensing range in meters. 8.0 (default) matches "
            "config/slam_toolbox_params.yaml's max_laser_range; 4.0 is the documented "
            "Kinect-realistic profile."
        ),
    )

    fake_lidar_node = Node(
        package='walker_nav',
        executable='fake_lidar_node',
        name='walker_nav_fake_lidar',
        output='screen',
        parameters=[{
            'num_beams': FAKE_LIDAR_NUM_BEAMS,
            'max_range_m': ParameterValue(LaunchConfiguration('max_range_m'), value_type=float),
            'scan_rate_hz': FAKE_LIDAR_SCAN_RATE_HZ,
            'fov_deg': ParameterValue(LaunchConfiguration('fov_deg'), value_type=float),
        }],
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            # online_async_launch.py declares use_sim_time with
            # default_value='true' and applies it AFTER slam_params_file,
            # so it silently overrides anything set in the YAML - must be
            # passed here explicitly. Every other node in this project
            # (fake_lidar_node, walker_motor_driver) stamps with the wall
            # clock; leaving slam_toolbox on simulated time with no
            # /clock publisher works today only by accident.
            'slam_params_file': params_file,
            'use_sim_time': 'false',
        }.items(),
    )

    return LaunchDescription([fov_deg_arg, max_range_m_arg, fake_lidar_node, slam_toolbox_launch])
```

(Only the two new `DeclareLaunchArgument` actions, the `ParameterValue`-wrapped
`max_range_m`/`fov_deg` entries, and the corresponding new imports are actual changes; everything
else is copied unchanged from the existing file.)

- [ ] **Step 2: Build and verify the arguments are visible with correct defaults**

```bash
source /opt/ros/humble/setup.bash
cd src
PYTHONNOUSERSITE=1 colcon build --packages-select walker_nav --symlink-install
source install/setup.bash
ros2 launch walker_nav walker_nav.launch.py --show-args
```

Expected: output lists both `fov_deg` (default `'360.0'`) and `max_range_m` (default `'8.0'`),
with the descriptions from Step 1. This command only inspects the launch file — it does not
start any nodes.

- [ ] **Step 3: Re-run the pure-module regression suite**

```bash
python3 -m pytest walker_nav/test/ -v
```

Expected: PASS (this task touches no pytest-covered code, but re-confirming costs nothing).

- [ ] **Step 4: Commit**

```bash
git add src/walker_nav/launch/walker_nav.launch.py
git commit -m "walker_nav: expose fov_deg/max_range_m as launch arguments"
```

---

## Task 4: Re-verify SLAM/Nav2 against the full-circle and Kinect profiles

**Files:**
- Modify: `src/walker_nav/README.md` (document the new parameter/arguments and record the
  experiment's actual outcome)

**Interfaces:**
- Consumes: `walker_nav.launch.py`'s `fov_deg`/`max_range_m` launch arguments (Task 3);
  `tools/verify_slam.py`/`tools/verify_nav2.py` unmodified (spec §3 leaves open whether these
  need code changes — decided here: **no code changes needed**, since neither script depends on
  the sensor's angular coverage directly, only on the resulting `/map`/`/tf`/Nav2-action
  outcomes, which is exactly what makes the Kinect-profile run a genuine, undetermined
  experiment rather than a foregone conclusion).

This is the experiment the whole design exists to run (spec §2.3): **do not assume the outcome.**
Run every step below for real and record what actually happens.

- [ ] **Step 1: Build everything needed**

```bash
source /opt/ros/humble/setup.bash
cd src
PYTHONNOUSERSITE=1 colcon build --packages-select walker_motor_driver walker_nav --symlink-install
source install/setup.bash
```

- [ ] **Step 2: Regression run — SLAM, full-circle profile (defaults)**

```bash
ros2 launch walker_motor_driver motor_driver.launch.py &
ros2 launch walker_nav walker_nav.launch.py &
sleep 3
python3 walker_nav/tools/verify_slam.py   # or /usr/bin/python3, per its docstring
```

Expected: `PASS: final pose ... inside Room 2, /map has ... known cells, map->odom TF observed`
— this must still pass exactly as before (spec §4, "must still pass exactly as before"). If it
does not, the Task 2/3 wiring broke the default path — stop and fix before continuing (this is a
real regression, not the experiment).

Kill both launched processes when done (check `ps aux` for `fake_lidar_node`,
`async_slam_toolbox_node`, `motor_driver_node` still running and kill explicitly — `ros2 launch`'s
child nodes don't die from a plain kill on the launch parent, per the README's existing note).

- [ ] **Step 3: Regression run — Nav2, full-circle profile (defaults)**

```bash
ros2 launch walker_motor_driver motor_driver.launch.py &
ros2 launch walker_nav walker_nav.launch.py &
ros2 launch walker_nav nav2.launch.py &
sleep 10
python3 walker_nav/tools/verify_nav2.py
```

Expected: `PASS: navigate_to_pose SUCCEEDED, final pose ... is ...m from the goal` — must still
pass exactly as before. Same regression-vs-experiment logic as Step 2: a failure here means the
new code broke the default path, not a finding about the Kinect profile.

Kill all three launched processes when done (same `ps aux` caveat, three launch trees now).

- [ ] **Step 4: Experiment run — SLAM, Kinect profile**

```bash
ros2 launch walker_motor_driver motor_driver.launch.py &
ros2 launch walker_nav walker_nav.launch.py fov_deg:=57.0 max_range_m:=4.0 &
sleep 3
python3 walker_nav/tools/verify_slam.py
```

Record the exact PASS/FAIL line printed. Kill both processes when done.

- [ ] **Step 5: Experiment run — Nav2, Kinect profile**

```bash
ros2 launch walker_motor_driver motor_driver.launch.py &
ros2 launch walker_nav walker_nav.launch.py fov_deg:=57.0 max_range_m:=4.0 &
ros2 launch walker_nav nav2.launch.py &
sleep 10
python3 walker_nav/tools/verify_nav2.py
```

Record the exact PASS/FAIL line printed. Kill all three processes when done.

- [ ] **Step 6: Document the new parameter/arguments and the experiment's outcome**

In `src/walker_nav/README.md`'s "Layout" section, extend the existing
`walker_nav/fake_lidar_node.py` bullet's `Parameters: num_beams (default 360), max_range_m
(default 8.0 — must match config/slam_toolbox_params.yaml's max_laser_range), scan_rate_hz
(default 5.0).` sentence by appending a fourth parameter to that same list (same bullet, not a
new one):

```
, `fov_deg` (default 360 — full circle; <360 produces a non-wrapping
  forward arc via `room_map.py`'s `fov_to_scan_params`, e.g. 57 for a
  Kinect-realistic profile).
```

Add a new subsection right after "Running the end-to-end Nav2 check" (before "The room's origin
is the robot's start pose"):

```markdown
## Running the Kinect-realistic sensor profile

`walker_nav.launch.py` exposes `fov_deg` (default `360.0`) and `max_range_m` (default `8.0`) as
launch arguments — no separate launch file. The documented Kinect v1 profile
(`docs/superpowers/specs/2026-09-01-walker-nav-kinect-design.md` Sec 2.3) is `fov_deg=57`,
`max_range_m=4.0`:

```bash
ros2 launch walker_nav walker_nav.launch.py fov_deg:=57.0 max_range_m:=4.0
```

Substitute this for the plain `ros2 launch walker_nav walker_nav.launch.py` line in either the
SLAM or Nav2 end-to-end check above to re-run it against the narrow-FOV profile.

**Result of the first Kinect-profile run (`fov_deg=57`, `max_range_m=4.0`) against the existing
two-room floor plan and Nav2 tuning:** [FILL IN: paste the exact PASS/FAIL lines from Steps 4-5
above for both `verify_slam.py` and `verify_nav2.py`, e.g. "verify_slam.py: PASS — final pose
(...) inside Room 2 ...; verify_nav2.py: PASS — navigate_to_pose SUCCEEDED ..." or the FAIL
line(s) with their reason if either failed]. [If either FAILED: state that plainly here as a
known limitation, and do not attempt a fix — deciding what to do about it (revisit the room
layout, the doorway maneuver, or accept a documented degraded-navigation limitation) is
out of scope for this pass, per the design spec Sec 2.3 and Sec 5.]
```

(The `[FILL IN: ...]` bracket above is an instruction to the person/agent executing this step,
not text to leave in the committed README — replace it with the real recorded outcome before
committing.)

- [ ] **Step 7: Commit**

```bash
git add src/walker_nav/README.md
git commit -m "walker_nav: document fov_deg/max_range_m launch args and Kinect-profile re-verification"
```

---

## Out of scope for this plan (already documented in the spec)

Part 2 of the spec — the real Kinect-backed sensing backend (`lidar_backend:=sim|kinect` launch
argument, the `openni2_camera`-vs-`libfreenect` driver choice, `kinect_depth_bridge_node.py`, and
`docs/kinect_bring_up.md`) — is deliberately not implemented here. It has no automated
verification available until a Kinect is physically connected and bring-up confirms which
depth-camera driver path actually works (spec §2.5); the spec document itself is Part 2's design
deliverable for this pass. Plan it separately once hardware bring-up begins, mirroring
`walker_anomaly_detection`'s own "design now, hardware bring-up later" split (spec §2.1, §5).
