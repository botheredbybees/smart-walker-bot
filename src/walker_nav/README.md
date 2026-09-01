# walker_nav (SLAM + Nav2)

Simulated-LiDAR + `slam_toolbox` mapping, and `nav2_bringup`'s navigation
stack configured against that live map, for smart-walker-bot. See
`docs/superpowers/specs/2026-08-30-walker-nav-design.md` (SLAM pass) and
`docs/superpowers/specs/2026-08-30-walker-nav-nav2-design.md` (Nav2 pass)
for the full design (this is a summary).

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
  the room via `room_map.py`. Parameters: `num_beams` (default 360),
  `max_range_m` (default 8.0 — must match
  `config/slam_toolbox_params.yaml`'s `max_laser_range` unless
  deliberately overridden at launch — see "Running the Kinect-realistic
  sensor profile" below), `scan_rate_hz` (default 5.0), `fov_deg`
  (default 360 — full circle; <360 produces a non-wrapping forward arc
  via `room_map.py`'s `fov_to_scan_params`, e.g. 57 for a
  Kinect-realistic profile).
- `config/slam_toolbox_params.yaml` — binds `odom_frame`/`base_frame`/
  `map_frame`/`scan_topic`/`resolution`/`max_laser_range`; everything
  else is `slam_toolbox`'s own default (except `use_sim_time`, which
  the launch file overrides explicitly — see Known limitations).
- `launch/walker_nav.launch.py` — starts `fake_lidar_node` and
  `slam_toolbox`'s `online_async` node together.
- `tools/verify_slam.py` — scripted (not pytest) end-to-end check: with
  `walker_motor_driver` and this package both launched, drives the
  simulated robot through the doorway into Room 2 and confirms the
  final pose is genuinely inside Room 2, `/map` has accumulated known
  cells past what Room 1 alone could produce, and `slam_toolbox`
  publishes `map`→`odom` on `/tf`.
- `config/nav2_params.yaml` — `nav2_bringup`'s own reference config
  with one deliberate change (`robot_radius`); see the file's own
  header comment for the full provenance and what's left inert.
- `launch/nav2.launch.py` — includes `nav2_bringup`'s
  `navigation_launch.py` against `config/nav2_params.yaml`. No AMCL,
  no map-saving — Nav2 navigates against `slam_toolbox`'s live map.
- `tools/verify_nav2.py` — scripted (not pytest) end-to-end check:
  with `walker_motor_driver`, this package's SLAM launch, and this
  package's Nav2 launch all running, primes the map with the same
  maneuver `verify_slam.py` uses, then sends a `navigate_to_pose` goal
  back to the start pose and confirms Nav2 plans and drives the return
  trip on its own — checking both the action's own `SUCCEEDED` status
  and an independent `map`→`base_link` TF lookup, and explicitly
  cancelling the goal on any failure path so a timed-out or rejected
  goal never leaves the robot driving unsupervised.

## Running the pure-module tests

```bash
cd src/walker_nav
python3 -m pytest test/ -v
```

No ROS environment or colcon build needed for these.

## Running the end-to-end SLAM check

Requires both `walker_motor_driver` and `walker_nav` built and this
workspace sourced:

```bash
source /opt/ros/humble/setup.bash
cd src
PYTHONNOUSERSITE=1 colcon build --packages-select walker_motor_driver walker_nav --symlink-install
source install/setup.bash

ros2 launch walker_motor_driver motor_driver.launch.py &
ros2 launch walker_nav walker_nav.launch.py &
sleep 3
python3 walker_nav/tools/verify_slam.py   # or /usr/bin/python3, see its docstring
```

Kill both launched processes when done — `ros2 launch`'s spawned child
nodes (`fake_lidar_node`, `async_slam_toolbox_node`, `motor_driver_node`)
don't die from a plain `kill` on the launch parent; check `ps aux` for
anything still running and kill it explicitly.

## Running the end-to-end Nav2 check

Requires `walker_motor_driver` and `walker_nav` built and this
workspace sourced, and all three launches running together, in this
order:

```bash
source /opt/ros/humble/setup.bash
cd src
PYTHONNOUSERSITE=1 colcon build --packages-select walker_motor_driver walker_nav --symlink-install
source install/setup.bash

ros2 launch walker_motor_driver motor_driver.launch.py &
ros2 launch walker_nav walker_nav.launch.py &
ros2 launch walker_nav nav2.launch.py &
sleep 10
python3 walker_nav/tools/verify_nav2.py   # or /usr/bin/python3, see its docstring
```

The `sleep 10` gives Nav2's seven lifecycle nodes time to reach
`active` before the script sends a goal — a shorter wait can cause the
goal to be rejected outright. Kill all three launched processes when
done, same `ps aux` caveat as the SLAM check above — three launch
trees now, not two, so more child processes to account for.

## Running the Kinect-realistic sensor profile

`walker_nav.launch.py` exposes `fov_deg` (default `360.0`) and
`max_range_m` (default `8.0`) as launch arguments — no separate launch
file. The documented Kinect v1 profile
(`docs/superpowers/specs/2026-09-01-walker-nav-kinect-design.md` Sec 2.3)
is `fov_deg=57`, `max_range_m=4.0`:

```bash
ros2 launch walker_nav walker_nav.launch.py fov_deg:=57.0 max_range_m:=4.0
```

Substitute this for the plain `ros2 launch walker_nav walker_nav.launch.py`
line in either the SLAM or Nav2 end-to-end check above to re-run it
against the narrow-FOV profile.

Note this deliberately leaves `config/slam_toolbox_params.yaml`'s
`max_laser_range` at `8.0` — this branch doesn't touch that file. That's
benign: `slam_toolbox`'s `SetRangeThreshold` (upstream Karto SDK) clips
its range threshold into `[minimum_range, maximum_range]`, where the
maximum comes from the incoming scan's own `range_max` field, not the
YAML value — so the effective range threshold still tracks the scan's
actual `max_range_m=4.0`, no-return readings stay no-returns, and no
phantom obstacle arc gets mapped out at the old 8m limit. The FAIL
below is therefore genuine FOV/coverage behavior, not an artifact of
the `max_range_m`/`max_laser_range` mismatch.

**Result of the first Kinect-profile run (`fov_deg=57`, `max_range_m=4.0`)
against the existing two-room floor plan and Nav2 tuning** (each of
`verify_slam.py` and `verify_nav2.py` run once, n=1):

- `tools/verify_slam.py`: **FAIL** — `/map has only 2414 known cells,
  expected at least 5000 - slam_toolbox may not have mapped past Room 1`.
  The drive maneuver itself still completed (the script's earlier
  final-pose-inside-Room-2 check passed; it only failed the known-cell
  threshold), so this is a *mapping-coverage* shortfall, not a
  navigation failure: a 57° forward arc capped at 4m simply sweeps far
  less of the two rooms in the same scripted maneuver than the
  full-circle 8m profile does (which maps ~6100 cells on the same run).
- `tools/verify_nav2.py`: **PASS** — `navigate_to_pose SUCCEEDED, final
  pose (-0.21, -0.13) [map frame] is 0.25m from the goal`. Nav2 still
  plans and drives the return trip successfully on the sparser map,
  because the return goal is the start pose, which the robot has
  already observed.

This is a **known limitation, recorded as-is** — no fix was attempted.
Deciding what to do about it (revisit the room layout, the doorway
maneuver, `verify_slam.py`'s coverage threshold, or accept a documented
degraded-mapping limitation) is out of scope for this pass, per the
design spec Sec 2.3 and Sec 5.

## The room's origin is the robot's start pose

`room_map.py`'s walls are defined so `(0, 0)` is both the room's local
origin and `walker_motor_driver`'s odometry origin (which always starts
at `(0, 0, 0)` by construction) — so `fake_lidar_node.py` reads `/odom`
directly as room coordinates, no offset math anywhere. `fake_lidar_node.py`
validates incoming poses are finite and warns (without crashing) if the
first `/odom` pose is far from the origin, since nothing else would
detect this assumption silently breaking.

## Known limitations

- **No collision detection, and this matters more under autonomous
  control than it did for a hand-scripted maneuver.** `room_map.py`'s
  walls exist only for ray-casting — `walker_motor_driver`'s simulated
  backend knows nothing about them, so the simulated robot can drive
  straight through a wall. `tools/verify_slam.py`'s drive timing is
  tuned to stay inside the rooms, but once Nav2 owns the path —
  including its recovery behaviors (`spin`, `backup`,
  `drive_on_heading`), which can move the robot along headings nobody
  hand-validated — there's no equivalent check. If the robot ever did
  exit a room, `room_map.py` would start reporting ray-casts against
  the *outside* faces of the walls, `slam_toolbox` would map a
  nonsense world from that, and Nav2 would plan against it — a
  self-reinforcing failure loop that can't occur while a human-written
  script is driving. `tools/verify_nav2.py`'s final pose/tolerance
  check would catch this after the fact (the run would FAIL), but
  nothing prevents it from happening.
- **`tools/verify_slam.py` and `tools/verify_nav2.py` both hardcode
  `walker_motor_driver`'s placeholder physical constants**
  (`wheel_radius_m=0.03`, `max_wheel_speed_rad_s=10.0`) to compute
  their drive timing (identical values in both scripts, not shared
  code — `tools/` isn't set up as an importable package). Those get
  recalibrated at hardware bring-up (see that package's README) — when
  they change, both scripts' timing needs revisiting.
- **`tools/verify_slam.py`'s known-cell count is sampled from a single
  `/map` publish and is genuinely flaky under machine load — a FAIL in
  the 4400s usually just needs a re-run.** `config/slam_toolbox_params.yaml`
  doesn't set `map_update_interval`, so `slam_toolbox`'s own default of
  **10.0s** applies (upstream's `mapper_params_online_async.yaml` would
  have set 5.0, but passing our own `slam_params_file` replaces it).
  `/map` is therefore published only about every 10s, while
  `verify_slam.py`'s whole maneuver is ~13s of wall-clock time and it
  reads whatever `/map` arrived last. In practice the check is decided
  by *one* map sample taken ~10s after `slam_toolbox` starts, so the
  result depends entirely on how far the robot had driven by that
  instant. On an idle machine the robot is already through the doorway
  and the count lands ~6100 (PASS); on a loaded machine, node startup
  and DDS discovery delay the drive by a couple of seconds, the sample
  catches the robot still short of Room 2, and the count lands ~4400-4500
  (FAIL) — reproducibly enough to look like a real regression rather
  than noise. This was measured directly (probing `/map` arrival times
  showed publishes at t≈10s, 20s, 30s, with the 20s sample already at
  ~6400 cells). Both this and `tools/verify_nav2.py` are wall-clock
  timed against a CPU-bound `slam_toolbox`, so on a busy shared desktop
  expect occasional failures and re-run before concluding anything
  broke. A future pass could de-flake this by pinning a shorter
  `map_update_interval` in `config/slam_toolbox_params.yaml` and/or
  having `verify_slam.py` wait for a fresh `/map` after the maneuver
  instead of accepting a stale one.
- `slam_toolbox`'s upstream `online_async_launch.py` defaults
  `use_sim_time` to `true` and applies it after any params file, so
  `walker_nav.launch.py` must (and does) override it explicitly to
  `false` to match every other node in this project. `nav2_bringup`'s
  `navigation_launch.py` already defaults `use_sim_time` to `false` on
  its own (verified against the installed file) — `nav2.launch.py`
  still passes it explicitly, for clarity and so a future
  `nav2_bringup` version bump can't silently flip that default without
  this project noticing.
- **`inflation_radius` (0.55, in `config/nav2_params.yaml`'s costmap
  sections) was left at `nav2_bringup`'s stock TurtleBot3-scale value**
  without being evaluated against this project's 1m-wide doorway —
  see the file's own header comment for the doorway-cost arithmetic
  and why it's safe today but worth revisiting if a second route
  through a similarly narrow gap is ever added.
