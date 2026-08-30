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
  `config/slam_toolbox_params.yaml`'s `max_laser_range`), `scan_rate_hz`
  (default 5.0).
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
