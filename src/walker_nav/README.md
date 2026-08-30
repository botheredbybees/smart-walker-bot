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

## The room's origin is the robot's start pose

`room_map.py`'s walls are defined so `(0, 0)` is both the room's local
origin and `walker_motor_driver`'s odometry origin (which always starts
at `(0, 0, 0)` by construction) — so `fake_lidar_node.py` reads `/odom`
directly as room coordinates, no offset math anywhere. `fake_lidar_node.py`
validates incoming poses are finite and warns (without crashing) if the
first `/odom` pose is far from the origin, since nothing else would
detect this assumption silently breaking.

## Known limitations

- **No collision detection.** `room_map.py`'s walls exist only for
  ray-casting — `walker_motor_driver`'s simulated backend knows nothing
  about them, so the simulated robot can drive straight through a wall.
  `tools/verify_slam.py`'s drive timing is tuned to stay inside the
  rooms; nothing enforces that automatically.
- **`tools/verify_slam.py` hardcodes `walker_motor_driver`'s placeholder
  physical constants** (`wheel_radius_m=0.03`, `max_wheel_speed_rad_s=10.0`)
  to compute its drive timing. Those get recalibrated at hardware
  bring-up (see that package's README) — when they change, this script's
  timing needs revisiting too.
- `slam_toolbox`'s upstream `online_async_launch.py` defaults
  `use_sim_time` to `true` and applies it after any params file, so
  `walker_nav.launch.py` must (and does) override it explicitly to
  `false` to match every other node in this project.
