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
