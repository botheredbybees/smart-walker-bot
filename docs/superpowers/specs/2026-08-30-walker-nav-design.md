# walker_nav Design (SLAM pass)

**Date:** 2026-08-30
**Status:** Approved by user; ready for implementation planning
**Scope:** Step 3 of the revised Phase 1 roadmap
(`docs/superpowers/specs/2026-08-30-phase1-roadmap-design.md` §3): a thin integration/config
layer over upstream `slam_toolbox`, configured against `walker_motor_driver`'s
`/cmd_vel`→`/odom`+TF interface, plus a lightweight simulated LiDAR to exercise it before real
hardware exists. Does **not** cover Nav2 (path planning/goal navigation) — that's deliberately
split into a separate follow-up spec/plan, decided during brainstorming because Nav2's full
stack (costmaps, planner/controller servers, behavior tree, recovery behaviors) is substantial
scope on its own, and SLAM (mapping) and Nav2 (navigating) are separable capabilities.

## 1. Problem

The roadmap step 3 description ("thin integration/config layer on top of upstream
`slam_toolbox` and `nav2`, not a reimplementation of SLAM or path planning") is a roadmap-level
sketch, not a design — it doesn't say what `slam_toolbox` needs to actually run, and in
particular doesn't address that no `/scan` data source exists at all yet: no real LiDAR, and no
simulator produces one. `walker_motor_driver` solved the equivalent problem for `/cmd_vel`/`/odom`
with `SimMotorBackend`; `walker_nav` needs an analogous simulated LiDAR before `slam_toolbox` has
anything to consume.

## 2. Decisions

### 2.1 SLAM only in this pass; Nav2 is a separate follow-up

Decided during brainstorming: bundling both SLAM and the full Nav2 stack (costmap config,
planner/controller servers, behavior tree navigator, recovery behaviors) into one plan was
judged too large for a single implementation plan, and the two capabilities (build a map;
navigate using a map) are naturally separable. This spec produces a working map-building
pipeline; a second spec configures Nav2 against the map this one produces.

### 2.2 A simple fixed room, ray-cast from the robot's real tracked pose — not Gazebo, not random data

Two rejected alternatives, and why:
- **Random/synthetic scan data, not tied to real geometry or pose** — rejected because
  `slam_toolbox`'s resulting map would be meaningless noise; this would confirm topics/TF are
  wired correctly but never demonstrate SLAM actually working, which is the whole point of this
  step.
- **A full simulator (Gazebo)** — rejected for the same reason the roadmap design rejected it for
  `walker_motor_driver` (`docs/superpowers/specs/2026-08-30-phase1-roadmap-design.md` §2.4):
  heavier setup and maintenance cost than this project's "lightweight, not physics-realistic"
  simulation approach calls for.

Instead: a small hardcoded 2D room (two connected rectangular rooms via a 1m doorway — see §3.1)
that a fake LiDAR node ray-casts against, using the robot's actual pose from
`walker_motor_driver`'s real, live-tracked odometry (not a separately-simulated position). This
is cheap to build (a pure ray-casting function, no level-editor or asset pipeline) while still
giving `slam_toolbox` genuine, pose-consistent geometry to build a real map from.

### 2.3 The room's coordinate origin is the robot's start pose — no offset math anywhere

The room is defined so that the robot's start position `(0, 0, 0)` is also the room's local
origin. `walker_motor_driver`'s `OdometryTracker` always starts at `(0, 0, 0)` in the `odom`
frame by construction (`docs/superpowers/specs/2026-08-30-walker-motor-driver-design.md`) — so
the two origins coincide, and the fake LiDAR node can read `/odom`'s `(x, y, theta)` directly as
room coordinates with no translation/rotation offset to apply or get wrong.

### 2.4 Fake LiDAR node: subscribes /odom, publishes /scan, frame_id='base_link'

A thin `rclpy` node (`fake_lidar_node.py`) wraps the pure ray-casting module (`room_map.py`):
subscribes `nav_msgs/Odometry` on `/odom`, decodes `(x, y, theta)` (yaw from the orientation
quaternion — the inverse of `walker_motor_driver`'s `yaw_to_quaternion`, valid for the
Z-axis-only rotations this project's ground robots ever produce), calls `room_map.py` for each
beam, and publishes `sensor_msgs/LaserScan` on `/scan`. The scan's `frame_id` is `'base_link'`
directly — the sim treats the LiDAR as exactly co-located with the robot (zero offset), so no
extra static-transform node is needed. Mirrors the pure-core/thin-ROS-wrapper split
`watchdog_logic.py`/`main.py` and `diff_drive_kinematics.py`/`motor_driver_node.py` already
established in this project.

### 2.5 slam_toolbox config: bind the frames/topics that matter, defaults for the rest

A params YAML sets `odom_frame: odom`, `base_frame: base_link`, `map_frame: map`,
`scan_topic: /scan`, map resolution, and `max_laser_range` (matching the fake LiDAR's configured
range) — everything else stays at `slam_toolbox`'s own defaults. This is what "thin
integration/config layer... not a reimplementation" means concretely: bind the interface,
don't touch SLAM internals.

## 3. Room map and ray-casting

### 3.1 Room geometry

Two connected rooms as line-segment walls, robot starting at the origin, centered so no offset
math is needed (§2.3):

```
Room 1: x in [-2.0, 2.0], y in [-1.5, 1.5]  (4m x 3m, robot starts at (0, 0), facing +x)
Doorway: gap in Room 1's y=1.5 wall, from x=-0.5 to x=0.5 (1m wide)
Room 2: x in [-1.0, 1.0], y in [1.5, 3.5]   (2m x 2m, entered through the doorway)
```

### 3.2 Ray-casting

Pure function: given a pose `(x, y, theta)` and a beam angle, find the nearest wall-segment
intersection using standard ray/line-segment intersection (ray parameter `t >= 0`, segment
parameter `0 <= u <= 1`), returning `max_range_m` if nothing is hit within range. A second pure
function (`scan_room`) calls this once per beam across a full `sensor_msgs/LaserScan`-style
angle range (`angle_min`, `angle_increment`, beam count), so its output maps directly onto the
message fields the ROS node needs to fill in.

## 4. File structure

New `ament_python` package, matching `walker_motor_driver`'s established pattern:

```
src/walker_nav/
  package.xml, setup.py, setup.cfg, resource/walker_nav
  walker_nav/
    __init__.py
    room_map.py           (pure: wall segments + ray-casting, §3)
    fake_lidar_node.py    (rclpy node: /odom in, /scan out, §2.4)
  config/
    slam_toolbox_params.yaml   (§2.5)
  launch/
    walker_nav.launch.py       (fake_lidar_node + slam_toolbox's online_async node)
  test/
    conftest.py
    test_room_map.py
  tools/
    verify_slam.py             (scripted E2E check, §5)
```

## 5. Testing

`room_map.py` is pure Python — no `rclpy` imports — so it's fully unit-tested with pytest, the
same pattern `diff_drive_kinematics.py` and `watchdog_logic.py` used. `fake_lidar_node.py` and
the `slam_toolbox` integration, like `walker_motor_driver`'s node, run for real in this
environment (full ROS2 Humble install, no missing-hardware problem), so they get a scripted
end-to-end check rather than a unit test.

`tools/verify_slam.py` launches both `walker_motor_driver` (for `/odom`) and `walker_nav` (for
`/scan` + `slam_toolbox`) together, drives the simulated robot around — forward, turn, through
the doorway into Room 2 — then checks that `/map` (`nav_msgs/OccupancyGrid`) actually contains a
meaningful number of known (free/occupied, not just "unknown") cells, and that `map`→`odom` is
being published on `/tf`. This confirms SLAM is genuinely running and building a map, not just
that the topics are wired up — the same bar `walker_motor_driver`'s final review held its own
verification script to.

## 6. Out of scope

- Nav2 (path planning, costmaps, behavior tree, recovery behaviors) — explicitly deferred to a
  separate follow-up spec/plan (§2.1).
- Saving the built map to disk (`slam_toolbox`'s map-save service) — this pass only confirms SLAM
  produces a map on `/map` during a live session, not persistence between sessions.
- Any hardware-facing LiDAR driver or `GpioMotorBackend`-equivalent — `room_map.py`'s simulated
  environment is the only data source in this pass; a real LiDAR integration is future work at
  the hardware bring-up checkpoint, analogous to `walker_motor_driver`'s deferred
  `GpioMotorBackend`.
