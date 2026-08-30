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
