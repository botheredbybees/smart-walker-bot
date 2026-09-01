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

```bash
cd src/walker_gait_metrics
python3 -m pytest test/ -v
```

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
