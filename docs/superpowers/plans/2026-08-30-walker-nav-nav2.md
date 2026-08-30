# walker_nav (Nav2 pass) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add autonomous navigation to `walker_nav`: `nav2_bringup`'s navigation stack, configured against the live `slam_toolbox` map, verified by sending one `navigate_to_pose` goal that plans and drives a real return trip through the doorway.

**Architecture:** Reuse `nav2_bringup`'s own `navigation_launch.py` (controller server, planner server, behavior server, `bt_navigator`, velocity smoother, lifecycle manager) rather than hand-assembling those nodes — no AMCL, no map-saving, since `slam_toolbox` (already running from the SLAM pass) supplies `map`→`odom` directly. `config/nav2_params.yaml` is `nav2_bringup`'s own reference config with the minimum verified changes this project's placeholder robot geometry needs. Extends the existing `src/walker_nav/` package; no SLAM-pass files change.

**Tech Stack:** ROS2 Humble, `nav2_bringup`/`nav2_msgs` (already installed on this workstation), Python 3 + `rclpy` for the verification script only (no new pure-Python logic — this pass is entirely upstream integration and configuration).

**Spec:** `docs/superpowers/specs/2026-08-30-walker-nav-nav2-design.md` (§2 for decisions, §3 for file structure, §4 for testing).

## Global Constraints

- **`use_sim_time` — verified, not the same footgun as `slam_toolbox`.** Unlike `slam_toolbox`'s `online_async_launch.py` (which defaulted `use_sim_time` to `true` and silently overrode the YAML), `nav2_bringup`'s `navigation_launch.py` already declares its own `use_sim_time` launch argument with `default_value='false'` (verified directly against `/opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py` while writing this plan) — and it applies the override correctly via a `RewrittenYaml` substitution mechanism that rewrites every section's `use_sim_time` value, not just some. No bug to work around here. This plan still passes `use_sim_time: 'false'` explicitly in the include, for clarity and so a future `nav2_bringup` version change can't silently flip the default without this project noticing.
- **`robot_radius` is the only verified-necessary customization to `nav2_params.yaml`.** The stock file's frame names (`map`/`odom`/`base_link`), topic names (`/scan`, `/odom`), and costmap `resolution` (`0.05`, matching `slam_toolbox`'s map resolution) already match this project's established conventions exactly (verified directly against `/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml` while writing this plan — the design spec §2.4 anticipated needing to override these too, but they turned out to already be correct). The one real change: `robot_radius: 0.22` (TurtleBot3-sized default) → `0.15` (this project's placeholder footprint, consistent with `walker_motor_driver`'s placeholder physical constants), in both the `local_costmap` and `global_costmap` sections.
- **`cmd_vel` topology inside `navigation_launch.py` (context, not something to build):** `controller_server` publishes to `/cmd_vel_nav`, which `velocity_smoother` subscribes to and republishes as `/cmd_vel` (smoothed) — this is `navigation_launch.py`'s own internal remapping, already correct for `walker_motor_driver`'s `/cmd_vel` subscription. Nothing in this plan needs to touch it; useful to know when debugging why a command doesn't reach the robot instantly.
- Extends the existing `src/walker_nav/` package — `walker_nav/room_map.py`, `walker_nav/fake_lidar_node.py`, `config/slam_toolbox_params.yaml`, and `launch/walker_nav.launch.py` (all from the SLAM pass) are unchanged by this plan.
- The end-to-end verification reuses the SLAM pass's exact drive maneuver constants (`tools/verify_slam.py`'s `TURN_ANGULAR_Z_RAD_S`/`TURN_DURATION_S`/`DRIVE_LINEAR_X`/`DRIVE_DURATION_S`/`SETTLE_DURATION_S`) to prime `slam_toolbox`'s map with real coverage before sending a `navigate_to_pose` goal — sending a goal on a mostly-unknown map would be a less deterministic first test (spec §2.5).
- `ros2 launch`'s spawned child processes don't die from a plain `kill` on the launch parent (established operational fact from this session's `walker_nav` SLAM pass) — every verification step that launches processes must confirm via `ps aux` (or similar) that nothing is left running afterward.
- This workstation needs `PYTHONNOUSERSITE=1` set for any `colcon build` to succeed (pre-existing, unrelated environment issue, not a defect).

---

## Task 1: nav2_params.yaml + Package Metadata

**Files:**
- Create: `src/walker_nav/config/nav2_params.yaml`
- Modify: `src/walker_nav/package.xml`
- Modify: `src/walker_nav/setup.py`

**Interfaces:**
- Produces: `config/nav2_params.yaml`, installed to the package's share directory, consumed by Task 2's launch file (`params_file` launch argument to `navigation_launch.py`).

- [ ] **Step 1: Create nav2_params.yaml**

Create `src/walker_nav/config/nav2_params.yaml` with this exact content — it is `nav2_bringup`'s own reference config (`/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml`) with `robot_radius: 0.22` changed to `robot_radius: 0.15` in both the `local_costmap` and `global_costmap` sections (search for both occurrences below and confirm they read `0.15`, not `0.22`) — everything else is byte-identical to the stock file:

```yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    wait_for_service_timeout: 1000
    # 'default_nav_through_poses_bt_xml' and 'default_nav_to_pose_bt_xml' are use defaults:
    # nav2_bt_navigator/navigate_to_pose_w_replanning_and_recovery.xml
    # nav2_bt_navigator/navigate_through_poses_w_replanning_and_recovery.xml
    # They can be set here or via a RewrittenYaml remap from a parent launch file to Nav2.
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_compute_path_through_poses_action_bt_node
      - nav2_smooth_path_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_assisted_teleop_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_drive_on_heading_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_globally_updated_goal_condition_bt_node
      - nav2_is_path_valid_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_truncate_path_action_bt_node
      - nav2_truncate_path_local_action_bt_node
      - nav2_goal_updater_node_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_path_expiring_timer_condition
      - nav2_distance_traveled_condition_bt_node
      - nav2_single_trigger_bt_node
      - nav2_goal_updated_controller_bt_node
      - nav2_is_battery_low_condition_bt_node
      - nav2_navigate_through_poses_action_bt_node
      - nav2_navigate_to_pose_action_bt_node
      - nav2_remove_passed_goals_action_bt_node
      - nav2_planner_selector_bt_node
      - nav2_controller_selector_bt_node
      - nav2_goal_checker_selector_bt_node
      - nav2_controller_cancel_bt_node
      - nav2_path_longer_on_approach_bt_node
      - nav2_wait_cancel_bt_node
      - nav2_spin_cancel_bt_node
      - nav2_back_up_cancel_bt_node
      - nav2_assisted_teleop_cancel_bt_node
      - nav2_drive_on_heading_cancel_bt_node
      - nav2_is_battery_charging_condition_bt_node

bt_navigator_navigate_through_poses_rclcpp_node:
  ros__parameters:
    use_sim_time: True

bt_navigator_navigate_to_pose_rclcpp_node:
  ros__parameters:
    use_sim_time: True

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"] # "precise_goal_checker"
    controller_plugins: ["FollowPath"]

    # Progress checker parameters
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    # Goal checker parameters
    #precise_goal_checker:
    #  plugin: "nav2_controller::SimpleGoalChecker"
    #  xy_goal_tolerance: 0.25
    #  yaw_goal_tolerance: 0.25
    #  stateful: True
    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
    # DWB parameters
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.26
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.26
      min_speed_theta: 0.0
      # Add high threshold velocity for turtlebot 3 issue.
      # https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/75
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2
      vx_samples: 20
      vy_samples: 5
      vtheta_samples: 20
      sim_time: 1.7
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.25
      trans_stopped_velocity: 0.25
      short_circuit_trajectory_evaluation: True
      stateful: True
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      PathAlign.forward_point_distance: 0.1
      GoalAlign.scale: 24.0
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.15
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      always_send_full_costmap: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.15
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True

map_server:
  ros__parameters:
    use_sim_time: True
    # Overridden in launch by the "map" launch configuration or provided default value.
    # To use in yaml, remove the default "map" value in the tb3_simulation_launch.py file & provide full path to map below.
    yaml_filename: ""

map_saver:
  ros__parameters:
    use_sim_time: True
    save_map_timeout: 5.0
    free_thresh_default: 0.25
    occupied_thresh_default: 0.65
    map_subscribe_transient_local: True

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

smoother_server:
  ros__parameters:
    use_sim_time: True
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: True

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "drive_on_heading", "assisted_teleop", "wait"]
    spin:
      plugin: "nav2_behaviors/Spin"
    backup:
      plugin: "nav2_behaviors/BackUp"
    drive_on_heading:
      plugin: "nav2_behaviors/DriveOnHeading"
    wait:
      plugin: "nav2_behaviors/Wait"
    assisted_teleop:
      plugin: "nav2_behaviors/AssistedTeleop"
    global_frame: odom
    robot_base_frame: base_link
    transform_tolerance: 0.1
    use_sim_time: true
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2

robot_state_publisher:
  ros__parameters:
    use_sim_time: True

waypoint_follower:
  ros__parameters:
    use_sim_time: True
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: True
      waypoint_pause_duration: 200

velocity_smoother:
  ros__parameters:
    use_sim_time: True
    smoothing_frequency: 20.0
    scale_velocities: False
    feedback: "OPEN_LOOP"
    max_velocity: [0.26, 0.0, 1.0]
    min_velocity: [-0.26, 0.0, -1.0]
    max_accel: [2.5, 0.0, 3.2]
    max_decel: [-2.5, 0.0, -3.2]
    odom_topic: "odom"
    odom_duration: 0.1
    deadband_velocity: [0.0, 0.0, 0.0]
    velocity_timeout: 1.0
```

- [ ] **Step 2: Update package.xml**

Modify `src/walker_nav/package.xml`. Change the `<description>` line from:
```xml
  <description>SLAM integration layer for smart-walker-bot: a simulated LiDAR (ray-cast against a fixed room) feeding slam_toolbox, until real hardware exists.</description>
```
to:
```xml
  <description>SLAM + Nav2 integration layer for smart-walker-bot: a simulated LiDAR feeding slam_toolbox, and nav2_bringup's navigation stack configured against the live map, until real hardware exists.</description>
```

Add these two lines after the existing `<exec_depend>slam_toolbox</exec_depend>` line:
```xml
  <depend>nav2_msgs</depend>
  <exec_depend>nav2_bringup</exec_depend>
```

- [ ] **Step 3: Update setup.py's data_files**

Modify `src/walker_nav/setup.py`. Replace the `data_files=[...]` block with:

```python
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/walker_nav.launch.py',
            'launch/nav2.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/slam_toolbox_params.yaml',
            'config/nav2_params.yaml',
        ]),
    ],
```

Note: `launch/nav2.launch.py` doesn't exist until Task 2 — if this task's own build-verification (Step 4) fails because it's missing, create a placeholder first:

```python
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([])
```

Task 2 overwrites this placeholder with the real file.

- [ ] **Step 4: Verify the package builds**

```bash
source /opt/ros/humble/setup.bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src
PYTHONNOUSERSITE=1 colcon build --packages-select walker_nav --symlink-install
```

Expected: `Summary: 1 package finished`. If it fails on the missing `launch/nav2.launch.py`, create the placeholder from Step 3's note and retry.

- [ ] **Step 5: Commit**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot
git add src/walker_nav/config/nav2_params.yaml src/walker_nav/package.xml src/walker_nav/setup.py
git commit -m "$(cat <<'EOF'
Add walker_nav nav2_params.yaml and package metadata for Nav2 pass

nav2_params.yaml is nav2_bringup's own reference config with
robot_radius changed from the TurtleBot3 default (0.22) to this
project's placeholder footprint (0.15) in both costmap sections -
verified against the installed file that frame names, topic names, and
costmap resolution already match this project's conventions, so no
other changes were needed.
EOF
)"
```

---

## Task 2: nav2.launch.py + Stack Activation Smoke Test

**Files:**
- Create: `src/walker_nav/launch/nav2.launch.py` (overwrites Task 1's placeholder, if one was created)

**Interfaces:**
- Consumes: `config/nav2_params.yaml` (Task 1).
- Produces: a running Nav2 navigation stack exposing the `navigate_to_pose` action, consumed by Task 3's verification script.

- [ ] **Step 1: Write the launch file**

Create `src/walker_nav/launch/nav2.launch.py`:

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    walker_nav_share = get_package_share_directory('walker_nav')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    params_file = os.path.join(walker_nav_share, 'config', 'nav2_params.yaml')

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': params_file,
            # navigation_launch.py's own use_sim_time launch argument
            # already defaults to 'false' (verified against the
            # installed nav2_bringup package) - unlike slam_toolbox's
            # online_async_launch.py, no override is strictly required,
            # but it's passed explicitly here for clarity and so a
            # future nav2_bringup version change can't silently flip it.
            'use_sim_time': 'false',
        }.items(),
    )

    return LaunchDescription([navigation_launch])
```

- [ ] **Step 2: Build the package**

```bash
source /opt/ros/humble/setup.bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src
PYTHONNOUSERSITE=1 colcon build --packages-select walker_nav --symlink-install
source install/setup.bash
```

Expected: `Summary: 1 package finished`.

- [ ] **Step 3: Smoke-test the Nav2 stack activates cleanly**

This launches all three packages together (`walker_motor_driver` provides `/odom`, `walker_nav`'s existing SLAM launch provides `/scan`+`/map`, this new launch is the thing under test) and confirms the navigation stack's lifecycle nodes reach the `active` state and the `navigate_to_pose` action server is available — it does NOT send a navigation goal yet, that's Task 3.

```bash
source /opt/ros/humble/setup.bash
source /home/peter_sha/sourcecode/smart-walker-bot/src/install/setup.bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src

ros2 launch walker_motor_driver motor_driver.launch.py > /tmp/motor_driver_smoke.log 2>&1 &
MOTOR_PID=$!
ros2 launch walker_nav walker_nav.launch.py > /tmp/walker_nav_smoke.log 2>&1 &
NAV_SLAM_PID=$!
ros2 launch walker_nav nav2.launch.py > /tmp/nav2_smoke.log 2>&1 &
NAV2_PID=$!
sleep 10

if kill -0 $NAV2_PID 2>/dev/null; then
    echo "nav2 launch is still running after 10s (expected)"
else
    echo "nav2 launch exited early (unexpected) - check /tmp/nav2_smoke.log"
fi

echo "--- checking navigate_to_pose action server ---"
ros2 action list | grep navigate_to_pose && echo "PRESENT" || echo "MISSING"

echo "--- checking lifecycle node states (expect 'active' on each) ---"
for node in controller_server planner_server behavior_server bt_navigator smoother_server waypoint_follower velocity_smoother; do
    echo -n "$node: "
    ros2 lifecycle get /$node 2>&1
done

echo "--- process cleanup ---"
kill $NAV2_PID $NAV_SLAM_PID $MOTOR_PID 2>/dev/null
sleep 1
ps aux | grep -E 'ros2|controller_server|planner_server|behavior_server|bt_navigator|smoother_server|waypoint_follower|velocity_smoother|lifecycle_manager|fake_lidar_node|async_slam_toolbox_node|motor_driver_node' | grep -v grep

cat /tmp/nav2_smoke.log
```

Expected: `navigate_to_pose` reports `PRESENT`; all seven lifecycle nodes report `active`; the log shows no Python traceback (a handful of early "waiting for transform"-style warnings before the SLAM/motor-driver launches finish starting up are fine, as long as they stop and the nodes reach `active`). The final `ps aux` line should show nothing related to this test — if any process is still listed, `kill` it explicitly (by PID, from the `ps aux` output) and re-check before finishing, per the `ros2 launch` child-process cleanup requirement in Global Constraints.

- [ ] **Step 4: Commit**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot
git add src/walker_nav/launch/nav2.launch.py
git commit -m "$(cat <<'EOF'
Add walker_nav nav2.launch.py, smoke-tested for clean stack activation

Includes nav2_bringup's navigation_launch.py against
config/nav2_params.yaml. Smoke-tested with walker_motor_driver and
walker_nav's SLAM launch also running: all seven lifecycle nodes reach
'active' and the navigate_to_pose action server is available. Sending
an actual navigation goal is Task 3.
EOF
)"
```

---

## Task 3: End-to-End Nav2 Verification

**Files:**
- Create: `src/walker_nav/tools/verify_nav2.py`

**Interfaces:**
- Consumes: `walker_motor_driver`'s `/cmd_vel` (in) / `/odom` (out), `walker_nav`'s `/scan`/`/map` (from the SLAM launch), and the `navigate_to_pose` action server (from Task 2's Nav2 launch) — all via the ROS graph, not Python imports.
- Produces: nothing consumed by a later task — this is the last task in this plan.

- [ ] **Step 1: Write the end-to-end verification script**

Create `src/walker_nav/tools/verify_nav2.py`:

```python
#!/usr/bin/env python3
"""Scripted end-to-end check for walker_nav's Nav2 pass - not a pytest
test. Assumes walker_motor_driver's node (backend:=sim), walker_nav's
SLAM launch (fake_lidar_node + slam_toolbox), and walker_nav's Nav2
launch are all already running (see this package's README for the
launch commands).

Reuses the SLAM pass's exact drive-through-the-doorway maneuver
(tools/verify_slam.py's constants) to give slam_toolbox's map real
coverage of both rooms before handing control to Nav2 - sending a
navigate_to_pose goal immediately on a mostly-unknown map would be a
less deterministic first test. Then sends a navigate_to_pose action
goal back near the start pose (0, 0), letting Nav2 plan and drive the
return trip through the doorway on its own, and confirms the action
reports SUCCEEDED with the final /odom pose close to the goal.

Usage: python3 tools/verify_nav2.py
(On this project's dev workstation, use /usr/bin/python3 if plain
python3 can't import rclpy - see walker_motor_driver's
verify_motor_driver.py docstring for why.)

Exits 0 and prints PASS on success, exits 1 and prints FAIL otherwise.
"""
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node

# Same maneuver walker_nav's SLAM pass verification uses
# (tools/verify_slam.py) to give the map real coverage before Nav2
# needs to plan through it.
TURN_ANGULAR_Z_RAD_S = math.pi / 2
TURN_DURATION_S = 1.0
DRIVE_LINEAR_X = 1.0
DRIVE_DURATION_S = 9.0
SETTLE_DURATION_S = 3.0
REPUBLISH_INTERVAL_S = 0.1
FIRST_ODOM_TIMEOUT_S = 5.0

GOAL_X_M = 0.0
GOAL_Y_M = 0.0
GOAL_XY_TOLERANCE_M = 0.5
NAV2_ACTION_TIMEOUT_S = 60.0


class VerifyNav2Node(Node):
    def __init__(self):
        super().__init__('walker_nav_verify_nav2')
        self.latest_odom = None
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _on_odom(self, msg):
        self.latest_odom = msg

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


def _send_nav_goal_and_wait(node):
    """Send a navigate_to_pose goal and block (via spinning) until it
    completes. Returns the GoalStatus constant, or None on a timeout,
    rejection, or unavailable action server."""
    if not node.nav_to_pose_client.wait_for_server(timeout_sec=10.0):
        return None

    goal_msg = NavigateToPose.Goal()
    goal_msg.pose = PoseStamped()
    goal_msg.pose.header.frame_id = 'map'
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = GOAL_X_M
    goal_msg.pose.pose.position.y = GOAL_Y_M
    goal_msg.pose.pose.orientation.w = 1.0

    send_goal_future = node.nav_to_pose_client.send_goal_async(goal_msg)
    deadline = time.monotonic() + NAV2_ACTION_TIMEOUT_S
    while not send_goal_future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)

    if not send_goal_future.done():
        return None
    goal_handle = send_goal_future.result()
    if not goal_handle.accepted:
        return None

    result_future = goal_handle.get_result_async()
    while not result_future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)

    if not result_future.done():
        return None
    return result_future.result().status


def main():
    rclpy.init()
    node = VerifyNav2Node()

    try:
        # Wait for the first /odom before priming - a publish immediately
        # after node creation can race DDS discovery of walker_motor_driver's
        # subscriber and be silently dropped (same guard verify_motor_driver.py
        # and verify_slam.py already use).
        deadline = time.monotonic() + FIRST_ODOM_TIMEOUT_S
        while node.latest_odom is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)

        if node.latest_odom is None:
            print('FAIL: no /odom message received within '
                  f'{FIRST_ODOM_TIMEOUT_S}s - is walker_motor_driver running?')
            return 1

        # Prime the map: drive through the doorway into Room 2 (same
        # maneuver as walker_nav's SLAM pass verification), so
        # slam_toolbox's map covers the path Nav2 will need to plan
        # the return trip through.
        _drive_phase(node, linear_x=0.0, angular_z=TURN_ANGULAR_Z_RAD_S, duration_s=TURN_DURATION_S)
        _drive_phase(node, linear_x=DRIVE_LINEAR_X, angular_z=0.0, duration_s=DRIVE_DURATION_S)
        _drive_phase(node, linear_x=0.0, angular_z=0.0, duration_s=SETTLE_DURATION_S)

        # Hand off to Nav2 for the return trip - don't publish /cmd_vel
        # ourselves from here on, or we'd fight with Nav2's own output.
        status = _send_nav_goal_and_wait(node)

        if status is None:
            print('FAIL: navigate_to_pose action did not complete within '
                  f'{NAV2_ACTION_TIMEOUT_S}s (or the action server/goal was '
                  'rejected) - is the Nav2 stack running and active?')
            return 1

        if status != GoalStatus.STATUS_SUCCEEDED:
            print(f'FAIL: navigate_to_pose finished with status {status}, '
                  f'expected STATUS_SUCCEEDED ({GoalStatus.STATUS_SUCCEEDED})')
            return 1

        final_x = node.latest_odom.pose.pose.position.x
        final_y = node.latest_odom.pose.pose.position.y
        distance_from_goal_m = math.hypot(final_x - GOAL_X_M, final_y - GOAL_Y_M)
        if distance_from_goal_m > GOAL_XY_TOLERANCE_M:
            print(f'FAIL: navigate_to_pose reported SUCCEEDED but final pose '
                  f'({final_x:.2f}, {final_y:.2f}) is {distance_from_goal_m:.2f}m '
                  f'from the goal ({GOAL_X_M}, {GOAL_Y_M}), expected within '
                  f'{GOAL_XY_TOLERANCE_M}m')
            return 1

        print(f'PASS: navigate_to_pose SUCCEEDED, final pose ({final_x:.2f}, '
              f'{final_y:.2f}) is {distance_from_goal_m:.2f}m from the goal')
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
python3 -m py_compile /home/peter_sha/sourcecode/smart-walker-bot/src/walker_nav/tools/verify_nav2.py
```

Expected: no output, exit code 0.

- [ ] **Step 3: Run the full end-to-end verification**

All three packages must already be built (`walker_safety` needs no build; `walker_motor_driver` and `walker_nav` need `colcon build --packages-select walker_motor_driver walker_nav --symlink-install` with `PYTHONNOUSERSITE=1` — rebuild if `src/install/` was cleaned since a prior session).

```bash
source /opt/ros/humble/setup.bash
source /home/peter_sha/sourcecode/smart-walker-bot/src/install/setup.bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src

ros2 launch walker_motor_driver motor_driver.launch.py > /tmp/motor_driver_node.log 2>&1 &
MOTOR_PID=$!
ros2 launch walker_nav walker_nav.launch.py > /tmp/walker_nav.log 2>&1 &
NAV_SLAM_PID=$!
ros2 launch walker_nav nav2.launch.py > /tmp/nav2.log 2>&1 &
NAV2_PID=$!
sleep 10

python3 walker_nav/tools/verify_nav2.py
VERIFY_EXIT=$?

echo "verify_nav2.py exit code: $VERIFY_EXIT"

kill $NAV2_PID $NAV_SLAM_PID $MOTOR_PID 2>/dev/null
sleep 1
echo "--- checking for stray processes ---"
ps aux | grep -E 'ros2|controller_server|planner_server|behavior_server|bt_navigator|smoother_server|waypoint_follower|velocity_smoother|lifecycle_manager|fake_lidar_node|async_slam_toolbox_node|motor_driver_node' | grep -v grep

echo "--- motor_driver_node.log ---"
cat /tmp/motor_driver_node.log
echo "--- walker_nav.log ---"
cat /tmp/walker_nav.log
echo "--- nav2.log ---"
cat /tmp/nav2.log
```

Expected: `verify_nav2.py` prints `PASS: navigate_to_pose SUCCEEDED, final pose (...) is ...m from the goal` and `VERIFY_EXIT` is `0`. The script itself takes roughly 15-25 seconds for the priming maneuver plus however long Nav2 takes to plan and drive the return trip (comfortably under the script's own 60-second action timeout) — this is expected, not a hang. The `ps aux` check after cleanup should show nothing related to this test; if anything is still listed, kill it explicitly by PID and re-check. If the verification fails, use the three log files to diagnose: `motor_driver_node.log` for anything wrong with the sim/odometry side, `walker_nav.log` for the fake LiDAR or `slam_toolbox` side, `nav2.log` for the navigation stack itself (a `navigate_to_pose` goal rejected outright often means a lifecycle node isn't `active` yet — check whether `sleep 10` was long enough, or increase it).

- [ ] **Step 4: Commit**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot
git add src/walker_nav/tools/verify_nav2.py
git commit -m "$(cat <<'EOF'
Add walker_nav end-to-end Nav2 verification

Primes slam_toolbox's map with the SLAM pass's proven drive maneuver,
then hands off to Nav2 for a navigate_to_pose goal back near the start
pose - confirms the action reports SUCCEEDED and the robot's final
odometry pose is actually close to the goal, not just that the action
server exists. No physical hardware needed.
EOF
)"
```

---

## Self-Review Notes

- **Spec coverage:** §2.1 (live SLAM map, no AMCL) — `nav2.launch.py` includes only `navigation_launch.py`, never `localization_launch.py`/AMCL; `map_server`/`map_saver`/`amcl` sections remain in `nav2_params.yaml` (they're part of `nav2_bringup`'s reference file) but are inert since nothing in `navigation_launch.py`'s `lifecycle_nodes` list launches those nodes — confirmed by reading the actual installed launch file while writing this plan. §2.2 (extend existing package) — no new package created, no SLAM-pass files touched. §2.3 (reuse `nav2_bringup`'s launch/BT) — Task 2, no custom BT written. §2.4 (config based on reference, minimal customization) — Task 1, with the actual verified diff (just `robot_radius` in two places) documented in Global Constraints rather than the spec's more speculative pre-verification enumeration. §2.5 (verification reuses the SLAM maneuver, checks `SUCCEEDED` + pose tolerance) — Task 3. §3 (file structure) — matches exactly (`config/nav2_params.yaml`, `launch/nav2.launch.py`, `tools/verify_nav2.py`). §4 (no new pytest suite) — no pure-Python logic introduced by this plan, matches. §5 (out of scope: AMCL, multi-goal, custom BT, hardware tuning) — none of these appear anywhere in this plan.
- **Placeholder scan:** no TBD/TODO in any step. Task 1's placeholder `nav2.launch.py` (only needed if Task 1's own build hits the missing-file case) is real, minimal, valid content, overwritten by Task 2 regardless of whether it was needed.
- **Type/name consistency:** `verify_nav2.py`'s drive-phase constants and `_drive_phase` function match `verify_slam.py`'s naming and behavior exactly (same constant names, same values, same republish-loop structure), as the design intended by "reusing" that maneuver. `GOAL_X_M`/`GOAL_Y_M` (0, 0) is the robot's actual start pose, matching `room_map.py`'s room-origin convention established in the SLAM pass. Topic/frame names (`/cmd_vel`, `/odom`, `map`, `base_link`) are used identically to how `walker_motor_driver` and the SLAM pass already established them.
