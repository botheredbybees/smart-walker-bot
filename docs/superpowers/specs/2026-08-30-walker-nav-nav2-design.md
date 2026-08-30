# walker_nav Design (Nav2 pass)

**Date:** 2026-08-30
**Status:** Approved by user; ready for implementation planning
**Scope:** The Nav2 follow-up deferred by the SLAM pass
(`docs/superpowers/specs/2026-08-30-walker-nav-design.md` §2.1): configuring upstream `nav2`
(costmaps, planner/controller servers, `bt_navigator`, recovery behaviors) against the existing
`/cmd_vel`→`/odom`+TF→`/scan`→`slam_toolbox` `/map` pipeline, extending the existing `walker_nav`
package rather than creating a new one.

## 1. Problem

The SLAM pass produced a working mapping pipeline but explicitly deferred autonomous navigation.
Nothing in this project yet commands `/cmd_vel` based on a planned path — the SLAM pass's own
verification script hand-scripts a fixed drive sequence. This design adds the missing piece: a
`navigate_to_pose` goal that Nav2 plans and executes on its own, using the map `slam_toolbox`
already builds live.

## 2. Decisions

### 2.1 Live SLAM map, no AMCL

Nav2 navigates against `slam_toolbox`'s continuously-updating `/map`, not a saved static map with
AMCL localization. This is the standard "SLAM + Nav2" pattern: `slam_toolbox` (already running,
unchanged from the SLAM pass) supplies `map`→`odom`, so Nav2's own localization stack
(`localization_launch.py`/AMCL) isn't used at all — only its navigation stack is. Rejected
alternative: adding map-saving + AMCL for a static-map pipeline, which would reopen a scope
decision the SLAM pass already made deliberately (`docs/superpowers/specs/2026-08-30-walker-nav-design.md`
§6: "Saving the built map to disk... this pass only confirms SLAM produces a map on `/map` during
a live session, not persistence between sessions") — matching that pass's own scope discipline
rather than expanding it here.

### 2.2 Extend the existing walker_nav package

The roadmap has always described `walker_nav` as covering both `slam_toolbox` and `nav2`
together (`docs/superpowers/specs/2026-08-30-phase1-roadmap-design.md` §3: "walker_nav — thin
integration/config layer on top of upstream `slam_toolbox` and `nav2`"). This design adds
`config/nav2_params.yaml` and `launch/nav2.launch.py` alongside the SLAM pass's existing
`config/slam_toolbox_params.yaml` and `launch/walker_nav.launch.py`, rather than creating a
separate package with no precedent elsewhere in this project.

### 2.3 Reuse nav2_bringup's own launch file and behavior tree

`navigation_launch.py` (from the `nav2_bringup` package) brings up the controller server,
planner server, behavior server, `bt_navigator`, velocity smoother, and lifecycle manager as a
unit — this project includes it rather than hand-assembling those nodes individually, the same
"integrate, don't reimplement" principle the SLAM pass applied to `slam_toolbox`. No custom
behavior tree: `bt_navigator`'s stock `navigate_to_pose_w_replanning_and_recovery.xml` handles a
single-goal `navigate_to_pose` action, which is all this pass needs (§2.5).

`nav2_bringup`'s own `localization_launch.py`/AMCL is explicitly NOT included — see §2.1.

### 2.4 nav2_params.yaml is based on nav2_bringup's reference config, not written from scratch

A real difference from the SLAM pass's `slam_toolbox_params.yaml`, worth being explicit about:
`slam_toolbox`'s parameters are individually optional in code, so binding 6 keys and leaving
everything else at defaults was sufficient. Nav2's parameter surface doesn't work the same way —
costmap plugin chains, controller/planner plugin IDs, and the recovery behavior list don't have
a single sensible universal default, so `config/nav2_params.yaml` starts from `nav2_bringup`'s
own reference `nav2_params.yaml` (the file it ships as its documented starting point) rather than
being hand-written. What gets customized on top of that reference, explicitly:

- `robot_radius: 0.15` — a placeholder footprint, consistent with `walker_motor_driver`'s
  placeholder physical constants (`wheel_separation_m=0.2`), flagged for recalibration at
  hardware bring-up like every other placeholder physical parameter in this project
  (`docs/superpowers/specs/2026-08-30-walker-motor-driver-design.md` §2.5).
- `global_frame: map`, `robot_base_frame: base_link`, `odom_topic: /odom` — matching the frame
  names `walker_motor_driver` and the SLAM pass already established.
- Costmap `resolution: 0.05` — matches `slam_toolbox`'s map resolution so costmap cells align
  with the SLAM map cell-for-cell.
- `use_sim_time: false`, set explicitly wherever `nav2_bringup`'s launch/params reference it —
  carrying forward the exact lesson from the SLAM pass's final review, where `slam_toolbox`'s own
  launch file silently defaulted this to `true` and overrode the YAML. Whether
  `navigation_launch.py` has the identical footgun needs to be checked directly against the
  installed file (the same way the SLAM pass's fix did), not assumed away.

Left at `nav2_bringup`'s stock defaults: controller/planner plugin choices (DWB/NavFn or
whatever the reference config ships), recovery behavior parameters, the behavior tree itself.

### 2.5 Verification reuses the SLAM pass's maneuver rather than inventing a new one

A fresh `slam_toolbox` map is mostly "unknown," and Nav2's planner treats unknown space
conservatively — sending a `navigate_to_pose` goal immediately on startup risks an unreliable,
non-deterministic automated check. Instead, the verification script:

1. Replays the SLAM pass's existing drive-through-the-doorway maneuver (the same turn/drive
   sequence `tools/verify_slam.py` already uses) to give the map real coverage of both rooms and
   the doorway — reusing a proven maneuver rather than writing a new one.
2. Sends a `navigate_to_pose` action goal back near the start pose `(0, 0)`, handing control to
   Nav2 for the return trip. Nav2 must genuinely plan and drive back through the doorway — the
   part actually being tested — but over territory the map already covers, which is
   deterministic enough for an automated pass/fail check.
3. Confirms the action reports `SUCCEEDED` and the robot's final `/odom` pose is close to the
   goal (a tolerance, not exact — matching the tolerance-based checks the SLAM and motor-driver
   passes' own verification scripts already use).

Chosen over sending a `navigate_to_pose` goal into unknown space from the very first test
(rejected as less deterministic for this first pass) and over a broader multi-goal/obstacle-
avoidance demo (rejected during brainstorming as more scope than a first Nav2 pass needs — this
mirrors the SLAM pass's own "single maneuver, not a broader demo" scope decision).

## 3. File structure

Additions to the existing `src/walker_nav/` package (SLAM pass files unchanged):

```
src/walker_nav/
  config/
    nav2_params.yaml        (based on nav2_bringup's reference config, customized per §2.4)
  launch/
    nav2.launch.py           (includes nav2_bringup's navigation_launch.py)
  tools/
    verify_nav2.py            (scripted E2E check, §2.5 — not pytest, same rationale
                                verify_slam.py and verify_motor_driver.py already established:
                                needs a live rclpy context and running nodes)
```

No changes to `walker_nav/room_map.py`, `walker_nav/fake_lidar_node.py`,
`config/slam_toolbox_params.yaml`, or `launch/walker_nav.launch.py` — this pass adds to the
package, it doesn't modify the SLAM pass's work.

## 4. Testing

No new pure-Python logic is introduced by this pass (unlike the SLAM pass's `room_map.py`) — it's
entirely upstream `nav2` integration and configuration, so there's no equivalent pytest suite to
add. `tools/verify_nav2.py` is the sole verification, per §2.5, matching the scripted-E2E-check
pattern this project has used for every ROS2-node-level pass so far (`verify_motor_driver.py`,
`verify_slam.py`).

## 5. Out of scope

- Map persistence, AMCL, and static-map localization — explicitly rejected in §2.1, not merely
  deferred; revisit only if the project's needs genuinely change (e.g., a future requirement to
  navigate without re-mapping every session).
- Multi-goal sequences, waypoint following, or an obstacle-avoidance-specific demo — rejected in
  §2.5 as more scope than this first pass needs.
- Any hardware-facing changes — this pass is entirely simulation-based, like every ROS2 package
  in this project so far; real-hardware Nav2 tuning (costmap inflation, controller gains, the
  placeholder `robot_radius`) happens at the hardware bring-up checkpoint
  (`docs/superpowers/specs/2026-08-30-phase1-roadmap-design.md` §3 step 4), same as every other
  placeholder physical parameter.
- Custom behavior trees or custom recovery behaviors — §2.3/§2.4 use `nav2_bringup`'s stock
  versions; a custom BT is future work if the stock one proves insufficient once the platform
  exists.
