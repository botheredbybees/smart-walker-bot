# walker_nav Kinect Sensing Redesign

**Date:** 2026-09-01
**Status:** Approved by user; ready for implementation planning
**Scope:** First design pass for replacing `walker_nav`'s planned real-LiDAR sensing path with a
real Xbox 360 Kinect (v1) depth camera, since no donor vacuum with a spinning LiDAR could be
sourced (only an old Roomba 630 — a bump-navigation 600-series model with no LiDAR to begin
with). Produces (1) a real, buildable, testable update to the existing simulation modeling the
Kinect's narrow field of view, re-verified against the existing SLAM/Nav2 test suite, and (2) a
documented — not yet built or hardware-tested — design for the real Kinect-backed sensing
backend. Does not cover actually wiring up the Kinect (separate future hardware bring-up, same
posture as `walker_anomaly_detection`'s ESP32 work, tracked at
https://github.com/botheredbybees/smart-walker-bot/issues/8 for that package; this redesign's own
hardware bring-up should get its own follow-up issue once implemented — see §5).

## 1. Problem

`walker_nav` (`docs/superpowers/specs/2026-08-30-walker-nav-design.md`,
`docs/superpowers/specs/2026-08-30-walker-nav-nav2-design.md`) was designed and built assuming a
spinning 360° LiDAR feeding `slam_toolbox`/`nav2`, simulated for now by `fake_lidar_node.py`
against a fixed two-room floor plan (`room_map.py`), with the sim/real boundary sitting at the
`/scan` (`sensor_msgs/LaserScan`) topic — the same sim/real split pattern
`walker_motor_driver`'s `MotorBackend` uses.

The user has been unable to source a donor robot vacuum with a real spinning LiDAR (only an old
Roomba 630, a 600-series bump-navigation model with no LiDAR at all), but has an Xbox 360 Kinect
(v1) on hand instead. A Kinect v1 is fundamentally different sensing hardware: a fixed,
forward-facing structured-light depth camera (~57° horizontal field of view, ~0.8-4m usable
range), not a 360° scanning rangefinder. This is the first real departure from `walker_nav`'s
original sensing assumption since that package was built and reviewed.

## 2. Decisions

### 2.1 Real hardware integration deferred; design now, against an updated simulation

Per user preference (mirroring `walker_anomaly_detection`'s own "design now, hardware bring-up
later" posture): this design pass produces a real, implementable simulation update (§2.2-2.3) now,
plus a documented but not-yet-built real-sensor backend design (§2.4-2.6) for a future
implementation pass once the Kinect is physically wired up and its exact working driver path is
confirmed.

### 2.2 Simulation gets a backward-compatible narrow-FOV mode

`fake_lidar_node.py` gains a new `fov_deg` parameter, default `360` — preserving today's exact
full-circle behavior and passing today's existing tests unchanged, no regression. When set
narrower than 360, the node computes a non-wrapping arc's `angle_min_rad`/`angle_increment_rad`
instead of a full circle's.

`room_map.py`'s `scan_room`/`cast_ray` need **zero changes** — confirmed by reading the actual
code during design: `scan_room` already accepts `angle_min_rad`/`angle_increment_rad` directly as
parameters, it was never hardcoded to a full circle. Only the angle-parameter *computation*
(currently inline in `fake_lidar_node.py`'s `__init__`) changes.

That computation is extracted as a new pure function in `room_map.py` — not left inline in
`fake_lidar_node.py`, which (like every `rclpy` node in this project) isn't pytest-tested. This
matches the existing pattern of `yaw_from_quaternion` living in `room_map.py` for exactly this
testability reason (see that function's own docstring).

```python
def fov_to_scan_params(fov_deg, num_beams) -> (angle_min_rad, angle_increment_rad):
```

- **`fov_deg >= 360`**: reproduces the current formula exactly — `angle_min_rad = -pi`,
  `angle_increment_rad = 2*pi / num_beams`. A full circle deliberately does not place a beam at
  both -180° and +180° (the same physical direction), hence dividing by `num_beams`.
- **`fov_deg < 360`**: a non-wrapping arc should have its first and last beams land exactly on
  the FOV's two edges, so `angle_min_rad = -fov_rad / 2`,
  `angle_increment_rad = fov_rad / (num_beams - 1)` (requires `num_beams >= 2`).

### 2.3 Re-verify existing SLAM/Nav2 behavior against a Kinect-realistic sensor profile

Once §2.2 is built, re-run `walker_nav/tools/verify_slam.py` and `verify_nav2.py` — or new copies,
if their existing pass/fail assertions turn out to assume full-circle coverage in a way that
doesn't generalize — against a documented "Kinect profile" parameter set (`fov_deg=57`,
`max_range_m=4.0`, `num_beams` unchanged), launched via `fake_lidar_node`'s existing
parameter-override mechanism (no new launch file needed — same `walker_nav.launch.py`, different
parameter values).

This is a genuine experiment, not an assumed pass. The existing two-room floor plan
(`room_map.py`'s `ROOM_WALLS`: a 4m x 3m room connected to a 2m x 2m room via a 1m doorway) and its
Nav2 tuning (including `nav2_params.yaml`'s `inflation_radius` doorway-cost arithmetic — see that
file's own header comment) were designed and tuned assuming full-circle sensing. A 57°/4m
forward-only sensor may or may not localize and navigate through the doorway reliably using the
existing maneuver. If it doesn't, that finding is valuable to have now rather than at hardware
bring-up — but deciding what to do about it (revisit the room layout, the doorway-approach
maneuver, or accept a documented degraded-navigation limitation) is future work, not part of this
design.

### 2.4 Real backend: compose upstream packages, not custom SLAM/perception code

Matches `walker_nav`'s own established philosophy (`README.md` §5.2; this package's prior design
docs). The real Kinect-backed sensing path is two upstream ROS2 packages composed together,
publishing the same `/scan` (`sensor_msgs/LaserScan`) shape the simulated backend already
produces — so `slam_toolbox`/`nav2` configuration needs **zero changes** for the real-hardware
path. The sim/real boundary sits exactly at `/scan`, identical in spirit to
`walker_motor_driver`'s `MotorBackend` split.

- A depth-camera driver publishing `sensor_msgs/Image` (depth encoding) — see §2.5 for which one.
- `ros-humble-depthimage-to-laserscan` (already installed on this dev workstation, confirmed
  during design) — takes a depth `Image` + `CameraInfo`, produces a `sensor_msgs/LaserScan` by
  slicing a horizontal band of the depth image. Configured via its own launch/params, not
  reimplemented.

### 2.5 Depth-camera driver: try openni2_camera first; libfreenect custom bridge as documented fallback

`ros-humble-openni2-camera` is available via apt (confirmed during design) and is the
"prefer-upstream" ideal — if it detects and drives this Kinect v1, zero custom code is needed for
the depth-image-source half of the pipeline either. **This is not confirmed to work**: OpenNI2's
driver stack historically targets PrimeSense/ASUS-branded hardware by vendor ID, and the Xbox 360
Kinect enumerates under Microsoft's vendor ID — whether stock `openni2_camera` (or the underlying
`libopenni2` driver already installed on this workstation) recognizes it is a genuine open
question, resolvable only once the Kinect is physically connected.

Documented fallback if `openni2_camera` doesn't detect the device: a small custom node inside
`walker_nav` using `libfreenect` (available via apt as the `freenect` metapackage /
`libfreenect-dev`, not yet installed) to grab depth frames directly and publish them as
`sensor_msgs/Image` (+ a static/hardcoded `CameraInfo` using the Kinect v1's known
factory-calibration-ballpark intrinsics — per-unit calibration is out of scope for a first pass,
§5) — still handing off to the same upstream `depthimage_to_laserscan` for the image-to-scan
step. This keeps custom code to the thinnest possible sliver (a depth-frame-to-`Image`-message
adapter, nothing more) even in the fallback case.

Which path is actually used is a bring-up-time decision, not a design-time one — this section
documents both so implementation can proceed immediately once bring-up confirms which driver
works, without needing a second design pass.

### 2.6 Real backend selected via a launch argument, mirroring walker_motor_driver's pattern

`walker_nav.launch.py` gains a `lidar_backend` argument (`sim` default, `kinect` the real option)
— mirroring `walker_motor_driver.launch.py`'s `backend:=sim|real` argument exactly. When `kinect`
is selected, the launch file starts the depth-camera driver + `depthimage_to_laserscan`
(§2.4-2.5) instead of `fake_lidar_node`. `slam_toolbox`'s own launch inclusion is unaffected
either way, since it only ever consumes `/scan` + the `odom`->`base_link` TF, never caring which
backend produced them.

### 2.7 No coupling to walker_safety/walker_motor_driver beyond what already exists

This redesign doesn't change `walker_nav`'s existing relationship to either package (it already
subscribes to `walker_motor_driver`'s `/odom` and TF; it has never coupled to `walker_safety`,
matching every other package's established "no coupling" invariant). Stated explicitly so it
isn't forgotten mid-implementation.

## 3. Package structure changes

**Modified this pass (Part 1 — buildable and testable now):**

```
src/walker_nav/
  walker_nav/
    room_map.py         (+ fov_to_scan_params, pure, new pytest coverage)
    fake_lidar_node.py  (+ fov_deg param, backward-compatible default 360)
  test/
    test_room_map.py    (+ tests for fov_to_scan_params)
  tools/
    verify_slam.py, verify_nav2.py  (re-run against both sim profiles; may need
                                      parameterizing or a documented second invocation
                                      pattern rather than code changes - determined
                                      during implementation planning)
```

**Designed, not built this pass (Part 2 — future implementation, after hardware bring-up
confirms the driver path):**

```
src/walker_nav/
  launch/walker_nav.launch.py  (+ lidar_backend:=sim|kinect argument)
  walker_nav/
    kinect_depth_bridge_node.py  (ONLY if openni2_camera doesn't detect the device -
                                   libfreenect-based depth Image publisher; may not be
                                   needed at all if openni2_camera works)
  docs/
    kinect_bring_up.md  (wiring/power notes - Kinect v1 needs its own power supply,
                          not just USB; mirrors walker_safety/e_stop_wiring.md and
                          walker_anomaly_detection/docs/bring_up.md's pattern)
```

## 4. Testing

**Part 1** is fully testable now, same discipline as every other pure-module change in this
project: `fov_to_scan_params` is pure, unit-tested with pytest (full-circle case
unchanged/regression-tested against existing expectations; narrow-arc case tested for exact edge
placement — beam 0 lands on `-fov_rad/2`, beam `num_beams-1` lands on `+fov_rad/2`).
`fake_lidar_node.py`'s own behavior (not pytest-tested, matching every `rclpy` node in this
project) gets exercised via the existing `verify_slam.py`/`verify_nav2.py` scripts, run against
both the existing full-circle profile (regression check — must still pass exactly as before) and
the new Kinect profile (the actual experiment, §2.3).

**Part 2** has no automated verification available this pass — no hardware exists yet. Mirrors
`walker_anomaly_detection`'s firmware treatment exactly: acceptance criteria for whatever future
implementation task covers Part 2 is "code is written and self-consistent with Part 1's `/scan`
contract," with real verification deferred to hardware bring-up.

## 5. Out of scope

- Actually plugging in and testing the Kinect — separate hardware bring-up work, to be tracked as
  its own follow-up issue once Part 2 is implemented (same pattern as issue #8 for the ESP32).
- A servo/rotating mount for wider composite coverage — explicitly deferred per user's answer
  during design; accepted narrow-FOV as a v1 limitation instead, not merely postponed pending a
  mount design.
- Changing `slam_toolbox`/`nav2` configuration itself — the whole point of the `/scan`-boundary
  design (§2.4) is that neither needs to change.
- Deciding what to do if §2.3's re-verification reveals the existing room/doorway isn't navigable
  with narrow-FOV sensing — that's a finding to act on in a follow-up, not a decision this design
  makes preemptively.
- Per-unit Kinect camera calibration (real `CameraInfo` intrinsics) — a hardcoded
  factory-ballpark `CameraInfo` is accepted as a placeholder for the fallback custom-bridge path
  (§2.5), consistent with this project's "placeholder now, calibrate at bring-up" pattern already
  used for `walker_motor_driver`'s physical constants.
- Continuing to search for a real spinning LiDAR in parallel — user's answer accepted the Kinect
  as the working path for this design, not merely one option among several being pursued at once.
