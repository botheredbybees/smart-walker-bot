# walker_gait_metrics Design

**Date:** 2026-09-01
**Status:** Approved by user; ready for implementation planning
**Scope:** First design pass for wellness/gait metrics: step count and step length, computed
from the existing IMU stream (`walker_anomaly_detection`) and wheel odometry
(`walker_motor_driver`), published on a new topic and surfaced on `walker_companion_app`'s
dashboard. Two related ideas from the same brainstorming session are explicitly deferred:
grip strength (needs handle hardware that doesn't exist yet — tracked in
`docs/ideas-backlog.md`) and Kinect-based gait/fitness analysis (its own, larger, separately
speced idea). Wiring these metrics into `walker_llm_bridge` so they're answerable
conversationally — the direction `CLAUDE.md`'s wellness-feature design principle points at —
is a named follow-up, not built this pass.

## 1. Problem

`docs/ideas-backlog.md` collected several wellness-metric ideas from research the user added to
their own knowledge base (Kinect-based gait analysis, grip strength, step count, step length).
Step count and step length are the two immediately buildable ones: unlike grip strength (needs a
handle sensor that doesn't exist) or Kinect gait analysis (needs the nav Kinect re-purposed or a
second, differently-mounted sensor — a separate open question), both step metrics can be derived
entirely from hardware this project already has or has already speced — `walker_anomaly_detection`'s
9-axis IMU and `walker_motor_driver`'s wheel odometry — with no new hardware and no changes to
either of those packages' own detection/control logic.

This is also the first feature built against `CLAUDE.md`'s new wellness/monitoring design
principle (added the same session): favor conversational, user-facing framing over passive
surveillance. This pass doesn't build the conversational piece (`walker_llm_bridge` integration
is deferred, §2.9), but the metrics-publishing package is built so that follow-up is a
straightforward subscribe-and-respond addition, not a restructuring.

## 2. Decisions

### 2.1 New package `walker_gait_metrics`, not folded into `walker_anomaly_detection`

Matches this project's established one-subsystem-per-package pattern (the same reasoning
`walker_anomaly_detection`'s own design spec §2.1 used when it was kept separate from
`walker_motor_driver`). `walker_anomaly_detection`'s own README frames it narrowly as
fall/anomaly detection; folding wellness metrics into it would blur that scope. Rejected:
extending `walker_anomaly_detection` directly (its detection logic and this package's
gait-tracking logic are unrelated concerns that happen to share an input stream, not one
concern).

### 2.2 `walker_anomaly_detection` republishes raw IMU samples on a new topic

`anomaly_detection_node.py` gains one small, additive change: alongside its existing
`/anomaly_detected` publisher, it republishes every parsed sample (the same dict
`imu_serial.parse_sample_line` already returns) as JSON on a new `/imu/raw_sample` topic. No
change to `FallDetector`/`TiltDetector` or the detection logic itself.

Chosen over `walker_gait_metrics` opening its own second connection to the same serial port
(rejected: two processes can't share one serial device; this also would have duplicated
`walker_anomaly_detection`'s own pty-virtual-serial testing story for no benefit).

On the receiving side, `gait_metrics_node.py` parses `/imu/raw_sample`'s JSON with its own small
local parse function — a duplicate of `imu_serial.parse_sample_line`'s validation logic (required
keys present, values numeric), not a cross-package import of it. Matches this project's accepted
precedent for small duplication across independently-buildable `ament_python` packages (e.g.
`verify_slam.py`/`verify_nav2.py`'s duplicated physical constants) — importing one package's
internal module from another creates a tighter build-order coupling than a few duplicated lines
of validation justify. A malformed or incomplete `/imu/raw_sample` message is silently skipped
(the callback returns without calling into `GaitTracker`), the same "return `None`/skip rather
than raise" contract `parse_sample_line` itself uses.

### 2.3 `/imu/raw_sample` payload: the same raw sample dict, not `sensor_msgs/Imu`

`std_msgs/String` with a JSON payload — `{"ax":.., "ay":.., "az":.., "gx":.., "gy":.., "gz":..,
"mx":.., "my":.., "mz":.., "t_ms":..}`, identical shape to what `imu_serial.parse_sample_line`
already produces. Mirrors `/anomaly_detected`'s already-established JSON-over-`String` pattern.
Rejected `sensor_msgs/Imu`: that message type expects `m/s²`/`rad/s` while these samples are in
`g`/deg-per-s-ish units (per `walker_anomaly_detection`'s design spec §2.3), forcing a
conversion this package doesn't otherwise need; it also has no field for magnetometer data and
would need an unpopulated orientation field with a `covariance[0] = -1` convention for "not
provided." Not worth the machinery for one consumer.

### 2.4 Step detection: threshold-crossing on accelerometer magnitude, with debounce

`step_counter.py`'s pure `StepCounter` mirrors `FallDetector`/`TiltDetector`'s exact style:
`StepCounter(step_threshold_g, min_step_interval_s)`; `.update(accel_magnitude_g: float, now_s:
float) -> bool`, called once per sample, returns `True` exactly on the sample confirming a new
step (magnitude crosses `step_threshold_g` and at least `min_step_interval_s` has elapsed since
the last confirmed step — the debounce prevents one footstep's impact-and-settle from being
counted twice). Takes an already-computed magnitude, same as `FallDetector`, not raw axes.
Thresholds are placeholder constructor defaults, not calibrated against real data — same
"placeholder now, recalibrate at bring-up" treatment as `walker_motor_driver`'s physical
constants and `walker_anomaly_detection`'s own seven threshold params.

### 2.5 Step detection from a frame-mounted IMU is a genuine experiment, not a validated assumption

`walker_anomaly_detection`'s IMU monitors the **walker frame's own** motion (fall/tilt of the
robot itself) — it is not worn by the person. Whether a person's footsteps transmit a clean,
threshold-crossable jolt through a wheeled, motorized frame (as opposed to a rigid, four-legged
rollator, or a body-worn accelerometer) is a genuine open question this design does not resolve
by assumption. Per user decision: build `StepCounter`/`GaitTracker` now, pytest-verified against
synthetic accelerometer sequences (which validates the *algorithm's* logic, not whether real
frame-transmitted vibration actually contains a detectable step signature) — real-world signal
quality is a bring-up-time finding, documented as an addition to
`walker_anomaly_detection/docs/bring_up.md` (this package owns no hardware of its own, so it gets
no `docs/bring_up.md` of its own). Mirrors `walker_nav`'s Kinect-narrow-FOV pass exactly: build
the testable part now, treat the real-hardware question as a real experiment, not a foregone
conclusion. Deciding what to do if bring-up finds the frame-mounted signal is unusable (e.g.
switching to a wheel-odometry-based step signal, or a person/handle-mounted sensor instead) is a
follow-up decision, not something this design resolves preemptively.

### 2.6 Step length: wheel odometry ÷ step count, not IMU double-integration

Pure double-integration of accelerometer data into step length is drift-prone — it's why the
wearable-IMU gait literature the user's research covers mostly reports speed/cadence rather than
step length directly. This project can sidestep that: `gait_tracker.py`'s pure `GaitTracker`
accumulates `total_distance_m` from consecutive `/odom` pose deltas (Euclidean distance between
successive `(x, y)` positions — `walker_motor_driver` already publishes accurate wheel odometry)
and reports `avg_step_length_m = total_distance_m / step_count` (`0.0` when `step_count == 0`,
guarding the division). This assumes the person's pace is coupled to the walker's own motion
(not lagging, leading, or working the frame back and forth while stationary) — a known, stated
limitation, not solved this pass.

`GaitTracker` composes a `StepCounter` internally: `on_imu_sample(sample: dict, now_s: float)`
feeds the sample's accelerometer magnitude into it and increments `step_count` on a detected
step; `on_odom_pose(x_m: float, y_m: float)` accumulates distance from the previous call's pose
(the first call has no previous pose to diff against, so it seeds `_last_pose` and adds no
distance). Exposes `step_count`, `total_distance_m`, `avg_step_length_m` as read properties.

### 2.7 Metrics are a running total for the node's lifetime — no persistence, no daily reset

`GaitTracker`'s counters start at zero when `gait_metrics_node` starts and never reset or persist
to disk. Matches this project's general "keep state minimal until there's a concrete need"
pattern — `walker_anomaly_detection` doesn't persist alerts either; `conversation_log.py` is the
only place in the whole project with actual disk persistence, for an unrelated concern. A
"per-day" or "reset" story is explicit future work (§6), not attempted here.

### 2.8 `/gait_metrics`: periodic timer publish, not event-only

`std_msgs/String`, JSON payload `{"step_count": int, "total_distance_m": float,
"avg_step_length_m": float, "timestamp": float}`, published on a 1 Hz timer — matches
`fake_lidar_node.py`'s timer-publish pattern, not `/anomaly_detected`'s event-only style, because
these are continuously-valid cumulative values a consumer (the dashboard now, `walker_llm_bridge`
later) wants to read at any time, not discrete events.

### 2.9 `walker_companion_app` dashboard wiring is included this pass; `walker_llm_bridge` is a named follow-up

Per user decision (revising the initial "defer both" scope mid-design): the dashboard update is
small and mirrors an exactly-existing pattern (§3), so it's included now. `walker_llm_bridge`
wiring — the direction `CLAUDE.md`'s wellness design principle actually points at — needs new
intent-recognition work (mirroring `stop_intent.py`'s pattern) and is deferred as a named
follow-up, the same posture `walker_anomaly_detection`'s own design spec §2.9 used for deferring
its dashboard wiring.

`dashboard_app_node.py` subscribes to `/gait_metrics` and calls a new
`SharedState.set_gait_metrics(...)`. `SharedState.status_snapshot()` gains a `gait` key
(`step_count`, `total_distance_m`, `avg_step_length_m`) alongside the existing `pose`/`nav_status`
— folded into the existing `/api/status` endpoint rather than a new one, since gait metrics are
the same kind of small, frequently-polled live data as pose/nav-status, unlike the chunkier
`/api/map`/`/api/conversation` endpoints. `web/index.html` gets a new "Gait" section (matching
the existing Status/Alerts/Conversation section style), populated by extending the existing
`pollStatus()` JS function — no new endpoint, no new polling loop.

### 2.10 No coupling to `walker_safety`

Matches every other package's established invariant. This package only publishes an
observational metric (and read-only consumes `/odom`, the same read-only relationship
`walker_nav` already has to `walker_motor_driver`) — it never subscribes to safety topics and
never publishes anything that could stop or control the robot.

### 2.11 Automated node-level verification via synthetic topic publishes, not hardware

`tools/verify_gait_metrics.py` needs no real IMU, serial connection, or `pty` trick — unlike
`walker_anomaly_detection`'s node, `gait_metrics_node` only ever consumes ROS topics. The script
publishes synthetic `/imu/raw_sample` messages (a constructed sequence crossing the step
threshold a known number of times) and synthetic `/odom` messages (a known total displacement),
then subscribes to `/gait_metrics` and confirms `step_count`, `total_distance_m`, and
`avg_step_length_m` match the constructed expectation — fully automated, matching every other
package's scripted `verify_X.py` pattern.

## 3. Package structure

New `ament_python` package:

```
src/walker_gait_metrics/
  package.xml, setup.py, setup.cfg, resource/walker_gait_metrics
  walker_gait_metrics/
    __init__.py
    step_counter.py       (pure: StepCounter)
    gait_tracker.py        (pure: GaitTracker, composes StepCounter + odom-distance accumulation)
    gait_metrics_node.py   (rclpy node: subscribes /imu/raw_sample + /odom, publishes /gait_metrics
                             on a 1 Hz timer)
  launch/gait_metrics.launch.py
  test/
    conftest.py
    test_step_counter.py
    test_gait_tracker.py
  tools/
    verify_gait_metrics.py (scripted, fully automated: publishes synthetic /imu/raw_sample and
                             /odom, checks /gait_metrics - no hardware needed, per §2.11)
```

Modified existing packages:

```
src/walker_anomaly_detection/
  walker_anomaly_detection/anomaly_detection_node.py  (+ publish /imu/raw_sample, per §2.2)
  docs/bring_up.md                                     (+ note: step-detectability from the
                                                          frame-mounted IMU is an open bring-up
                                                          finding, per §2.5)

src/walker_companion_app/
  walker_companion_app/dashboard_app_node.py  (+ subscribe /gait_metrics)
  walker_companion_app/shared_state.py         (+ set_gait_metrics(); status_snapshot() gains
                                                 a `gait` key)
  web/index.html                                (+ Gait section; pollStatus() extended)
```

## 4. Interface

**Node:** `walker_gait_metrics` (entry point `gait_metrics_node`)

**Params:**
| Param | Default | Notes |
|---|---|---|
| `step_threshold_g` | `1.2` | accel magnitude above this (in g) counts as a step candidate |
| `min_step_interval_s` | `0.3` | debounce; caps detectable cadence at 200 steps/min |
| `publish_rate_hz` | `1.0` | `/gait_metrics` publish rate |

All three are placeholders per §2.4/§2.5 — recalibrate (or replace the underlying signal
entirely, if bring-up finds the frame-mounted IMU doesn't carry a usable step signature) once
real data exists.

**Topics subscribed:**
- `/imu/raw_sample` (`std_msgs/String`, JSON — same shape as `walker_anomaly_detection`'s parsed
  samples)
- `/odom` (`nav_msgs/Odometry`, from `walker_motor_driver`)

**Topics published:**
- `/gait_metrics` (`std_msgs/String`, JSON: `{"step_count": int, "total_distance_m": float,
  "avg_step_length_m": float, "timestamp": float}`)

**Changes to existing packages' interfaces:**
- `walker_anomaly_detection` gains a new published topic: `/imu/raw_sample` (`std_msgs/String`,
  JSON — the raw sample dict, per §2.2/§2.3).
- `walker_companion_app`'s `/api/status` JSON response gains a `gait` key (per §2.9).

## 5. Testing

`step_counter.py` and `gait_tracker.py` are pure Python — unit-tested with pytest, no ROS
sourcing or colcon build required, same `test/conftest.py` pattern as every other package. Test
cases include: a constructed sequence of N threshold-crossings at least `min_step_interval_s`
apart counts N steps; two crossings closer together than `min_step_interval_s` count as one step
(debounce); a sequence of `/odom` poses with known total displacement and a known step count
produces the expected `avg_step_length_m`; `avg_step_length_m` is `0.0`, not a `ZeroDivisionError`,
when `step_count` is `0`; the very first `on_odom_pose` call adds no distance (nothing to diff
against yet).

`gait_metrics_node.py` is not pytest-testable (same reason every other package's `rclpy` node
isn't), but per §2.11 it IS fully, automatically verified: `tools/verify_gait_metrics.py`
publishes synthetic `/imu/raw_sample` and `/odom` messages and confirms `/gait_metrics` reflects
the expected values — no real hardware needed.

`anomaly_detection_node.py`'s new `/imu/raw_sample` publisher is covered by extending its
existing `tools/verify_anomaly_detection.py` script to also confirm that topic is published
alongside the pty-fed synthetic samples it already sends — a small addition, not a new
verification story.

`shared_state.py`'s new `set_gait_metrics()`/`status_snapshot()` change is pure and covered by
extending the existing `test_shared_state.py` pattern. `dashboard_app_node.py`'s new subscription
isn't pytest-tested itself (matches that package's convention), covered by extending
`tools/verify_dashboard_app.py`.

Real-world step-detectability from the frame-mounted IMU (§2.5) is explicitly **not** automatable
— it's a bring-up-time finding, documented as an addition to
`walker_anomaly_detection/docs/bring_up.md`, mirroring `walker_anomaly_detection`'s own treatment
of "does the sensor itself work" as a separate manual step from "does the pipeline wiring work."

## 6. Out of scope

- Grip strength — needs handle hardware that doesn't exist yet; tracked as its own future spec in
  `docs/ideas-backlog.md`.
- Kinect-based gait/fitness analysis — a separate, larger idea with its own open mounting-geometry
  question; tracked in `docs/ideas-backlog.md`.
- `walker_llm_bridge` conversational exposure ("how many steps have I taken") — a named follow-up
  per §2.9, not built this pass.
- Persistence or daily reset of `step_count`/`total_distance_m` — explicitly deferred per §2.7,
  not merely forgotten.
- Deciding what to do if bring-up finds the frame-mounted IMU can't reliably detect footsteps —
  a finding to act on in a follow-up per §2.5, not a decision this design makes preemptively.
- Calibrating the three placeholder params against real hardware — deferred to bring-up, same
  treatment as every other package's placeholder physical/threshold constants.
- Any coupling to `walker_safety` — explicitly rejected per §2.10.
- A custom ROS2 `.msg` type — rejected per §2.3, same reasoning `walker_anomaly_detection`'s own
  design spec §2.7 already gave for `/anomaly_detected`.
