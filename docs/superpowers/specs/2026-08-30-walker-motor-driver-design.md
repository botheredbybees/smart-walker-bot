# walker_motor_driver Design

**Date:** 2026-08-30
**Status:** Approved by user; ready for implementation planning
**Scope:** Step 2 of the revised Phase 1 roadmap
(`docs/superpowers/specs/2026-08-30-phase1-roadmap-design.md` §3): the real driver-board
(L298N/BTS7960) ROS2 node interface, backed by a lightweight kinematic simulator until real
motor hardware exists. Does not cover `walker_nav` (step 3) or the hardware bring-up
checkpoint (step 4) itself — this design produces what step 4 later swaps a backend into.

## 1. Problem

The roadmap design's step 2 (`docs/superpowers/specs/2026-08-30-phase1-roadmap-design.md` §3)
specifies a driver-board node interface "backed by the lightweight kinematic sim... until real
hardware exists," with "the sim/real boundary sits exactly at this node's interface, so
swapping in real GPIO later doesn't require changes upstream." That's a roadmap-level
description, not a design — it doesn't say what the node's actual ROS2 topics/messages are,
how the sim/real boundary is structured internally, what language it's written in, or how it's
tested. This design fills that in.

Unlike `walker_safety`, this package's dependents are known and concrete: `walker_nav` (step 3)
configures upstream `slam_toolbox`/`nav2` against whatever this package publishes, so the
topic/message interface chosen here is a real commitment other code will be written against.

## 2. Decisions

### 2.1 Python (rclpy), not C++

Matches `walker_safety`'s Python-first approach and the rest of the project's tone. This
workstation already has the full ROS2 Humble Python toolchain installed
(`docs/superpowers/specs/2026-08-30-phase1-roadmap-design.md` §4). C++'s lower latency isn't
needed at this control-loop scale (tens of Hz, not hard real-time), and it would add a build
step and boilerplate the project hasn't used anywhere else.

### 2.2 Real ROS2/colcon package, unlike walker_safety

`walker_safety` deliberately opted out of being a colcon package because its watchdog runs on
a physically separate Pico, not in the ROS2 graph at all
(`docs/superpowers/specs/2026-08-30-phase1-roadmap-design.md` §2.2). `walker_motor_driver` has
no such reason to opt out — it's a real ROS2 node that other ROS2 nodes (`walker_nav`) will
launch and talk to over topics, so it's a standard `ament_python` package with
`package.xml`/`setup.py`/`setup.cfg`, buildable with `colcon build`.

### 2.3 Backend abstraction for the sim/real boundary

A small `MotorBackend` interface, not a "swap the sim call for a real call later" approach:

```python
class MotorBackend:
    def apply_wheel_speeds(self, left_rad_s: float, right_rad_s: float) -> None: ...
    def read_wheel_deltas(self) -> tuple[float, float]: ...  # (left_rad, right_rad) since last read
```

`SimMotorBackend` implements this now with an idealized kinematic model (commanded wheel speed
achieved instantly, no motor dynamics, no slip). At the hardware bring-up checkpoint
(roadmap design §3 step 4), a `GpioMotorBackend` gets added alongside it, selected by a launch
argument (`backend:=sim|real`) — the ROS2 node's control logic (parameter handling, `/cmd_vel`
subscription, `/odom`/TF publishing, kinematics/odometry calls) never changes at bring-up. Only
the backend-construction branch in `__init__` (currently a single `if backend_name == 'sim':`)
gains a new `elif` for the added backend, and `MotorBackend` declares a `stop()` lifecycle
method (called from `main()`'s shutdown path) so a future hardware backend has a defined place
to de-energize motors on clean shutdown, not just on E-stop/watchdog cutoff.
This mirrors `walker_safety`'s split
between pure logic (`watchdog_logic.py`) and the hardware-facing entry point (`main.py`), and
was chosen over "call the sim directly, edit the node's internals later" specifically because
the roadmap design's stated goal (§2.1) is to avoid rework when the board/hardware decision is
finally made — a direct-call approach would have reintroduced exactly the rework the roadmap
was trying to defer.

### 2.4 Standard ROS2/Nav2 topic and message conventions

- **Command in:** `geometry_msgs/Twist` on `/cmd_vel` — what Nav2 publishes by default.
- **Feedback out:** `nav_msgs/Odometry` on `/odom`, plus an `odom`→`base_link` TF broadcast via
  `tf2_ros` — what `slam_toolbox`/`nav2` expect for localization.

Chosen over a custom message format so `walker_nav` (step 3) can configure `nav2`/`slam_toolbox`
against this package with zero custom glue code, using their standard, documented expectations.

### 2.5 Physical parameters are placeholders, not guesses to get right now

`wheel_radius_m`, `wheel_separation_m`, and `max_wheel_speed_rad_s` are ROS2 node parameters
with placeholder defaults (typical small robot-vacuum dimensions: `wheel_radius_m=0.03`,
`wheel_separation_m=0.2`, `max_wheel_speed_rad_s=10.0`), not values derived from real
measurements — the actual salvaged-vacuum dimensions aren't known until vacuums are stripped
(`README.md` §6 step 1). Documented as needing recalibration at bring-up, the same treatment
`walker_safety` gave the MOSFET/relay part choice
(`docs/superpowers/specs/2026-08-30-phase1-roadmap-design.md` §2.3).

### 2.6 No coupling to walker_safety

The hardware E-stop and Pico watchdog cut motor power physically, in series with the driver
board's power rail, independent of any software this package runs
(`docs/superpowers/specs/2026-08-30-phase1-roadmap-design.md` §2.2, §5.4 in the project's root
`README.md`). `walker_motor_driver` does not check watchdog state, does not publish a heartbeat,
and does not need to know the watchdog exists — adding a software interlock here would be
redundant with, and could create false confidence alongside, the physical cutoff that's meant
to be independent of exactly this kind of software. This is a boundary worth stating explicitly
so it isn't "fixed" by someone adding coupling later.

This is distinct from `motor_driver_node.py`'s local `cmd_vel_timeout_s` behavior: if no
`/cmd_vel` command arrives within that timeout, the node zeroes wheel speeds itself, using only
information it already has (when it last received a command). That's ordinary defensive behavior
for a velocity-command interface, not a heartbeat protocol, not a check against `walker_safety`'s
watchdog state, and not a substitute for the hardware E-stop. The boundary this section draws is
against *coupling to walker_safety specifically*, not against this package having any fail-safe
behavior of its own.

## 3. Package structure

New `ament_python` package:

```
src/walker_motor_driver/
  package.xml, setup.py, setup.cfg, resource/walker_motor_driver
  walker_motor_driver/
    __init__.py
    diff_drive_kinematics.py   (pure: twist -> wheel speeds, wheel deltas -> odometry)
    motor_backend.py            (MotorBackend interface)
    sim_backend.py               (SimMotorBackend)
    motor_driver_node.py        (rclpy node, wires the above together)
  launch/motor_driver.launch.py (backend:=sim|real argument, default sim)
  test/
    conftest.py
    test_diff_drive_kinematics.py
    test_sim_backend.py
  tools/
    verify_motor_driver.py       (scripted end-to-end check, not pytest)
```

## 4. Testing

`diff_drive_kinematics.py` and `sim_backend.py` are pure Python — no `rclpy` or hardware
imports — so they're fully unit-tested with pytest, the same pattern `watchdog_logic.py` used.

`motor_driver_node.py` is different from `walker_safety`'s `main.py` in one important way: it
*can* actually run in this environment, since this workstation has the full ROS2 Humble
install — there's no missing-hardware-module problem here the way `machine` was missing for
MicroPython. So instead of "untestable, verify manually with real hardware," the node gets a
manual verification procedure that doesn't require any physical hardware at all: launch it with
`backend:=sim`, `ros2 topic pub` a `/cmd_vel` command, `ros2 topic echo /odom` and confirm the
numbers move as the kinematics predict, and check `/tf` for the broadcast. This is documented
in the package but isn't pytest-automatable within a single task's TDD loop the way the pure
modules are.

## 5. Out of scope

- `GpioMotorBackend` and any real GPIO/L298N/BTS7960 wiring — deferred to the hardware bring-up
  checkpoint (roadmap design §3 step 4), same as `walker_safety`'s E-stop circuit specifics.
- `walker_nav` itself (roadmap design §3 step 3) — this design produces the topic interface it
  will configure `slam_toolbox`/`nav2` against, but doesn't touch that package.
- Recalibrating `wheel_radius_m`/`wheel_separation_m`/`max_wheel_speed_rad_s` to real measured
  values — explicitly deferred to bring-up per §2.5.
- Any coupling to `walker_safety`'s watchdog — explicitly rejected per §2.6, not merely deferred.
