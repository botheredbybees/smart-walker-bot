# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project status

Early scaffold stage. No ROS2 packages exist yet — `src/` is an empty `colcon` workspace root with only a
planning README. There is no build, lint, or test tooling to run yet. When packages start landing, update this
file (and `src/README.md`) with the real commands.

## What this project is

A DIY companion/monitoring robot built by repurposing parts (drive motors, encoders, LiDAR, bump/cliff sensors)
salvaged from two dead robot vacuums, controlled by a Raspberry Pi 5 running ROS2, with voice interaction via a
locally-hosted LLM (Ollama on a separate home server with an Nvidia 5060 Ti). Full rationale, architecture, and
references live in `README.md` — read it before proposing design changes, since most decisions here trace back to
specific sections there.

## The Phase 1 / Phase 2 boundary (critical, read before touching safety or motion code)

The project has a hard, deliberate scope split (`README.md` §4):

- **Phase 1 — the actual deliverable**: a mobile robot that navigates, converses, and monitors for falls/anomalies,
  but is **never** leaned on or physically depended on for balance. This is the only phase with a build order.
- **Phase 2 — explicitly out of scope**: a real weight-bearing walker frame. It would need purpose-built, load-rated
  hardware (rigid frame, load-rated casters, mechanical brake, motors sized for continuous torque under a person's
  weight) that a salvaged vacuum drivetrain cannot provide. It is documented only as future aspirational context,
  not a next step.

Do not write or suggest code that treats this platform as load-bearing, fall-arresting, or leanable-on — that would
contradict the project's explicit non-goals (§4.1) and its stated risk (§7, "Phase conflation").

## Safety-layer invariants (§5.4, §7)

These constraints shape how motor-control and nav code must be structured, not just what the docs say:

- **Hardware E-stop is independent of software.** It's a physical switch wired directly into the motor driver's
  power line, cutting drive power regardless of what the Pi, ROS2, or the LLM are doing. Per the build order (§6
  step 2), this must exist and be wired *before* any motor — even teleoperated — is put under program control.
- **Software watchdog is independent of the ROS2 nav loop.** It halts motors on a lost heartbeat and must not share
  a failure domain with the navigation stack it's supervising — don't implement it as just another node in the same
  process/executor as `walker_nav`.
- **Voice "stop" commands are a convenience layer only.** LLM inference and STT/TTS round-trip over the local
  network, which is too slow to be a primary safety mechanism. Never wire a spoken command as the sole or primary
  path to stopping the robot — it must go through (or be backed by) the hardware E-stop / watchdog path.

## Planned architecture (`src/README.md`)

`src/` is a `colcon` workspace. Planned packages, in build order (README.md §6):

1. **`walker_safety`** — hardware E-stop wiring notes + the software watchdog node. First package, before any
   motor is under program control.
2. **`walker_motor_driver`** — L298N/BTS7960 driver interface translating motion commands into wheel speeds.
   Reference pattern: https://github.com/dblanding/diy-ROS-robot for Pi → driver-board wiring.
3. **`walker_nav`** — thin integration/config layer over upstream `slam_toolbox` and `nav2`. Deliberately *not* a
   reimplementation of SLAM or path planning — prefer configuring upstream packages over writing custom nav logic.
4. **`walker_llm_bridge`** — voice I/O (STT/TTS) and the connection to the Ollama server for conversation and
   natural-language nav commands (e.g. "take me back to the shed" → a Nav2 goal).
5. **`walker_companion_app`** — optional local-network-only phone dashboard (status, location, alerts, conversation
   log). Last in the build order; a plain local log file of anomaly events is an acceptable fallback until this
   exists.

Data flow, end to end (README.md §5):

```
Salvaged motors/wheels -> Motor driver (L298N/BTS7960) -> Motion controller
Salvaged LiDAR/IMU/bump sensors -> Pi 5 sensor fusion -> ROS2 nav stack (SLAM + obstacle avoidance)
ROS2 nav stack -> Motion controller -> Motor driver
Hardware E-stop -> Motor driver directly (bypasses Pi/ROS2/LLM entirely)
Software watchdog -> Motor driver (independent of the ROS2 nav loop; halts on lost heartbeat)
Local LLM (Ollama, remote 5060 Ti server) <-> Voice I/O (mic/speaker + STT/TTS)
Local LLM <-> Sensor fusion / nav stack
Sensor fusion -> Fall/anomaly detection -> Companion app
```

## Setup (once packages exist)

```bash
sudo apt install ros-humble-desktop ros-humble-slam-toolbox ros-humble-navigation2
cd src
colcon build --symlink-install
source install/setup.bash
```
