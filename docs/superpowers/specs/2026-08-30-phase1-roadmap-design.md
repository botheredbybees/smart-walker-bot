# Phase 1 Roadmap Design: Simulation-First Compute Deferral

**Date:** 2026-08-30
**Status:** Approved by user; ready for implementation planning
**Scope:** Adjusts and sequences `README.md` §6's build order for the five planned Phase 1 packages
(`README.md` §5.2/§5.5, `src/README.md`). Does not revisit the Phase 1/Phase 2 boundary (`README.md` §4) or
the hardware E-stop's physical requirement (`README.md` §5.4) — both stand as originally specified.

## 1. Problem

The original build order (`README.md` §6) assumes hardware — salvaged motors, a chosen onboard board, a wired
E-stop — is available from step 1 onward, and assumes a Raspberry Pi 5 as the onboard controller (§5.1). Neither
holds:

- Vacuums are not yet stripped; no salvaged motors/LiDAR are on the bench.
- The user has had reliability problems with Pi 4/5 boards in past projects (SD-card corruption after power loss,
  and under-voltage brownouts), and has stable prior experience with Pi Zero boards. A Pi 5 is not committed to,
  and neither is any other board yet.
- The user's actual on-hand inventory is broader than the proposal assumed: Pi Zero, Pi 4, Pi Pico, and ESP32
  boards, not just a hypothetical Pi 5.

This design resolves the onboard-compute uncertainty by deferring it rather than guessing, and restructures the
early build order so software work can start immediately without waiting on hardware decisions or salvage work.

## 2. Decisions

### 2.1 Defer onboard-compute choice; develop in simulation first

Do not choose between Pi Zero / Pi Zero 2 W / Pi 4 now. Build `walker_safety`, `walker_motor_driver`, and
`walker_nav` against a simulated robot on the dev workstation. Choose the onboard board at the hardware
bring-up checkpoint (§3, step 4 below), using real benchmarks from running the actual candidate boards rather
than datasheet comparisons.

This was chosen over two alternatives considered and rejected:
- Committing now to "Pi Zero as thin I/O node, heavy compute offboard" — rejected because it forecloses the Pi 4
  option before the reliability concerns below are addressed, and because no hardware exists yet to make the
  choice meaningful.
- Committing now to a Pi 5-class upgrade — rejected outright given the user's stated reliability concerns with
  Pi 4/5 boards.

### 2.2 Software watchdog moves to a physically independent MCU (Pi Pico)

`README.md` §5.4 requires the software watchdog to be "independent of the ROS2 nav loop." The original framing
(a separate process on the same SBC) only gives process-level isolation — an OS-level hang, kernel panic, or
power brownout on the main board takes the watchdog down with everything else it's meant to guard against.

Decision: the watchdog runs as firmware on a Pi Pico, wired between the motor driver and the E-stop line,
monitoring a heartbeat from the main ROS2 side over USB serial. This is physical isolation, not just process
isolation — a stronger fit for the independence requirement than what was originally scoped.

ESP32 is held in reserve for a future non-safety-critical role (not scoped in this design); its wireless stack
is judged an unnecessary addition to a safety-critical path, which is why the Pico (no wireless) was chosen for
the watchdog specifically.

### 2.3 Reliability hardening applied at bring-up, regardless of board choice

The user's two reported Pi 4/5 problems have known fixes that are not board-specific, so they're applied to
whichever board is chosen in step 4, not treated as reasons to exclude the Pi 4:

- **SD-card corruption after power loss** → mount root read-only with a RAM-backed overlay (Raspberry Pi OS's
  built-in overlay filesystem via `raspi-config`, or `overlayroot` on other Debian-derived images). No writes
  happen to the card in normal operation, so an unclean power-off can't corrupt it.
- **Under-voltage brownouts** → this is a power-distribution problem, not a board defect: motor stall current
  can sag a shared rail enough to brownout the SBC. Fix is a dedicated, adequately-rated regulated 5V supply for
  compute, electrically isolated from the motor driver's power rail. Add this to the power budget in
  `README.md` §5.6 when hardware work begins.

### 2.4 Simulation fidelity: lightweight custom sim, not Gazebo

For steps 2–3 below, "simulation" means a plain ROS2 node that fakes encoder feedback from commanded velocity
(basic kinematics) plus a simple fake-obstacle LiDAR publisher — not a full physics simulator. This is enough to
validate node interfaces and exercise `slam_toolbox`/`nav2` configuration end-to-end, without the setup and
maintenance cost of a Gazebo world/URDF model. `rviz2` (part of `ros-humble-desktop`) is used to visualize the
sim's fake sensor output and the resulting SLAM map / Nav2 plans.

## 3. Revised package roadmap

Replaces `README.md` §6 steps 2–4. Steps 1 (strip vacuums) and 5–6 (LLM bridge, companion app) are unchanged
from the original proposal and not repeated here.

1. **`walker_safety`** — E-stop wiring documentation, plus watchdog firmware for the Pico, developed and tested
   against a fake heartbeat source (no real hardware needed yet).
2. **`walker_motor_driver`** — the real driver-board (L298N/BTS7960) node interface (subscribe to velocity
   commands, publish encoder feedback), backed by the lightweight kinematic sim from §2.4 until real hardware
   exists. The sim/real boundary sits exactly at this node's interface, so swapping in real GPIO later doesn't
   require changes upstream.
3. **`walker_nav`** — `slam_toolbox` + `nav2` configuration, exercised against the sim's fake LiDAR and the
   simulated motor driver from step 2. Thin integration/config layer only, per `src/README.md` — not a
   reimplementation of SLAM or path planning.
4. **Hardware bring-up checkpoint** — once vacuums are stripped and a candidate board is in hand: wire the real
   E-stop, connect the Pico watchdog, swap the simulated motor driver for real GPIO/driver-board code, apply the
   read-only-rootfs and power-isolation hardening from §2.3, and benchmark the candidate board(s) to make the
   final onboard-compute decision from real data.
5. **`walker_llm_bridge`** — unchanged from `README.md` §6 step 5.
6. **`walker_companion_app`** — unchanged from `README.md` §6 step 6, still optional and last.

## 4. Dev environment (this workstation)

Target: Linux Mint 21.3 (Ubuntu 22.04 "Jammy" base) — compatible with ROS2 Humble's apt packages.

Packages: `ros-humble-desktop` (full desktop install, not base — `rviz2` is needed to visualize the simulation),
`ros-humble-slam-toolbox`, `ros-humble-navigation2`, `ros-humble-nav2-bringup`,
`python3-colcon-common-extensions`, `python3-rosdep`. Requires adding the ROS2 apt repository and signing key
first, since neither is currently configured on this machine — `src/README.md`'s existing setup section assumes
this step already happened and will be updated to include it.

## 5. Out of scope

- The Phase 1/Phase 2 boundary and non-goals (`README.md` §4) — unchanged, not revisited by this design.
- `walker_llm_bridge` and `walker_companion_app` design — deferred to their own design pass when steps 1–4 are
  closer to done, per the original build order's own sequencing.
- ESP32's eventual role — not scoped; noted in §2.2 as reserved for later.
- Final onboard-compute board choice — explicitly deferred to step 4, not decided by this design.
