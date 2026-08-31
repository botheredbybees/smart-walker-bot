# ROS2 workspace

This directory is a `colcon` workspace root. Three packages now exist —
`walker_safety`, `walker_motor_driver` and `walker_nav` (the first three
build-order steps); the remaining two below are still planned. This file
records the layout so build-order work (see the main
[README](../README.md) §6) lands in a consistent place.

Note that `walker_safety` is deliberately *not* a colcon package — its
watchdog runs on a physically separate Pico, outside the ROS2 graph
entirely (see its own README). `walker_motor_driver` and `walker_nav` are
real `ament_python` packages.

## Planned packages

- **Built.** **`walker_safety`** — hardware E-stop wiring notes + the software
  watchdog node (main README §5.4). Build-order step 2, before any motor
  is under program control.
- **Built.** **`walker_motor_driver`** — L298N/BTS7960 interface translating motion
  commands into wheel speeds (§5.2). Reference:
  [dblanding/diy-ROS-robot](https://github.com/dblanding/diy-ROS-robot) for
  the Pi → driver-board wiring pattern.
- **Built (SLAM + Nav2).** **`walker_nav`** — thin integration/config layer on top of upstream
  `slam_toolbox` and `nav2`, not a reimplementation of SLAM or path
  planning (§5.2, §7 risk notes on scope).
- **Built (text bridge).** **`walker_llm_bridge`** — text-based
  conversational bridge to the Ollama server (§5.3); real STT/TTS and
  nav-goal translation still deferred (see the package's own README).
- **Built.** **`walker_companion_app`** — local-network web dashboard:
  robot pose, Nav2 status, live map, and the `walker_llm_bridge`
  conversation log (§5.5). Fall/anomaly alerts are a static placeholder
  — see the package's own README.
- **Built (pure logic + node; hardware bring-up pending).** **`walker_anomaly_detection`** —
  fall/anomaly detection via a real ESP32-streamed 9-axis IMU: free-fall+impact and
  sustained-tilt detection, publishing `/anomaly_detected` alerts. A new addition beyond the
  original five-package roadmap — root `README.md` §5.2 originally assigned this to
  `walker_motor_driver`, but it was never implemented there. First package developed against
  real hardware rather than simulation; see the package's own README and `docs/bring_up.md`.
  Wiring `/anomaly_detected` into `walker_companion_app`'s dashboard is a separate follow-up.

## Setup

Dev workstation target: Ubuntu 22.04 "Jammy" (or a Jammy-based distro, e.g. Linux Mint 21.3) — matches
ROS2 Humble. See the Phase 1 roadmap design
(`docs/superpowers/specs/2026-08-30-phase1-roadmap-design.md`) for why onboard-board choice is deferred
and why this workstation is where `walker_safety`/`walker_motor_driver`/`walker_nav` get developed first,
against a lightweight simulation, before any hardware is involved.

```bash
# ROS2 apt repo isn't configured by default — add it first
sudo apt-get update
sudo apt-get install -y curl gnupg lsb-release ca-certificates
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt-get update
sudo apt-get install -y ros-humble-desktop ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup python3-colcon-common-extensions python3-rosdep

sudo rosdep init
rosdep update
```

Build/test `walker_motor_driver`:

```bash
source /opt/ros/humble/setup.bash
cd src
PYTHONNOUSERSITE=1 colcon build --packages-select walker_motor_driver --symlink-install
source install/setup.bash

python3 -m pytest walker_motor_driver/test/ -v   # pure-module unit tests, no ROS sourcing needed
```

Build/test `walker_nav`:

```bash
source /opt/ros/humble/setup.bash
cd src
PYTHONNOUSERSITE=1 colcon build --packages-select walker_nav --symlink-install
source install/setup.bash

python3 -m pytest walker_nav/test/ -v   # pure-module unit tests, no ROS sourcing needed
```

Build/test `walker_llm_bridge`:

```bash
source /opt/ros/humble/setup.bash
cd src
PYTHONNOUSERSITE=1 colcon build --packages-select walker_llm_bridge --symlink-install
source install/setup.bash

python3 -m pytest walker_llm_bridge/test/ -v   # pure-module unit tests, no ROS sourcing needed
```

Build/test `walker_companion_app`:

```bash
source /opt/ros/humble/setup.bash
cd src
PYTHONNOUSERSITE=1 colcon build --packages-select walker_companion_app --symlink-install
source install/setup.bash

python3 -m pytest walker_companion_app/test/ -v   # pure-module unit tests, no ROS sourcing needed
```

Build/test `walker_anomaly_detection`:

```bash
source /opt/ros/humble/setup.bash
cd src
PYTHONNOUSERSITE=1 colcon build --packages-select walker_anomaly_detection --symlink-install
source install/setup.bash

python3 -m pytest walker_anomaly_detection/test/ -v   # pure-module unit tests, no ROS sourcing needed
```

(`PYTHONNOUSERSITE=1` works around a workstation-specific setuptools/jaraco.functools version
mismatch between this machine's user-site Python packages and what ROS2 Humble's apt packages
expect — not a general ROS2 requirement, and may not be needed on other machines.)
