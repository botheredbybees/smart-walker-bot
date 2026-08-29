# ROS2 workspace

This directory is a `colcon` workspace root. No packages exist yet — this
file just records the planned layout so build-order work (see the main
[README](../README.md) §6) lands in a consistent place.

## Planned packages

- **`walker_safety`** — hardware E-stop wiring notes + the software
  watchdog node (main README §5.4). Build-order step 2, before any motor
  is under program control.
- **`walker_motor_driver`** — L298N/BTS7960 interface translating motion
  commands into wheel speeds (§5.2). Reference:
  [dblanding/diy-ROS-robot](https://github.com/dblanding/diy-ROS-robot) for
  the Pi → driver-board wiring pattern.
- **`walker_nav`** — thin integration/config layer on top of upstream
  `slam_toolbox` and `nav2`, not a reimplementation of SLAM or path
  planning (§5.2, §7 risk notes on scope).
- **`walker_llm_bridge`** — voice I/O (STT/TTS) and the connection to the
  Ollama server for the conversational layer (§5.3).
- **`walker_companion_app`** — the optional local dashboard (§5.5), last
  in the build order.

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

Once packages exist under `src/`:

```bash
cd src
colcon build --symlink-install
source install/setup.bash
```
