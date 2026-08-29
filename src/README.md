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

## Setup (once packages exist)

```bash
sudo apt install ros-humble-desktop ros-humble-slam-toolbox ros-humble-navigation2
cd src
colcon build --symlink-install
source install/setup.bash
```
