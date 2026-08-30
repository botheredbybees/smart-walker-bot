# walker_safety

E-stop wiring design and the software watchdog for the smart-walker-bot
project. See `docs/superpowers/specs/2026-08-30-phase1-roadmap-design.md`
§2.2 for why the watchdog runs on a physically separate Pi Pico rather
than as a ROS2 node.

This directory intentionally has no `package.xml`/`CMakeLists.txt` — it
is not a ROS2/colcon package. `colcon build` skips it silently.

## Layout

- `docs/e_stop_wiring.md` — physical wiring design and fail-safe
  rationale.
- `firmware/` — Pico firmware (MicroPython). `watchdog_logic.py` and
  `heartbeat_framing.py` are pure Python, unit-tested with pytest on the
  desktop (`firmware/tests/`); `main.py` is the on-device entry point
  and can only be verified on real hardware (see the wiring doc's
  "Firmware bring-up" section).
- `tools/send_fake_heartbeats.py` — PC-side script that sends fake
  heartbeats over serial, used for manual hardware verification.

## Running the tests

```bash
cd src/walker_safety/firmware
python3 -m pytest tests/ -v
```

## Latching vs auto-recovery

The watchdog currently auto-recovers: if heartbeats resume after a trip,
`is_tripped()` returns `False` again on the next heartbeat, re-enabling
motors without any manual reset step. This matches `README.md` §5.4's
literal description ("halts motors if it stops receiving heartbeat
signals"). A latching design (requiring an explicit reset before motors
re-enable) is arguably safer for a real E-stop and worth reconsidering
once there's a physical robot to test against — deferred rather than
guessed at now.
