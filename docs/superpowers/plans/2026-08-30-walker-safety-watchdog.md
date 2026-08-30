# walker_safety (E-Stop Docs + Pico Watchdog) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce the `walker_safety` package: E-stop wiring documentation, and Pico watchdog firmware developed and unit-tested against a fake heartbeat source, with no salvaged hardware required to complete this plan.

**Architecture:** Split the watchdog into a pure-Python core (`watchdog_logic.py`, `heartbeat_framing.py`) with zero hardware/MicroPython dependencies, unit-tested with pytest on the desktop, and a thin MicroPython entry point (`main.py`) that wires that core to real GPIO/serial I/O on a Pi Pico. The pure core is what "tested against a fake heartbeat source" means in practice — tests feed it fabricated timestamps and bytes with no serial port or Pico involved. `main.py` can't be unit tested off-device (it imports `machine`, which only exists in MicroPython on real hardware), so it gets a manual hardware-verification procedure instead, backed by a small PC-side script that sends real fake heartbeats over serial for a human to observe.

**Tech Stack:** MicroPython (Pico firmware), Python 3 + pytest (desktop unit tests), pyserial (manual hardware-verification tooling).

**Spec:** `docs/superpowers/specs/2026-08-30-phase1-roadmap-design.md` (§2.2 for the Pico-independence rationale, §3 step 1 for scope, §5 for what's explicitly out of scope).

## Global Constraints

- Fail-safe default: `Watchdog.is_tripped()` must return `True` before any heartbeat has ever been received — a Pico that's unpowered, crashed, or freshly booted fails toward "motors off." (spec §2.2, `README.md` §5.4)
- `watchdog_logic.py` and `heartbeat_framing.py` must have zero MicroPython-specific or hardware imports, so they're testable on the desktop via pytest without a Pico attached. (spec §3 step 1)
- `src/walker_safety/` is not a colcon/ROS2 package — no `package.xml` or `CMakeLists.txt`. The watchdog is MicroPython firmware, not a ROS2 node; `colcon build` skips this directory silently. (spec §2.2)
- Onboard-compute board choice, read-only-rootfs hardening, and power-rail isolation are explicitly out of scope for this plan — deferred to the hardware bring-up checkpoint. (spec §2.1, §2.3, §5)
- Whether the watchdog latches (requires manual reset) or auto-recovers is explicitly deferred — this plan implements auto-recovery only, documented as a revisit item, not decided silently. (see Task 1)

---

## Task 1: E-Stop Wiring Documentation + Package Scaffold

**Files:**
- Create: `src/walker_safety/README.md`
- Create: `src/walker_safety/docs/e_stop_wiring.md`

**Interfaces:**
- Produces: no code interfaces. Documents the pin assignment (`GPIO15` / `ENABLE_PIN_NUM`) and timeout (`0.5s` / `TIMEOUT_S`) that Task 4's firmware must match exactly.

- [ ] **Step 1: Create the package directories**

```bash
mkdir -p /home/peter_sha/sourcecode/smart-walker-bot/src/walker_safety/docs
mkdir -p /home/peter_sha/sourcecode/smart-walker-bot/src/walker_safety/firmware/tests
mkdir -p /home/peter_sha/sourcecode/smart-walker-bot/src/walker_safety/tools
```

- [ ] **Step 2: Write the E-stop wiring doc**

Create `src/walker_safety/docs/e_stop_wiring.md`:

```markdown
# E-Stop Wiring (walker_safety)

Status: design document, not yet built — no motors or driver board are on
the bench yet. Written now per `README.md` §6 step 2: "the E-stop must
exist before any motor is under program control."

## Topology

Two independent ways to cut drive-motor power, either one sufficient on
its own:

1. **Manual hardware E-stop**: a physical latching push-button switch
   wired in series with the motor driver's main power input (the +V rail
   feeding the L298N/BTS7960 board), placed between the battery and the
   driver. Pressing it opens the circuit directly — no microcontroller,
   ROS2, or Pico involved.
2. **Pico watchdog cutoff**: the Pico's `ENABLE_PIN_NUM` (GPIO15, see
   `firmware/main.py`) drives the gate of a MOSFET (or a relay coil)
   placed in series with the same +V rail, downstream of the manual
   E-stop switch. The Pico holds this line HIGH only while it's
   receiving heartbeats from the Pi; on boot, on losing heartbeats, or
   if the Pico itself loses power or crashes, the line defaults LOW and
   cuts power.

```
Battery (+) --[Manual E-Stop]--[Pico-driven MOSFET/relay]--[Motor Driver +V]
                                        ^
                                Pico GPIO15 (ENABLE_PIN_NUM)
```

Either switch opening (manual button, or Pico dropping `ENABLE_PIN_NUM`
low) cuts motor power. They're in series, so both must be "closed"
(button not pressed, AND Pico actively heartbeat-fed) for motors to
receive power.

## Fail-safe requirement

The Pico's GPIO must default LOW (motors disabled) on boot and stay LOW
whenever it isn't actively receiving heartbeats — see
`firmware/watchdog_logic.py`'s `Watchdog.is_tripped()`, which returns
`True` before any heartbeat is ever received. A Pico that is unpowered,
crashed, or freshly booted fails toward "motors off," not "motors on."

## Not yet decided / deferred

- Exact MOSFET/relay part number and gate-drive circuit — depends on the
  salvaged motor driver's voltage/current draw, which won't be known
  until vacuums are stripped (`README.md` §6 step 1).
- Whether the cutoff needs to be latching (requiring manual reset) rather
  than auto-recovering once heartbeats resume — see
  `../README.md`'s "Latching vs auto-recovery" note. Current firmware
  auto-recovers; revisit at the hardware bring-up checkpoint (roadmap
  design §3 step 4) once real-world testing exists.

## Firmware bring-up (manual verification, once a Pico is wired up)

1. Flash `firmware/watchdog_logic.py`, `firmware/heartbeat_framing.py`,
   and `firmware/main.py` onto the Pico (e.g. `mpremote cp *.py :` from
   inside `firmware/`, or via Thonny's "Save as > Raspberry Pi Pico").
2. Connect the Pico over USB; note its serial device path
   (`ls /dev/ttyACM*`).
3. Wire an LED + resistor from `ENABLE_PIN_NUM` (GPIO15) to ground as a
   stand-in for the real MOSFET/relay, so trip state is visible without
   a motor driver attached yet.
4. Reset the Pico. Confirm the LED is OFF (fail-safe default — no
   heartbeat received yet).
5. From the PC, run:
   `python3 src/walker_safety/tools/send_fake_heartbeats.py /dev/ttyACM0`
   Confirm the LED turns ON within `TIMEOUT_S` (0.5s).
6. Stop the script (Ctrl+C). Confirm the LED turns back OFF within
   `TIMEOUT_S` of the last heartbeat.
```

- [ ] **Step 3: Write the package README**

Create `src/walker_safety/README.md`:

```markdown
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
```

- [ ] **Step 4: Review for placeholders**

Read both files back and confirm neither contains "TBD"/"TODO" as an
unresolved placeholder (the "Not yet decided / deferred" section is
intentional content, not a placeholder — it names exactly what's
deferred and why).

- [ ] **Step 5: Commit**

```bash
git add src/walker_safety/README.md src/walker_safety/docs/e_stop_wiring.md
git commit -m "$(cat <<'EOF'
Add walker_safety E-stop wiring doc and package README

Documents the physical E-stop topology and the Pico watchdog's
fail-safe GPIO requirement per the Phase 1 roadmap design, before any
motor hardware exists.
EOF
)"
```

---

## Task 2: Watchdog Timeout Logic (TDD)

**Files:**
- Create: `src/walker_safety/firmware/watchdog_logic.py`
- Create: `src/walker_safety/firmware/tests/conftest.py`
- Test: `src/walker_safety/firmware/tests/test_watchdog_logic.py`

**Interfaces:**
- Produces: `Watchdog(timeout_s: float)` — raises `ValueError` if `timeout_s <= 0`. Methods: `on_heartbeat(now_s: float) -> None`, `is_tripped(now_s: float) -> bool`. Consumed by Task 4 (`main.py`).

- [ ] **Step 1: Confirm pytest is available**

```bash
python3 -m pytest --version
```

If this errors with "No module named pytest", install it without sudo:

```bash
python3 -m pip install --user pytest
```

- [ ] **Step 2: Write the test-path conftest**

Create `src/walker_safety/firmware/tests/conftest.py` — inserts the
`firmware/` directory (parent of `tests/`) onto `sys.path` so tests can
`import watchdog_logic` directly, without needing a `setup.py`:

```python
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
```

- [ ] **Step 3: Write the failing tests**

Create `src/walker_safety/firmware/tests/test_watchdog_logic.py`:

```python
import pytest

from watchdog_logic import Watchdog


def test_no_heartbeat_received_is_tripped():
    wd = Watchdog(timeout_s=0.5)
    assert wd.is_tripped(now_s=0.0) is True


def test_recent_heartbeat_is_not_tripped():
    wd = Watchdog(timeout_s=0.5)
    wd.on_heartbeat(now_s=10.0)
    assert wd.is_tripped(now_s=10.2) is False


def test_stale_heartbeat_is_tripped():
    wd = Watchdog(timeout_s=0.5)
    wd.on_heartbeat(now_s=10.0)
    assert wd.is_tripped(now_s=10.6) is True


def test_exactly_at_timeout_boundary_is_tripped():
    wd = Watchdog(timeout_s=0.5)
    wd.on_heartbeat(now_s=10.0)
    assert wd.is_tripped(now_s=10.5) is True


def test_new_heartbeat_recovers_from_trip():
    wd = Watchdog(timeout_s=0.5)
    wd.on_heartbeat(now_s=10.0)
    assert wd.is_tripped(now_s=10.6) is True  # tripped
    wd.on_heartbeat(now_s=100.0)
    assert wd.is_tripped(now_s=100.0) is False  # recovered


def test_repeated_heartbeats_use_latest_time():
    wd = Watchdog(timeout_s=0.5)
    wd.on_heartbeat(now_s=1.0)
    wd.on_heartbeat(now_s=2.0)
    assert wd.is_tripped(now_s=2.3) is False  # 0.3s since latest (2.0)
    assert wd.is_tripped(now_s=2.6) is True


def test_negative_or_zero_timeout_rejected():
    with pytest.raises(ValueError):
        Watchdog(timeout_s=0)
    with pytest.raises(ValueError):
        Watchdog(timeout_s=-1.0)
```

- [ ] **Step 4: Run tests to verify they fail**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_safety/firmware
python3 -m pytest tests/test_watchdog_logic.py -v
```

Expected: `ModuleNotFoundError: No module named 'watchdog_logic'` (the
module doesn't exist yet).

- [ ] **Step 5: Implement the minimal watchdog logic**

Create `src/walker_safety/firmware/watchdog_logic.py`:

```python
"""Pure heartbeat-timeout logic for the walker_safety watchdog.

No hardware or MicroPython-specific imports here — this module is
shared between the Pico firmware (main.py) and the desktop pytest
suite, so the same logic that runs on real hardware is exactly what
the tests exercise.
"""


class Watchdog:
    """Tracks whether motors should be cut based on heartbeat recency.

    Fails safe: before the first heartbeat is received, is_tripped()
    always returns True (motors disabled). This matches README.md
    Sec 5.4's requirement that the watchdog halts motors on any loss
    of heartbeat signal, including "never started."
    """

    def __init__(self, timeout_s):
        if timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        self.timeout_s = timeout_s
        self._last_heartbeat_s = None

    def on_heartbeat(self, now_s):
        """Record a heartbeat received at now_s (seconds, monotonic)."""
        self._last_heartbeat_s = now_s

    def is_tripped(self, now_s):
        """Return True if motors should be cut given the current time."""
        if self._last_heartbeat_s is None:
            return True
        elapsed_s = now_s - self._last_heartbeat_s
        return elapsed_s >= self.timeout_s
```

- [ ] **Step 6: Run tests to verify they pass**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_safety/firmware
python3 -m pytest tests/test_watchdog_logic.py -v
```

Expected: 7 passed.

- [ ] **Step 7: Commit**

```bash
git add src/walker_safety/firmware/watchdog_logic.py \
        src/walker_safety/firmware/tests/conftest.py \
        src/walker_safety/firmware/tests/test_watchdog_logic.py
git commit -m "$(cat <<'EOF'
Add walker_safety watchdog timeout logic with tests

Pure-Python heartbeat/timeout core, unit-tested against fake
timestamps with no hardware or serial port needed. main.py (Task 4)
wires this to real GPIO on the Pico.
EOF
)"
```

---

## Task 3: Heartbeat Wire-Format Helper (TDD)

**Files:**
- Create: `src/walker_safety/firmware/heartbeat_framing.py`
- Test: `src/walker_safety/firmware/tests/test_heartbeat_framing.py`

**Interfaces:**
- Produces: `HEARTBEAT_BYTE: bytes` (`b"\x01"`), `is_heartbeat_byte(data: bytes) -> bool`. Consumed by Task 4 (`main.py`) and Task 4's PC-side sender script.

- [ ] **Step 1: Write the failing tests**

Create `src/walker_safety/firmware/tests/test_heartbeat_framing.py`:

```python
from heartbeat_framing import HEARTBEAT_BYTE, is_heartbeat_byte


def test_recognizes_heartbeat_byte():
    assert is_heartbeat_byte(HEARTBEAT_BYTE) is True


def test_rejects_different_byte():
    assert is_heartbeat_byte(b"\x00") is False


def test_rejects_empty_bytes():
    assert is_heartbeat_byte(b"") is False


def test_rejects_multi_byte_payload():
    assert is_heartbeat_byte(b"\x01\x01") is False
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_safety/firmware
python3 -m pytest tests/test_heartbeat_framing.py -v
```

Expected: `ModuleNotFoundError: No module named 'heartbeat_framing'`.

- [ ] **Step 3: Implement the minimal framing helper**

Create `src/walker_safety/firmware/heartbeat_framing.py`:

```python
"""Wire-format helpers for the watchdog's heartbeat protocol.

Kept separate from main.py so the byte-level protocol can be unit
tested on the desktop without a Pico attached, same rationale as
watchdog_logic.py.
"""

HEARTBEAT_BYTE = b"\x01"


def is_heartbeat_byte(data):
    """Return True if data is exactly one heartbeat marker byte."""
    return data == HEARTBEAT_BYTE
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_safety/firmware
python3 -m pytest tests/test_heartbeat_framing.py -v
```

Expected: 4 passed.

- [ ] **Step 5: Commit**

```bash
git add src/walker_safety/firmware/heartbeat_framing.py \
        src/walker_safety/firmware/tests/test_heartbeat_framing.py
git commit -m "$(cat <<'EOF'
Add walker_safety heartbeat wire-format helper with tests

Single-byte heartbeat marker, isolated from main.py so the protocol
is unit-testable without a Pico attached.
EOF
)"
```

---

## Task 4: Pico Firmware Entry Point + Manual Verification Tooling

**Files:**
- Create: `src/walker_safety/firmware/main.py`
- Create: `src/walker_safety/tools/send_fake_heartbeats.py`

**Interfaces:**
- Consumes: `Watchdog` from `watchdog_logic.py` (Task 2); `is_heartbeat_byte` from `heartbeat_framing.py` (Task 3).
- Produces: nothing consumed by a later task in this plan — this is the top of the current build order. A future `walker_motor_driver` plan will wire the physical `ENABLE_PIN_NUM` signal into the real motor driver's power gating, not a Python interface.

- [ ] **Step 1: Write the Pico firmware entry point**

Create `src/walker_safety/firmware/main.py`. This cannot be unit tested
with pytest — it imports `machine`, which exists only in MicroPython
running on real hardware — so verify it per the "Firmware bring-up"
section of `docs/e_stop_wiring.md` (Task 1) once a Pico is available:

```python
"""Pico watchdog firmware entry point.

Runs standalone on the Pico, independent of the Pi/ROS2 stack per
docs/superpowers/specs/2026-08-30-phase1-roadmap-design.md Sec 2.2.
Reads single-byte heartbeats over USB serial (stdin) and drives
ENABLE_PIN_NUM high only while heartbeats are recent; on boot, and on
any gap longer than TIMEOUT_S, the pin is driven low (motors disabled).

Not unit-testable on the desktop (uses the `machine` module, which
only exists in MicroPython on-device) - verify manually per
../docs/e_stop_wiring.md's "Firmware bring-up" section.
"""
import sys
import time

try:
    import uselect as select
except ImportError:
    import select

from machine import Pin

from heartbeat_framing import is_heartbeat_byte
from watchdog_logic import Watchdog

ENABLE_PIN_NUM = 15
TIMEOUT_S = 0.5
POLL_INTERVAL_S = 0.05


def main():
    enable_pin = Pin(ENABLE_PIN_NUM, Pin.OUT)
    enable_pin.value(0)  # fail-safe: motors disabled until first heartbeat
    watchdog = Watchdog(timeout_s=TIMEOUT_S)
    poller = select.poll()
    poller.register(sys.stdin, select.POLLIN)

    while True:
        now = time.time()
        if poller.poll(0):
            byte = sys.stdin.buffer.read(1)
            if is_heartbeat_byte(byte):
                watchdog.on_heartbeat(now)
        enable_pin.value(0 if watchdog.is_tripped(now) else 1)
        time.sleep(POLL_INTERVAL_S)


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Syntax-check main.py without a Pico attached**

`py_compile` only checks syntax, not imports, so this succeeds even
though `machine` isn't installed on the desktop:

```bash
python3 -m py_compile /home/peter_sha/sourcecode/smart-walker-bot/src/walker_safety/firmware/main.py
```

Expected: no output, exit code 0.

- [ ] **Step 3: Install pyserial for the manual-verification tool**

```bash
python3 -m pip install --user pyserial
```

- [ ] **Step 4: Write the PC-side fake-heartbeat sender**

Create `src/walker_safety/tools/send_fake_heartbeats.py`:

```python
"""Manual test helper: sends a fake heartbeat byte to a Pico running
walker_safety firmware, at a fixed interval, so a human can verify the
watchdog trips/recovers correctly (e.g. by watching an LED/multimeter
on ENABLE_PIN_NUM) without needing the full ROS2 stack running.

Usage: python3 send_fake_heartbeats.py /dev/ttyACM0 --interval 0.2
Stop sending (Ctrl+C) to observe the watchdog trip after TIMEOUT_S.
"""
import argparse
import time

import serial

HEARTBEAT_BYTE = b"\x01"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="Serial port, e.g. /dev/ttyACM0")
    parser.add_argument(
        "--interval", type=float, default=0.2, help="Seconds between heartbeats"
    )
    args = parser.parse_args()

    with serial.Serial(args.port, baudrate=115200, timeout=1) as ser:
        print(f"Sending heartbeats to {args.port} every {args.interval}s. Ctrl+C to stop.")
        while True:
            ser.write(HEARTBEAT_BYTE)
            time.sleep(args.interval)


if __name__ == "__main__":
    main()
```

- [ ] **Step 5: Syntax-check the sender script**

```bash
python3 -m py_compile /home/peter_sha/sourcecode/smart-walker-bot/src/walker_safety/tools/send_fake_heartbeats.py
```

Expected: no output, exit code 0.

- [ ] **Step 6: Commit**

```bash
git add src/walker_safety/firmware/main.py src/walker_safety/tools/send_fake_heartbeats.py
git commit -m "$(cat <<'EOF'
Add walker_safety Pico firmware entry point and manual test tool

main.py wires watchdog_logic + heartbeat_framing to real GPIO/serial
on the Pico; not unit-testable without hardware, so it's paired with
a PC-side fake-heartbeat sender for manual verification per
docs/e_stop_wiring.md's "Firmware bring-up" section.
EOF
)"
```

- [ ] **Step 7: Manual hardware verification (human, once a Pico is on hand)**

Follow `src/walker_safety/docs/e_stop_wiring.md`'s "Firmware bring-up"
section (flash the three firmware files, wire an LED to `ENABLE_PIN_NUM`,
run `send_fake_heartbeats.py`, confirm LED on/off behavior). This step
cannot be automated in this environment — no Pico is attached — and is
the actual acceptance test for this package's firmware, so don't
consider `walker_safety` done until it's been run at least once.

---

## Self-Review Notes

- **Spec coverage:** §3 step 1 ("E-stop wiring documentation, plus
  watchdog firmware for the Pico, developed and tested against a fake
  heartbeat source") — Task 1 covers the docs, Tasks 2–4 cover the
  firmware and its fake-heartbeat-based tests/tooling. §2.2 (Pico
  independence, fail-safe default) — encoded in `Watchdog.is_tripped()`
  and the wiring doc. §2.3 (rootfs/power hardening), §2.1 (board choice)
  — explicitly out of scope, called out in Global Constraints, not
  silently dropped.
- **Placeholder scan:** no TBD/TODO in any step; the wiring doc's "Not
  yet decided" section names concrete, genuinely-deferred hardware
  specifics rather than standing in for unwritten content.
- **Type/name consistency:** `ENABLE_PIN_NUM = 15` and `TIMEOUT_S = 0.5`
  match between the wiring doc (Task 1) and `main.py` (Task 4).
  `Watchdog(timeout_s).on_heartbeat(now_s)/.is_tripped(now_s)` and
  `is_heartbeat_byte(data)` are used identically everywhere they're
  referenced across Tasks 2–4.
