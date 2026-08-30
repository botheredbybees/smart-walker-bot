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
