# walker_anomaly_detection Hardware Bring-Up

Not yet done as of this package's initial build — wiring the ESP32 is "a small project of its
own." This document records what's needed when that happens.

## Hardware

- An ESP32 dev board, flashed with MicroPython.
- An MPU-9250-style 9-axis IMU breakout (or ICM-20948 — see `firmware/imu_reader.py`'s header
  comment if so; the register map differs and the firmware needs adjusting).

## Wiring (I2C)

| IMU pin | ESP32 pin |
|---|---|
| VCC | 3.3V |
| GND | GND |
| SCL | GPIO22 (default I2C0 SCL) |
| SDA | GPIO21 (default I2C0 SDA) |

Adjust `firmware/imu_reader.py`'s `Pin(22)`/`Pin(21)` if using different GPIOs.

## Flashing MicroPython

1. Install `esptool` and flash the MicroPython firmware for ESP32 (see micropython.org's ESP32
   download page for the current `.bin`).
2. Copy `firmware/imu_reader.py` onto the device as `main.py` so it runs automatically on boot
   (e.g. `mpremote cp firmware/imu_reader.py :main.py`, or `ampy`/`rshell`).
3. Connect the ESP32 to this workstation via USB — it should enumerate as `/dev/ttyUSB0` or
   `/dev/ttyACM0` (check `dmesg` after plugging in). Update the `serial_port` launch argument if
   it's different from the default.

## Verifying the sensor itself works

Once wired and flashed:

```bash
# from this workstation, with nothing else using the serial port
python3 -c "
import serial
s = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)
for _ in range(10):
    print(s.readline().decode().strip())
"
```

Expected: ten lines of JSON, each with `ax`/`ay`/`az` roughly summing to ~1g in magnitude when the
board is stationary (e.g. `az` close to 1.0 if the IMU is lying flat, others close to 0.0). If
values look wildly wrong (all zeros, all identical, or magnitude far from 1g), check the I2C
wiring and the `_MPU9250_ADDR`/register constants in `firmware/imu_reader.py` against your
specific breakout board's datasheet.

Once real samples look sane:

```bash
ros2 launch walker_anomaly_detection anomaly_detection.launch.py
```

and, in another terminal, `ros2 topic echo /anomaly_detected` while manually performing a real
drop/catch motion and a real sustained-tilt motion with the IMU in hand, confirming each produces
the expected `fall`/`tilt` event. This manual step is the only thing
`tools/verify_anomaly_detection.py`'s automated `pty`-based check (design spec §2.10) can't
cover — it proves the node's wiring works, not that a real accelerometer produces sensible
values.
