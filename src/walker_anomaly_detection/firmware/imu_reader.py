"""ESP32 firmware (MicroPython): reads an MPU-9250 9-axis IMU over I2C
and streams JSON-line samples over USB serial (MicroPython's REPL/UART0
doubles as the USB-serial connection on typical ESP32 boards). No
detection logic here - untestable except on real hardware, mirrors
walker_safety/firmware/main.py's role as the hardware-facing entry
point with no pure logic of its own. See
docs/superpowers/specs/2026-08-31-walker-anomaly-detection-design.md
Sec 2.2, 2.3.

Targets the MPU-9250 register map specifically. If the actual chip in
hand turns out to be an ICM-20948 instead, this file's register
addresses and scaling need to be swapped for that chip's (different)
register map - the exact chip wasn't confirmed before this was
written (design spec Sec 1), same "placeholder now, adjust at bring-up"
treatment walker_motor_driver's physical constants got.
"""
import time

import ujson
from machine import I2C, Pin

_MPU9250_ADDR = 0x68
_PWR_MGMT_1 = 0x6B
_ACCEL_XOUT_H = 0x3B
_ACCEL_FS_SCALE_LSB_PER_G = 16384.0        # default +/-2g full-scale range
_GYRO_FS_SCALE_LSB_PER_DEG_S = 131.0       # default +/-250 deg/s full-scale range

_SAMPLE_INTERVAL_MS = 20  # ~50 Hz


def _read_word_signed(i2c, addr, reg):
    high, low = i2c.readfrom_mem(addr, reg, 2)
    value = (high << 8) | low
    if value >= 0x8000:
        value -= 0x10000
    return value


def _wake_up(i2c):
    i2c.writeto_mem(_MPU9250_ADDR, _PWR_MGMT_1, bytes([0x00]))


def _read_sample(i2c):
    ax_raw = _read_word_signed(i2c, _MPU9250_ADDR, _ACCEL_XOUT_H)
    ay_raw = _read_word_signed(i2c, _MPU9250_ADDR, _ACCEL_XOUT_H + 2)
    az_raw = _read_word_signed(i2c, _MPU9250_ADDR, _ACCEL_XOUT_H + 4)
    # ACCEL_XOUT_H + 6/7 is temperature (2 bytes) - skipped, not used.
    gx_raw = _read_word_signed(i2c, _MPU9250_ADDR, _ACCEL_XOUT_H + 8)
    gy_raw = _read_word_signed(i2c, _MPU9250_ADDR, _ACCEL_XOUT_H + 10)
    gz_raw = _read_word_signed(i2c, _MPU9250_ADDR, _ACCEL_XOUT_H + 12)

    return {
        'ax': ax_raw / _ACCEL_FS_SCALE_LSB_PER_G,
        'ay': ay_raw / _ACCEL_FS_SCALE_LSB_PER_G,
        'az': az_raw / _ACCEL_FS_SCALE_LSB_PER_G,
        'gx': gx_raw / _GYRO_FS_SCALE_LSB_PER_DEG_S,
        'gy': gy_raw / _GYRO_FS_SCALE_LSB_PER_DEG_S,
        'gz': gz_raw / _GYRO_FS_SCALE_LSB_PER_DEG_S,
        # Magnetometer (AK8963, behind the MPU-9250's I2C bypass) isn't
        # wired up this first pass - stream zeros so the JSON shape
        # always matches the design spec's protocol (Sec 2.3), even
        # though nothing consumes these fields yet (Sec 2.3's own note).
        'mx': 0.0,
        'my': 0.0,
        'mz': 0.0,
        't_ms': time.ticks_ms(),
    }


def main():
    i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
    _wake_up(i2c)
    time.sleep_ms(100)

    while True:
        sample = _read_sample(i2c)
        print(ujson.dumps(sample))
        time.sleep_ms(_SAMPLE_INTERVAL_MS)


if __name__ == '__main__':
    main()
