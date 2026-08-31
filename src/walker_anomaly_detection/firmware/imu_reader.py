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

The accelerometer (+/-8g) and gyro (+/-500 deg/s) full-scale ranges are
explicitly written to ACCEL_CONFIG/GYRO_CONFIG at startup (see
_configure_ranges()) rather than left at the chip's power-on-reset
default (+/-2g) - see docs/bring_up.md for why the accelerometer range
and anomaly_detection_node.py's impact_threshold_g parameter must be
considered together whenever either changes.
"""
import time

import ujson
from machine import I2C, Pin

_MPU9250_ADDR = 0x68
_PWR_MGMT_1 = 0x6B
_ACCEL_CONFIG = 0x1C
_GYRO_CONFIG = 0x1B
_ACCEL_XOUT_H = 0x3B

# Full-scale ranges are explicitly configured below (not left at the
# chip's power-on-reset default) - see _configure_ranges(). AFS_SEL=2
# selects +/-8g; FS_SEL=1 selects +/-500 deg/s.
_ACCEL_FS_SCALE_LSB_PER_G = 4096.0         # +/-8g full-scale range (AFS_SEL=2)
_GYRO_FS_SCALE_LSB_PER_DEG_S = 65.5        # +/-500 deg/s full-scale range (FS_SEL=1)

_SAMPLE_INTERVAL_MS = 20  # ~50 Hz


def _decode_signed_word(high, low):
    value = (high << 8) | low
    if value >= 0x8000:
        value -= 0x10000
    return value


def _wake_up(i2c):
    i2c.writeto_mem(_MPU9250_ADDR, _PWR_MGMT_1, bytes([0x00]))


def _configure_ranges(i2c):
    """Explicitly set the accelerometer and gyro full-scale ranges,
    rather than relying on the chip's power-on-reset default (+/-2g),
    which saturates at ~1.9999g - too low for impact_threshold_g
    (anomaly_detection_node.py's default is 2.0g) to ever be reachable.
    AFS_SEL=2 (bits 4:3 = 0b10 = 0x10) selects +/-8g.
    FS_SEL=1 (bits 4:3 = 0b01 = 0x08) selects +/-500 deg/s."""
    i2c.writeto_mem(_MPU9250_ADDR, _ACCEL_CONFIG, bytes([0x10]))
    i2c.writeto_mem(_MPU9250_ADDR, _GYRO_CONFIG, bytes([0x08]))


def _read_sample(i2c):
    # Read all 14 output registers (accel x/y/z, temp, gyro x/y/z) in a
    # single atomic I2C transaction rather than six separate 2-byte
    # reads - the output registers can update between separate reads,
    # so six independent reads could straddle two different sensor
    # instants and inject noise directly into the accel magnitude the
    # fall detector keys on.
    raw = i2c.readfrom_mem(_MPU9250_ADDR, _ACCEL_XOUT_H, 14)

    ax_raw = _decode_signed_word(raw[0], raw[1])
    ay_raw = _decode_signed_word(raw[2], raw[3])
    az_raw = _decode_signed_word(raw[4], raw[5])
    # raw[6:8] is temperature - skipped, not used.
    gx_raw = _decode_signed_word(raw[8], raw[9])
    gy_raw = _decode_signed_word(raw[10], raw[11])
    gz_raw = _decode_signed_word(raw[12], raw[13])

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
    _configure_ranges(i2c)
    time.sleep_ms(100)

    while True:
        try:
            sample = _read_sample(i2c)
            print(ujson.dumps(sample))
        except OSError as e:
            # A loose jumper or brief brownout shouldn't crash the
            # firmware to the REPL and permanently end the data stream.
            # This print goes out over the same USB serial connection
            # as JSON samples, but since it isn't valid JSON,
            # imu_serial.py's parse_sample_line will just skip it.
            print('I2C read error, retrying:', e)
        time.sleep_ms(_SAMPLE_INTERVAL_MS)


if __name__ == '__main__':
    main()
