import math

import pytest

from walker_anomaly_detection.tilt_detector import TiltDetector, tilt_from_accel_deg

TILT_THRESHOLD_DEG = 45.0
TILT_SUSTAINED_DURATION_S = 3.0


def _make_detector():
    return TiltDetector(
        tilt_threshold_deg=TILT_THRESHOLD_DEG,
        tilt_sustained_duration_s=TILT_SUSTAINED_DURATION_S,
    )


def test_tilt_from_accel_deg_upright_is_zero():
    assert tilt_from_accel_deg(0.0, 0.0, 9.8) == pytest.approx(0.0, abs=1e-6)


def test_tilt_from_accel_deg_on_its_side_is_90():
    assert tilt_from_accel_deg(9.8, 0.0, 0.0) == pytest.approx(90.0, rel=1e-6)


def test_tilt_from_accel_deg_45_degrees():
    value = 9.8 / math.sqrt(2)
    assert tilt_from_accel_deg(value, 0.0, value) == pytest.approx(45.0, rel=1e-6)


def test_upright_never_triggers():
    detector = _make_detector()
    for t in range(0, 10):
        assert detector.update(0.0, float(t)) is False


def test_tilt_below_threshold_never_triggers():
    detector = _make_detector()
    for t in range(0, 10):
        assert detector.update(30.0, float(t)) is False


def test_sustained_tilt_triggers_once_after_duration():
    detector = _make_detector()
    assert detector.update(60.0, 0.0) is False
    assert detector.update(60.0, 1.0) is False
    assert detector.update(60.0, 3.0) is True
    assert detector.update(60.0, 4.0) is False  # doesn't re-trigger while still tilted


def test_transient_tilt_recovering_before_duration_does_not_trigger():
    detector = _make_detector()
    detector.update(60.0, 0.0)
    detector.update(60.0, 1.0)
    assert detector.update(20.0, 2.0) is False
    assert detector.update(60.0, 2.1) is False  # fresh start, not enough time elapsed yet


def test_retriggers_after_recovering_and_tilting_again():
    detector = _make_detector()
    detector.update(60.0, 0.0)
    detector.update(60.0, 1.0)
    assert detector.update(60.0, 3.0) is True
    detector.update(20.0, 3.5)  # recovers upright
    detector.update(60.0, 3.6)
    assert detector.update(60.0, 6.6) is True
