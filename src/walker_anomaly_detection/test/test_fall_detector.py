from walker_anomaly_detection.fall_detector import FallDetector

FREE_FALL_THRESHOLD_G = 0.3
FREE_FALL_MIN_DURATION_S = 0.05
IMPACT_THRESHOLD_G = 2.0
IMPACT_WINDOW_S = 0.5


def _make_detector():
    return FallDetector(
        free_fall_threshold_g=FREE_FALL_THRESHOLD_G,
        free_fall_min_duration_s=FREE_FALL_MIN_DURATION_S,
        impact_threshold_g=IMPACT_THRESHOLD_G,
        impact_window_s=IMPACT_WINDOW_S,
    )


def test_normal_readings_never_trigger():
    detector = _make_detector()
    for t in (0.0, 0.1, 0.2, 0.3):
        assert detector.update(1.0, t) is False


def test_free_fall_then_immediate_impact_triggers():
    detector = _make_detector()
    assert detector.update(1.0, 0.00) is False
    assert detector.update(0.1, 0.01) is False
    assert detector.update(0.1, 0.02) is False
    assert detector.update(0.1, 0.07) is False  # min_duration reached, still below threshold
    assert detector.update(2.5, 0.15) is True   # impact spike right as free-fall ends


def test_free_fall_then_delayed_impact_within_window_triggers():
    detector = _make_detector()
    detector.update(1.0, 0.00)
    detector.update(0.1, 0.01)
    detector.update(0.1, 0.07)  # confirms free-fall
    assert detector.update(1.0, 0.15) is False   # recovers, but not an impact yet
    assert detector.update(3.0, 0.20) is True    # impact within window


def test_free_fall_with_no_impact_within_window_does_not_trigger():
    detector = _make_detector()
    detector.update(1.0, 0.00)
    detector.update(0.1, 0.01)
    detector.update(0.1, 0.07)  # confirms free-fall
    detector.update(1.0, 0.15)  # recovers, normal reading, window starts
    assert detector.update(1.0, 0.70) is False  # window (0.5s) has elapsed, no impact


def test_brief_dip_below_min_duration_never_confirms_and_does_not_trigger():
    detector = _make_detector()
    detector.update(1.0, 0.00)
    detector.update(0.1, 0.01)   # dips below threshold
    detector.update(0.1, 0.02)   # still below, but duration (0.01s) < min_duration (0.05s)
    detector.update(1.0, 0.03)   # recovers before free-fall confirmed
    assert detector.update(5.0, 0.04) is False  # even a huge spike right after doesn't trigger


def test_impact_spike_without_preceding_free_fall_does_not_trigger():
    detector = _make_detector()
    detector.update(1.0, 0.00)
    assert detector.update(5.0, 0.01) is False


def test_state_resets_after_a_confirmed_fall_so_a_second_fall_can_be_detected():
    detector = _make_detector()
    detector.update(1.0, 0.00)
    detector.update(0.1, 0.01)
    detector.update(0.1, 0.07)
    assert detector.update(2.5, 0.15) is True

    # second, independent fall sequence
    detector.update(1.0, 1.00)
    detector.update(0.1, 1.01)
    detector.update(0.1, 1.07)
    assert detector.update(2.5, 1.15) is True
