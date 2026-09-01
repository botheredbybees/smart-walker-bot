from walker_gait_metrics.step_counter import StepCounter

STEP_THRESHOLD_G = 1.2
MIN_STEP_INTERVAL_S = 0.3


def _make_counter():
    return StepCounter(step_threshold_g=STEP_THRESHOLD_G, min_step_interval_s=MIN_STEP_INTERVAL_S)


def test_values_below_threshold_never_trigger():
    counter = _make_counter()
    for t in (0.0, 0.1, 0.2, 0.3):
        assert counter.update(1.0, t) is False


def test_first_crossing_triggers_immediately():
    counter = _make_counter()
    assert counter.update(1.5, 0.0) is True


def test_crossings_spaced_past_min_interval_each_count():
    counter = _make_counter()
    assert counter.update(1.5, 0.0) is True
    assert counter.update(1.5, 0.3) is True
    assert counter.update(1.5, 0.6) is True


def test_crossings_closer_than_min_interval_count_once():
    counter = _make_counter()
    assert counter.update(1.5, 0.0) is True
    assert counter.update(1.5, 0.1) is False   # only 0.1s since last step, debounced
    assert counter.update(1.5, 0.29) is False  # still within debounce window


def test_crossing_exactly_at_min_interval_boundary_counts():
    counter = _make_counter()
    counter.update(1.5, 0.0)
    assert counter.update(1.5, 0.3) is True  # exactly min_step_interval_s later


def test_a_debounced_sample_does_not_reset_the_debounce_window():
    counter = _make_counter()
    counter.update(1.5, 0.0)                   # step 1
    counter.update(1.5, 0.1)                   # debounced, must not move the "last step" time
    assert counter.update(1.5, 0.35) is True   # 0.35s since step 1 (not since the debounced 0.1)


def test_floating_point_boundary_case_at_decimal_timestamps():
    """Regression test for FP precision issue: 1.2 - 0.9 = 0.29999999999999993 < 0.3
    With epsilon tolerance, this boundary case must count as a step, not debounce."""
    counter = _make_counter()
    counter.update(1.5, 0.0)   # step 1 at t=0.0
    counter.update(1.5, 0.3)   # step 2 at t=0.3
    counter.update(1.5, 0.6)   # step 3 at t=0.6
    counter.update(1.5, 0.9)   # step 4 at t=0.9
    # The critical case: t=1.2 should count even though 1.2 - 0.9 has FP precision loss
    assert counter.update(1.5, 1.2) is True   # step 5 must not be debounced
