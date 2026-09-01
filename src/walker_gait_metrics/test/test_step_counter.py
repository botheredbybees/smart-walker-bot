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
