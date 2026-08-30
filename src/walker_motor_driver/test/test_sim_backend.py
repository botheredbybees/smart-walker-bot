import pytest

from walker_motor_driver.sim_backend import SimMotorBackend


def test_zero_speed_gives_zero_delta():
    backend = SimMotorBackend(now_s=0.0)
    backend.apply_wheel_speeds(0.0, 0.0)
    left, right = backend.read_wheel_deltas(now_s=1.0)
    assert left == pytest.approx(0.0, abs=1e-9)
    assert right == pytest.approx(0.0, abs=1e-9)


def test_constant_speed_gives_proportional_delta():
    backend = SimMotorBackend(now_s=0.0)
    backend.apply_wheel_speeds(2.0, 3.0)
    left, right = backend.read_wheel_deltas(now_s=1.0)
    assert left == pytest.approx(2.0, rel=1e-6)
    assert right == pytest.approx(3.0, rel=1e-6)


def test_successive_reads_only_count_new_elapsed_time():
    backend = SimMotorBackend(now_s=0.0)
    backend.apply_wheel_speeds(1.0, 1.0)
    first_left, first_right = backend.read_wheel_deltas(now_s=1.0)
    second_left, second_right = backend.read_wheel_deltas(now_s=2.0)
    assert first_left == pytest.approx(1.0, rel=1e-6)
    assert first_right == pytest.approx(1.0, rel=1e-6)
    assert second_left == pytest.approx(1.0, rel=1e-6)
    assert second_right == pytest.approx(1.0, rel=1e-6)


def test_speed_change_only_affects_subsequent_interval():
    backend = SimMotorBackend(now_s=0.0)
    backend.apply_wheel_speeds(1.0, 1.0)
    backend.read_wheel_deltas(now_s=1.0)
    backend.apply_wheel_speeds(5.0, 5.0)
    left, right = backend.read_wheel_deltas(now_s=2.0)
    assert left == pytest.approx(5.0, rel=1e-6)
    assert right == pytest.approx(5.0, rel=1e-6)


def test_asymmetric_speeds_give_independent_deltas():
    backend = SimMotorBackend(now_s=0.0)
    backend.apply_wheel_speeds(1.0, -2.0)
    left, right = backend.read_wheel_deltas(now_s=1.0)
    assert left == pytest.approx(1.0, rel=1e-6)
    assert right == pytest.approx(-2.0, rel=1e-6)


def test_backwards_time_rejected():
    backend = SimMotorBackend(now_s=10.0)
    with pytest.raises(ValueError):
        backend.read_wheel_deltas(now_s=5.0)
