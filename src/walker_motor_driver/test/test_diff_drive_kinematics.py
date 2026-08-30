import math

import pytest

from walker_motor_driver.diff_drive_kinematics import (
    OdometryTracker,
    clamp_wheel_speeds,
    twist_to_wheel_speeds,
    yaw_to_quaternion,
)

WHEEL_RADIUS_M = 0.03
WHEEL_SEPARATION_M = 0.2


def test_straight_line_gives_equal_wheel_speeds():
    left, right = twist_to_wheel_speeds(1.0, 0.0, WHEEL_RADIUS_M, WHEEL_SEPARATION_M)
    assert left == pytest.approx(33.3333333, rel=1e-6)
    assert right == pytest.approx(33.3333333, rel=1e-6)


def test_pure_rotation_gives_opposite_wheel_speeds():
    left, right = twist_to_wheel_speeds(0.0, 1.0, WHEEL_RADIUS_M, WHEEL_SEPARATION_M)
    assert left == pytest.approx(-3.3333333, rel=1e-6)
    assert right == pytest.approx(3.3333333, rel=1e-6)


def test_combined_motion_gives_asymmetric_wheel_speeds():
    left, right = twist_to_wheel_speeds(1.0, 1.0, WHEEL_RADIUS_M, WHEEL_SEPARATION_M)
    assert left == pytest.approx(30.0, rel=1e-6)
    assert right == pytest.approx(36.6666667, rel=1e-6)


def test_twist_zero_wheel_radius_rejected():
    with pytest.raises(ValueError):
        twist_to_wheel_speeds(1.0, 0.0, 0.0, WHEEL_SEPARATION_M)


def test_twist_negative_wheel_separation_rejected():
    with pytest.raises(ValueError):
        twist_to_wheel_speeds(1.0, 0.0, WHEEL_RADIUS_M, -0.1)


def test_clamp_within_limit_unchanged():
    left, right = clamp_wheel_speeds(5.0, -3.0, 10.0)
    assert left == pytest.approx(5.0)
    assert right == pytest.approx(-3.0)


def test_clamp_symmetric_saturation_scales_both_equally():
    left, right = clamp_wheel_speeds(20.0, 20.0, 10.0)
    assert left == pytest.approx(10.0)
    assert right == pytest.approx(10.0)


def test_clamp_asymmetric_saturation_preserves_ratio():
    left, right = clamp_wheel_speeds(30.0, 36.6666667, 10.0)
    scale = 10.0 / 36.6666667
    assert left == pytest.approx(30.0 * scale, rel=1e-6)
    assert right == pytest.approx(36.6666667 * scale, rel=1e-6)
    assert left / right == pytest.approx(30.0 / 36.6666667, rel=1e-6)


def test_clamp_rejects_nan_as_stop():
    left, right = clamp_wheel_speeds(float('nan'), 5.0, 10.0)
    assert left == 0.0
    assert right == 0.0


def test_clamp_rejects_infinite_as_stop():
    left, right = clamp_wheel_speeds(float('inf'), 5.0, 10.0)
    assert left == 0.0
    assert right == 0.0


def test_clamp_zero_max_speed_forces_stop():
    left, right = clamp_wheel_speeds(1.0, 1.0, 0.0)
    assert left == pytest.approx(0.0, abs=1e-9)
    assert right == pytest.approx(0.0, abs=1e-9)


def test_straight_line_update_moves_forward():
    tracker = OdometryTracker(WHEEL_RADIUS_M, WHEEL_SEPARATION_M)
    linear_x, angular_z = tracker.update(10.0, 10.0, 1.0)
    assert tracker.x_m == pytest.approx(0.3, rel=1e-6)
    assert tracker.y_m == pytest.approx(0.0, abs=1e-9)
    assert tracker.theta_rad == pytest.approx(0.0, abs=1e-9)
    assert linear_x == pytest.approx(0.3, rel=1e-6)
    assert angular_z == pytest.approx(0.0, abs=1e-9)


def test_pure_rotation_update_changes_heading_only():
    tracker = OdometryTracker(WHEEL_RADIUS_M, WHEEL_SEPARATION_M)
    linear_x, angular_z = tracker.update(-5.0, 5.0, 1.0)
    assert tracker.x_m == pytest.approx(0.0, abs=1e-9)
    assert tracker.y_m == pytest.approx(0.0, abs=1e-9)
    assert tracker.theta_rad == pytest.approx(1.5, rel=1e-6)
    assert linear_x == pytest.approx(0.0, abs=1e-9)
    assert angular_z == pytest.approx(1.5, rel=1e-6)


def test_multiple_updates_accumulate_pose():
    tracker = OdometryTracker(WHEEL_RADIUS_M, WHEEL_SEPARATION_M)
    tracker.update(10.0, 10.0, 1.0)
    tracker.update(10.0, 10.0, 1.0)
    assert tracker.x_m == pytest.approx(0.6, rel=1e-6)
    assert tracker.y_m == pytest.approx(0.0, abs=1e-9)
    assert tracker.theta_rad == pytest.approx(0.0, abs=1e-9)


def test_odometry_non_positive_dt_rejected():
    tracker = OdometryTracker(WHEEL_RADIUS_M, WHEEL_SEPARATION_M)
    with pytest.raises(ValueError):
        tracker.update(1.0, 1.0, 0.0)


def test_odometry_constructor_rejects_non_positive_wheel_radius():
    with pytest.raises(ValueError):
        OdometryTracker(0.0, WHEEL_SEPARATION_M)


def test_odometry_constructor_rejects_non_positive_wheel_separation():
    with pytest.raises(ValueError):
        OdometryTracker(WHEEL_RADIUS_M, -0.1)


def test_zero_yaw_gives_identity_quaternion():
    x, y, z, w = yaw_to_quaternion(0.0)
    assert (x, y, z, w) == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1e-9)


def test_half_turn_yaw_gives_expected_quaternion():
    x, y, z, w = yaw_to_quaternion(math.pi)
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(0.0, abs=1e-9)
    assert z == pytest.approx(1.0, rel=1e-6)
    assert w == pytest.approx(0.0, abs=1e-9)


def test_quarter_turn_yaw_gives_expected_quaternion():
    x, y, z, w = yaw_to_quaternion(math.pi / 2)
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(0.0, abs=1e-9)
    assert z == pytest.approx(math.sqrt(2) / 2, rel=1e-6)
    assert w == pytest.approx(math.sqrt(2) / 2, rel=1e-6)
