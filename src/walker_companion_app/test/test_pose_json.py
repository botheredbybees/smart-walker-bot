import math

import pytest

from walker_companion_app.pose_json import pose_to_json, yaw_from_quaternion


def test_zero_yaw_from_identity_quaternion():
    assert yaw_from_quaternion(0.0, 1.0) == pytest.approx(0.0, abs=1e-9)


def test_half_turn_yaw():
    assert yaw_from_quaternion(1.0, 0.0) == pytest.approx(math.pi, rel=1e-6)


def test_quarter_turn_yaw():
    assert yaw_from_quaternion(math.sqrt(2) / 2, math.sqrt(2) / 2) == pytest.approx(math.pi / 2, rel=1e-6)


def test_pose_to_json_fields():
    result = pose_to_json(1.5, -2.5, 0.0, 1.0)
    assert result == pytest.approx({'x': 1.5, 'y': -2.5, 'theta': 0.0}, abs=1e-9)
