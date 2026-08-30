import math

import pytest

from walker_nav.room_map import cast_ray, scan_room, yaw_from_quaternion


def test_cast_ray_hits_right_wall_at_known_distance():
    distance = cast_ray(0.0, 0.0, 0.0, max_range_m=8.0)
    assert distance == pytest.approx(2.0, rel=1e-6)


def test_cast_ray_facing_away_hits_left_wall():
    distance = cast_ray(0.0, 0.0, math.pi, max_range_m=8.0)
    assert distance == pytest.approx(2.0, rel=1e-6)


def test_cast_ray_through_doorway_hits_room2_far_wall():
    distance = cast_ray(0.0, 0.0, math.pi / 2, max_range_m=8.0)
    assert distance == pytest.approx(3.5, rel=1e-6)


def test_cast_ray_max_range_when_nothing_within_range():
    distance = cast_ray(0.0, 0.0, 0.0, max_range_m=1.0)
    assert distance == pytest.approx(1.0, rel=1e-9)


def test_cast_ray_rejects_non_positive_max_range():
    with pytest.raises(ValueError):
        cast_ray(0.0, 0.0, 0.0, max_range_m=0.0)


def test_scan_room_returns_one_reading_per_beam():
    ranges = scan_room(
        0.0, 0.0, 0.0,
        angle_min_rad=-math.pi, angle_increment_rad=(2 * math.pi) / 8,
        num_beams=8, max_range_m=8.0,
    )
    assert len(ranges) == 8


def test_scan_room_first_beam_matches_direct_cast_ray():
    angle_min_rad = -math.pi
    angle_increment_rad = (2 * math.pi) / 8
    ranges = scan_room(
        0.0, 0.0, 0.0,
        angle_min_rad=angle_min_rad, angle_increment_rad=angle_increment_rad,
        num_beams=8, max_range_m=8.0,
    )
    expected_first = cast_ray(0.0, 0.0, angle_min_rad, max_range_m=8.0)
    assert ranges[0] == pytest.approx(expected_first, rel=1e-9)


def test_scan_room_rejects_non_positive_num_beams():
    with pytest.raises(ValueError):
        scan_room(0.0, 0.0, 0.0, angle_min_rad=-math.pi, angle_increment_rad=0.1,
                  num_beams=0, max_range_m=8.0)


def test_cast_ray_asymmetric_position_distinguishes_forward_from_backward_hits():
    distance = cast_ray(1.0, 0.0, 0.0, max_range_m=8.0)
    assert distance == pytest.approx(1.0, rel=1e-6)


def test_cast_ray_doorway_edge_blocked_just_outside_gap():
    distance = cast_ray(0.6, 0.0, math.pi / 2, max_range_m=8.0)
    assert distance == pytest.approx(1.5, rel=1e-6)


def test_cast_ray_doorway_edge_open_just_inside_gap():
    distance = cast_ray(0.4, 0.0, math.pi / 2, max_range_m=8.0)
    assert distance == pytest.approx(3.5, rel=1e-6)


def test_scan_room_nonzero_theta_shifts_beams():
    angle_min_rad = -math.pi
    angle_increment_rad = (2 * math.pi) / 8
    ranges_theta_zero = scan_room(
        0.0, 0.0, 0.0,
        angle_min_rad=angle_min_rad, angle_increment_rad=angle_increment_rad,
        num_beams=8, max_range_m=8.0,
    )
    ranges_theta_shifted = scan_room(
        0.0, 0.0, angle_increment_rad,
        angle_min_rad=angle_min_rad, angle_increment_rad=angle_increment_rad,
        num_beams=8, max_range_m=8.0,
    )
    assert ranges_theta_shifted[0] == pytest.approx(ranges_theta_zero[1], rel=1e-6)


def test_yaw_from_quaternion_identity_gives_zero():
    assert yaw_from_quaternion(0.0, 1.0) == pytest.approx(0.0, abs=1e-9)


def test_yaw_from_quaternion_half_turn():
    assert yaw_from_quaternion(1.0, 0.0) == pytest.approx(math.pi, rel=1e-6)


def test_yaw_from_quaternion_quarter_turn():
    half = math.pi / 4
    assert yaw_from_quaternion(math.sin(half), math.cos(half)) == pytest.approx(math.pi / 2, rel=1e-6)


def test_yaw_from_quaternion_round_trips_for_several_angles():
    for yaw in (0.3, -1.2, 2.5):
        half = yaw / 2.0
        recovered = yaw_from_quaternion(math.sin(half), math.cos(half))
        assert recovered == pytest.approx(yaw, rel=1e-6)
