from walker_gait_metrics.gait_tracker import GaitTracker

STEP_THRESHOLD_G = 1.2
MIN_STEP_INTERVAL_S = 0.3


def _make_tracker():
    return GaitTracker(step_threshold_g=STEP_THRESHOLD_G, min_step_interval_s=MIN_STEP_INTERVAL_S)


def _sample(ax, ay, az):
    return {'ax': ax, 'ay': ay, 'az': az}


def test_step_count_starts_at_zero():
    tracker = _make_tracker()
    assert tracker.step_count == 0


def test_on_imu_sample_below_threshold_does_not_increment_step_count():
    tracker = _make_tracker()
    tracker.on_imu_sample(_sample(0.0, 0.0, 1.0), 0.0)
    assert tracker.step_count == 0


def test_on_imu_sample_above_threshold_increments_step_count():
    tracker = _make_tracker()
    tracker.on_imu_sample(_sample(0.0, 0.0, 1.5), 0.0)
    assert tracker.step_count == 1


def test_multiple_debounced_steps_increment_step_count_correctly():
    tracker = _make_tracker()
    tracker.on_imu_sample(_sample(0.0, 0.0, 1.5), 0.0)
    tracker.on_imu_sample(_sample(0.0, 0.0, 1.5), 0.1)  # debounced
    tracker.on_imu_sample(_sample(0.0, 0.0, 1.5), 0.3)  # counts
    assert tracker.step_count == 2


def test_first_odom_pose_adds_no_distance():
    tracker = _make_tracker()
    tracker.on_odom_pose(1.0, 2.0)
    assert tracker.total_distance_m == 0.0


def test_odom_poses_accumulate_distance():
    tracker = _make_tracker()
    tracker.on_odom_pose(0.0, 0.0)
    tracker.on_odom_pose(3.0, 4.0)  # 3-4-5 triangle: 5.0m
    assert tracker.total_distance_m == 5.0
    tracker.on_odom_pose(3.0, 4.0)  # no movement
    assert tracker.total_distance_m == 5.0
    tracker.on_odom_pose(6.0, 8.0)  # another 5.0m
    assert tracker.total_distance_m == 10.0


def test_avg_step_length_is_zero_when_no_steps_taken():
    tracker = _make_tracker()
    tracker.on_odom_pose(0.0, 0.0)
    tracker.on_odom_pose(3.0, 4.0)
    assert tracker.avg_step_length_m == 0.0


def test_avg_step_length_computes_distance_over_steps():
    tracker = _make_tracker()
    tracker.on_odom_pose(0.0, 0.0)
    tracker.on_odom_pose(10.0, 0.0)  # 10m traveled
    tracker.on_imu_sample(_sample(0.0, 0.0, 1.5), 0.0)
    tracker.on_imu_sample(_sample(0.0, 0.0, 1.5), 0.3)
    tracker.on_imu_sample(_sample(0.0, 0.0, 1.5), 0.6)
    tracker.on_imu_sample(_sample(0.0, 0.0, 1.5), 0.9)
    tracker.on_imu_sample(_sample(0.0, 0.0, 1.5), 1.2)  # 5 steps
    assert tracker.avg_step_length_m == 2.0
