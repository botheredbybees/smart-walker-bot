import pytest

from watchdog_logic import Watchdog


def test_no_heartbeat_received_is_tripped():
    wd = Watchdog(timeout_s=0.5)
    assert wd.is_tripped(now_s=0.0) is True


def test_recent_heartbeat_is_not_tripped():
    wd = Watchdog(timeout_s=0.5)
    wd.on_heartbeat(now_s=10.0)
    assert wd.is_tripped(now_s=10.2) is False


def test_stale_heartbeat_is_tripped():
    wd = Watchdog(timeout_s=0.5)
    wd.on_heartbeat(now_s=10.0)
    assert wd.is_tripped(now_s=10.6) is True


def test_exactly_at_timeout_boundary_is_tripped():
    wd = Watchdog(timeout_s=0.5)
    wd.on_heartbeat(now_s=10.0)
    assert wd.is_tripped(now_s=10.5) is True


def test_new_heartbeat_recovers_from_trip():
    wd = Watchdog(timeout_s=0.5)
    wd.on_heartbeat(now_s=10.0)
    assert wd.is_tripped(now_s=10.6) is True  # tripped
    wd.on_heartbeat(now_s=100.0)
    assert wd.is_tripped(now_s=100.0) is False  # recovered


def test_repeated_heartbeats_use_latest_time():
    wd = Watchdog(timeout_s=0.5)
    wd.on_heartbeat(now_s=1.0)
    wd.on_heartbeat(now_s=2.0)
    assert wd.is_tripped(now_s=2.3) is False  # 0.3s since latest (2.0)
    assert wd.is_tripped(now_s=2.6) is True


def test_negative_or_zero_timeout_rejected():
    with pytest.raises(ValueError):
        Watchdog(timeout_s=0)
    with pytest.raises(ValueError):
        Watchdog(timeout_s=-1.0)
