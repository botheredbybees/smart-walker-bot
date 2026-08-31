import pytest

from walker_anomaly_detection.imu_serial import parse_sample_line, read_samples

VALID_LINE = (
    '{"ax": 0.1, "ay": 0.2, "az": 1.0, "gx": 0.0, "gy": 0.0, "gz": 0.0, '
    '"mx": 10.0, "my": 20.0, "mz": 30.0, "t_ms": 1000}'
)


def test_valid_line_parses_all_keys():
    sample = parse_sample_line(VALID_LINE)
    assert sample == {
        'ax': 0.1, 'ay': 0.2, 'az': 1.0,
        'gx': 0.0, 'gy': 0.0, 'gz': 0.0,
        'mx': 10.0, 'my': 20.0, 'mz': 30.0,
        't_ms': 1000,
    }


def test_malformed_json_returns_none():
    assert parse_sample_line('{"ax": 0.1, "ay"') is None


def test_non_dict_json_returns_none():
    assert parse_sample_line('[1, 2, 3]') is None


def test_missing_key_returns_none():
    incomplete = '{"ax": 0.1, "ay": 0.2, "az": 1.0}'
    assert parse_sample_line(incomplete) is None


def test_empty_string_returns_none():
    assert parse_sample_line('') is None


def test_wrong_value_type_returns_none():
    line = (
        '{"ax": null, "ay": 0.2, "az": 1.0, "gx": 0.0, "gy": 0.0, "gz": 0.0, '
        '"mx": 10.0, "my": 20.0, "mz": 30.0, "t_ms": 1000}'
    )
    assert parse_sample_line(line) is None


class _TestComplete(Exception):
    """Sentinel exception to signal end of test data."""
    pass


class _FakeSerial:
    def __init__(self, lines):
        self._lines = iter(lines)

    def readline(self):
        try:
            return next(self._lines)
        except StopIteration:
            raise _TestComplete()


def test_read_samples_calls_callback_per_valid_line():
    lines = [
        (VALID_LINE + '\n').encode('utf-8'),
        b'garbage not json\n',
        (VALID_LINE + '\n').encode('utf-8'),
    ]
    fake = _FakeSerial(lines)
    received = []
    with pytest.raises(_TestComplete):
        read_samples(fake, received.append)
    assert len(received) == 2


def test_read_samples_continues_past_empty_bytes_timeout():
    """Empty/timeout reads (b'') should NOT stop the loop; a real line
    after one or more empty reads should still be processed. Only a raised
    exception stops the loop."""
    lines = [
        b'',  # simulates a timeout with no data
        (VALID_LINE + '\n').encode('utf-8'),
    ]
    fake = _FakeSerial(lines)
    received = []
    with pytest.raises(_TestComplete):
        read_samples(fake, received.append)
    assert len(received) == 1
