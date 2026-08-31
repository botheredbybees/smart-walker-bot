from walker_anomaly_detection.imu_serial import parse_sample_line, read_samples

VALID_LINE = (
    '{"ax": 0.1, "ay": 0.2, "az": 9.8, "gx": 0.0, "gy": 0.0, "gz": 0.0, '
    '"mx": 10.0, "my": 20.0, "mz": 30.0, "t_ms": 1000}'
)


def test_valid_line_parses_all_keys():
    sample = parse_sample_line(VALID_LINE)
    assert sample == {
        'ax': 0.1, 'ay': 0.2, 'az': 9.8,
        'gx': 0.0, 'gy': 0.0, 'gz': 0.0,
        'mx': 10.0, 'my': 20.0, 'mz': 30.0,
        't_ms': 1000,
    }


def test_malformed_json_returns_none():
    assert parse_sample_line('{"ax": 0.1, "ay"') is None


def test_non_dict_json_returns_none():
    assert parse_sample_line('[1, 2, 3]') is None


def test_missing_key_returns_none():
    incomplete = '{"ax": 0.1, "ay": 0.2, "az": 9.8}'
    assert parse_sample_line(incomplete) is None


def test_empty_string_returns_none():
    assert parse_sample_line('') is None


class _FakeSerial:
    def __init__(self, lines):
        self._lines = iter(lines)

    def readline(self):
        try:
            return next(self._lines)
        except StopIteration:
            return b''


def test_read_samples_calls_callback_per_valid_line():
    lines = [
        (VALID_LINE + '\n').encode('utf-8'),
        b'garbage not json\n',
        (VALID_LINE + '\n').encode('utf-8'),
    ]
    fake = _FakeSerial(lines)
    received = []
    read_samples(fake, received.append)
    assert len(received) == 2


def test_read_samples_stops_on_empty_bytes():
    fake = _FakeSerial([b''])
    received = []
    read_samples(fake, received.append)
    assert received == []
