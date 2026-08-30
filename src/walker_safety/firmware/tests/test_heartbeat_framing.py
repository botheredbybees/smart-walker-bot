from heartbeat_framing import HEARTBEAT_BYTE, is_heartbeat_byte


def test_recognizes_heartbeat_byte():
    assert is_heartbeat_byte(HEARTBEAT_BYTE) is True


def test_rejects_different_byte():
    assert is_heartbeat_byte(b"\x00") is False


def test_rejects_empty_bytes():
    assert is_heartbeat_byte(b"") is False


def test_rejects_multi_byte_payload():
    assert is_heartbeat_byte(b"\x01\x01") is False


def test_rejects_none():
    assert is_heartbeat_byte(None) is False
