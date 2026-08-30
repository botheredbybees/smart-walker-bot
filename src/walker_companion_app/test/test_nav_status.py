import pytest

from walker_companion_app.nav_status import status_code_to_label


def test_empty_list_returns_idle():
    assert status_code_to_label([]) == 'idle'


@pytest.mark.parametrize('code,label', [
    (0, 'idle'),
    (1, 'accepted'),
    (2, 'navigating'),
    (3, 'canceling'),
    (4, 'succeeded'),
    (5, 'canceled'),
    (6, 'aborted'),
])
def test_each_known_code_maps_correctly(code, label):
    assert status_code_to_label([code]) == label


def test_unknown_code_returns_unknown_label():
    assert status_code_to_label([99]) == 'unknown'


def test_uses_last_entry_when_multiple():
    assert status_code_to_label([2, 4]) == 'succeeded'
