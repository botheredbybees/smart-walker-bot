import pytest

from walker_llm_bridge.stop_intent import is_stop_utterance


@pytest.mark.parametrize('text', [
    'stop', 'Stop', 'STOP', 'halt', 'Halt',
    'stop now', 'emergency stop', '  stop  ',
])
def test_recognized_stop_phrases_detected(text):
    assert is_stop_utterance(text) is True


@pytest.mark.parametrize('text', [
    "don't stop the car",
    'what time is it',
    'stopwatch',
    '',
    'please continue',
])
def test_non_stop_phrases_not_detected(text):
    assert is_stop_utterance(text) is False
