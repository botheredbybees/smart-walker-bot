"""Pure stop-utterance detection for walker_llm_bridge. See
docs/superpowers/specs/2026-08-30-walker-llm-bridge-design.md Sec 2.3 -
this is publish-only, deliberately not wired to anything that acts.

Exact match, not substring match, against a small fixed phrase list -
substring matching would misfire on sentences that merely contain
"stop" as a word (e.g. "don't stop the car", "stopwatch").
"""

STOP_PHRASES = ('stop', 'halt', 'stop now', 'emergency stop')


def is_stop_utterance(text):
    """Case-insensitive, whitespace-stripped exact match against
    STOP_PHRASES."""
    normalized = text.strip().lower()
    return normalized in STOP_PHRASES
