from walker_llm_bridge.wellness_context import build_wellness_context_message

GAIT = {'step_count': 42, 'total_distance_m': 84.0, 'avg_step_length_m': 2.0}


def test_no_gait_and_no_alerts_returns_none():
    assert build_wellness_context_message(None, {}, None) is None


def test_gait_only_mentions_step_count_distance_and_step_length():
    message = build_wellness_context_message(GAIT, {}, None)
    assert message['role'] == 'system'
    assert '42' in message['content']
    assert '84.0' in message['content']
    assert '2.0' in message['content']
    assert 'No anomaly alerts reported so far.' in message['content']


def test_gait_only_does_not_claim_alerts_happened():
    message = build_wellness_context_message(GAIT, {}, None)
    assert 'fall' not in message['content'].lower()
    assert 'tilt' not in message['content'].lower()


def test_alerts_only_does_not_claim_gait_data():
    message = build_wellness_context_message(None, {'fall': 1}, 'fall')
    assert 'step' not in message['content'].lower()
    assert '1 fall alert' in message['content']


def test_single_alert_type_uses_singular_phrasing():
    message = build_wellness_context_message(None, {'fall': 1}, 'fall')
    assert '1 fall alert reported' in message['content']
    assert 'fall alerts' not in message['content']


def test_multiple_of_one_alert_type_uses_plural_phrasing():
    message = build_wellness_context_message(None, {'tilt': 2}, 'tilt')
    assert '2 tilt alerts reported' in message['content']


def test_multiple_alert_types_are_joined_with_and():
    message = build_wellness_context_message(None, {'fall': 1, 'tilt': 2}, 'tilt')
    assert '1 fall alert and 2 tilt alerts reported' in message['content']


def test_mentions_most_recent_alert_type():
    message = build_wellness_context_message(None, {'fall': 1, 'tilt': 2}, 'tilt')
    assert 'most recently a tilt' in message['content']


def test_gait_and_alerts_together():
    message = build_wellness_context_message(GAIT, {'fall': 1}, 'fall')
    assert '42' in message['content']
    assert '1 fall alert reported' in message['content']


def test_content_instructs_assistant_not_to_volunteer_unprompted():
    message = build_wellness_context_message(GAIT, {}, None)
    assert 'only if relevant' in message['content'].lower()


def test_float_step_count_renders_without_trailing_zero():
    gait = {'step_count': 42.0, 'total_distance_m': 84.0, 'avg_step_length_m': 2.0}
    message = build_wellness_context_message(gait, {}, None)
    assert 'Steps so far: 42.' in message['content']
    assert '42.0' not in message['content']


def test_distance_and_step_length_round_to_one_decimal():
    gait = {'step_count': 42, 'total_distance_m': 84.567, 'avg_step_length_m': 2.049}
    message = build_wellness_context_message(gait, {}, None)
    assert '84.6' in message['content']
    assert '2.0' in message['content']


def test_three_alert_types_joined_with_commas_and_final_and():
    # Alert types are listed alphabetically (fall, stumble, tilt), not
    # insertion/chronological order - latest_alert_type is the separate,
    # explicit way chronology is conveyed.
    message = build_wellness_context_message(None, {'fall': 1, 'tilt': 2, 'stumble': 3}, 'stumble')
    assert '1 fall alert, 3 stumble alerts and 2 tilt alerts reported' in message['content']
