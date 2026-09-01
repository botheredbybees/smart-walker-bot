from walker_companion_app.conversation_log import ConversationLog
from walker_companion_app.shared_state import SharedState


def _make_state(tmp_path):
    log = ConversationLog(str(tmp_path / 'conv.jsonl'), buffer_size=50)
    return SharedState(log)


def test_default_status_snapshot(tmp_path):
    state = _make_state(tmp_path)
    snapshot = state.status_snapshot(timestamp=123.0)
    assert snapshot == {
        'pose': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
        'nav_status': 'idle',
        'gait': {'step_count': 0, 'total_distance_m': 0.0, 'avg_step_length_m': 0.0},
        'timestamp': 123.0,
    }


def test_default_map_snapshot(tmp_path):
    state = _make_state(tmp_path)
    assert state.map_snapshot() == {
        'width': 0, 'height': 0, 'resolution': 0.0, 'origin_x': 0.0, 'origin_y': 0.0, 'data': [],
    }


def test_default_conversation_snapshot_empty(tmp_path):
    state = _make_state(tmp_path)
    assert state.conversation_snapshot() == []


def test_set_pose_reflected_in_status_snapshot(tmp_path):
    state = _make_state(tmp_path)
    state.set_pose({'x': 1.0, 'y': 2.0, 'theta': 0.5})
    snapshot = state.status_snapshot(timestamp=1.0)
    assert snapshot['pose'] == {'x': 1.0, 'y': 2.0, 'theta': 0.5}


def test_set_nav_status_reflected_in_status_snapshot(tmp_path):
    state = _make_state(tmp_path)
    state.set_nav_status('navigating')
    snapshot = state.status_snapshot(timestamp=1.0)
    assert snapshot['nav_status'] == 'navigating'


def test_set_gait_metrics_reflected_in_status_snapshot(tmp_path):
    state = _make_state(tmp_path)
    state.set_gait_metrics({'step_count': 42, 'total_distance_m': 84.0, 'avg_step_length_m': 2.0})
    snapshot = state.status_snapshot(timestamp=1.0)
    assert snapshot['gait'] == {'step_count': 42, 'total_distance_m': 84.0, 'avg_step_length_m': 2.0}


def test_status_snapshot_gait_returns_a_copy(tmp_path):
    state = _make_state(tmp_path)
    state.set_gait_metrics({'step_count': 1, 'total_distance_m': 1.0, 'avg_step_length_m': 1.0})
    snapshot = state.status_snapshot(timestamp=1.0)
    snapshot['gait']['step_count'] = 999
    assert state.status_snapshot(timestamp=1.0)['gait']['step_count'] == 1


def test_set_map_reflected_in_map_snapshot(tmp_path):
    state = _make_state(tmp_path)
    grid = {'width': 2, 'height': 2, 'resolution': 0.1, 'origin_x': 0.0, 'origin_y': 0.0, 'data': [0, 0, 0, 100]}
    state.set_map(grid)
    assert state.map_snapshot() == grid


def test_map_snapshot_returns_a_copy(tmp_path):
    state = _make_state(tmp_path)
    grid = {'width': 1, 'height': 1, 'resolution': 0.1, 'origin_x': 0.0, 'origin_y': 0.0, 'data': [0]}
    state.set_map(grid)
    snapshot = state.map_snapshot()
    snapshot['data'].append(99)
    assert state.map_snapshot()['data'] == [0]


def test_status_snapshot_pose_returns_a_copy(tmp_path):
    state = _make_state(tmp_path)
    state.set_pose({'x': 1.0, 'y': 2.0, 'theta': 0.0})
    snapshot = state.status_snapshot(timestamp=1.0)
    snapshot['pose']['x'] = 999.0
    assert state.status_snapshot(timestamp=1.0)['pose']['x'] == 1.0


def test_add_conversation_entry_reflected_in_conversation_snapshot(tmp_path):
    state = _make_state(tmp_path)
    state.add_conversation_entry('user', 'hello', 1000.0)
    assert state.conversation_snapshot() == [{'role': 'user', 'text': 'hello', 'timestamp': 1000.0}]
