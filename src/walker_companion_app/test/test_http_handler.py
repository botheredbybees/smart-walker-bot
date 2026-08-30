import json

from walker_companion_app.http_handler import build_response

STATUS_SNAPSHOT = {'pose': {'x': 1.0, 'y': 2.0, 'theta': 0.0}, 'nav_status': 'idle', 'timestamp': 123.0}
MAP_SNAPSHOT = {'width': 2, 'height': 1, 'resolution': 0.1, 'origin_x': 0.0, 'origin_y': 0.0, 'data': [0, 100]}
CONVERSATION_SNAPSHOT = [{'role': 'user', 'text': 'hi', 'timestamp': 1.0}]
INDEX_HTML = '<html><body>dashboard</body></html>'


def test_root_path_returns_index_html():
    status, content_type, body = build_response('/', STATUS_SNAPSHOT, MAP_SNAPSHOT, CONVERSATION_SNAPSHOT, INDEX_HTML)
    assert status == 200
    assert content_type == 'text/html; charset=utf-8'
    assert body == INDEX_HTML.encode('utf-8')


def test_status_path_returns_json_status():
    status, content_type, body = build_response('/api/status', STATUS_SNAPSHOT, MAP_SNAPSHOT, CONVERSATION_SNAPSHOT, INDEX_HTML)
    assert status == 200
    assert content_type == 'application/json'
    assert json.loads(body) == STATUS_SNAPSHOT


def test_map_path_returns_json_map():
    status, content_type, body = build_response('/api/map', STATUS_SNAPSHOT, MAP_SNAPSHOT, CONVERSATION_SNAPSHOT, INDEX_HTML)
    assert status == 200
    assert content_type == 'application/json'
    assert json.loads(body) == MAP_SNAPSHOT


def test_conversation_path_returns_json_conversation():
    status, content_type, body = build_response('/api/conversation', STATUS_SNAPSHOT, MAP_SNAPSHOT, CONVERSATION_SNAPSHOT, INDEX_HTML)
    assert status == 200
    assert content_type == 'application/json'
    assert json.loads(body) == CONVERSATION_SNAPSHOT


def test_unknown_path_returns_404():
    status, content_type, body = build_response('/nonexistent', STATUS_SNAPSHOT, MAP_SNAPSHOT, CONVERSATION_SNAPSHOT, INDEX_HTML)
    assert status == 404
    assert content_type == 'text/plain; charset=utf-8'
