"""HTTP layer for walker_companion_app: build_response holds all actual
response-building logic (pure, no sockets), and DashboardRequestHandler
is a thin BaseHTTPRequestHandler binding it to real connections. See
docs/superpowers/specs/2026-08-30-walker-companion-app-design.md Sec
2.2.
"""
import json
import time
from http.server import BaseHTTPRequestHandler


def build_response(path, status_snapshot, map_snapshot, conversation_snapshot, index_html):
    """Returns (status_code, content_type, body_bytes) for a GET request
    to path. Pure - takes already-computed snapshots and the pre-loaded
    index page, no I/O of its own."""
    if path == '/':
        return 200, 'text/html; charset=utf-8', index_html.encode('utf-8')
    if path == '/api/status':
        return 200, 'application/json', json.dumps(status_snapshot).encode('utf-8')
    if path == '/api/map':
        return 200, 'application/json', json.dumps(map_snapshot).encode('utf-8')
    if path == '/api/conversation':
        return 200, 'application/json', json.dumps(conversation_snapshot).encode('utf-8')
    return 404, 'text/plain; charset=utf-8', b'Not Found'


def make_handler_class(shared_state, index_html):
    """Binds build_response to a real BaseHTTPRequestHandler, closing
    over shared_state/index_html - http.server.HTTPServer constructs a
    handler instance per request itself, so this factory is how those
    two dependencies get in."""

    class DashboardRequestHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            status_snapshot = shared_state.status_snapshot(time.time())
            map_snapshot = shared_state.map_snapshot()
            conversation_snapshot = shared_state.conversation_snapshot()
            status_code, content_type, body = build_response(
                self.path, status_snapshot, map_snapshot, conversation_snapshot, index_html
            )
            try:
                self.send_response(status_code)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass  # client disconnected mid-response - not our problem, don't log a traceback

        def log_message(self, format, *args):
            pass  # suppress BaseHTTPRequestHandler's default stderr access log

    return DashboardRequestHandler
