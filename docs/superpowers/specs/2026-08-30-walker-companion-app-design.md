# walker_companion_app Design

**Date:** 2026-08-30
**Status:** Approved by user; ready for implementation planning
**Scope:** First design pass for `README.md` §6 step 6 / §5.5's companion app: a local-network
web dashboard showing robot pose, Nav2 status, a live map, and the `walker_llm_bridge`
conversation log, developed sim-first against the existing simulated stack (`walker_motor_driver`,
`walker_nav`, `walker_llm_bridge`), same posture the prior four packages took. Does not cover real
fall/anomaly detection (no subsystem exists to feed it) or any hardware-specific concerns.

## 1. Problem

`README.md` §5.5 describes the companion app as "a lightweight, local-network-only phone
dashboard surfacing current location/status, fall/anomaly alerts, and a basic conversation log,"
with a stated minimal-viable-build of "a simple web page served from the Pi, polling status over
the home network." None of this has a concrete design yet, and one of its three stated data
sources doesn't exist: fall/anomaly detection was assigned to a lightweight IMU tilt/deceleration
monitor in `README.md` §5.2, but no such subsystem was ever built anywhere in this codebase —
`walker_motor_driver` only implements diff-drive kinematics, no IMU integration at all. This
design scopes the package to what's actually buildable and testable today: a dashboard over the
robot's real live data (pose, Nav2 navigation status, the SLAM map, and the LLM bridge's
conversation topics), with the alerts panel left as an honest placeholder rather than wired to
data that doesn't exist.

## 2. Decisions

### 2.1 Real ROS2/colcon package, stdlib HTTP server, no new runtime dependency

Same shape as the other four packages: a standard `ament_python` package. The HTTP layer is
Python's stdlib `http.server.ThreadingHTTPServer`, not Flask — Flask isn't installed in this
environment, and adding it would be a new runtime dependency for what README §5.5 itself frames
as a "simple" page. Chosen over `rosbridge_suite` + `roslibjs` (considered and rejected): a
websocket bridge plus a client-side ROS pub/sub library is a heavier, more "ROS-native" approach
better suited to a richer app than a polling status page, and would add an upstream package this
project doesn't otherwise depend on for a page this simple.

### 2.2 Pure core / thin ROS-and-HTTP binding, matching the project's established pattern

Every prior package splits pure, ROS-free logic (unit-tested with pytest) from a thin binding
layer that wires it to `rclpy`: `diff_drive_kinematics.py`/`motor_driver_node.py`,
`room_map.py`/`fake_lidar_node.py`, `ollama_client.py`/`llm_bridge_node.py`. This package
follows the same split for its HTTP layer specifically: `build_response(path, state_snapshot,
index_html) -> (status_code, content_type, body_bytes)` is a pure function containing all actual
response-building logic (what JSON to return for `/api/status`, what to return for an unknown
path, etc.), unit-tested directly against plain Python inputs — no real socket, no
`BaseHTTPRequestHandler` needed for its tests. `DashboardRequestHandler(BaseHTTPRequestHandler)`
is a thin wrapper whose `do_GET` calls `build_response` and writes the result to the real
connection. This was chosen over testing the request handler directly (rejected: exercising
`BaseHTTPRequestHandler.do_GET` requires either a real listening socket or non-trivial mocking of
its request/socket internals, neither of which fits this project's fast, ROS-free pytest pattern
used everywhere else).

### 2.3 Lock-guarded shared state between the rclpy callback thread and the HTTP thread

`rclpy.spin()` runs subscription callbacks (updating pose/map/nav-status/conversation) on one
thread while `ThreadingHTTPServer` serves requests reading that same state on other threads. A
small `SharedState` class (pure Python, no ROS import) wraps all of it behind a single
`threading.Lock`, with getters returning a consistent snapshot (a plain dict/namedtuple) rather
than exposing mutable internals. Chosen over relying on Python's GIL for "good enough" safety
(rejected: a multi-field read — e.g. pose x/y/theta together — needs a consistent snapshot, which
individual attribute reads don't guarantee even under the GIL) and over a per-field lock
(rejected as unnecessary complexity for this data volume/update rate).

### 2.4 Conversation log: in-memory ring buffer backed by an append-only local file

`ConversationLog` (pure, no ROS import) keeps the last 50 exchanges in memory for fast serving,
and separately appends every new entry to a local JSON-lines log file
(`{"role": "user"|"assistant", "text": str, "timestamp": float}` per line) so history survives a
restart — chosen per user preference over an in-memory-only buffer (which `walker_llm_bridge`
itself uses for its own conversation history, but that package's history is LLM-context-window
management, a different concern from this dashboard's user-facing log). On startup,
`ConversationLog` reads the file's existing lines to repopulate the in-memory buffer. Each
`/llm_bridge/text_in` message becomes a `"user"` entry and each `/llm_bridge/text_out` message
becomes an `"assistant"` entry, in arrival order — no request/response correlation attempted,
since `walker_llm_bridge` doesn't publish anything that would let this package pair them
reliably. No log rotation or size cap on the file itself (unbounded append-only) — acceptable for
a hobby project's first pass; noted as a known limitation.

### 2.5 Nav2 status via a plain topic subscription, not an ActionClient

Nav2's `navigate_to_pose` action server automatically publishes `action_msgs/msg/GoalStatusArray`
on `/navigate_to_pose/_action/status` for any action server, independent of who (if anyone) is
actively calling it. Subscribing to this topic directly gives a live "is Nav2 currently
navigating" signal without this package needing to be a Nav2 action client itself — chosen over
adding an `ActionClient` (rejected: unnecessary complexity and a tighter coupling to Nav2's
action API for a package that only wants to *display* status, not *drive* navigation).
`nav_status.py`'s pure `status_code_to_label(status_codes: list[int]) -> str` maps the latest
status entry's integer code (the `action_msgs/msg/GoalStatus` enum: `UNKNOWN`, `ACCEPTED`,
`EXECUTING`, `CANCELING`, `SUCCEEDED`, `CANCELED`, `ABORTED`) to a human label; the node extracts
the int list from the message before calling it, keeping the function itself pure. An empty list
(no `GoalStatusArray` ever received, or one with zero entries — both happen before any Nav2 goal
has ever been sent) maps to `"idle"`.

### 2.6 Live map: server-side primitives to JSON, client-side canvas rendering

`occupancy_grid_json.py`'s pure `grid_to_json(width, height, resolution, origin_x, origin_y,
data) -> dict` takes primitive values (extracted by the node from the `nav_msgs/OccupancyGrid`
message) and produces a JSON-serializable dict; `web/index.html`'s JavaScript draws it into a
`<canvas>` on each poll. Chosen per user preference over server-side PNG encoding (which would
need `numpy`/`Pillow`-based image generation and adds a per-request encoding cost) — the
trade-off is that the canvas-drawing logic itself is untested JavaScript, accepted as a known gap
consistent with this project never having added a frontend test framework.

### 2.7 Alerts panel: static placeholder, no topic, no endpoint

The dashboard's alerts section is hardcoded HTML text ("No anomaly detection configured yet") in
`web/index.html` — no ROS topic, no HTTP endpoint, no subscription. Chosen per user preference,
and consistent with this project's established discipline of not stubbing interfaces for
subsystems that don't exist yet (contrast with `walker_llm_bridge`'s `/llm_bridge/stop_requested`,
which *is* a real published topic with no consumer — that's the opposite shape: a real producer
with a deliberately absent consumer, appropriate there because the producer/detection logic
already exists; here, neither exists, so a fake topic would only create false confidence that
something is being monitored).

### 2.8 No wiring to `/llm_bridge/stop_requested`

This package does not subscribe to or display `/llm_bridge/stop_requested` — it's a voice
"stop"-convenience signal (`README.md` §5.3, CLAUDE.md safety invariants), not an anomaly alert,
and wiring it into this dashboard wasn't requested. Stated explicitly so it isn't added later as
an assumed extension of the alerts panel.

### 2.9 Pose extraction: pure yaw-from-quaternion, mirroring `walker_nav`'s own simplification

`nav_msgs/Odometry`'s orientation is a quaternion, not a heading — §2.5/§2.6 gave the grid and
nav-status data their own pure conversion functions, and pose needs the same treatment rather
than inline, untested math in the node. `pose_json.py`'s pure `pose_to_json(x, y, quat_z,
quat_w) -> dict` returns `{"x": x, "y": y, "theta": yaw}`, computing `yaw = 2 * atan2(quat_z,
quat_w)` — valid because this is a planar ground robot with zero roll/pitch, the same
simplification `walker_nav/walker_nav/room_map.py`'s own `yaw_from_quaternion` uses, for the
same reason. Implemented independently in this package rather than importing `walker_nav`'s
version (rejected: no package in this project currently imports another package's Python code
across the `src/` boundary, and adding the first such cross-package import for one three-line
trig formula isn't worth the coupling — package.xml would need a `<depend>walker_nav</depend>`
for a package that is otherwise about serving already-published topics, not about SLAM/nav
internals).

### 2.10 Startup defaults before any subscribed topic has published

`SharedState` is constructed with sane defaults, not `None`/absent fields, so `/api/status` and
`/api/map` never need to handle a not-yet-populated case as an error: pose `{"x": 0.0, "y": 0.0,
"theta": 0.0}`, `nav_status` `"idle"` (per §2.5's empty-list default), and an empty/zero-size map
(`{"width": 0, "height": 0, "resolution": 0.0, "origin_x": 0.0, "origin_y": 0.0, "data": []}`)
until `/map` publishes at least once. `/api/conversation` starts as whatever `ConversationLog`
loaded from its file at startup (an empty list if the file doesn't exist yet).

## 3. Package structure

New `ament_python` package:

```
src/walker_companion_app/
  package.xml, setup.py, setup.cfg, resource/walker_companion_app
  walker_companion_app/
    __init__.py
    conversation_log.py      (pure: ConversationLog - ring buffer + JSON-lines file I/O)
    pose_json.py             (pure: pose_to_json, yaw_from_quaternion)
    occupancy_grid_json.py   (pure: grid_to_json)
    nav_status.py            (pure: status_code_to_label)
    shared_state.py          (pure: SharedState, threading.Lock-guarded)
    http_handler.py          (pure: build_response; DashboardRequestHandler binds it to sockets)
    dashboard_app_node.py    (rclpy node: subscriptions + HTTP server thread lifecycle)
  web/
    index.html               (static page: polling JS, canvas map rendering, static alerts text)
  launch/dashboard_app.launch.py (http_port argument, default 8080)
  test/
    conftest.py
    test_conversation_log.py
    test_pose_json.py
    test_occupancy_grid_json.py
    test_nav_status.py
    test_shared_state.py
    test_http_handler.py
  tools/
    verify_dashboard_app.py  (scripted end-to-end check, not pytest)
```

## 4. Interface

**Node:** `walker_companion_app` (entry point `dashboard_app_node`)

**Params:**
| Param | Default | Notes |
|---|---|---|
| `http_port` | `8080` | |
| `conversation_log_path` | `~/.walker_companion_app/conversation.jsonl` | JSON-lines file; directory created if missing |
| `conversation_buffer_size` | `50` | in-memory ring buffer entry count |

**Topics subscribed:**
- `/odom` (`nav_msgs/Odometry`) — pose.
- `/map` (`nav_msgs/OccupancyGrid`) — live map.
- `/navigate_to_pose/_action/status` (`action_msgs/msg/GoalStatusArray`) — Nav2 status.
- `/llm_bridge/text_in`, `/llm_bridge/text_out` (both `std_msgs/String`) — conversation log.

**HTTP endpoints** (default `http://localhost:8080`):
- `GET /` — the dashboard page (`web/index.html`, loaded once at startup).
- `GET /api/status` — JSON: `{"pose": {"x", "y", "theta"}, "nav_status": str, "timestamp": float}`.
- `GET /api/map` — JSON: `{"width", "height", "resolution", "origin_x", "origin_y", "data": [int...]}`.
- `GET /api/conversation` — JSON: `[{"role", "text", "timestamp"}, ...]` (most recent
  `conversation_buffer_size` entries).
- Any other path — `404`.

## 5. Testing

`conversation_log.py`, `pose_json.py`, `occupancy_grid_json.py`, `nav_status.py`, `shared_state.py`, and
`http_handler.py`'s `build_response` are pure Python — unit-tested with pytest, no ROS sourcing
or colcon build required, same `test/conftest.py` `sys.path` pattern as every other package here.
`conversation_log.py`'s file I/O is tested against a real temp file (via `tmp_path`), not mocked
— the actual read/write round-trip is what matters.

`tools/verify_dashboard_app.py` — scripted (not pytest) end-to-end check: with
`walker_motor_driver`, `walker_nav` (SLAM + Nav2), `walker_llm_bridge`, and this package's node
all launched, it drives the same maneuver `walker_nav/tools/verify_slam.py` uses (so a real map
exists), sends a `/cmd_vel` command and confirms `/api/status`'s pose changes accordingly, sends
a Nav2 goal and confirms `/api/status`'s `nav_status` transitions away from `"idle"`, confirms
`/api/map` returns non-trivial grid data, and drives one utterance through `walker_llm_bridge`
(same FIFO-stdin mechanism `walker_llm_bridge/tools/verify_llm_bridge.py` uses) and confirms
`/api/conversation` shows both the user and assistant entries. Uses stdlib `urllib.request` for
the HTTP calls — no new dependency.

## 6. Out of scope

- Real fall/anomaly detection and any topic/endpoint for it — no IMU subsystem exists to feed
  it; the alerts panel is static placeholder text only, per §2.7.
- Any wiring to `/llm_bridge/stop_requested` — explicitly rejected per §2.8, not merely deferred.
- Log file rotation or size limits on the conversation log file — unbounded append-only, accepted
  as a known limitation for this pass.
- A JavaScript test framework for `web/index.html`'s polling/canvas-rendering logic — untested,
  consistent with this project not having added frontend test machinery anywhere else.
- `rosbridge_suite`/`roslibjs` or any push-based (as opposed to polling) update mechanism —
  rejected per §2.1, not merely deferred.
- Authentication/access control on the HTTP server — matches README §5.5's "local-network-only"
  framing; no internet exposure is assumed or supported.
