# walker_companion_app

Local-network web dashboard for smart-walker-bot. See
`docs/superpowers/specs/2026-08-30-walker-companion-app-design.md` for
the full design (this is a summary).

Real `ament_python` package — build it with
`colcon build --packages-select walker_companion_app` from `src/` (this
repo's colcon workspace root).

## Layout

- `walker_companion_app/conversation_log.py` — pure Python:
  `ConversationLog`, an in-memory ring buffer backed by an append-only
  local JSON-lines file. No ROS import; unit-tested with pytest against
  a real temp file.
- `walker_companion_app/pose_json.py` — pure Python: `pose_to_json`,
  `yaw_from_quaternion` — converts an Odometry-derived (x, y,
  quaternion z/w) into a JSON pose dict with a 2D heading.
- `walker_companion_app/occupancy_grid_json.py` — pure Python:
  `grid_to_json`, converts primitive occupancy-grid fields into a
  JSON-serializable dict.
- `walker_companion_app/nav_status.py` — pure Python:
  `status_code_to_label`, maps Nav2's `action_msgs/GoalStatus` codes to
  a human label.
- `walker_companion_app/shared_state.py` — pure Python: `SharedState`,
  the sole `threading.Lock`-guarded boundary between the `rclpy`
  callback thread (writers) and the HTTP server threads (readers) —
  wraps pose, map, nav status, and the `ConversationLog` instance.
- `walker_companion_app/http_handler.py` — `build_response`, a pure
  function holding all HTTP response-building logic; `DashboardRequestHandler`
  is a thin `BaseHTTPRequestHandler` binding it to real sockets.
- `walker_companion_app/dashboard_app_node.py` — the `rclpy` node:
  subscribes `/odom`, `/map`, `/navigate_to_pose/_action/status`,
  `/llm_bridge/text_in`, `/llm_bridge/text_out`; runs the HTTP server in
  a background thread.
- `web/index.html` — the dashboard page: polls `/api/status`,
  `/api/map`, `/api/conversation` on an interval, renders the map on a
  `<canvas>`, and shows a static (unwired) alerts placeholder.
- `launch/dashboard_app.launch.py` — launch file with an `http_port`
  argument (default `8080`).
- `tools/verify_dashboard_app.py` — a scripted (not pytest) end-to-end
  check against the full simulated stack. See this file's own docstring
  for usage.

## Running the pure-module tests

```bash
cd src/walker_companion_app
python3 -m pytest test/ -v
```

No ROS environment or colcon build needed for these.

## Fall/anomaly alerts are not wired up

The dashboard's alerts panel is static placeholder text — no topic, no
endpoint. No fall/anomaly detection subsystem exists anywhere in this
project yet (root `README.md` §5.2 assigns it to an IMU monitor that was
never built). See the design spec §2.7 for the reasoning.
