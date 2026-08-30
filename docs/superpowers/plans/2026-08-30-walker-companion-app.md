# walker_companion_app Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce the `walker_companion_app` ROS2 package: a local-network web dashboard serving robot pose, Nav2 status, a live map, and the `walker_llm_bridge` conversation log over a stdlib HTTP server, with a static (unwired) alerts panel.

**Architecture:** Five small pure-Python modules (conversation log, pose/grid/nav-status JSON translation, thread-safe shared state), each unit-tested with pytest, plus a pure `build_response` function holding all HTTP response logic — all following this project's established pure-core/thin-ROS-binding pattern. A thin `rclpy` node subscribes to `/odom`, `/map`, `/navigate_to_pose/_action/status`, and `walker_llm_bridge`'s conversation topics, updates the shared state, and runs a stdlib `ThreadingHTTPServer` in a background thread serving a static polling/canvas-rendering HTML page plus three JSON endpoints. Verified end-to-end against the full simulated stack (`walker_motor_driver`, `walker_nav`, `walker_llm_bridge`).

**Tech Stack:** Python 3 + `rclpy` (ROS2 Humble), pytest (pure-module unit tests), stdlib `http.server`/`threading`/`json` (no new runtime dependency), standard ROS2 messages (`nav_msgs/Odometry`, `nav_msgs/OccupancyGrid`, `action_msgs/msg/GoalStatusArray`, `std_msgs/String`).

**Spec:** `docs/superpowers/specs/2026-08-30-walker-companion-app-design.md` (§2 for decisions, §3 for file structure, §4 for interface, §5 for testing approach).

## Global Constraints

- Real `ament_python` colcon package, buildable with `colcon build --packages-select walker_companion_app` from `src/`. (spec §2.1)
- HTTP server: stdlib `http.server.ThreadingHTTPServer`, binds `0.0.0.0` (all interfaces) — README §5.5 wants this reachable from a phone on the home network, not just this workstation; the spec's "default `http://localhost:8080`" phrasing describes how to reach it from *this* machine for testing, not a loopback-only binding. Default port `8080`, overridable via the launch file's `http_port` argument. (spec §2.1, §4, clarified)
- `build_response(path, status_snapshot, map_snapshot, conversation_snapshot, index_html) -> (status_code, content_type, body_bytes)` — refined beyond the spec's single-`state_snapshot` sketch signature into three separate snapshot params, so each can be constructed independently in tests without a combined bundling object. Pure; no sockets, no I/O. (spec §2.2, refined)
- `SharedState` is the **sole** thread-safety boundary for all shared state, including the conversation log — `ConversationLog` itself has no internal locking; `SharedState.add_conversation_entry`/`.conversation_snapshot` acquire `SharedState`'s one lock before touching the wrapped `ConversationLog` instance. (spec §2.3, §2.4)
- `pose_json.py`'s `yaw_from_quaternion(qz, qw)` computes `2.0 * math.atan2(qz, qw)` — verified identical to `walker_nav/walker_nav/room_map.py`'s own `yaw_from_quaternion(qz, qw)`, implemented independently (no cross-package Python import). (spec §2.9)
- `nav_status.py`'s `status_code_to_label`: empty list → `"idle"`; code `0` (UNKNOWN) → `"idle"`; codes `1`-`6` → `"accepted"`/`"navigating"`/`"canceling"`/`"succeeded"`/`"canceled"`/`"aborted"`; any other code → `"unknown"`. Uses the *last* entry in the list. (spec §2.5)
- `SharedState` starts with defaults, never `None`: pose `{"x": 0.0, "y": 0.0, "theta": 0.0}`, `nav_status` `"idle"`, map `{"width": 0, "height": 0, "resolution": 0.0, "origin_x": 0.0, "origin_y": 0.0, "data": []}`, conversation whatever `ConversationLog` loaded at startup (empty list if no log file exists yet). (spec §2.10)
- Alerts panel is static HTML text only ("No anomaly detection configured yet") — no topic, no endpoint, no subscription. No wiring to `/llm_bridge/stop_requested`. Neither is stubbed. (spec §2.7, §2.8)
- No real fall/anomaly detection, no JS test framework, no `rosbridge_suite`/push-based updates, no auth on the HTTP server — all explicitly out of scope. (spec §6)
- Pure modules (`conversation_log.py`, `pose_json.py`, `occupancy_grid_json.py`, `nav_status.py`, `shared_state.py`, `http_handler.py`'s `build_response`) have zero `rclpy` imports — tests run with plain `python3 -m pytest`, no ROS sourcing or colcon build required, same `test/conftest.py` `sys.path` pattern as every other package here.
- `tools/verify_dashboard_app.py` requires `walker_motor_driver`, `walker_nav` (SLAM + Nav2), and this package's own node **already launched** (via `ros2 launch`, documented in this package's README) — it launches `walker_llm_bridge`'s node itself, reusing that package's own FIFO-stdin + process-group-cleanup pattern (`tools/verify_llm_bridge.py`), since `ros2 launch` can't feed that node's stdin. Requires the real Ollama server reachable, same as `walker_llm_bridge`'s own E2E check.

---

## Task 1: Package Scaffold

**Files:**
- Create: `src/walker_companion_app/package.xml`
- Create: `src/walker_companion_app/setup.py`
- Create: `src/walker_companion_app/setup.cfg`
- Create: `src/walker_companion_app/resource/walker_companion_app`
- Create: `src/walker_companion_app/walker_companion_app/__init__.py`
- Create: `src/walker_companion_app/README.md`

**Interfaces:**
- Produces: an installable, buildable `ament_python` package shell. `console_scripts` entry point `dashboard_app_node = walker_companion_app.dashboard_app_node:main` is declared now even though `dashboard_app_node.py` doesn't exist until Task 6 — `colcon build` doesn't import entry-point targets at build time.

- [ ] **Step 1: Create the package directories**

```bash
mkdir -p /home/peter_sha/sourcecode/smart-walker-bot/src/walker_companion_app/walker_companion_app
mkdir -p /home/peter_sha/sourcecode/smart-walker-bot/src/walker_companion_app/resource
mkdir -p /home/peter_sha/sourcecode/smart-walker-bot/src/walker_companion_app/web
mkdir -p /home/peter_sha/sourcecode/smart-walker-bot/src/walker_companion_app/launch
mkdir -p /home/peter_sha/sourcecode/smart-walker-bot/src/walker_companion_app/test
mkdir -p /home/peter_sha/sourcecode/smart-walker-bot/src/walker_companion_app/tools
```

- [ ] **Step 2: Write package.xml**

Create `src/walker_companion_app/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>walker_companion_app</name>
  <version>0.0.1</version>
  <description>Local-network web dashboard for smart-walker-bot: robot pose, Nav2 status, live map, and the walker_llm_bridge conversation log.</description>
  <maintainer email="botheredbybees@gmail.com">botheredbybees</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>action_msgs</depend>
  <depend>ament_index_python</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 3: Write setup.py**

Create `src/walker_companion_app/setup.py`:

```python
from setuptools import find_packages, setup

package_name = 'walker_companion_app'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/dashboard_app.launch.py']),
        ('share/' + package_name + '/web', ['web/index.html']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='botheredbybees',
    maintainer_email='botheredbybees@gmail.com',
    description='Local-network web dashboard for smart-walker-bot: robot pose, Nav2 status, live map, and the walker_llm_bridge conversation log.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard_app_node = walker_companion_app.dashboard_app_node:main',
        ],
    },
)
```

Note: `launch/dashboard_app.launch.py` and `web/index.html` are referenced here but don't exist until Tasks 6 and 5 respectively. If Step 6's build-verification fails because either is missing, create placeholders first:

`launch/dashboard_app.launch.py`:
```python
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([])
```

`web/index.html`:
```html
<!DOCTYPE html>
<html><body>placeholder</body></html>
```

Both get overwritten by their real content in later tasks.

- [ ] **Step 4: Write setup.cfg**

Create `src/walker_companion_app/setup.cfg`:

```ini
[develop]
script_dir=$base/lib/walker_companion_app
[install]
install_scripts=$base/lib/walker_companion_app
```

- [ ] **Step 5: Create the resource marker and package __init__**

```bash
touch /home/peter_sha/sourcecode/smart-walker-bot/src/walker_companion_app/resource/walker_companion_app
touch /home/peter_sha/sourcecode/smart-walker-bot/src/walker_companion_app/walker_companion_app/__init__.py
```

- [ ] **Step 6: Verify the package builds**

```bash
source /opt/ros/humble/setup.bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src
PYTHONNOUSERSITE=1 colcon build --packages-select walker_companion_app --symlink-install
```

Expected: build succeeds (`Summary: 1 package finished`). If it fails because `launch/dashboard_app.launch.py` or `web/index.html` is missing, create the placeholders from Step 3's note and retry.

- [ ] **Step 7: Write the package README**

Create `src/walker_companion_app/README.md`:

```markdown
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

\`\`\`bash
cd src/walker_companion_app
python3 -m pytest test/ -v
\`\`\`

No ROS environment or colcon build needed for these.

## Fall/anomaly alerts are not wired up

The dashboard's alerts panel is static placeholder text — no topic, no
endpoint. No fall/anomaly detection subsystem exists anywhere in this
project yet (root `README.md` §5.2 assigns it to an IMU monitor that was
never built). See the design spec §2.7 for the reasoning.
```

- [ ] **Step 8: Commit**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot
git add src/walker_companion_app/
git commit -m "$(cat <<'EOF'
Add walker_companion_app package scaffold

ament_python ROS2 package shell: package.xml, setup.py/cfg, resource
marker, and package README. colcon build verified working before any
node code exists.
EOF
)"
```

---

## Task 2: Conversation Log (TDD)

**Files:**
- Create: `src/walker_companion_app/walker_companion_app/conversation_log.py`
- Create: `src/walker_companion_app/test/conftest.py`
- Test: `src/walker_companion_app/test/test_conversation_log.py`

**Interfaces:**
- Produces: `ConversationLog(log_path, buffer_size)` with `.append(role, text, timestamp) -> None` and `.entries() -> list[dict]` (each `{'role', 'text', 'timestamp'}`, most-recent-last, returns a fresh copy). Consumed by Task 4 (`shared_state.py`) and Task 6 (`dashboard_app_node.py`).

- [ ] **Step 1: Confirm pytest is available**

```bash
python3 -m pytest --version
```

- [ ] **Step 2: Write the test-path conftest**

Create `src/walker_companion_app/test/conftest.py`:

```python
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
```

Same pattern as every other package here: inserts the *outer*
`src/walker_companion_app/` directory onto `sys.path`, so tests use the
same fully-qualified import style (`from walker_companion_app.conversation_log
import ...`) the real node uses.

- [ ] **Step 3: Write the failing tests**

Create `src/walker_companion_app/test/test_conversation_log.py`:

```python
import json

from walker_companion_app.conversation_log import ConversationLog


def test_new_log_starts_empty(tmp_path):
    log = ConversationLog(str(tmp_path / 'conv.jsonl'), buffer_size=50)
    assert log.entries() == []


def test_append_adds_entry(tmp_path):
    log = ConversationLog(str(tmp_path / 'conv.jsonl'), buffer_size=50)
    log.append('user', 'hello', 1000.0)
    assert log.entries() == [{'role': 'user', 'text': 'hello', 'timestamp': 1000.0}]


def test_append_persists_to_file_and_reloads(tmp_path):
    log_path = str(tmp_path / 'conv.jsonl')
    log1 = ConversationLog(log_path, buffer_size=50)
    log1.append('user', 'hello', 1000.0)
    log1.append('assistant', 'hi there', 1001.0)

    log2 = ConversationLog(log_path, buffer_size=50)
    assert log2.entries() == [
        {'role': 'user', 'text': 'hello', 'timestamp': 1000.0},
        {'role': 'assistant', 'text': 'hi there', 'timestamp': 1001.0},
    ]


def test_buffer_caps_at_buffer_size(tmp_path):
    log = ConversationLog(str(tmp_path / 'conv.jsonl'), buffer_size=2)
    log.append('user', 'one', 1.0)
    log.append('user', 'two', 2.0)
    log.append('user', 'three', 3.0)
    entries = log.entries()
    assert [e['text'] for e in entries] == ['two', 'three']


def test_load_existing_caps_at_buffer_size(tmp_path):
    log_path = tmp_path / 'conv.jsonl'
    with open(log_path, 'w') as f:
        for i in range(5):
            f.write(json.dumps({'role': 'user', 'text': str(i), 'timestamp': float(i)}) + '\n')

    log = ConversationLog(str(log_path), buffer_size=2)
    entries = log.entries()
    assert [e['text'] for e in entries] == ['3', '4']


def test_entries_returns_a_copy(tmp_path):
    log = ConversationLog(str(tmp_path / 'conv.jsonl'), buffer_size=50)
    log.append('user', 'hello', 1000.0)
    entries = log.entries()
    entries.append({'role': 'user', 'text': 'sneaky', 'timestamp': 0.0})
    assert len(log.entries()) == 1


def test_directory_created_if_missing(tmp_path):
    nested_path = tmp_path / 'nested' / 'dir' / 'conv.jsonl'
    log = ConversationLog(str(nested_path), buffer_size=50)
    log.append('user', 'hello', 1000.0)
    assert nested_path.exists()


def test_blank_lines_in_file_skipped(tmp_path):
    log_path = tmp_path / 'conv.jsonl'
    with open(log_path, 'w') as f:
        f.write(json.dumps({'role': 'user', 'text': 'hi', 'timestamp': 1.0}) + '\n')
        f.write('\n')
        f.write(json.dumps({'role': 'assistant', 'text': 'hello', 'timestamp': 2.0}) + '\n')

    log = ConversationLog(str(log_path), buffer_size=50)
    assert len(log.entries()) == 2
```

- [ ] **Step 4: Run tests to verify they fail**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_companion_app
python3 -m pytest test/test_conversation_log.py -v
```

Expected: `ModuleNotFoundError: No module named 'walker_companion_app.conversation_log'`.

- [ ] **Step 5: Implement ConversationLog**

Create `src/walker_companion_app/walker_companion_app/conversation_log.py`:

```python
"""Pure conversation log for walker_companion_app: an in-memory ring
buffer backed by an append-only local JSON-lines file, so history
survives a restart. No ROS import, and no internal thread-safety of its
own - shared_state.py (Task 4) is the sole thread-safety boundary for
this and every other piece of shared state. See
docs/superpowers/specs/2026-08-30-walker-companion-app-design.md Sec
2.3, 2.4.
"""
import json
import os


class ConversationLog:
    def __init__(self, log_path, buffer_size):
        self._log_path = log_path
        self._buffer_size = buffer_size
        self._entries = []

        log_dir = os.path.dirname(log_path)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)

        self._load_existing()

    def _load_existing(self):
        if not os.path.exists(self._log_path):
            return
        with open(self._log_path, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                self._entries.append(json.loads(line))
        self._trim()

    def _trim(self):
        if len(self._entries) > self._buffer_size:
            self._entries = self._entries[-self._buffer_size:]

    def append(self, role, text, timestamp):
        entry = {'role': role, 'text': text, 'timestamp': timestamp}
        self._entries.append(entry)
        self._trim()

        with open(self._log_path, 'a') as f:
            f.write(json.dumps(entry) + '\n')

    def entries(self):
        """Return a copy of the current buffer (most-recent-last)."""
        return list(self._entries)
```

- [ ] **Step 6: Run tests to verify they pass**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_companion_app
python3 -m pytest test/test_conversation_log.py -v
```

Expected: 8 passed.

- [ ] **Step 7: Commit**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot
git add src/walker_companion_app/walker_companion_app/conversation_log.py \
        src/walker_companion_app/test/conftest.py \
        src/walker_companion_app/test/test_conversation_log.py
git commit -m "$(cat <<'EOF'
Add walker_companion_app ConversationLog

In-memory ring buffer backed by an append-only local JSON-lines file,
unit-tested against real temp files - no ROS dependency, no internal
locking (shared_state.py, Task 4, owns that).
EOF
)"
```

---

## Task 3: Pure Translation Modules — Pose, Grid, Nav Status (TDD)

**Files:**
- Create: `src/walker_companion_app/walker_companion_app/pose_json.py`
- Create: `src/walker_companion_app/walker_companion_app/occupancy_grid_json.py`
- Create: `src/walker_companion_app/walker_companion_app/nav_status.py`
- Test: `src/walker_companion_app/test/test_pose_json.py`
- Test: `src/walker_companion_app/test/test_occupancy_grid_json.py`
- Test: `src/walker_companion_app/test/test_nav_status.py`

**Interfaces:**
- Produces: `pose_to_json(x, y, qz, qw) -> dict`, `yaw_from_quaternion(qz, qw) -> float`; `grid_to_json(width, height, resolution, origin_x, origin_y, data) -> dict`; `status_code_to_label(status_codes: list) -> str`. All consumed by Task 6 (`dashboard_app_node.py`).

- [ ] **Step 1: Write the failing tests**

Create `src/walker_companion_app/test/test_pose_json.py`:

```python
import math

import pytest

from walker_companion_app.pose_json import pose_to_json, yaw_from_quaternion


def test_zero_yaw_from_identity_quaternion():
    assert yaw_from_quaternion(0.0, 1.0) == pytest.approx(0.0, abs=1e-9)


def test_half_turn_yaw():
    assert yaw_from_quaternion(1.0, 0.0) == pytest.approx(math.pi, rel=1e-6)


def test_quarter_turn_yaw():
    assert yaw_from_quaternion(math.sqrt(2) / 2, math.sqrt(2) / 2) == pytest.approx(math.pi / 2, rel=1e-6)


def test_pose_to_json_fields():
    result = pose_to_json(1.5, -2.5, 0.0, 1.0)
    assert result == pytest.approx({'x': 1.5, 'y': -2.5, 'theta': 0.0}, abs=1e-9)
```

Create `src/walker_companion_app/test/test_occupancy_grid_json.py`:

```python
from walker_companion_app.occupancy_grid_json import grid_to_json


def test_grid_to_json_basic_fields():
    result = grid_to_json(width=4, height=3, resolution=0.05, origin_x=-1.0, origin_y=-1.0, data=[0, 100, -1, 50])
    assert result == {
        'width': 4,
        'height': 3,
        'resolution': 0.05,
        'origin_x': -1.0,
        'origin_y': -1.0,
        'data': [0, 100, -1, 50],
    }


def test_grid_to_json_converts_data_to_plain_list():
    result = grid_to_json(width=2, height=1, resolution=0.1, origin_x=0.0, origin_y=0.0, data=(1, 2))
    assert result['data'] == [1, 2]
    assert isinstance(result['data'], list)


def test_grid_to_json_empty_data():
    result = grid_to_json(width=0, height=0, resolution=0.1, origin_x=0.0, origin_y=0.0, data=[])
    assert result['data'] == []
```

Create `src/walker_companion_app/test/test_nav_status.py`:

```python
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
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_companion_app
python3 -m pytest test/test_pose_json.py test/test_occupancy_grid_json.py test/test_nav_status.py -v
```

Expected: three `ModuleNotFoundError`s, one per missing module.

- [ ] **Step 3: Implement pose_json.py**

Create `src/walker_companion_app/walker_companion_app/pose_json.py`:

```python
"""Pure pose extraction for walker_companion_app: converts a planar
robot's (x, y, quaternion z/w) into a JSON-serializable pose dict with a
2D heading. No ROS import - the node extracts these primitives from a
nav_msgs/Odometry message before calling this. See
docs/superpowers/specs/2026-08-30-walker-companion-app-design.md Sec 2.9.
"""
import math


def yaw_from_quaternion(qz, qw):
    """Recover a 2D heading (radians) from a Z-axis-only quaternion -
    valid only for a pure yaw rotation (qx=qy=0), which is all this
    project's ground robots ever produce. Same formula
    walker_nav/walker_nav/room_map.py's own yaw_from_quaternion uses,
    implemented independently here rather than imported across the
    package boundary (see design spec Sec 2.9)."""
    return 2.0 * math.atan2(qz, qw)


def pose_to_json(x, y, qz, qw):
    return {'x': x, 'y': y, 'theta': yaw_from_quaternion(qz, qw)}
```

- [ ] **Step 4: Implement occupancy_grid_json.py**

Create `src/walker_companion_app/walker_companion_app/occupancy_grid_json.py`:

```python
"""Pure conversion of primitive occupancy-grid fields to a JSON-serializable
dict, for walker_companion_app's /api/map endpoint. No ROS import - the
node extracts these primitives from a nav_msgs/OccupancyGrid message
before calling this. See
docs/superpowers/specs/2026-08-30-walker-companion-app-design.md Sec 2.6.
"""


def grid_to_json(width, height, resolution, origin_x, origin_y, data):
    return {
        'width': width,
        'height': height,
        'resolution': resolution,
        'origin_x': origin_x,
        'origin_y': origin_y,
        'data': list(data),
    }
```

- [ ] **Step 5: Implement nav_status.py**

Create `src/walker_companion_app/walker_companion_app/nav_status.py`:

```python
"""Pure Nav2 status-code-to-label mapping for walker_companion_app. See
docs/superpowers/specs/2026-08-30-walker-companion-app-design.md Sec 2.5.
"""

_STATUS_LABELS = {
    0: 'idle',        # action_msgs/GoalStatus.STATUS_UNKNOWN
    1: 'accepted',    # STATUS_ACCEPTED
    2: 'navigating',  # STATUS_EXECUTING
    3: 'canceling',   # STATUS_CANCELING
    4: 'succeeded',   # STATUS_SUCCEEDED
    5: 'canceled',    # STATUS_CANCELED
    6: 'aborted',     # STATUS_ABORTED
}


def status_code_to_label(status_codes):
    """status_codes: list of int action_msgs/GoalStatus codes, in the
    order GoalStatusArray reported them (most-recent goal last). Returns
    a human label for the latest entry; an empty list (no goal ever
    sent) maps to 'idle'."""
    if not status_codes:
        return 'idle'
    return _STATUS_LABELS.get(status_codes[-1], 'unknown')
```

- [ ] **Step 6: Run tests to verify they pass**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_companion_app
python3 -m pytest test/test_pose_json.py test/test_occupancy_grid_json.py test/test_nav_status.py -v
```

Expected: 17 passed (4 + 3 + 10).

- [ ] **Step 7: Commit**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot
git add src/walker_companion_app/walker_companion_app/pose_json.py \
        src/walker_companion_app/walker_companion_app/occupancy_grid_json.py \
        src/walker_companion_app/walker_companion_app/nav_status.py \
        src/walker_companion_app/test/test_pose_json.py \
        src/walker_companion_app/test/test_occupancy_grid_json.py \
        src/walker_companion_app/test/test_nav_status.py
git commit -m "$(cat <<'EOF'
Add walker_companion_app pure translation modules

pose_json.py (quaternion -> 2D heading, matching walker_nav's own
formula), occupancy_grid_json.py, and nav_status.py - all pure, no ROS
dependency, unit-tested. dashboard_app_node.py (Task 6) wires these to
real topics.
EOF
)"
```

---

## Task 4: SharedState (TDD)

**Files:**
- Create: `src/walker_companion_app/walker_companion_app/shared_state.py`
- Test: `src/walker_companion_app/test/test_shared_state.py`

**Interfaces:**
- Consumes: `ConversationLog` from `conversation_log` (Task 2).
- Produces: `SharedState(conversation_log)` with `.set_pose(pose_dict)`, `.set_nav_status(label)`, `.set_map(grid_dict)`, `.add_conversation_entry(role, text, timestamp)`, `.status_snapshot(timestamp) -> dict`, `.map_snapshot() -> dict`, `.conversation_snapshot() -> list`. Consumed by Task 5 (`http_handler.py`) and Task 6 (`dashboard_app_node.py`).

- [ ] **Step 1: Write the failing tests**

Create `src/walker_companion_app/test/test_shared_state.py`:

```python
from walker_companion_app.conversation_log import ConversationLog
from walker_companion_app.shared_state import SharedState


def _make_state(tmp_path):
    log = ConversationLog(str(tmp_path / 'conv.jsonl'), buffer_size=50)
    return SharedState(log)


def test_default_status_snapshot(tmp_path):
    state = _make_state(tmp_path)
    snapshot = state.status_snapshot(timestamp=123.0)
    assert snapshot == {'pose': {'x': 0.0, 'y': 0.0, 'theta': 0.0}, 'nav_status': 'idle', 'timestamp': 123.0}


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
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_companion_app
python3 -m pytest test/test_shared_state.py -v
```

Expected: `ModuleNotFoundError: No module named 'walker_companion_app.shared_state'`.

- [ ] **Step 3: Implement SharedState**

Create `src/walker_companion_app/walker_companion_app/shared_state.py`:

```python
"""Thread-safe shared state for walker_companion_app: written by rclpy
subscription callbacks (one thread), read by the HTTP server threads.
No ROS import - the node extracts primitives from messages before
calling these setters. This class is the sole thread-safety boundary
for all shared state, including the conversation log: ConversationLog
itself has no internal locking, and is only ever touched here, under
this class's one lock. See
docs/superpowers/specs/2026-08-30-walker-companion-app-design.md Sec
2.3, 2.10.
"""
import threading


class SharedState:
    def __init__(self, conversation_log):
        self._lock = threading.Lock()
        self._conversation_log = conversation_log
        self._pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self._nav_status = 'idle'
        self._map = {
            'width': 0, 'height': 0, 'resolution': 0.0,
            'origin_x': 0.0, 'origin_y': 0.0, 'data': [],
        }

    def set_pose(self, pose):
        with self._lock:
            self._pose = dict(pose)

    def set_nav_status(self, label):
        with self._lock:
            self._nav_status = label

    def set_map(self, grid):
        with self._lock:
            self._map = {**grid, 'data': list(grid['data'])}

    def add_conversation_entry(self, role, text, timestamp):
        with self._lock:
            self._conversation_log.append(role, text, timestamp)

    def status_snapshot(self, timestamp):
        with self._lock:
            return {'pose': dict(self._pose), 'nav_status': self._nav_status, 'timestamp': timestamp}

    def map_snapshot(self):
        with self._lock:
            return {**self._map, 'data': list(self._map['data'])}

    def conversation_snapshot(self):
        with self._lock:
            return self._conversation_log.entries()
```

Note: `set_map`/`map_snapshot` both rebuild the `'data'` list explicitly
(`{**grid, 'data': list(grid['data'])}`), not just `dict(grid)` — a
shallow copy would leave `'data'` as the *same* list object shared
between the stored state and whatever the caller (or a snapshot
consumer) holds, defeating the "returns a copy" guarantee for the one
field that's actually mutable. Caught by `test_map_snapshot_returns_a_copy`
in Step 1 — a plain `dict(self._map)` passes every other test but fails
that one.

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_companion_app
python3 -m pytest test/test_shared_state.py -v
```

Expected: 9 passed.

- [ ] **Step 5: Commit**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot
git add src/walker_companion_app/walker_companion_app/shared_state.py \
        src/walker_companion_app/test/test_shared_state.py
git commit -m "$(cat <<'EOF'
Add walker_companion_app SharedState

Sole thread-safety boundary between the rclpy callback thread and the
HTTP server threads, wrapping pose/map/nav-status/conversation behind
one lock. Snapshots deep-copy the one mutable field (map 'data') so
callers can't mutate internal state through a returned snapshot.
EOF
)"
```

---

## Task 5: HTTP Handler + Static Dashboard Page (TDD)

**Files:**
- Create: `src/walker_companion_app/walker_companion_app/http_handler.py`
- Create: `src/walker_companion_app/web/index.html` (overwrites Task 1's placeholder, if one was created)
- Test: `src/walker_companion_app/test/test_http_handler.py`

**Interfaces:**
- Produces: `build_response(path, status_snapshot, map_snapshot, conversation_snapshot, index_html) -> (status_code, content_type, body_bytes)`; `make_handler_class(shared_state, index_html) -> a BaseHTTPRequestHandler subclass`. Consumed by Task 6 (`dashboard_app_node.py`).

- [ ] **Step 1: Write the failing tests**

Create `src/walker_companion_app/test/test_http_handler.py`:

```python
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
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_companion_app
python3 -m pytest test/test_http_handler.py -v
```

Expected: `ModuleNotFoundError: No module named 'walker_companion_app.http_handler'`.

- [ ] **Step 3: Implement http_handler.py**

Create `src/walker_companion_app/walker_companion_app/http_handler.py`:

```python
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
            self.send_response(status_code)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, format, *args):
            pass  # suppress BaseHTTPRequestHandler's default stderr access log

    return DashboardRequestHandler
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_companion_app
python3 -m pytest test/test_http_handler.py -v
```

Expected: 5 passed.

- [ ] **Step 5: Write the dashboard page**

Create `src/walker_companion_app/web/index.html`:

```html
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Walker Companion</title>
<style>
  body { font-family: sans-serif; margin: 1rem; }
  canvas#map { border: 1px solid #333; background: #eee; }
  #conversation { max-height: 200px; overflow-y: auto; border: 1px solid #ccc; padding: 0.5rem; }
  .role-user { font-weight: bold; }
  .role-assistant { color: #333; }
  #alerts { background: #f0f0f0; padding: 0.5rem; border-radius: 4px; }
</style>
</head>
<body>
<h1>Walker Companion</h1>

<section id="status-section">
  <h2>Status</h2>
  <p>Pose: x=<span id="pose-x">-</span>, y=<span id="pose-y">-</span>, theta=<span id="pose-theta">-</span></p>
  <p>Nav status: <span id="nav-status">-</span></p>
</section>

<section id="map-section">
  <h2>Map</h2>
  <canvas id="map" width="400" height="400"></canvas>
</section>

<section id="alerts-section">
  <h2>Alerts</h2>
  <div id="alerts">No anomaly detection configured yet.</div>
</section>

<section id="conversation-section">
  <h2>Conversation</h2>
  <div id="conversation"></div>
</section>

<script>
const POLL_INTERVAL_MS = 2000;

async function pollStatus() {
  try {
    const res = await fetch('/api/status');
    const data = await res.json();
    document.getElementById('pose-x').textContent = data.pose.x.toFixed(2);
    document.getElementById('pose-y').textContent = data.pose.y.toFixed(2);
    document.getElementById('pose-theta').textContent = data.pose.theta.toFixed(2);
    document.getElementById('nav-status').textContent = data.nav_status;
  } catch (e) {
    console.error('status poll failed', e);
  }
}

async function pollMap() {
  try {
    const res = await fetch('/api/map');
    const grid = await res.json();
    const canvas = document.getElementById('map');
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    if (grid.width === 0 || grid.height === 0) return;

    const scaleX = canvas.width / grid.width;
    const scaleY = canvas.height / grid.height;
    const imageData = ctx.createImageData(canvas.width, canvas.height);

    for (let gy = 0; gy < grid.height; gy++) {
      for (let gx = 0; gx < grid.width; gx++) {
        const value = grid.data[gy * grid.width + gx];
        const shade = value < 0 ? 128 : 255 - Math.round((value / 100) * 255);
        const px = Math.floor(gx * scaleX);
        // Flip Y: row 0 is the grid's bottom row in ROS convention, but canvas row 0 is the top.
        const py = Math.floor((grid.height - 1 - gy) * scaleY);
        const idx = (py * canvas.width + px) * 4;
        imageData.data[idx] = shade;
        imageData.data[idx + 1] = shade;
        imageData.data[idx + 2] = shade;
        imageData.data[idx + 3] = 255;
      }
    }
    ctx.putImageData(imageData, 0, 0);
  } catch (e) {
    console.error('map poll failed', e);
  }
}

async function pollConversation() {
  try {
    const res = await fetch('/api/conversation');
    const entries = await res.json();
    const container = document.getElementById('conversation');
    container.innerHTML = '';
    for (const entry of entries) {
      const p = document.createElement('p');
      p.className = 'role-' + entry.role;
      p.textContent = entry.role + ': ' + entry.text;
      container.appendChild(p);
    }
  } catch (e) {
    console.error('conversation poll failed', e);
  }
}

function pollAll() {
  pollStatus();
  pollMap();
  pollConversation();
}

pollAll();
setInterval(pollAll, POLL_INTERVAL_MS);
</script>
</body>
</html>
```

- [ ] **Step 6: Commit**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot
git add src/walker_companion_app/walker_companion_app/http_handler.py \
        src/walker_companion_app/test/test_http_handler.py \
        src/walker_companion_app/web/index.html
git commit -m "$(cat <<'EOF'
Add walker_companion_app HTTP handler and dashboard page

build_response holds all response-building logic (pure, unit-tested);
DashboardRequestHandler is a thin BaseHTTPRequestHandler binding it to
real sockets. index.html polls the JSON endpoints and renders the map
on a canvas - untested JavaScript, per the design spec's accepted gap.
EOF
)"
```

---

## Task 6: ROS2 Node + Launch File

**Files:**
- Create: `src/walker_companion_app/walker_companion_app/dashboard_app_node.py`
- Create: `src/walker_companion_app/launch/dashboard_app.launch.py` (overwrites Task 1's placeholder, if one was created)

**Interfaces:**
- Consumes: `ConversationLog` (Task 2); `pose_to_json`, `grid_to_json`, `status_code_to_label` (Task 3); `SharedState` (Task 4); `make_handler_class` (Task 5).
- Produces: the running HTTP server and topic subscriptions that Task 7's verification script exercises. Nothing later in this plan consumes it as a Python interface.

- [ ] **Step 1: Write the ROS2 node**

Create `src/walker_companion_app/walker_companion_app/dashboard_app_node.py`:

```python
"""walker_companion_app's ROS2 node: subscribes to pose/map/nav-status/
conversation topics, updates SharedState, and runs a stdlib HTTP server
serving the dashboard. See
docs/superpowers/specs/2026-08-30-walker-companion-app-design.md for the
full design.
"""
import os
import threading
from http.server import ThreadingHTTPServer

import rclpy
from action_msgs.msg import GoalStatusArray
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from walker_companion_app.conversation_log import ConversationLog
from walker_companion_app.http_handler import make_handler_class
from walker_companion_app.nav_status import status_code_to_label
from walker_companion_app.occupancy_grid_json import grid_to_json
from walker_companion_app.pose_json import pose_to_json
from walker_companion_app.shared_state import SharedState


class DashboardAppNode(Node):
    def __init__(self):
        super().__init__('walker_companion_app')

        self.declare_parameter('http_port', 8080)
        self.declare_parameter('conversation_log_path', '~/.walker_companion_app/conversation.jsonl')
        self.declare_parameter('conversation_buffer_size', 50)

        http_port = self.get_parameter('http_port').value
        log_path = os.path.expanduser(self.get_parameter('conversation_log_path').value)
        buffer_size = self.get_parameter('conversation_buffer_size').value

        conversation_log = ConversationLog(log_path, buffer_size)
        self._state = SharedState(conversation_log)

        index_html_path = os.path.join(
            get_package_share_directory('walker_companion_app'), 'web', 'index.html'
        )
        with open(index_html_path, 'r') as f:
            index_html = f.read()

        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, 10)
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status', self._on_nav_status, 10
        )
        self.create_subscription(String, '/llm_bridge/text_in', self._on_text_in, 10)
        self.create_subscription(String, '/llm_bridge/text_out', self._on_text_out, 10)

        handler_class = make_handler_class(self._state, index_html)
        # 0.0.0.0, not just localhost - README Sec 5.5 wants this reachable
        # from a phone on the home network, not just this workstation.
        self._http_server = ThreadingHTTPServer(('0.0.0.0', http_port), handler_class)
        self._http_thread = threading.Thread(target=self._http_server.serve_forever, daemon=True)
        self._http_thread.start()

    def _on_odom(self, msg):
        orientation = msg.pose.pose.orientation
        pose = pose_to_json(msg.pose.pose.position.x, msg.pose.pose.position.y, orientation.z, orientation.w)
        self._state.set_pose(pose)

    def _on_map(self, msg):
        grid = grid_to_json(
            msg.info.width, msg.info.height, msg.info.resolution,
            msg.info.origin.position.x, msg.info.origin.position.y, msg.data,
        )
        self._state.set_map(grid)

    def _on_nav_status(self, msg):
        codes = [status.status for status in msg.status_list]
        self._state.set_nav_status(status_code_to_label(codes))

    def _on_text_in(self, msg):
        self._state.add_conversation_entry('user', msg.data, self.get_clock().now().nanoseconds / 1e9)

    def _on_text_out(self, msg):
        self._state.add_conversation_entry('assistant', msg.data, self.get_clock().now().nanoseconds / 1e9)

    def stop(self):
        self._http_server.shutdown()
        self._http_thread.join(timeout=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DashboardAppNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Syntax-check the node**

```bash
source /opt/ros/humble/setup.bash
python3 -m py_compile /home/peter_sha/sourcecode/smart-walker-bot/src/walker_companion_app/walker_companion_app/dashboard_app_node.py
```

Expected: no output, exit code 0.

- [ ] **Step 3: Write the launch file**

Create (overwrite) `src/walker_companion_app/launch/dashboard_app.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    http_port_arg = DeclareLaunchArgument(
        'http_port',
        default_value='8080',
        description='Port for the dashboard HTTP server.',
    )

    dashboard_app_node = Node(
        package='walker_companion_app',
        executable='dashboard_app_node',
        name='walker_companion_app',
        output='screen',
        parameters=[{
            'http_port': LaunchConfiguration('http_port'),
            'conversation_log_path': '~/.walker_companion_app/conversation.jsonl',
            'conversation_buffer_size': 50,
        }],
    )

    return LaunchDescription([http_port_arg, dashboard_app_node])
```

- [ ] **Step 4: Build the package**

```bash
source /opt/ros/humble/setup.bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src
PYTHONNOUSERSITE=1 colcon build --packages-select walker_companion_app --symlink-install
source install/setup.bash
```

Expected: `Summary: 1 package finished`.

- [ ] **Step 5: Standalone smoke test — confirm the node starts and serves defaults**

Nothing else needs to be running for this check — it confirms the node
and HTTP server work correctly on their own, before Task 7's full
cross-package verification.

```bash
source /opt/ros/humble/setup.bash
source /home/peter_sha/sourcecode/smart-walker-bot/src/install/setup.bash

ros2 launch walker_companion_app dashboard_app.launch.py > /tmp/dashboard_app_node.log 2>&1 &
NODE_PID=$!
sleep 2

echo "--- / ---"
curl -s http://localhost:8080/ | head -c 100
echo
echo "--- /api/status ---"
curl -s http://localhost:8080/api/status
echo
echo "--- /api/map ---"
curl -s http://localhost:8080/api/map
echo
echo "--- /api/conversation ---"
curl -s http://localhost:8080/api/conversation
echo

kill $NODE_PID 2>/dev/null
wait $NODE_PID 2>/dev/null
cat /tmp/dashboard_app_node.log
```

Expected:
- `/` returns the start of the HTML page (starts with `<!DOCTYPE html>`).
- `/api/status` returns `{"pose": {"x": 0.0, "y": 0.0, "theta": 0.0}, "nav_status": "idle", "timestamp": <some float>}`.
- `/api/map` returns `{"width": 0, "height": 0, "resolution": 0.0, "origin_x": 0.0, "origin_y": 0.0, "data": []}`.
- `/api/conversation` returns `[]`.

If it fails, check `/tmp/dashboard_app_node.log` for parameter errors,
import errors, or a port-already-in-use exception (something else on
this workstation may already be using 8080 — retry with
`ros2 launch walker_companion_app dashboard_app.launch.py http_port:=8081`
and adjust the `curl` commands' port accordingly). Same `ps aux`
zombie-process caveat as every other `ros2 launch`-based check in this
project's other packages — confirm nothing's still running afterward.

- [ ] **Step 6: Verify the backend-parameter error path is absent (no invalid-value case exists for this node)**

Unlike `walker_motor_driver`'s `backend` param or `walker_llm_bridge`'s
`voice_io_backend` param, this node has no "unknown value" branch to
verify — `http_port`, `conversation_log_path`, and
`conversation_buffer_size` are all accepted as given (an invalid port
number surfaces as a normal `OSError` from `ThreadingHTTPServer`'s own
bind call, which is already exercised implicitly if Step 5's port is
ever in conflict). No separate check needed; this step exists only to
record that the omission was considered, not overlooked.

- [ ] **Step 7: Commit**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot
git add src/walker_companion_app/walker_companion_app/dashboard_app_node.py \
        src/walker_companion_app/launch/dashboard_app.launch.py
git commit -m "$(cat <<'EOF'
Add walker_companion_app ROS2 node and launch file

dashboard_app_node.py wires ConversationLog + pose/grid/nav-status
translation + SharedState + the HTTP handler together, subscribing to
/odom, /map, /navigate_to_pose/_action/status, and walker_llm_bridge's
conversation topics. Standalone-verified: the HTTP server starts and
serves correct startup defaults with nothing else running. Full
cross-package data flow verified in Task 7.
EOF
)"
```

---

## Task 7: End-to-End Verification + Docs

**Files:**
- Create: `src/walker_companion_app/tools/verify_dashboard_app.py`
- Modify: `src/walker_companion_app/README.md` (add "Running the end-to-end check" section)
- Modify: `src/README.md`
- Modify: `CLAUDE.md`

**Interfaces:**
- Consumes: the full running sim stack (`walker_motor_driver`, `walker_nav` SLAM + Nav2, `walker_llm_bridge`, this package's node) over their public ROS2 topics/actions and this package's HTTP endpoints. Nothing later in this plan consumes it as a Python interface — this is the last task.

- [ ] **Step 1: Write the end-to-end verification script**

Create `src/walker_companion_app/tools/verify_dashboard_app.py`:

```python
#!/usr/bin/env python3
"""Scripted end-to-end check for walker_companion_app - not a pytest
test.

Assumes walker_motor_driver, walker_nav (SLAM), walker_nav (Nav2), and
this package's own node are ALREADY launched (see this package's
README's "Running the end-to-end check" section for the exact
sequence). This script launches walker_llm_bridge's node itself, the
same FIFO-stdin way walker_llm_bridge/tools/verify_llm_bridge.py does -
see that file's docstring for why: /llm_bridge/text_in is published BY
that node, so driving a conversation through it needs real stdin, which
only `ros2 run` (not `ros2 launch`) provides - and requires the real
Ollama server to be reachable for the round-trip, same as that script.

Usage (after the four pre-launched nodes are running, per the README):

    python3 tools/verify_dashboard_app.py

Exits 0 and prints PASS on success, exits 1 and prints FAIL otherwise.
"""
import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import urllib.request

import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

HTTP_BASE = 'http://localhost:8080'


def _get_json(path):
    with urllib.request.urlopen(HTTP_BASE + path, timeout=5) as response:
        return json.loads(response.read())


class VerifyDriverNode(Node):
    def __init__(self):
        super().__init__('walker_companion_app_verify')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')


def main():
    fifo_dir = tempfile.mkdtemp(prefix='walker_companion_app_verify_')
    fifo_path = os.path.join(fifo_dir, 'stdin_fifo')
    os.mkfifo(fifo_path)

    llm_bridge_process = subprocess.Popen(
        f'exec ros2 run walker_llm_bridge llm_bridge_node < {fifo_path}',
        shell=True, start_new_session=True,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    # Blocks until llm_bridge_process's shell redirection opens the FIFO's
    # read end - standard FIFO open-pairing, same as verify_llm_bridge.py.
    fifo_write = open(fifo_path, 'w')

    rclpy.init()
    node = VerifyDriverNode()

    try:
        time.sleep(3.0)  # let llm_bridge_node and the pre-launched nodes settle

        # --- Pose changes after a /cmd_vel command ---
        before = _get_json('/api/status')
        twist = Twist()
        twist.linear.x = 1.0
        node.cmd_pub.publish(twist)
        time.sleep(2.0)
        after = _get_json('/api/status')
        if not (after['pose']['x'] > before['pose']['x']):
            print(f"FAIL: /api/status pose.x did not increase ({before['pose']['x']} -> {after['pose']['x']})")
            return 1

        # --- Nav2 status transitions away from idle after a goal ---
        if not node.nav_client.wait_for_server(timeout_sec=10.0):
            print('FAIL: navigate_to_pose action server not available')
            return 1
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = 0.0
        goal_msg.pose.pose.position.y = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        send_goal_future = node.nav_client.send_goal_async(goal_msg)
        deadline = time.monotonic() + 10.0
        while not send_goal_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)

        time.sleep(2.0)
        status = _get_json('/api/status')
        if status['nav_status'] == 'idle':
            print("FAIL: /api/status nav_status still 'idle' after sending a Nav2 goal")
            return 1
        print(f"Nav2 status after goal: {status['nav_status']!r}")

        # --- Map has real data ---
        grid = _get_json('/api/map')
        if grid['width'] == 0 or grid['height'] == 0:
            print(f"FAIL: /api/map returned an empty grid (width={grid['width']}, height={grid['height']})")
            return 1

        # --- Conversation log picks up an llm_bridge round-trip ---
        fifo_write.write('hello there\n')
        fifo_write.flush()

        deadline = time.monotonic() + 40.0
        conversation = []
        got_response = False
        while time.monotonic() < deadline:
            conversation = _get_json('/api/conversation')
            if any(e['role'] == 'assistant' for e in conversation):
                got_response = True
                break
            time.sleep(1.0)

        if not got_response:
            print('FAIL: no assistant entry appeared in /api/conversation within 40s')
            return 1

        if not any(e['role'] == 'user' and e['text'] == 'hello there' for e in conversation):
            print("FAIL: no matching user entry ('hello there') found in /api/conversation")
            return 1

        print('PASS: pose update, Nav2 status transition, live map, and conversation log all verified')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
        fifo_write.close()
        try:
            os.killpg(os.getpgid(llm_bridge_process.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            llm_bridge_process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(llm_bridge_process.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            llm_bridge_process.wait()
        shutil.rmtree(fifo_dir, ignore_errors=True)


if __name__ == '__main__':
    sys.exit(main())
```

- [ ] **Step 2: Syntax-check the verification script**

```bash
source /opt/ros/humble/setup.bash
python3 -m py_compile /home/peter_sha/sourcecode/smart-walker-bot/src/walker_companion_app/tools/verify_dashboard_app.py
```

Expected: no output, exit code 0.

- [ ] **Step 3: Confirm Ollama is reachable before the end-to-end run**

```bash
curl -s --max-time 30 http://192.168.1.20:11434/api/chat -d '{"model":"qwen2.5:14b","messages":[{"role":"user","content":"Say hi in five words or less."}],"stream":false}'
```

Expected: a JSON response with non-empty `message.content` within 30s.
If this fails, fix Ollama reachability before Step 5 — same reasoning
as `walker_llm_bridge`'s own Step 7 (being listed by `/api/tags` isn't
proof of a working round-trip).

- [ ] **Step 4: Launch the four prerequisite nodes**

```bash
source /opt/ros/humble/setup.bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src
PYTHONNOUSERSITE=1 colcon build --packages-select walker_motor_driver walker_nav walker_llm_bridge walker_companion_app --symlink-install
source install/setup.bash

ros2 launch walker_motor_driver motor_driver.launch.py > /tmp/verify_motor_driver.log 2>&1 &
ros2 launch walker_nav walker_nav.launch.py > /tmp/verify_nav_slam.log 2>&1 &
sleep 3
ros2 launch walker_nav nav2.launch.py > /tmp/verify_nav_nav2.log 2>&1 &
sleep 10
ros2 launch walker_companion_app dashboard_app.launch.py > /tmp/verify_dashboard_app.log 2>&1 &
sleep 2
```

The `sleep 10` before Nav2 matches `walker_nav`'s own documented
requirement (its seven lifecycle nodes need time to reach `active`).

- [ ] **Step 5: Run the end-to-end verification**

```bash
source /opt/ros/humble/setup.bash
source /home/peter_sha/sourcecode/smart-walker-bot/src/install/setup.bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_companion_app

python3 tools/verify_dashboard_app.py
echo "verify_dashboard_app.py exit code: $?"
```

Expected: prints `Nav2 status after goal: '...'`, then
`PASS: pose update, Nav2 status transition, live map, and conversation
log all verified`, exit code `0`. If it fails, check the four log files
from Step 4 for the pre-launched nodes and inspect which assertion
failed to narrow down which topic/subscription isn't wired correctly.

- [ ] **Step 6: Clean up all launched processes**

```bash
pkill -f 'ros2 launch walker_motor_driver' 2>/dev/null
pkill -f 'ros2 launch walker_nav' 2>/dev/null
pkill -f 'ros2 launch walker_companion_app' 2>/dev/null
sleep 1
ps aux | grep -E 'motor_driver_node|async_slam_toolbox_node|dashboard_app_node|llm_bridge_node' | grep -v grep
```

Expected: the final `ps aux` line prints nothing — confirm no orphaned
processes remain (same caveat as every other multi-node check in this
project: `ros2 launch`'s spawned children don't always die from a plain
`kill`/`pkill` on the launch parent). Kill anything still listed
explicitly.

- [ ] **Step 7: Update this package's README**

Read `src/walker_companion_app/README.md`, then add a new section after
"Running the pure-module tests":

```markdown
## Running the end-to-end check

Requires four nodes launched first, in this order (matches
`walker_nav`'s own documented sequencing for the shared SLAM/Nav2
prerequisites):

\`\`\`bash
source /opt/ros/humble/setup.bash
cd src
PYTHONNOUSERSITE=1 colcon build --packages-select walker_motor_driver walker_nav walker_llm_bridge walker_companion_app --symlink-install
source install/setup.bash

ros2 launch walker_motor_driver motor_driver.launch.py &
ros2 launch walker_nav walker_nav.launch.py &
sleep 3
ros2 launch walker_nav nav2.launch.py &
sleep 10
ros2 launch walker_companion_app dashboard_app.launch.py &
sleep 2

python3 walker_companion_app/tools/verify_dashboard_app.py
\`\`\`

This script launches `walker_llm_bridge`'s node itself (the same
FIFO-stdin trick `walker_llm_bridge/tools/verify_llm_bridge.py` uses)
and requires the real Ollama server reachable for the conversation-log
check. Kill all four launched processes when done, and check
`ps aux` for anything still running — see this package's own script's
docstring and `walker_nav`'s README for why a plain `kill` isn't always
enough.

## Visiting the dashboard yourself

With the stack above running, open `http://localhost:8080/` (or
`http://<this-machine's-LAN-IP>:8080/` from another device on the same
home network, e.g. a phone — the server binds all interfaces, not just
localhost).
```

- [ ] **Step 8: Update src/README.md**

Read `src/README.md`, then in the "Planned packages" list change:

```markdown
- **`walker_companion_app`** — the optional local dashboard (§5.5), last
  in the build order.
```

to:

```markdown
- **Built.** **`walker_companion_app`** — local-network web dashboard:
  robot pose, Nav2 status, live map, and the `walker_llm_bridge`
  conversation log (§5.5). Fall/anomaly alerts are a static placeholder
  — see the package's own README.
```

And after the existing "Build/test `walker_llm_bridge`" bash block, add:

```markdown
Build/test `walker_companion_app`:

\`\`\`bash
source /opt/ros/humble/setup.bash
cd src
PYTHONNOUSERSITE=1 colcon build --packages-select walker_companion_app --symlink-install
source install/setup.bash

python3 -m pytest walker_companion_app/test/ -v   # pure-module unit tests, no ROS sourcing needed
\`\`\`
```

- [ ] **Step 9: Update CLAUDE.md**

Read `CLAUDE.md`, then in the "Project status" section change:

```markdown
Four packages exist under `src/`: `walker_safety` (E-stop wiring docs + Pico watchdog
firmware - not a colcon package, see its own README), `walker_motor_driver` (a real
`ament_python` ROS2 node - differential-drive motor control backed by a simulator until real
hardware exists), `walker_nav` (a real `ament_python` ROS2 package - a simulated LiDAR
feeding `slam_toolbox` for mapping, backed by a fixed hardcoded room until real hardware
exists; Nav2 navigates autonomously against that live map, using `nav2_bringup`'s own
navigation stack), and `walker_llm_bridge` (a real `ament_python` ROS2 package - a
text-based conversational bridge to an Ollama server; real STT/TTS and nav-goal
translation still deferred to hardware bring-up).
```

to:

```markdown
Five packages exist under `src/`: `walker_safety` (E-stop wiring docs + Pico watchdog
firmware - not a colcon package, see its own README), `walker_motor_driver` (a real
`ament_python` ROS2 node - differential-drive motor control backed by a simulator until real
hardware exists), `walker_nav` (a real `ament_python` ROS2 package - a simulated LiDAR
feeding `slam_toolbox` for mapping, backed by a fixed hardcoded room until real hardware
exists; Nav2 navigates autonomously against that live map, using `nav2_bringup`'s own
navigation stack), `walker_llm_bridge` (a real `ament_python` ROS2 package - a
text-based conversational bridge to an Ollama server; real STT/TTS and nav-goal
translation still deferred to hardware bring-up), and `walker_companion_app` (a real
`ament_python` ROS2 package - a local-network web dashboard over a stdlib HTTP server,
serving robot pose, Nav2 status, a live map, and the conversation log; fall/anomaly
alerts are a static placeholder, no IMU subsystem exists yet).
```

And after the existing "Build/test `walker_llm_bridge`" bash block, add:

```markdown
Build/test `walker_companion_app`:

\`\`\`bash
source /opt/ros/humble/setup.bash
cd src
PYTHONNOUSERSITE=1 colcon build --packages-select walker_companion_app --symlink-install
python3 -m pytest walker_companion_app/test/ -v   # pure-module unit tests, no ROS sourcing needed
\`\`\`
```

And change:

```markdown
The remaining planned package (`walker_companion_app`) doesn't exist yet.
```

to:

```markdown
All five planned Phase 1 packages now exist.
```

- [ ] **Step 10: Commit**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot
git add src/walker_companion_app/tools/verify_dashboard_app.py \
        src/walker_companion_app/README.md \
        src/README.md \
        CLAUDE.md
git commit -m "$(cat <<'EOF'
Add walker_companion_app end-to-end verification and docs

verify_dashboard_app.py checks pose updates, Nav2 status transitions,
live map data, and the conversation log against the full simulated
stack - launching walker_llm_bridge's node itself via the same
FIFO-stdin pattern that package's own verify script uses. Verified
passing against the real reachable Ollama server. Updates src/README.md
and CLAUDE.md to reflect all five planned Phase 1 packages now existing.
EOF
)"
```

---

## Self-Review Notes

- **Spec coverage:** §2.1 (stdlib HTTP server) — Task 5/6. §2.2 (pure core / thin binding) — Task 5's `build_response`/`DashboardRequestHandler` split. §2.3 (lock-guarded shared state) — Task 4. §2.4 (conversation log ring buffer + file) — Task 2. §2.5 (Nav2 status via plain subscription) — Task 3's `nav_status.py` + Task 6's subscription (no `ActionClient` in the node itself — only the verify script uses one, to *send* a goal, a different role). §2.6 (map: server primitives to JSON, client canvas) — Task 3's `occupancy_grid_json.py` + Task 5's `index.html`. §2.7 (static alerts placeholder) — Task 5's `index.html`, no topic/endpoint anywhere in Tasks 1-7. §2.8 (no `stop_requested` wiring) — never referenced anywhere in this plan. §2.9 (pure pose extraction) — Task 3's `pose_json.py`. §2.10 (startup defaults) — Task 4's `SharedState.__init__`. §3 (file structure) — matches exactly. §4 (interface: params/topics/endpoints) — Task 6's `declare_parameter` calls and subscriptions, Task 5's `build_response` paths, all match the spec's tables verbatim. §5 (testing) — Tasks 2-5 are pytest-TDD; Task 6 gets a standalone smoke test; Task 7's `verify_dashboard_app.py` matches the spec's described cross-package checks (pose, Nav2 status, map, conversation) using the documented FIFO/`urllib.request` approach. §6 (out of scope) — no fall/anomaly detection, no `stop_requested` wiring, no log rotation, no JS test framework, no `rosbridge_suite`, no auth appears anywhere in this plan.
- **Placeholder scan:** no TBD/TODO in any step. Task 1 Step 3's placeholder launch file and `index.html` (used only if Task 1 Step 6's build fails without them) are real, valid, working content, and get overwritten by their real versions in Tasks 6 and 5 respectively.
- **Type/name consistency:** `ConversationLog(log_path, buffer_size)` / `.append(role, text, timestamp)` / `.entries()` used identically in Task 2's tests, Task 4's `SharedState`, and Task 6's node. `pose_to_json(x, y, qz, qw)`, `grid_to_json(width, height, resolution, origin_x, origin_y, data)`, `status_code_to_label(status_codes)` used identically in Task 3's tests and Task 6's node. `SharedState(conversation_log)` / `.set_pose`/`.set_nav_status`/`.set_map`/`.add_conversation_entry`/`.status_snapshot`/`.map_snapshot`/`.conversation_snapshot` used identically in Task 4's tests, Task 5's `make_handler_class`, and Task 6's node. `build_response(path, status_snapshot, map_snapshot, conversation_snapshot, index_html)` used identically in Task 5's tests and its own `make_handler_class`. Topic names (`/odom`, `/map`, `/navigate_to_pose/_action/status`, `/llm_bridge/text_in`, `/llm_bridge/text_out`) and HTTP paths (`/`, `/api/status`, `/api/map`, `/api/conversation`) match between Task 6's node, Task 5's tests, and Task 7's verify script.
