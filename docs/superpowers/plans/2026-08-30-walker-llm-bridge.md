# walker_llm_bridge Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce the `walker_llm_bridge` ROS2 package: a real `ament_python` node bridging a text-based conversational loop to the Ollama server at `192.168.1.20:11434`, with a `VoiceIOBackend` interface so real STT/TTS can be swapped in later without touching the node's control logic.

**Architecture:** Two pure-Python modules unit-tested with pytest — `ollama_client.py` (HTTP client for Ollama's `/api/chat`, `requests.post` mocked in tests) and `stop_intent.py` (exact-match stop-phrase detection). A `VoiceIOBackend` interface with `TextIoBackend` (stdin/stdout, injectable streams for deterministic testing) as the only implementation, mirroring `walker_motor_driver`'s `MotorBackend` sim/real boundary. A thin `rclpy` node (`llm_bridge_node.py`) wires these together, publishing conversation and stop-intent events on ROS2 topics for future observability (`walker_companion_app`). Verified end-to-end with a scripted check (`tools/verify_llm_bridge.py`) that launches the node with its stdin redirected from a named pipe and drives a real round-trip against the actual reachable Ollama server.

**Tech Stack:** Python 3 + `rclpy` (ROS2 Humble), `requests` (already present in this environment), pytest (pure-module unit tests), `std_msgs/String` and `std_msgs/Empty`.

**Spec:** `docs/superpowers/specs/2026-08-30-walker-llm-bridge-design.md` (§2 for decisions, §3 for file structure, §4 for interface, §5 for testing approach).

## Global Constraints

- Real `ament_python` colcon package, buildable with `colcon build --packages-select walker_llm_bridge` from `src/` (spec §2.1).
- `VoiceIOBackend` interface: `start(on_utterance) -> None` (registers a callback invoked with each utterance's text), `speak(text) -> None`, `stop() -> None`. `TextIoBackend` is the only implementation this pass. (spec §2.1)
- Ollama target: host `192.168.1.20`, port `11434`, model `qwen3.5-9b-64k:latest` — confirmed present and reachable via `curl http://192.168.1.20:11434/api/tags` during design. `ollama_client.py`'s `OllamaClient.chat(messages) -> str` raises `OllamaError` on any connection failure, timeout, or malformed response. (spec §2.2)
- Stop-intent matching (`stop_intent.py`'s `is_stop_utterance(text) -> bool`) is **exact match** against a fixed phrase list (`stop`, `halt`, `stop now`, `emergency stop`), case-insensitive, after stripping whitespace — not substring matching. This is a refinement beyond the spec's "simple... phrase list" wording, chosen specifically to avoid false positives on sentences merely containing "stop" as a word (e.g. "don't stop the car", "stopwatch"), decided during planning and enforced by `test_stop_intent.py`'s explicit negative cases. (spec §2.3)
- `/llm_bridge/stop_requested` (`std_msgs/Empty`) has no consumer this pass — publish-only, never wired to anything that acts. (spec §2.3)
- `/llm_bridge/text_in` and `/llm_bridge/text_out` (both `std_msgs/String`) are **published by the node** — `text_in` echoes what the backend heard, `text_out` carries the response. The node does not subscribe to either. (spec §2.4, corrected)
- Params: `voice_io_backend` (default `text`), `ollama_host` (`192.168.1.20`), `ollama_port` (`11434`), `ollama_model` (`qwen3.5-9b-64k:latest`), `ollama_timeout_s` (`30.0`), `system_prompt` (companion-robot persona default), `max_history_messages` (`20`). (spec §4)
- Conversation history is in-memory only, capped at `max_history_messages`, oldest entries dropped first — no disk persistence. (spec §2.4)
- No real STT/TTS backend and no nav-goal translation in this pass. (spec §6)
- `tools/verify_llm_bridge.py` launches the node itself (via `subprocess.Popen` with a named-pipe stdin), rather than assuming a separately-launched node the way `walker_motor_driver`/`walker_nav`'s verify scripts do — a deliberate deviation, since stdin redirection can only be wired up at process-creation time, not attached to an already-running process. (spec §5, refined during planning)
- Pure modules (`ollama_client.py`, `stop_intent.py`, `text_io_backend.py`, `voice_io_backend.py`) have zero `rclpy` imports — tests run with plain `python3 -m pytest`, no ROS sourcing or colcon build required, same `test/conftest.py` `sys.path` pattern as `walker_motor_driver`/`walker_nav`.

---

## Task 1: Package Scaffold

**Files:**
- Create: `src/walker_llm_bridge/package.xml`
- Create: `src/walker_llm_bridge/setup.py`
- Create: `src/walker_llm_bridge/setup.cfg`
- Create: `src/walker_llm_bridge/resource/walker_llm_bridge`
- Create: `src/walker_llm_bridge/walker_llm_bridge/__init__.py`
- Create: `src/walker_llm_bridge/README.md`

**Interfaces:**
- Produces: an installable, buildable `ament_python` package shell. `console_scripts` entry point `llm_bridge_node = walker_llm_bridge.llm_bridge_node:main` is declared now even though `llm_bridge_node.py` doesn't exist until Task 5 — `colcon build` doesn't import entry-point targets at build time, only when actually run.

- [ ] **Step 1: Create the package directories**

```bash
mkdir -p /home/peter_sha/sourcecode/smart-walker-bot/src/walker_llm_bridge/walker_llm_bridge
mkdir -p /home/peter_sha/sourcecode/smart-walker-bot/src/walker_llm_bridge/resource
mkdir -p /home/peter_sha/sourcecode/smart-walker-bot/src/walker_llm_bridge/launch
mkdir -p /home/peter_sha/sourcecode/smart-walker-bot/src/walker_llm_bridge/test
mkdir -p /home/peter_sha/sourcecode/smart-walker-bot/src/walker_llm_bridge/tools
```

- [ ] **Step 2: Write package.xml**

Create `src/walker_llm_bridge/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>walker_llm_bridge</name>
  <version>0.0.1</version>
  <description>Conversational bridge to Ollama for smart-walker-bot, backed by a text I/O backend until real STT/TTS hardware exists.</description>
  <maintainer email="botheredbybees@gmail.com">botheredbybees</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>python3-requests</depend>

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

Create `src/walker_llm_bridge/setup.py`:

```python
from setuptools import find_packages, setup

package_name = 'walker_llm_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/llm_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='botheredbybees',
    maintainer_email='botheredbybees@gmail.com',
    description='Conversational bridge to Ollama for smart-walker-bot, backed by a text I/O backend until real STT/TTS hardware exists.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_bridge_node = walker_llm_bridge.llm_bridge_node:main',
        ],
    },
)
```

Note: `launch/llm_bridge.launch.py` is referenced here but doesn't exist until Task 5. If Step 6's build-verification fails because the referenced launch file is missing, create an empty placeholder first:

```python
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([])
```

Task 5 will overwrite this placeholder with the real launch file.

- [ ] **Step 4: Write setup.cfg**

Create `src/walker_llm_bridge/setup.cfg`:

```ini
[develop]
script_dir=$base/lib/walker_llm_bridge
[install]
install_scripts=$base/lib/walker_llm_bridge
```

- [ ] **Step 5: Create the resource marker and package __init__**

```bash
touch /home/peter_sha/sourcecode/smart-walker-bot/src/walker_llm_bridge/resource/walker_llm_bridge
touch /home/peter_sha/sourcecode/smart-walker-bot/src/walker_llm_bridge/walker_llm_bridge/__init__.py
```

- [ ] **Step 6: Verify the package builds**

```bash
source /opt/ros/humble/setup.bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src
PYTHONNOUSERSITE=1 colcon build --packages-select walker_llm_bridge --symlink-install
```

Expected: build succeeds (`Summary: 1 package finished`). If it fails because `launch/llm_bridge.launch.py` is missing, create the placeholder from Step 3's note and retry.

- [ ] **Step 7: Write the package README**

Create `src/walker_llm_bridge/README.md`:

```markdown
# walker_llm_bridge

Conversational bridge to Ollama for smart-walker-bot. See
`docs/superpowers/specs/2026-08-30-walker-llm-bridge-design.md` for the
full design (this is a summary).

Real `ament_python` package — build it with
`colcon build --packages-select walker_llm_bridge` from `src/` (this
repo's colcon workspace root).

## Layout

- `walker_llm_bridge/ollama_client.py` — pure Python: `OllamaClient`
  wraps host/port/model/timeout and calls Ollama's `/api/chat`,
  raising `OllamaError` on any connection failure, timeout, or
  malformed response. No ROS import; unit-tested with `requests.post`
  mocked.
- `walker_llm_bridge/stop_intent.py` — pure Python:
  `is_stop_utterance(text)`, exact (not substring) case-insensitive
  match against a small fixed phrase list. Unit-tested, including
  explicit negative cases for sentences that merely contain "stop" as
  a word.
- `walker_llm_bridge/voice_io_backend.py` — the `VoiceIOBackend`
  interface separating the node from how utterances actually
  enter/exit — the sim/real boundary. `start(on_utterance)`,
  `speak(text)`, `stop()`.
- `walker_llm_bridge/text_io_backend.py` — `TextIoBackend`, the only
  implementation until hardware bring-up adds real STT/TTS. Reads
  stdin lines on a background daemon thread; `speak()` prints to
  stdout. Input/output streams are injectable, so it's unit-tested
  deterministically with `io.StringIO` rather than a real terminal.
- `walker_llm_bridge/llm_bridge_node.py` — the `rclpy` node wiring the
  above together: constructs the backend from the `voice_io_backend`
  param, keeps an in-memory conversation history, publishes
  `/llm_bridge/text_in` and `/llm_bridge/text_out`
  (`std_msgs/String`), and `/llm_bridge/stop_requested`
  (`std_msgs/Empty`) when a stop utterance is detected — unconsumed
  this pass, publish-only.
- `launch/llm_bridge.launch.py` — launch file with a
  `voice_io_backend` argument (default `text`).
- `tools/verify_llm_bridge.py` — scripted (not pytest) end-to-end
  check: launches the node itself with stdin redirected from a named
  pipe, writes an utterance, and confirms the `/llm_bridge/text_in`
  echo, a real `/llm_bridge/text_out` round-trip response from the
  actual reachable Ollama server, and that a stop utterance fires
  `/llm_bridge/stop_requested` without calling Ollama. See this file's
  own docstring for usage.

## Running the pure-module tests

\`\`\`bash
cd src/walker_llm_bridge
python3 -m pytest test/ -v
\`\`\`

No ROS environment or colcon build needed for these.

## Voice "stop" is a convenience signal only

`/llm_bridge/stop_requested` has no consumer in this pass. Nothing
about this package stops the robot — the hardware E-stop and Pico
watchdog (`walker_safety`) are the only real stop mechanisms, per the
project's root `README.md` §5.3/§5.4. This topic exists so a future
consumer has an interface to plug into, not to imply voice stop
currently does anything.

## Real STT/TTS and nav-goal translation are out of scope

Deferred until a mic/speaker exists to test a real `VoiceIOBackend`
against, and until `walker_nav` has named locations to target,
respectively. See the design spec §6.
```

- [ ] **Step 8: Commit**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot
git add src/walker_llm_bridge/
git commit -m "$(cat <<'EOF'
Add walker_llm_bridge package scaffold

ament_python ROS2 package shell: package.xml, setup.py/cfg, resource
marker, and package README. colcon build verified working before any
node code exists.
EOF
)"
```

---

## Task 2: Ollama Client (TDD)

**Files:**
- Create: `src/walker_llm_bridge/walker_llm_bridge/ollama_client.py`
- Create: `src/walker_llm_bridge/test/conftest.py`
- Test: `src/walker_llm_bridge/test/test_ollama_client.py`

**Interfaces:**
- Produces: `OllamaError(Exception)`; `OllamaClient(host, port, model, timeout_s)` with `.chat(messages) -> str`, where `messages` is a list of `{'role': str, 'content': str}` dicts. Consumed by Task 5 (`llm_bridge_node.py`).

- [ ] **Step 1: Confirm pytest and requests are available**

```bash
python3 -m pytest --version
python3 -c "import requests; print(requests.__version__)"
```

If either is missing: `python3 -m pip install --user pytest requests`.

- [ ] **Step 2: Write the test-path conftest**

Create `src/walker_llm_bridge/test/conftest.py`:

```python
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
```

Same pattern as `walker_motor_driver`/`walker_nav`: inserts the *outer* `src/walker_llm_bridge/` directory onto `sys.path`, so tests use the exact same fully-qualified import style (`from walker_llm_bridge.ollama_client import ...`) that `llm_bridge_node.py` (Task 5) uses.

- [ ] **Step 3: Write the failing tests**

Create `src/walker_llm_bridge/test/test_ollama_client.py`:

```python
from unittest.mock import MagicMock, patch

import pytest
import requests

from walker_llm_bridge.ollama_client import OllamaClient, OllamaError


def _make_response(json_data=None, raise_status_error=False):
    response = MagicMock()
    if raise_status_error:
        response.raise_for_status.side_effect = requests.exceptions.HTTPError('500 error')
    else:
        response.raise_for_status.return_value = None
    response.json.return_value = json_data if json_data is not None else {}
    return response


@patch('walker_llm_bridge.ollama_client.requests.post')
def test_chat_returns_message_content(mock_post):
    mock_post.return_value = _make_response(json_data={'message': {'content': 'hello there'}})
    client = OllamaClient('192.168.1.20', 11434, 'qwen3.5-9b-64k:latest', 30.0)

    result = client.chat([{'role': 'user', 'content': 'hi'}])

    assert result == 'hello there'


@patch('walker_llm_bridge.ollama_client.requests.post')
def test_chat_sends_expected_request(mock_post):
    mock_post.return_value = _make_response(json_data={'message': {'content': 'ok'}})
    client = OllamaClient('192.168.1.20', 11434, 'qwen3.5-9b-64k:latest', 30.0)
    messages = [{'role': 'system', 'content': 'sys'}, {'role': 'user', 'content': 'hi'}]

    client.chat(messages)

    mock_post.assert_called_once_with(
        'http://192.168.1.20:11434/api/chat',
        json={'model': 'qwen3.5-9b-64k:latest', 'messages': messages, 'stream': False},
        timeout=30.0,
    )


@patch('walker_llm_bridge.ollama_client.requests.post')
def test_chat_raises_ollama_error_on_connection_failure(mock_post):
    mock_post.side_effect = requests.exceptions.ConnectionError('refused')
    client = OllamaClient('192.168.1.20', 11434, 'qwen3.5-9b-64k:latest', 30.0)

    with pytest.raises(OllamaError):
        client.chat([{'role': 'user', 'content': 'hi'}])


@patch('walker_llm_bridge.ollama_client.requests.post')
def test_chat_raises_ollama_error_on_timeout(mock_post):
    mock_post.side_effect = requests.exceptions.Timeout('timed out')
    client = OllamaClient('192.168.1.20', 11434, 'qwen3.5-9b-64k:latest', 30.0)

    with pytest.raises(OllamaError):
        client.chat([{'role': 'user', 'content': 'hi'}])


@patch('walker_llm_bridge.ollama_client.requests.post')
def test_chat_raises_ollama_error_on_http_error_status(mock_post):
    mock_post.return_value = _make_response(raise_status_error=True)
    client = OllamaClient('192.168.1.20', 11434, 'qwen3.5-9b-64k:latest', 30.0)

    with pytest.raises(OllamaError):
        client.chat([{'role': 'user', 'content': 'hi'}])


@patch('walker_llm_bridge.ollama_client.requests.post')
def test_chat_raises_ollama_error_on_missing_message_key(mock_post):
    mock_post.return_value = _make_response(json_data={'unexpected': 'shape'})
    client = OllamaClient('192.168.1.20', 11434, 'qwen3.5-9b-64k:latest', 30.0)

    with pytest.raises(OllamaError):
        client.chat([{'role': 'user', 'content': 'hi'}])


@patch('walker_llm_bridge.ollama_client.requests.post')
def test_chat_raises_ollama_error_on_non_json_response(mock_post):
    response = MagicMock()
    response.raise_for_status.return_value = None
    response.json.side_effect = ValueError('not json')
    mock_post.return_value = response
    client = OllamaClient('192.168.1.20', 11434, 'qwen3.5-9b-64k:latest', 30.0)

    with pytest.raises(OllamaError):
        client.chat([{'role': 'user', 'content': 'hi'}])
```

- [ ] **Step 4: Run tests to verify they fail**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_llm_bridge
python3 -m pytest test/test_ollama_client.py -v
```

Expected: `ModuleNotFoundError: No module named 'walker_llm_bridge.ollama_client'`.

- [ ] **Step 5: Implement the Ollama client**

Create `src/walker_llm_bridge/walker_llm_bridge/ollama_client.py`:

```python
"""Pure Python client for Ollama's /api/chat endpoint - no ROS import,
shared between llm_bridge_node.py and the pytest suite (requests.post is
mocked in tests, never called for real). See
docs/superpowers/specs/2026-08-30-walker-llm-bridge-design.md Sec 2.2.
"""
import requests


class OllamaError(Exception):
    """Raised for any Ollama connection failure, timeout, or malformed
    response - callers never need to catch requests-specific exceptions."""


class OllamaClient:
    def __init__(self, host, port, model, timeout_s):
        self._url = f'http://{host}:{port}/api/chat'
        self._model = model
        self._timeout_s = timeout_s

    def chat(self, messages):
        """messages: list of {'role': 'system'|'user'|'assistant', 'content': str}.
        Returns the assistant's reply text. Raises OllamaError on any
        connection failure, timeout, or unexpected response shape."""
        try:
            response = requests.post(
                self._url,
                json={'model': self._model, 'messages': messages, 'stream': False},
                timeout=self._timeout_s,
            )
            response.raise_for_status()
        except requests.exceptions.RequestException as e:
            raise OllamaError(f'Ollama request failed: {e}') from e

        try:
            data = response.json()
            return data['message']['content']
        except (ValueError, KeyError, TypeError) as e:
            raise OllamaError(f'Unexpected Ollama response shape: {e}') from e
```

- [ ] **Step 6: Run tests to verify they pass**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_llm_bridge
python3 -m pytest test/test_ollama_client.py -v
```

Expected: 7 passed.

- [ ] **Step 7: Commit**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot
git add src/walker_llm_bridge/walker_llm_bridge/ollama_client.py \
        src/walker_llm_bridge/test/conftest.py \
        src/walker_llm_bridge/test/test_ollama_client.py
git commit -m "$(cat <<'EOF'
Add walker_llm_bridge Ollama client

Pure-Python OllamaClient wrapping /api/chat, unit-tested with
requests.post mocked - no ROS dependency, no real network call in
tests. llm_bridge_node.py (Task 5) wires this to the conversation loop.
EOF
)"
```

---

## Task 3: Stop-Intent Detection (TDD)

**Files:**
- Create: `src/walker_llm_bridge/walker_llm_bridge/stop_intent.py`
- Test: `src/walker_llm_bridge/test/test_stop_intent.py`

**Interfaces:**
- Produces: `is_stop_utterance(text) -> bool`. Consumed by Task 5 (`llm_bridge_node.py`).

- [ ] **Step 1: Write the failing tests**

Create `src/walker_llm_bridge/test/test_stop_intent.py`:

```python
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
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_llm_bridge
python3 -m pytest test/test_stop_intent.py -v
```

Expected: `ModuleNotFoundError: No module named 'walker_llm_bridge.stop_intent'`.

- [ ] **Step 3: Implement stop-intent detection**

Create `src/walker_llm_bridge/walker_llm_bridge/stop_intent.py`:

```python
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
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_llm_bridge
python3 -m pytest test/test_stop_intent.py -v
```

Expected: 13 passed.

- [ ] **Step 5: Commit**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot
git add src/walker_llm_bridge/walker_llm_bridge/stop_intent.py \
        src/walker_llm_bridge/test/test_stop_intent.py
git commit -m "$(cat <<'EOF'
Add walker_llm_bridge stop-intent detection

Exact-match (not substring) stop-phrase detection, unit-tested
including negative cases for sentences that merely contain "stop" as
a word. Publish-only per spec Sec 2.3 - not wired to anything that
acts on it.
EOF
)"
```

---

## Task 4: VoiceIOBackend Interface + TextIoBackend (TDD)

**Files:**
- Create: `src/walker_llm_bridge/walker_llm_bridge/voice_io_backend.py`
- Create: `src/walker_llm_bridge/walker_llm_bridge/text_io_backend.py`
- Test: `src/walker_llm_bridge/test/test_text_io_backend.py`

**Interfaces:**
- Produces: `VoiceIOBackend` (interface, `NotImplementedError` stubs for `start`/`speak`/`stop`); `TextIoBackend(input_stream=None, output_stream=None)` implementing it, with `.start(on_utterance)`, `.speak(text)`, `.stop()`. Consumed by Task 5 (`llm_bridge_node.py`).

- [ ] **Step 1: Write the failing tests**

Create `src/walker_llm_bridge/test/test_text_io_backend.py`:

```python
import io

from walker_llm_bridge.text_io_backend import TextIoBackend


def test_start_invokes_callback_per_nonempty_line():
    input_stream = io.StringIO('hello\n\nworld\n')
    output_stream = io.StringIO()
    backend = TextIoBackend(input_stream=input_stream, output_stream=output_stream)
    received = []

    backend.start(received.append)
    backend._thread.join(timeout=2.0)

    assert not backend._thread.is_alive()
    assert received == ['hello', 'world']


def test_speak_writes_prefixed_line_and_flushes():
    output_stream = io.StringIO()
    backend = TextIoBackend(input_stream=io.StringIO(''), output_stream=output_stream)

    backend.speak('hi there')

    assert output_stream.getvalue() == 'walker> hi there\n'


def test_stop_does_not_raise():
    backend = TextIoBackend(input_stream=io.StringIO(''), output_stream=io.StringIO())
    backend.start(lambda text: None)
    backend._thread.join(timeout=2.0)

    backend.stop()
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_llm_bridge
python3 -m pytest test/test_text_io_backend.py -v
```

Expected: `ModuleNotFoundError: No module named 'walker_llm_bridge.text_io_backend'`.

- [ ] **Step 3: Implement the backend interface**

Create `src/walker_llm_bridge/walker_llm_bridge/voice_io_backend.py`:

```python
"""Abstract interface separating walker_llm_bridge's ROS2 node from how
utterances actually enter/exit the system - the sim/real boundary
described in docs/superpowers/specs/2026-08-30-walker-llm-bridge-design.md
Sec 2.1. TextIoBackend (text_io_backend.py) is the only implementation
until hardware bring-up adds a real STT/TTS backend; llm_bridge_node.py's
control logic doesn't change when that happens.
"""


class VoiceIOBackend:
    def start(self, on_utterance):
        """Begin listening for utterances. on_utterance is called with
        each utterance's text (str) as it arrives."""
        raise NotImplementedError

    def speak(self, text):
        """Output a response utterance."""
        raise NotImplementedError

    def stop(self):
        """Release any resources. Called on node shutdown."""
        raise NotImplementedError
```

- [ ] **Step 4: Implement the text backend**

Create `src/walker_llm_bridge/walker_llm_bridge/text_io_backend.py`:

```python
"""stdin/stdout VoiceIOBackend implementation - the only backend until
hardware bring-up adds real STT/TTS. See
docs/superpowers/specs/2026-08-30-walker-llm-bridge-design.md Sec 2.1.
"""
import sys
import threading

from walker_llm_bridge.voice_io_backend import VoiceIOBackend


class TextIoBackend(VoiceIOBackend):
    """input_stream/output_stream default to sys.stdin/sys.stdout but are
    injectable so tests can exercise the read loop deterministically
    without a real terminal (e.g. io.StringIO)."""

    def __init__(self, input_stream=None, output_stream=None):
        self._input_stream = input_stream if input_stream is not None else sys.stdin
        self._output_stream = output_stream if output_stream is not None else sys.stdout
        self._thread = None

    def start(self, on_utterance):
        def _read_loop():
            while True:
                line = self._input_stream.readline()
                if line == '':
                    break
                text = line.strip()
                if text:
                    on_utterance(text)

        self._thread = threading.Thread(target=_read_loop, daemon=True)
        self._thread.start()

    def speak(self, text):
        self._output_stream.write(f'walker> {text}\n')
        self._output_stream.flush()

    def stop(self):
        # stdin.readline() can't be cleanly interrupted mid-block; the
        # read thread is a daemon and dies with the process. Nothing to
        # release here - same "nothing to release" rationale
        # SimMotorBackend.stop() uses for a sim with no physical motors.
        pass
```

- [ ] **Step 5: Run tests to verify they pass**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_llm_bridge
python3 -m pytest test/test_text_io_backend.py -v
```

Expected: 3 passed.

- [ ] **Step 6: Run the full pure-module suite**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_llm_bridge
python3 -m pytest test/ -v
```

Expected: 23 passed (7 ollama_client + 13 stop_intent + 3 text_io_backend).

- [ ] **Step 7: Commit**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot
git add src/walker_llm_bridge/walker_llm_bridge/voice_io_backend.py \
        src/walker_llm_bridge/walker_llm_bridge/text_io_backend.py \
        src/walker_llm_bridge/test/test_text_io_backend.py
git commit -m "$(cat <<'EOF'
Add walker_llm_bridge VoiceIOBackend interface and text backend

TextIoBackend is the only implementation until hardware bring-up adds
real STT/TTS. Input/output streams are injectable, so the stdin read
loop is unit-tested deterministically with io.StringIO rather than a
real terminal.
EOF
)"
```

---

## Task 5: ROS2 Node, Launch File, and End-to-End Verification

**Files:**
- Create: `src/walker_llm_bridge/walker_llm_bridge/llm_bridge_node.py`
- Create: `src/walker_llm_bridge/launch/llm_bridge.launch.py` (overwrites Task 1's placeholder, if one was created)
- Create: `src/walker_llm_bridge/tools/verify_llm_bridge.py`
- Modify: `src/README.md`
- Modify: `CLAUDE.md`

**Interfaces:**
- Consumes: `OllamaClient`/`OllamaError` from `ollama_client` (Task 2); `is_stop_utterance` from `stop_intent` (Task 3); `TextIoBackend` from `text_io_backend` (Task 4).
- Produces: the `/llm_bridge/text_in`, `/llm_bridge/text_out`, `/llm_bridge/stop_requested` topic interface that a future `walker_companion_app` observes. Nothing later in this plan consumes it as a Python interface.

- [ ] **Step 1: Write the ROS2 node**

Create `src/walker_llm_bridge/walker_llm_bridge/llm_bridge_node.py`:

```python
"""walker_llm_bridge's ROS2 node: bridges a VoiceIOBackend
(text_io_backend.py's TextIoBackend for now) to an Ollama chat model,
publishing conversation and stop-intent events on ROS2 topics. See
docs/superpowers/specs/2026-08-30-walker-llm-bridge-design.md for the
full design.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Empty, String

from walker_llm_bridge.ollama_client import OllamaClient, OllamaError
from walker_llm_bridge.stop_intent import is_stop_utterance
from walker_llm_bridge.text_io_backend import TextIoBackend

STOP_ACK_MESSAGE = (
    "Stop noted - this is a convenience signal only and isn't wired to "
    "the motors; the hardware E-stop is what actually stops the robot."
)
OLLAMA_UNREACHABLE_MESSAGE = "I can't reach the LLM server right now."


class LlmBridgeNode(Node):
    def __init__(self):
        super().__init__('walker_llm_bridge')

        self.declare_parameter('voice_io_backend', 'text')
        self.declare_parameter('ollama_host', '192.168.1.20')
        self.declare_parameter('ollama_port', 11434)
        self.declare_parameter('ollama_model', 'qwen3.5-9b-64k:latest')
        self.declare_parameter('ollama_timeout_s', 30.0)
        self.declare_parameter(
            'system_prompt',
            "You are a friendly companion robot's conversational voice. "
            "Keep replies short and warm.",
        )
        self.declare_parameter('max_history_messages', 20)

        backend_name = self.get_parameter('voice_io_backend').value
        ollama_host = self.get_parameter('ollama_host').value
        ollama_port = self.get_parameter('ollama_port').value
        ollama_model = self.get_parameter('ollama_model').value
        ollama_timeout_s = self.get_parameter('ollama_timeout_s').value
        self._system_prompt = self.get_parameter('system_prompt').value
        self._max_history_messages = self.get_parameter('max_history_messages').value

        if backend_name == 'text':
            self._backend = TextIoBackend()
        else:
            raise ValueError(
                f"Unknown voice_io_backend '{backend_name}' - only 'text' is implemented; "
                "a real STT/TTS backend is added at the hardware bring-up checkpoint."
            )

        self._ollama_client = OllamaClient(ollama_host, ollama_port, ollama_model, ollama_timeout_s)
        self._history = []

        self._text_in_pub = self.create_publisher(String, '/llm_bridge/text_in', 10)
        self._text_out_pub = self.create_publisher(String, '/llm_bridge/text_out', 10)
        self._stop_pub = self.create_publisher(Empty, '/llm_bridge/stop_requested', 10)

        self._backend.start(self._on_utterance)

    def _on_utterance(self, text):
        self._text_in_pub.publish(String(data=text))

        if is_stop_utterance(text):
            self._stop_pub.publish(Empty())
            self.get_logger().warning(
                f"Stop utterance detected ({text!r}) - convenience signal only, "
                "not wired to any motor/safety topic."
            )
            self._backend.speak(STOP_ACK_MESSAGE)
            return

        messages = [{'role': 'system', 'content': self._system_prompt}]
        messages.extend(self._history)
        messages.append({'role': 'user', 'content': text})

        try:
            response_text = self._ollama_client.chat(messages)
        except OllamaError as e:
            self.get_logger().error(f'Ollama call failed: {e}')
            response_text = OLLAMA_UNREACHABLE_MESSAGE
        else:
            self._history.append({'role': 'user', 'content': text})
            self._history.append({'role': 'assistant', 'content': response_text})
            if len(self._history) > self._max_history_messages:
                overflow = len(self._history) - self._max_history_messages
                del self._history[:overflow]

        self._text_out_pub.publish(String(data=response_text))
        self._backend.speak(response_text)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LlmBridgeNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node._backend.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Syntax-check the node**

```bash
source /opt/ros/humble/setup.bash
python3 -m py_compile /home/peter_sha/sourcecode/smart-walker-bot/src/walker_llm_bridge/walker_llm_bridge/llm_bridge_node.py
```

Expected: no output, exit code 0.

- [ ] **Step 3: Write the launch file**

Create (overwrite) `src/walker_llm_bridge/launch/llm_bridge.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    backend_arg = DeclareLaunchArgument(
        'voice_io_backend',
        default_value='text',
        description="Voice I/O backend: 'text' (default, stdin/stdout) - "
                    "a real STT/TTS backend is added at hardware bring-up.",
    )

    llm_bridge_node = Node(
        package='walker_llm_bridge',
        executable='llm_bridge_node',
        name='walker_llm_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'voice_io_backend': LaunchConfiguration('voice_io_backend'),
            'ollama_host': '192.168.1.20',
            'ollama_port': 11434,
            'ollama_model': 'qwen3.5-9b-64k:latest',
            'ollama_timeout_s': 30.0,
            'max_history_messages': 20,
        }],
    )

    return LaunchDescription([backend_arg, llm_bridge_node])
```

- [ ] **Step 4: Write the end-to-end verification script**

Create `src/walker_llm_bridge/tools/verify_llm_bridge.py`:

```python
#!/usr/bin/env python3
"""Scripted end-to-end check for walker_llm_bridge - not a pytest test.

llm_bridge_node.py's only real utterance path is its VoiceIOBackend (the
`text` backend reads stdin) - see
docs/superpowers/specs/2026-08-30-walker-llm-bridge-design.md Sec 2.1/2.4.
So driving the conversation from a script means writing into the node's
actual stdin, not publishing to a topic (/llm_bridge/text_in is
published BY the node, an echo for observability - subscribing a test
publisher to it would create a publish/subscribe loop back onto the
node's own callback).

This script launches the node itself (via subprocess, stdin redirected
from a named pipe it creates) rather than assuming a separately-launched
node the way other packages' verify scripts do, since stdin redirection
can only be wired up at process-creation time.

Requires the real Ollama server (config default: 192.168.1.20:11434,
model qwen3.5-9b-64k:latest) to be reachable - there is no mocking here,
unlike test_ollama_client.py.

Usage (after `colcon build --packages-select walker_llm_bridge` and
`source install/setup.bash` from src/):

    python3 tools/verify_llm_bridge.py

Exits 0 and prints PASS on success, exits 1 and prints FAIL otherwise.
"""
import os
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String


class VerifyNode(Node):
    def __init__(self):
        super().__init__('walker_llm_bridge_verify')
        self.text_in_messages = []
        self.text_out_messages = []
        self.stop_requested_count = 0
        self.create_subscription(String, '/llm_bridge/text_in', self._on_text_in, 10)
        self.create_subscription(String, '/llm_bridge/text_out', self._on_text_out, 10)
        self.create_subscription(Empty, '/llm_bridge/stop_requested', self._on_stop, 10)

    def _on_text_in(self, msg):
        self.text_in_messages.append(msg.data)

    def _on_text_out(self, msg):
        self.text_out_messages.append(msg.data)

    def _on_stop(self, msg):
        self.stop_requested_count += 1


def _spin_until(node, predicate, timeout_s):
    deadline = time.monotonic() + timeout_s
    while not predicate() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
    return predicate()


def main():
    fifo_dir = tempfile.mkdtemp(prefix='walker_llm_bridge_verify_')
    fifo_path = os.path.join(fifo_dir, 'stdin_fifo')
    os.mkfifo(fifo_path)

    node_process = subprocess.Popen(
        f'exec ros2 run walker_llm_bridge llm_bridge_node < {fifo_path}',
        shell=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    # Blocks until node_process's shell redirection opens the FIFO's read
    # end - standard FIFO open-pairing, no race or O_NONBLOCK trick needed.
    fifo_write = open(fifo_path, 'w')

    rclpy.init()
    node = VerifyNode()

    try:
        time.sleep(2.0)  # let the node finish declaring parameters/subscriptions

        fifo_write.write('hello there\n')
        fifo_write.flush()
        if not _spin_until(node, lambda: len(node.text_in_messages) >= 1, timeout_s=5.0):
            print('FAIL: no /llm_bridge/text_in echo received within 5s')
            return 1
        if node.text_in_messages[0] != 'hello there':
            print(f"FAIL: /llm_bridge/text_in echoed {node.text_in_messages[0]!r}, expected 'hello there'")
            return 1

        if not _spin_until(node, lambda: len(node.text_out_messages) >= 1, timeout_s=30.0):
            print('FAIL: no /llm_bridge/text_out response received within 30s (Ollama round-trip)')
            return 1
        print(f'Round-trip response: {node.text_out_messages[0]!r}')

        fifo_write.write('stop\n')
        fifo_write.flush()
        if not _spin_until(node, lambda: node.stop_requested_count >= 1, timeout_s=5.0):
            print('FAIL: /llm_bridge/stop_requested did not fire within 5s')
            return 1

        time.sleep(3.0)
        rclpy.spin_once(node, timeout_sec=0.5)
        if len(node.text_out_messages) != 1:
            print(
                'FAIL: expected no new /llm_bridge/text_out after stop utterance, '
                f'got {len(node.text_out_messages)} total messages'
            )
            return 1

        print('PASS: text_in echo, Ollama round-trip response, and stop-intent short-circuit all verified')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
        fifo_write.close()
        node_process.terminate()
        try:
            node_process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            node_process.kill()
            node_process.wait()


if __name__ == '__main__':
    sys.exit(main())
```

- [ ] **Step 5: Syntax-check the verification script**

```bash
source /opt/ros/humble/setup.bash
python3 -m py_compile /home/peter_sha/sourcecode/smart-walker-bot/src/walker_llm_bridge/tools/verify_llm_bridge.py
```

Expected: no output, exit code 0.

- [ ] **Step 6: Build the full package**

```bash
source /opt/ros/humble/setup.bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src
PYTHONNOUSERSITE=1 colcon build --packages-select walker_llm_bridge --symlink-install
source install/setup.bash
```

Expected: `Summary: 1 package finished`.

- [ ] **Step 7: Confirm Ollama is reachable before the end-to-end run**

```bash
curl -s --max-time 3 http://192.168.1.20:11434/api/tags | head -c 300
```

Expected: JSON containing `"models"` with `qwen3.5-9b-64k:latest` listed. If this fails, the Ollama server is down or unreachable from this workstation right now — fix that before running Step 8, since `verify_llm_bridge.py` has no fallback path for this case (unlike the node's own `OllamaError` handling, which this check does not exercise).

- [ ] **Step 8: Run the end-to-end verification**

```bash
source /opt/ros/humble/setup.bash
source /home/peter_sha/sourcecode/smart-walker-bot/src/install/setup.bash
cd /home/peter_sha/sourcecode/smart-walker-bot/src/walker_llm_bridge

python3 tools/verify_llm_bridge.py
echo "verify_llm_bridge.py exit code: $?"
```

Expected: prints `Round-trip response: '...'` then `PASS: text_in echo, Ollama round-trip response, and stop-intent short-circuit all verified`, exit code `0`. If the round-trip step times out, confirm the node actually started (`ros2 node list` while a manual run is active) and that Step 7's curl check still succeeds.

- [ ] **Step 9: Verify the backend-parameter error path**

```bash
source /opt/ros/humble/setup.bash
source /home/peter_sha/sourcecode/smart-walker-bot/src/install/setup.bash
timeout 5 ros2 run walker_llm_bridge llm_bridge_node --ros-args -p voice_io_backend:=bogus
echo "exit code: $?"
```

Expected: the node raises `ValueError: Unknown voice_io_backend 'bogus' - ...` and exits with a non-zero code, confirming the process actually exits rather than hangs.

- [ ] **Step 10: Update src/README.md**

Read `src/README.md`, then in the "Planned packages" list change:

```markdown
- **`walker_llm_bridge`** — voice I/O (STT/TTS) and the connection to the
  Ollama server for the conversational layer (§5.3).
```

to:

```markdown
- **Built (text bridge).** **`walker_llm_bridge`** — text-based
  conversational bridge to the Ollama server (§5.3); real STT/TTS and
  nav-goal translation still deferred (see the package's own README).
```

And after the existing "Build/test `walker_nav`" bash block, add:

```markdown
Build/test `walker_llm_bridge`:

\`\`\`bash
source /opt/ros/humble/setup.bash
cd src
PYTHONNOUSERSITE=1 colcon build --packages-select walker_llm_bridge --symlink-install
source install/setup.bash

python3 -m pytest walker_llm_bridge/test/ -v   # pure-module unit tests, no ROS sourcing needed
\`\`\`
```

- [ ] **Step 11: Update CLAUDE.md**

Read `CLAUDE.md`, then in the "Project status" section change:

```markdown
Three packages exist under `src/`: `walker_safety` (E-stop wiring docs + Pico watchdog
firmware - not a colcon package, see its own README), `walker_motor_driver` (a real
`ament_python` ROS2 node - differential-drive motor control backed by a simulator until real
hardware exists), and `walker_nav` (a real `ament_python` ROS2 package - a simulated LiDAR
feeding `slam_toolbox` for mapping, backed by a fixed hardcoded room until real hardware
exists; Nav2 navigates autonomously against that live map, using `nav2_bringup`'s own
navigation stack).
```

to:

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

And after the existing "Build/test `walker_nav`" bash block, add:

```markdown
Build/test `walker_llm_bridge`:

\`\`\`bash
source /opt/ros/humble/setup.bash
cd src
PYTHONNOUSERSITE=1 colcon build --packages-select walker_llm_bridge --symlink-install
python3 -m pytest walker_llm_bridge/test/ -v   # pure-module unit tests, no ROS sourcing needed
\`\`\`
```

And change:

```markdown
Remaining planned packages (`walker_llm_bridge`, `walker_companion_app`) don't exist yet.
```

to:

```markdown
The remaining planned package (`walker_companion_app`) doesn't exist yet.
```

- [ ] **Step 12: Commit**

```bash
cd /home/peter_sha/sourcecode/smart-walker-bot
git add src/walker_llm_bridge/walker_llm_bridge/llm_bridge_node.py \
        src/walker_llm_bridge/launch/llm_bridge.launch.py \
        src/walker_llm_bridge/tools/verify_llm_bridge.py \
        src/README.md \
        CLAUDE.md
git commit -m "$(cat <<'EOF'
Add walker_llm_bridge ROS2 node, launch file, and E2E verification

llm_bridge_node.py wires OllamaClient + stop_intent + TextIoBackend
together, publishing conversation and stop-intent events on ROS2
topics for future walker_companion_app observability. Verified
end-to-end against the real reachable Ollama server with
tools/verify_llm_bridge.py, which drives the node's stdin via a named
pipe since text_in/text_out are published (not subscribed) topics.
Updates src/README.md and CLAUDE.md to reflect the new package.
EOF
)"
```

---

## Self-Review Notes

- **Spec coverage:** §2.1 (VoiceIOBackend abstraction) — Task 4, wired into the node in Task 5. §2.2 (pure OllamaClient) — Task 2. §2.3 (stop-intent stub, publish-only) — Task 3 (detection) + Task 5 (publish, no consumer), with the exact-vs-substring-match refinement called out explicitly in Global Constraints rather than silently diverging from the spec's looser wording. §2.4 (in-memory history, text_in/text_out published for observability) — Task 5's `_on_utterance`, corrected direction reflected in Global Constraints and the spec fix committed before this plan was written. §2.5 (graceful Ollama-failure degradation) — Task 5's `except OllamaError` branch. §3 (file structure) — matches exactly. §4 (interface: params/topics) — Task 5's `declare_parameter` calls and publishers match the table verbatim. §5 (testing approach, corrected) — Tasks 2-4 are pytest-TDD; Task 5's node gets the FIFO-based scripted `verify_llm_bridge.py` check, run as a real pass/fail gate against the actual reachable Ollama server (confirmed reachable via `curl` during design and re-checked in Task 5 Step 7). §6 (out of scope) — no real STT/TTS, no nav-goal translation, no `/llm_bridge/stop_requested` consumer, no disk persistence, and no `walker_companion_app` work appears anywhere in this plan.
- **Placeholder scan:** no TBD/TODO in any step. Task 1 Step 3's placeholder launch file (used only if Task 1 Step 6's build fails without it) is a real, valid, working `LaunchDescription([])`, and gets overwritten by Task 5's real launch file regardless.
- **Type/name consistency:** `OllamaClient(host, port, model, timeout_s)` / `.chat(messages) -> str` and `OllamaError` are used identically in Task 2's tests and Task 5's node. `is_stop_utterance(text) -> bool` is used identically in Task 3's tests and Task 5's node. `VoiceIOBackend.start(on_utterance)`/`.speak(text)`/`.stop()` and `TextIoBackend(input_stream=None, output_stream=None)` are used identically in Task 4's tests and Task 5's node (`self._backend.start(...)`, `self._backend.speak(...)`, `node._backend.stop()`). Parameter names (`voice_io_backend`, `ollama_host`, `ollama_port`, `ollama_model`, `ollama_timeout_s`, `system_prompt`, `max_history_messages`) match between Task 5's node's `declare_parameter` calls and its launch file. Topic names (`/llm_bridge/text_in`, `/llm_bridge/text_out`, `/llm_bridge/stop_requested`) match between Task 5's node and its own verification script's subscriptions.
