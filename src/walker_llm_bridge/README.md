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

```bash
cd src/walker_llm_bridge
python3 -m pytest test/ -v
```

No ROS environment or colcon build needed for these.

## Running the node

For interactive use with the `text` backend (the only one implemented so far), run the node
directly with `ros2 run`, **not** `ros2 launch`:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run walker_llm_bridge llm_bridge_node --ros-args -p ollama_host:=192.168.1.20 -p ollama_model:=qwen2.5:14b
```

(override any of the other declared parameters the same way, e.g. `-p ollama_timeout_s:=30.0`).

`ros2 launch walker_llm_bridge llm_bridge.launch.py` does **not** work for the `text` backend
today: `TextIoBackend` reads `sys.stdin`, but `ros2 launch` never connects a launched node's
stdin to anything, so the node hangs forever waiting for input that never arrives, with no
error. This is exactly why `tools/verify_llm_bridge.py` launches the node via `ros2 run` (with
a named pipe standing in for a real terminal's stdin) instead of using this launch file. The
launch file still exists for future backends — e.g. once a real STT/TTS backend removes the
stdin dependency, `ros2 launch` will work normally.

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
