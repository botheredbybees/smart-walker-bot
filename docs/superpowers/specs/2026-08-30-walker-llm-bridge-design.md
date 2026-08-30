# walker_llm_bridge Design

**Date:** 2026-08-30
**Status:** Approved by user; ready for implementation planning
**Scope:** First design pass for `README.md` §6 step 5 / §5.3's conversational layer: the
Ollama connection and a text-based conversational bridge, developed sim-first against a text
I/O backend since this dev workstation has no mic/speaker or robot hardware yet (same posture
`walker_nav`/`walker_motor_driver` took toward LiDAR/motor hardware, per the deviation from
`docs/superpowers/specs/2026-08-30-phase1-roadmap-design.md` §5's original sequencing — approved
by user to start now rather than wait for the hardware bring-up checkpoint). Does not cover real
STT/TTS, natural-language nav-goal translation, or `walker_companion_app` (step 6).

## 1. Problem

`README.md` §5.3 describes three responsibilities for the conversational layer: voice I/O
(STT/TTS), a connection to the Ollama server for conversation, and translating natural-language
nav commands into Nav2 goals. None of these have a concrete design yet, and two of the three
have hard prerequisites this project doesn't have yet:

- Voice I/O needs a mic/speaker, which doesn't exist on this dev workstation or the (not yet
  assembled) robot.
- Nav-goal translation needs named locations to target, and `walker_nav`'s room map
  (`src/walker_nav/walker_nav/room_map.py`) is still a fixed, unnamed two-room sim floor plan
  with nothing to translate "the shed" or similar into.

The Ollama connection itself has no such blocker — the target server (`192.168.1.20:11434`) is
reachable from this workstation right now. (The model tag was later corrected during
implementation — see `docs/superpowers/plans/2026-08-30-walker-llm-bridge.md`'s Global
Constraints: `qwen3.5-9b-64k:latest` is listed by the server but hangs indefinitely on
`/api/chat`; `qwen2.5:14b` is the working default.) This design scopes the package to what's
actually buildable and testable today: a
conversational bridge over text, with the sim/real boundary drawn explicitly so voice I/O can be
added later without changing this pass's node logic — the same boundary discipline
`walker_motor_driver`'s `MotorBackend` used for the sim/GPIO split
(`docs/superpowers/specs/2026-08-30-walker-motor-driver-design.md` §2.3).

## 2. Decisions

### 2.1 Real ROS2/colcon package, `VoiceIOBackend` abstraction for the I/O boundary

Same shape as `walker_motor_driver`: a standard `ament_python` package, with a small backend
interface separating the node's conversation logic from how utterances actually enter/exit the
system.

```python
class VoiceIOBackend:
    def start(self, on_utterance) -> None: ...  # register a callback: str -> None
    def speak(self, text: str) -> None: ...
    def stop(self) -> None: ...
```

`TextIoBackend` implements this now: a background daemon thread reads lines from stdin and
invokes the callback per non-empty line; `speak()` prints the response to stdout with a
`walker>` prefix. `stop()` is a documented no-op — `stdin.readline()` can't be cleanly
interrupted mid-block, so the thread is a daemon and simply dies with the process, the same
"nothing to release" rationale `SimMotorBackend.stop()` uses for a sim with nothing physical to
de-energize. A future STT/TTS backend is added the same way `GpioMotorBackend` will be: a new
class, selected by a launch/node argument, with the node's control logic unchanged.

Chosen over building STT/TTS now (rejected — no mic/speaker hardware exists to test against, so
that code would be unverifiable and speculative) and over skipping the abstraction and wiring
stdin directly into the node (rejected for the same reason `walker_motor_driver` rejected a
direct sim call: it would reintroduce exactly the rework the boundary is meant to avoid once
real voice hardware exists).

### 2.2 Ollama client is a pure Python module, not folded into the node

`ollama_client.py`'s `OllamaClient` wraps `host`/`port`/`model`/`timeout_s` and exposes
`.chat(messages) -> str`, calling `POST http://{host}:{port}/api/chat` (`stream: False`) via
`requests` (already present in this environment; no new dependency). It raises a single
`OllamaError` on any connection failure, timeout, or malformed response, rather than leaking
`requests` exceptions or partial-response shapes to callers. No `rclpy` import — unit-testable
with `unittest.mock.patch('requests.post', ...)`, matching `diff_drive_kinematics.py`'s
pure-module pattern. Chosen over a thin inline call inside the node so the HTTP/JSON contract
with Ollama is testable without spinning up ROS2 or a real server.

### 2.3 Stop-intent detection is a stub: publish-only, no coupling to anything that acts

`stop_intent.py`'s `is_stop_utterance(text) -> bool` does simple case-insensitive matching
against a small fixed phrase list (`"stop"`, `"halt"`, `"emergency stop"`, `"stop now"`). When
the node detects it, it publishes `std_msgs/Empty` on `/llm_bridge/stop_requested`, logs a
warning that this is convenience-only, skips the Ollama call (avoids unnecessary latency on what
might be an urgent utterance), and has the backend `speak()` a short acknowledgment.

Nothing currently subscribes to `/llm_bridge/stop_requested` — there is no motor/safety topic to
wire it to yet, and this design deliberately does not invent one. This mirrors
`walker_motor_driver`'s explicit "no coupling to `walker_safety`"
(`docs/superpowers/specs/2026-08-30-walker-motor-driver-design.md` §2.6) and the project's
own invariant (`README.md` §5.3, CLAUDE.md): voice "stop" is a convenience layer only, must
never be the sole or primary stop path, and must go through (or be backed by) the hardware
E-stop/watchdog. Publishing an inert topic now establishes the interface future work (a real
consumer, once one exists) plugs into, without creating any false impression that saying "stop"
currently stops anything.

### 2.4 In-memory conversation history, no persistence

The node keeps a rolling list of `{role, content}` messages (capped at `max_history_messages`,
default 20) purely in memory, prepended with a configurable `system_prompt` param on each Ollama
call. No disk persistence and no conversation log — `README.md` §5.5 assigns a conversation log
to `walker_companion_app`, which doesn't exist yet; duplicating that here would be built ahead of
the package that actually needs it. `/llm_bridge/text_in` and `/llm_bridge/text_out`
(`std_msgs/String`) are published regardless of backend so a future `walker_companion_app` can
subscribe and log without any change to this package.

### 2.5 Graceful degradation on Ollama failure

If `OllamaClient.chat` raises `OllamaError` (server unreachable, timeout, bad response), the
node logs the error and has the backend `speak()` a fixed fallback ("I can't reach the LLM
server right now") rather than crashing or hanging. Chosen because the Ollama server is a
separate machine on the home network (`README.md` §5.6) reachable over infrastructure this
project doesn't control — treating it as unreliable-by-default is more honest than assuming it's
always up, and matches how `motor_driver_node.py` treats a stalled `/cmd_vel` stream (zero
speeds rather than hang) as ordinary defensive behavior for an external, possibly-absent input.

## 3. Package structure

New `ament_python` package:

```
src/walker_llm_bridge/
  package.xml, setup.py, setup.cfg, resource/walker_llm_bridge
  walker_llm_bridge/
    __init__.py
    ollama_client.py       (pure: OllamaClient, OllamaError)
    stop_intent.py          (pure: is_stop_utterance)
    voice_io_backend.py     (VoiceIOBackend interface)
    text_io_backend.py      (TextIoBackend)
    llm_bridge_node.py      (rclpy node, wires the above together)
  launch/llm_bridge.launch.py (voice_io_backend:=text argument, default text)
  test/
    conftest.py
    test_ollama_client.py
    test_stop_intent.py
  tools/
    verify_llm_bridge.py    (scripted end-to-end check, not pytest)
```

## 4. Interface

**Node:** `walker_llm_bridge` (entry point `llm_bridge_node`)

**Params:**
| Param | Default | Notes |
|---|---|---|
| `voice_io_backend` | `text` | only `text` implemented this pass |
| `ollama_host` | `192.168.1.20` | |
| `ollama_port` | `11434` | Ollama's default |
| `ollama_model` | `qwen2.5:14b` | corrected during implementation — `qwen3.5-9b-64k:latest` is listed by the server but hangs indefinitely on `/api/chat`; see plan Global Constraints |
| `ollama_timeout_s` | `30.0` | |
| `system_prompt` | short companion-robot persona string | |
| `max_history_messages` | `20` | |

**Topics published:**
- `/llm_bridge/text_in` (`std_msgs/String`) — each utterance, before any processing.
- `/llm_bridge/text_out` (`std_msgs/String`) — each LLM (or fallback) response.
- `/llm_bridge/stop_requested` (`std_msgs/Empty`) — stop-intent detected; unconsumed this pass.

## 5. Testing

`ollama_client.py` and `stop_intent.py` are pure Python — unit-tested with pytest,
`requests.post` mocked via `unittest.mock.patch`, no ROS sourcing needed, same pattern as
`diff_drive_kinematics.py`/`watchdog_logic.py`.

`tools/verify_llm_bridge.py` — scripted (not pytest) end-to-end check. `/llm_bridge/text_in` is
published *by* the node (§2.4) — the node's only real utterance path is the backend (§2.1) — so
driving the conversation from a script means injecting into the `text` backend's actual stdin,
not publishing to a topic. The script launches the node with its stdin redirected from a named
pipe (`mkfifo`), writes an utterance line to that pipe, and confirms: (1) that same text arrives
echoed on `/llm_bridge/text_in`, (2) a real round-trip response arrives on `/llm_bridge/text_out`
from the actual reachable Ollama server. A second case writes a stop utterance to the pipe and
confirms `/llm_bridge/stop_requested` fires and no new `/llm_bridge/text_out` message is produced
for that line (i.e. Ollama was not called).

## 6. Out of scope

- Real STT/TTS backend — deferred to hardware bring-up once a mic/speaker exists to test
  against; `VoiceIOBackend` is the interface it will implement.
- Natural-language nav-goal translation ("take me back to the shed") — deferred until
  `walker_nav` has named locations to target; not stubbed in this pass.
- Any consumer of `/llm_bridge/stop_requested`, or any behavior beyond publishing it — explicitly
  rejected per §2.3, not merely deferred, to avoid implying voice stop does anything today.
- Conversation persistence/logging to disk — `walker_companion_app`'s responsibility
  (`README.md` §5.5), not duplicated here.
- `walker_companion_app` itself (step 6) — designed separately once this spec is implemented.
