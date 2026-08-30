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
