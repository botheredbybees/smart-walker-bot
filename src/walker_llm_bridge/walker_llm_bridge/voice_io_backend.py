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
