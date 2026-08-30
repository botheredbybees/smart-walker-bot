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
