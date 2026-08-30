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
