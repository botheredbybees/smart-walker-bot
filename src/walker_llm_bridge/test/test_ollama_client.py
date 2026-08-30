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
