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
