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
