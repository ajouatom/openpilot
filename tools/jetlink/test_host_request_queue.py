"""Fragment assembly must not consume the logical inference-window queue."""
import queue
import threading
import pytest
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "third_party/jetlink"))
from host_reader import ReadAheadTransport
from jetlink import protocol as P, request_chunks as C
from jetlink.transport.base import Message, LinkError, LinkTimeout

class Input:
  protocol_version = 2
  def __init__(self, messages, target=None):
    self.target = len(messages) if target is None else target
    self.received = 0
    self.messages = queue.Queue()
    for msg in messages: self.messages.put(msg)
    self.drained = threading.Event()
    self.received_all = threading.Event()
  def recv(self, timeout):
    try:
      msg = self.messages.get(timeout=.02)
      self.received += 1
      if self.received >= self.target: self.received_all.set()
      return msg
    except queue.Empty:
      self.drained.set()
      raise LinkTimeout('idle')

def envelopes(kind, seq, raw):
  out = []
  def send(kind, seq, parts, **kw): out.append(Message(kind, seq, 0, memoryview(b''.join(parts))))
  C.send(send, kind, seq, (raw,))
  return out

@pytest.mark.parametrize('replay', [False, True])
@pytest.mark.parametrize('count', [2, 3])
def test_gpu_busy_reader_queues_two_complete_requests_not_fragments(replay, count):
  first = b'a' * 96000
  second = b'b' * 32792
  msgs = envelopes(0x4000, 8, first) + envelopes(0x4001, 9, second)
  if count == 3: msgs += envelopes(0x4001, 10, b'c' * 32792)
  if replay: msgs = [m for msg in msgs for m in (msg, msg)]
  source = Input(msgs, len(msgs) - int(count == 3 and replay)); reader = ReadAheadTransport(source)
  try:
    gate = source.drained if count == 2 else source.received_all
    # The third complete input may be reader-owned with two slots full;
    # its final duplicate can be drained after the GPU releases the consumer.
    assert gate.wait(.5), 'fragment queue blocked while GPU consumer is busy'
    assert reader.queue.maxsize == 2 and reader.queue.qsize() == 2
    a, b = reader.recv(.1), reader.recv(.1)
    assert (a.msg_type, a.seq, bytes(a.payload)) == (0x4000, 8, first)
    assert (b.msg_type, b.seq, bytes(b.payload)) == (0x4001, 9, second)
    if count == 3:
      c = reader.recv(.5)
      assert (c.msg_type, c.seq, bytes(c.payload)) == (0x4001, 10, b'c' * 32792)
  finally: reader.close()

def test_corrupt_request_never_reaches_inference_queue():
  msgs = envelopes(P.Msg.INFER_REQ, 7, b'x' * 100000)
  last = msgs[-1]
  last.payload = memoryview(bytes(last.payload[:-1]) + b'z')
  reader = ReadAheadTransport(Input(msgs))
  try:
    with pytest.raises(LinkError, match='chunked request'): reader.recv(.5)
  finally: reader.close()

def test_partial_request_does_not_cross_hello_and_old_replays_do_not_reassemble():
  part = envelopes(P.Msg.INFER_REQ, 7, b'old' * 20000)[0]
  hello = Message(P.Msg.HELLO_REQ, 1, 0, memoryview(b'hello'))
  fresh = envelopes(P.Msg.INFER_REQ, 2, b'new' * 20000)
  source = Input([part, hello, *fresh, *fresh]); reader = ReadAheadTransport(source)
  try:
    assert reader.recv(.5).msg_type == P.Msg.HELLO_REQ
    out = reader.recv(.5)
    assert out.seq == 2 and bytes(out.payload) == b'new' * 20000
    assert source.drained.wait(.5)
    with pytest.raises(LinkTimeout): reader.recv(.02)
  finally: reader.close()


def test_actual_session_executes_one_owned_input_after_reader_reassembly():
  from types import SimpleNamespace
  from server import CarrotSession
  raw = b'camera' * 75000
  msgs = envelopes(P.Msg.INFER_REQ, 7, raw)
  source = Input([m for msg in msgs for m in (msg, msg)])
  reader = ReadAheadTransport(source)
  session = object.__new__(CarrotSession)
  session.request_chunks = C.Receiver(); session.last_seq = 0
  session.t = SimpleNamespace(protocol_version=2); session.client = 'test'
  outputs = []; session.on_infer = lambda msg: outputs.append(bytes(msg.payload))
  try:
    session.handle(reader.recv(.5))
    assert source.drained.wait(.5)
    assert outputs == [raw] and session.last_seq == 7
    with pytest.raises(LinkTimeout): reader.recv(.02)
  finally: reader.close()

@pytest.mark.parametrize('version', [2, 3])
def test_interrupted_or_wrong_protocol_fails_without_partial_input(version):
  msgs = envelopes(P.Msg.INFER_REQ, 7, b'x' * 100000)[:1]
  msgs.append(Message(P.Msg.STATE_REQ, 8, 0, memoryview(b'')))
  source = Input(msgs); source.protocol_version = version
  reader = ReadAheadTransport(source)
  try:
    with pytest.raises(LinkError): reader.recv(.5)
  finally: reader.close()
  assert reader.requests.data is None and not reader.thread.is_alive()
