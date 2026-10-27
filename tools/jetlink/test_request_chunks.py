import errno
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'third_party/jetlink'))
from jetlink import protocol as P, request_chunks as C
from jetlink.transport.base import LinkError, LinkTimeout, Message, StreamTransport


def envelopes(raw, seq=7, kind=P.Msg.INFER_REQ):
  result = []
  C.send(lambda k, s, parts, **kw: result.append(Message(k, s, 0, memoryview(b''.join(parts)))), kind, seq, (raw,))
  return result


@pytest.mark.parametrize('size', [1, C.DATA_BYTES, C.DATA_BYTES + 1, 458996, 655608, C.MAX_BYTES])
def test_lossless_assembly_with_every_envelope_replayed(size):
  raw = bytes((i * 17 + 3) % 256 for i in range(size))
  receiver = C.Receiver(); completed = []; last = 0
  for msg in envelopes(raw):
    for _ in range(2):
      output = receiver.accept(msg, last)
      if output is not None:
        completed.append(bytes(output.payload)); last = output.seq
  assert completed == [raw]
  assert receiver.key is None and receiver.data is None


@pytest.mark.parametrize('fault', ['missing_first', 'missing_middle', 'changed_duplicate', 'changed_crc', 'wrong_seq', 'expired', 'unknown_kind', 'oversized'])
def test_corrupt_incomplete_or_expired_input_never_executes(fault, monkeypatch):
  now = [10.]; monkeypatch.setattr(C.time, 'monotonic', lambda: now[0])
  msgs = envelopes(b'x' * (C.DATA_BYTES * 3 + 7)); receiver = C.Receiver()
  if fault == 'missing_first': items = msgs[1:]
  else:
    assert receiver.accept(msgs[0], 0) is None
    items = msgs[1:]
    if fault == 'missing_middle': items = msgs[2:]
    if fault == 'changed_duplicate':
      payload = bytearray(msgs[0].payload); payload[-1] ^= 1
      items = [Message(C.MESSAGE, 7, 0, memoryview(payload))]
    if fault == 'changed_crc':
      payload = bytearray(msgs[1].payload); payload[-1] ^= 1
      items = [Message(C.MESSAGE, 7, 0, memoryview(payload)), *msgs[2:]]
    if fault == 'wrong_seq': items = [Message(C.MESSAGE, 8, 0, msgs[1].payload)]
    if fault == 'expired': now[0] += C.MAX_AGE + .001
    if fault in ('unknown_kind', 'oversized'):
      kind, flags, total, offset, checksum = C.HEADER.unpack_from(msgs[1].payload)
      payload = C.HEADER.pack(999 if fault == 'unknown_kind' else kind, flags,
                              C.MAX_BYTES + 1 if fault == 'oversized' else total, offset, checksum) + bytes(msgs[1].payload[C.HEADER.size:])
      items = [Message(C.MESSAGE, 7, 0, memoryview(payload))]
  with pytest.raises(LinkError):
    for msg in items: assert receiver.accept(msg, 0) is None
  assert receiver.data is None


def test_sender_deadline_covers_all_envelopes(monkeypatch):
  now = [0.]; monkeypatch.setattr(C.time, 'monotonic', lambda: now[0]); budgets = []
  def send(*args, timeout):
    budgets.append(timeout); now[0] += .06
  with pytest.raises(LinkTimeout): C.send(send, P.Msg.INFER_REQ, 9, (b'x' * 200000,), timeout=.15)
  assert budgets == pytest.approx([.15, .09, .03])


def test_clear_discards_old_partial_session():
  receiver = C.Receiver(); msgs = envelopes(b'x' * 100000)
  receiver.accept(msgs[0], 0); receiver.clear()
  with pytest.raises(LinkError): receiver.accept(msgs[1], 0)
  assert bytes(receiver.accept(envelopes(b'new', seq=1)[0], 0).payload) == b'new'


def test_descriptor_is_strict():
  assert C.supported(C.descriptor())
  for value in (True, None, {}, dict(C.descriptor(), version=True), dict(C.descriptor(), wire_bytes=16384)):
    assert not C.supported(value)


class WireReader(StreamTransport):
  rx_align = P.GADGET_TX_ALIGN
  def __init__(self, data): super().__init__(); self.data = bytearray(data)
  def _read_into(self, dest, timeout):
    n = min(len(dest), len(self.data)); dest[:n] = self.data[:n]; del self.data[:n]; return n
  def _write(self, parts): raise AssertionError('receive only')
  def close(self): pass


@pytest.mark.skipif(sys.platform != 'linux', reason='FunctionFS sender and signal module require Linux')
def test_actual_ffs_sender_reproduces_old_allocation_failure_and_survives_replay():
  from jetlink.transport.ffs import FfsTransport
  class MemoryGate(FfsTransport):
    def __init__(self, enabled):
      StreamTransport.__init__(self); self.request_chunks_enabled = enabled; self.writes = []
    def _write(self, parts):
      raw = b''.join(parts)
      if len(raw) > C.WIRE_BYTES: raise OSError(errno.ENOMEM, 'synthetic contiguous allocation pressure')
      self.writes.append(raw); return len(raw)
    def close(self): pass
  raw = bytes(i % 251 for i in range(458996))
  with pytest.raises(OSError) as exc: MemoryGate(False).send(P.Msg.INFER_REQ, 7, (raw,), timeout=.5)
  assert exc.value.errno == errno.ENOMEM
  tx = MemoryGate(True); tx.send(P.Msg.INFER_REQ, 7, (raw,), timeout=.5)
  assert len(tx.writes) == 15 and max(map(len, tx.writes)) == C.WIRE_BYTES
  # Repeat each complete USB write as observed on DWC3. Real stream parser must
  # stay aligned and reconstruct exactly once, not merely parse mocked objects.
  reader = WireReader(b''.join(packet * 2 for packet in tx.writes)); receiver = C.Receiver(); results = []; last = 0
  for _ in range(len(tx.writes) * 2):
    msg = reader.recv(.5); result = receiver.accept(msg, last)
    if result is not None: results.append(bytes(result.payload)); last = result.seq
  assert results == [raw]


def test_actual_session_executes_once_after_complete_request():
  import server
  session = object.__new__(server.CarrotSession)
  session.request_chunks = C.Receiver(); session.last_seq = 0
  session.t = NS(protocol_version=2); session.client = 'test'
  executions = []; session.on_infer = lambda msg: executions.append(bytes(msg.payload))
  raw = b'input' * 100000
  msgs = envelopes(raw)
  for msg in msgs[:-1]:
    session.handle(msg); session.handle(msg)
    assert not executions and session.last_seq == 0
  session.handle(msgs[-1]); session.handle(msgs[-1])
  assert executions == [raw] and session.last_seq == 7


def test_new_session_rejects_partial_and_interleaved_requests():
  import server
  session = object.__new__(server.CarrotSession)
  session.request_chunks = C.Receiver(); session.last_seq = 0
  session.t = NS(protocol_version=2); session.client = 'test'
  session.on_infer = lambda msg: pytest.fail('incomplete input executed')
  session.handle(envelopes(b'x' * 100000)[0])
  with pytest.raises(LinkError): session.handle(Message(P.Msg.INFER_REQ, 8, 0, memoryview(b'bad')))
  assert session.last_seq == 0 and session.request_chunks.data is None


@pytest.mark.parametrize('capability,host,allowed,expected', [(C.descriptor(), 'jetson', True, True),
  (None, 'jetson', True, False), (True, 'jetson', True, False), (C.descriptor(), 'mac', True, False),
  (C.descriptor(), 'jetson', False, False)])
def test_hello_negotiates_and_resets_per_connection(capability, host, allowed, expected):
  import json
  from jetlink.client import JetlinkClient
  class FakeTransport:
    protocol_version = 2
    request_chunks_enabled = True
    def _send_bounded_request(self): pass
    def _prepare_bounded_requests(self): return True
    def send_json(self, *args): pass
  transport = FakeTransport(); client = JetlinkClient(transport, allow_request_chunks=allowed)
  response = dict(protocol=2, carrot_host=host, stateful_v2=True)
  if capability is not None: response[C.CAPABILITY] = capability
  def expect(*args):
    assert transport.request_chunks_enabled is False
    return NS(payload=json.dumps(response).encode())
  client._expect = expect
  client.hello()
  assert transport.request_chunks_enabled is expected

@pytest.mark.skipif(sys.platform != 'linux', reason='FunctionFS writev path requires Linux')
def test_real_ffs_writev_retry_stalls_old_request_but_not_bounded_envelopes(monkeypatch):
  from jetlink.transport.ffs import FfsTransport
  import jetlink.transport.ffs as ffs
  now = [0.]
  monkeypatch.setattr(C.time, 'monotonic', lambda: now[0])
  class NativeWritePath(FfsTransport):
    def __init__(self, enabled):
      StreamTransport.__init__(self)
      self.request_chunks_enabled = enabled
      self.ep_in = -1
      self._write_aborted = False
      self._write_guard = NS(arm=lambda budget: True, disarm=lambda: True)
    def _ensure_epfiles(self): pass
    def close(self): pass
  import _signal, signal
  before = _signal.pthread_sigmask(signal.SIG_BLOCK, ())
  monkeypatch.setattr(signal, 'pthread_sigmask', lambda *args: pytest.fail('per-write enum conversion'))
  attempts = []
  def writev(fd, parts):
    assert int(signal.SIGUSR1) in _signal.pthread_sigmask(signal.SIG_BLOCK, ())
    size = sum(len(p) for p in parts); attempts.append(size)
    if size > C.WIRE_BYTES:
      now[0] += .08  # controlled simulated reclaim stall, not wall-clock sleep
      raise OSError(errno.ENOMEM, 'synthetic high-order allocation failure')
    now[0] += .0001
    return size
  monkeypatch.setattr(ffs.os, 'writev', writev)
  raw = b'x' * 458996
  NativeWritePath(False).send(P.Msg.INFER_REQ, 1, (raw,), timeout=.5)
  assert max(attempts) > 400000 and now[0] >= .32
  attempts.clear(); now[0] = 0.
  NativeWritePath(True).send(P.Msg.INFER_REQ, 1, (raw,), timeout=.5)
  assert len(attempts) == 15 and max(attempts) == C.WIRE_BYTES
  assert now[0] < .002
  # A small allocation failure must not shrink a framed envelope into raw bytes.
  attempts.clear()
  def fail_small(fd, parts):
    attempts.append(sum(len(p) for p in parts))
    raise OSError(errno.ENOMEM, 'synthetic small allocation failure')
  monkeypatch.setattr(ffs.os, 'writev', fail_small)
  tx = NativeWritePath(True)
  with pytest.raises(LinkError, match='small framed USB request allocation failed'):
    tx.send(P.Msg.INFER_REQ, 2, (raw,), timeout=.5)
  assert attempts == [C.WIRE_BYTES]
  assert tx._bounded_envelope is False
  assert _signal.pthread_sigmask(signal.SIG_BLOCK, ()) == before

@pytest.mark.skipif(sys.platform != 'linux', reason='FunctionFS sender integration')
def test_same_frame_writer_receives_complete_replay_safe_envelopes():
  from jetlink.transport.ffs import FfsTransport
  tx = object.__new__(FfsTransport)
  StreamTransport.__init__(tx)
  tx.request_chunks_enabled = True
  arms = []
  tx._write_guard = NS(arm=lambda budget: arms.append(budget) or True, disarm=lambda: True)
  captured = []
  tx._request_writer = NS(send=lambda packets, budget: captured.extend(packets))
  raw = b'input' * 91799
  tx.send(P.Msg.INFER_REQ, 42, (raw,), timeout=.15)
  assert len(arms) == 1 and 0 < arms[0] <= .15
  assert len(captured) == 15 and max(map(len, captured)) == C.WIRE_BYTES
  reader=WireReader(b''.join(packet * 2 for packet in captured));receiver=C.Receiver();results=[];last=0
  for _ in range(len(captured)*2):
    result=receiver.accept(reader.recv(.5),last)
    if result is not None:results.append(bytes(result.payload));last=result.seq
  assert results==[raw] and tx._had_host
  assert not hasattr(tx, '_collect_envelopes') and tx._bounded_envelope is False


@pytest.mark.parametrize('size', [1, 32720, 32721, 65440, 458996, 655608, 1048576])
@pytest.mark.parametrize('kind', sorted(C.KINDS))
def test_native_packet_builder_is_byte_exact_with_actual_stream_serializer(size, kind):
  class Wire(StreamTransport):
    tx_align = P.GADGET_TX_ALIGN
    def __init__(self): super().__init__(); self.packets = []
    def _write(self, parts):
      packet = b''.join(parts); self.packets.append(packet); return len(packet)
    def _read_into(self, *args): raise AssertionError
    def close(self): pass
  source = bytes(i % 251 for i in range(size))
  parts = (memoryview(source)[:3], memoryview(source)[3:])
  wire = Wire(); C.send(wire.send, kind, 987654, parts)
  assert C.packets(kind, 987654, parts) == wire.packets


@pytest.mark.skipif(sys.platform != 'linux', reason='FunctionFS completion integration')
def test_late_completion_failure_is_checked_before_send_and_after_receive(monkeypatch):
  from jetlink.transport.ffs import FfsTransport
  tx = object.__new__(FfsTransport)
  calls = []
  def failed():
    calls.append('poll')
    raise OSError(errno.ESHUTDOWN, 'late completion')
  tx._request_writer = NS(deferred=True, poll=failed)
  with pytest.raises(LinkError, match='completion failed'):
    tx.send(P.Msg.INFER_REQ, 1, (b'input',))
  assert calls == ['poll']
  calls.clear()
  def second_poll_fails():
    calls.append('poll')
    if len(calls) == 3: raise OSError(errno.ESHUTDOWN, 'late completion')
  def receive(self, timeout):
    calls.append('receive')
    return object()
  tx._request_writer.poll = second_poll_fails
  monkeypatch.setattr(StreamTransport, 'recv', receive)
  with pytest.raises(LinkError, match='completion failed'):
    tx.recv(.05)
  assert calls == ['poll', 'receive', 'poll']


@pytest.mark.skipif(sys.platform != 'linux', reason='FunctionFS cleanup integration')
def test_endpoint_release_closes_writer_before_rebind_and_blocks_on_failure():
  from jetlink.transport.ffs import FfsTransport
  import threading
  tx = object.__new__(FfsTransport)
  tx.gadget='/test'; tx._closing=False; tx.ep_out=12; tx.bound_udc='test'; tx._reader=None
  calls=[]
  tx.unbind=lambda: calls.append('unbind')
  tx._close_fds=lambda names, reader: calls.append('fds')
  tx.bind=lambda udc: calls.append('bind')
  def fail_close():
    calls.append('writer')
    raise OSError(errno.ETIMEDOUT, 'pending kernel buffer')
  writer=NS(close=fail_close)
  tx._request_writer=writer
  with pytest.raises(OSError): tx.release_endpoints()
  assert calls==['unbind','writer','fds'] and tx._request_writer is writer
  writer.close=lambda: calls.append('recovered')
  tx._close_request_writer()
  assert tx._request_writer is None


@pytest.mark.skipif(sys.platform != 'linux', reason='FunctionFS native preparation')
@pytest.mark.parametrize('defer,quantum,expected', [(None,None,(True,8192)), ('0',None,(False,32768)),
  ('1','32768',(True,32768)), ('0','8192',(False,32768)), ('bad','8192',(False,32768))])
def test_default_small_deferred_requests_and_explicit_diagnostic_fallback(monkeypatch, defer, quantum, expected):
  from jetlink.transport.ffs import FfsTransport
  from jetlink.transport import request_writer
  for key, value in [('JETLINK_DEFER_COMPLETIONS',defer), ('JETLINK_AIO_WRITE_BYTES',quantum)]:
    if value is None: monkeypatch.delenv(key,raising=False)
    else: monkeypatch.setenv(key,value)
  calls=[]
  monkeypatch.setattr(request_writer,'library',lambda:object())
  def create(fd,lib,**kw):
    calls.append((kw['deferred'],kw['quantum']))
    return NS(**kw)
  monkeypatch.setattr(request_writer,'Writer',create)
  tx=object.__new__(FfsTransport);tx.ep_in=7;tx._ensure_epfiles=lambda:None
  assert tx._prepare_bounded_requests() and tx._prepare_bounded_requests()
  assert calls==[expected]
