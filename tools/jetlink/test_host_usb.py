import sys
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'third_party/jetlink'))
from host_usb import BulkInput
from jetlink.transport.base import LinkError


class Transfer:
  def __init__(self):
    self.submitted = False
    self.submits = self.cancels = 0
    self.closed = False

  def setBulk(self, ep, buf, callback, timeout):
    assert not self.submitted and timeout == 0
    self.buf, self.callback = buf, callback

  def submit(self):
    self.submitted = True
    self.submits += 1

  def cancel(self):
    assert self.submitted
    self.cancels += 1

  def finish(self, data=b'', status=0):
    self.buf[:len(data)] = data
    self.count, self.status = len(data), status
    self.submitted = False
    self.callback(self)

  def getStatus(self): return self.status
  def getActualLength(self): return self.count
  def isSubmitted(self): return self.submitted
  def close(self):
    assert not self.submitted
    self.closed = True


def make_reader(monkeypatch, event):
  class NotFound(Exception): pass
  class Interrupted(Exception): pass
  monkeypatch.setitem(sys.modules, 'usb1', SimpleNamespace(
    TRANSFER_COMPLETED=0, USBErrorNotFound=NotFound, USBErrorInterrupted=Interrupted))
  transfer = Transfer()
  stopped = threading.Event()
  transport = SimpleNamespace(handle=SimpleNamespace(getTransfer=lambda: transfer), ep_in=0x81,
                              _clamp_read=len, context=SimpleNamespace(handleEventsTimeout=lambda tv: event(transfer, stopped)))
  return BulkInput(transport, stopped), transfer, stopped


def test_idle_polls_do_not_cancel_or_resubmit_and_buffer_is_owned(monkeypatch):
  polls = 0
  def event(t, stopped):
    nonlocal polls
    polls += 1
    assert t.submits == 1 and t.cancels == 0
    if polls == 40: t.finish(b'JLNK')
  reader, transfer, _ = make_reader(monkeypatch, event)
  dest = bytearray(16)
  assert reader.read(memoryview(dest), None) == 4
  assert dest[:4] == b'JLNK' and polls == 40
  reader.close()
  assert transfer.closed


def test_close_cancels_once_and_waits_for_callback_before_free(monkeypatch):
  polls = 0
  def event(t, stopped):
    nonlocal polls
    polls += 1
    if polls == 1: stopped.set()
    elif polls == 4: t.finish(b'partial', status=3)
    assert not t.closed
  reader, transfer, _ = make_reader(monkeypatch, event)
  with pytest.raises(LinkError, match='closed'):
    reader.read(memoryview(bytearray(16)), None)
  assert transfer.cancels == 1 and polls == 4
  reader.close()
  assert transfer.closed


def test_disconnect_is_link_error_not_a_valid_partial_message(monkeypatch):
  reader, transfer, _ = make_reader(monkeypatch, lambda t, stop: t.finish(b'partial', status=5))
  with pytest.raises(LinkError, match='status=5'):
    reader.read(memoryview(bytearray(16)), None)
  reader.close()


def test_teardown_refuses_submitted_transfer(monkeypatch):
  reader, transfer, _ = make_reader(monkeypatch, lambda *a: None)
  transfer.submitted = True
  with pytest.raises(LinkError, match='still submitted'):
    reader.close()
  assert not transfer.closed


def test_closed_session_restores_raw_io_for_upstream_desync_drain():
  from host_reader import ReadAheadTransport
  reader = object.__new__(ReadAheadTransport)
  calls = []
  reader.stopped = threading.Event()
  reader.thread = SimpleNamespace(join=lambda timeout: None, is_alive=lambda: False)
  reader.bulk_input = SimpleNamespace(close=lambda: calls.append('closed'))
  reader.raw_read = lambda *args: calls.append('drain')
  reader.transport = SimpleNamespace(_read_into=lambda *args: pytest.fail('closed async read used'))
  reader.close()
  reader.transport._read_into(None, 1.)
  reader.close()
  assert calls == ['closed', 'drain']

@pytest.mark.parametrize('replay', [False, True])
@pytest.mark.parametrize('size', [1, 32720, 32721, 65440, 458996, 1048576])
def test_prefetch_stops_at_request_boundary_even_with_whole_write_replays(replay, size):
  from jetlink import request_chunks as C, protocol as P
  from jetlink.transport.base import StreamTransport
  class Sink(StreamTransport):
    tx_align=P.GADGET_TX_ALIGN
    def __init__(self):super().__init__();self.packets=[]
    def _write(self,parts):
      raw=b''.join(parts);self.packets.append(raw);return len(raw)
    def _read_into(self,*a):raise AssertionError
    def close(self):pass
  sink=Sink();raw=bytes(i % 251 for i in range(size))
  C.send(sink.send,P.Msg.INFER_REQ,7,(raw,),timeout=.5)
  packets=sink.packets[:]
  sentinel=Sink();sentinel.send(P.Msg.STATE_REQ,8,(b'next',))
  wire=b''.join(p * (2 if replay else 1) for p in packets)
  class Source(StreamTransport):
    packet_size=1024;rx_align=P.GADGET_TX_ALIGN;read_chunk=256*1024;read_slack=1024
    def __init__(self):super().__init__(rx_size=2<<20);self.data=bytearray(wire+b''.join(sentinel.packets));self.reads=[]
    def _read_into(self,dest,timeout):
      self.reads.append(len(dest));n=min(len(dest),len(self.data));dest[:n]=self.data[:n];del self.data[:n]
      prefetch.prefetch_remaining=max(0,prefetch.prefetch_remaining-n)
      return n
    def _write(self,*a):raise AssertionError
    def close(self):pass
  source=Source();prefetch=object.__new__(BulkInput);prefetch.transport=source
  prefetch.raw_read_limit=source._read_limit;prefetch.prefetch_remaining=0;prefetch.prefetch_seq=None
  source._read_limit=prefetch._read_limit
  receiver=C.Receiver();results=[];last=0
  for _ in range(len(packets)*(2 if replay else 1)):
    msg=source.recv(.5);prefetch.note_message(msg);result=receiver.accept(msg,last)
    if result is not None:results.append(bytes(result.payload));last=result.seq
  assert results==[raw]
  assert source.data==b''.join(sentinel.packets)  # never read past logical input
  if not replay and size == 458996:assert len(source.reads)==4  # old path: 30 reads for 15 envelopes
  assert max(source.reads)<=source.read_chunk


def test_prefetch_ignores_invalid_headers_and_clears_on_hello():
  from jetlink import request_chunks as C, protocol as P
  from jetlink.transport.base import Message
  reader=object.__new__(BulkInput);reader.prefetch_seq=None;reader.prefetch_remaining=0
  for total,offset,kind in [(C.MAX_BYTES+1,0,8),(100000,1,8),(100000,0,999)]:
    msg=Message(C.MESSAGE,1,0,memoryview(C.HEADER.pack(kind,0,total,offset,0)+b'x'*C.DATA_BYTES))
    reader.note_message(msg);assert reader.prefetch_remaining==0
  reader.prefetch_seq=42;reader.prefetch_remaining=32768
  reader.note_message(Message(P.Msg.HELLO_REQ,1,0,memoryview(b'')))
  assert reader.prefetch_seq is None and reader.prefetch_remaining==0
