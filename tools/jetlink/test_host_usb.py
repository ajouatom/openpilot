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
