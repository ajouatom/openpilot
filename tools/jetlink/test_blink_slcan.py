import os
from pathlib import Path
import subprocess
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import blink_slcan as slcan


def test_classic_target_frames_and_unrelated_fd():
  assert slcan.frame(b't3C188F30300000000000') == (0x3c1, bytes.fromhex('8f30300000000000'))
  assert slcan.frame(b't41380000000000000000') == (0x413, bytes(8))
  for data in (b'D000003C1B'+b'00'*24, b't100100', b'T000004138'+b'00'*8, b''):
    assert slcan.frame(data) is None
  for data in (b't3C1700000000000000', b't4138'+b'xx'*8, b't4138'+b'00'*7):
    with pytest.raises(ValueError):
      slcan.frame(data)


def test_send_requires_armed_mode_and_encodes_exact_classic_frame():
  bus = slcan.Bus.__new__(slcan.Bus)
  sent = []
  bus.report = (b'T', 0, 0xffff)
  def command(data, **kwargs):
    sent.append(data)
    bus.report = (b'T', 1 if len(sent) == 1 else 2, 0)
  bus._command = command
  bus.transmit = False
  with pytest.raises(ValueError):
    bus.send(bytes(8))
  bus.transmit = True
  bus.send(bytes.fromhex('9cc0304010000000'))
  assert sent == [b't3C189CC0304010000000\rQ\r', b'Q\r']


@pytest.mark.parametrize('reply', [(b'L', 2, 11), (b'T', 3, 11), (b'T', 2, 10)])
def test_can_failure_silent_mode_or_stale_ack_never_reports_success(reply):
  bus = slcan.Bus.__new__(slcan.Bus)
  bus.transmit, bus.report = True, (b'T', 0, 10)
  bus._command = lambda *a, **k: setattr(bus, 'report', reply)
  with pytest.raises(OSError):
    bus.send(bytes(8))


def test_tx_completion_is_bounded(monkeypatch):
  bus = slcan.Bus.__new__(slcan.Bus)
  bus.transmit, bus.report = True, (b'T', 0, 10)
  bus._command = lambda *a, **k: setattr(bus, 'report', (b'T', 1, 11))
  ticks = iter((0., .001, .02))
  monkeypatch.setattr(slcan.time, 'monotonic', lambda: next(ticks))
  with pytest.raises(TimeoutError):
    bus.send(bytes(8))


def test_only_confirmed_arbitration_cancellation_retries_same_frame(monkeypatch):
  bus = slcan.Bus.__new__(slcan.Bus)
  bus.transmit, bus.report = True, (b'T', 0, 0)
  sent = []
  def command(data, **kwargs):
    sent.append(data)
    bus.report = (b'T', 4 if len(sent) < 3 else 2, len(sent))
  bus._command = command
  monkeypatch.setattr(slcan.time, 'sleep', lambda _: None)
  bus.send(bytes(8))
  assert sent == [b't3C18'+b'00'*8+b'\rQ\r']*3


def test_protocol_error_never_retries(monkeypatch):
  bus = slcan.Bus.__new__(slcan.Bus)
  bus.transmit, bus.report = True, (b'T', 0, 0)
  sent = []
  def command(data, **kwargs):
    sent.append(data)
    bus.report = (b'T', 3, 1)
  bus._command = command
  with pytest.raises(OSError, match='protocol error'):
    bus.send(bytes(8))
  assert len(sent) == 1


def test_fragmented_feedback_and_hardware_diagnostics(monkeypatch):
  bus = slcan.Bus.__new__(slcan.Bus)
  bus.sock, bus.next_lease = 42, float('inf')
  bus.buffer, bus.latest, bus.report_count = bytearray(), {}, 0
  chunks = iter([b't3C188F3030', b'0000000000\rQT20001000007070000000000000000\r'])
  def read(*args):
    try:
      return next(chunks)
    except StopIteration:
      raise BlockingIOError
  monkeypatch.setattr(slcan.os, 'read', read)
  bus.receive()
  assert bus.latest[0x3c1][1] == bytes.fromhex('8f30300000000000')
  assert bus.report == (b'T', 2, 1)
  assert bus.diagnostics == (0x707, 0, 0)


def test_auto_selection_keeps_first_adapter_then_can_probe_another(tmp_path, monkeypatch):
  import json
  import blink_protocol as protocol
  path = tmp_path/'profile.json'
  path.write_text(json.dumps({'enabled':True, 'profile':protocol.PROFILE, 'interface':'auto'}))
  original = Path.glob
  monkeypatch.setattr(Path, 'glob', lambda p, pattern: iter(()) if str(p).replace('\\', '/') == '/sys/class/net' else original(p, pattern))
  monkeypatch.setattr(slcan, 'identity', lambda _: ('verified',))
  monkeypatch.setattr(slcan, 'candidates', lambda: ['/dev/serial/by-id/canable-one'])
  assert protocol.configuration(path)['interface'] == '/dev/serial/by-id/canable-one'
  monkeypatch.setattr(slcan, 'candidates', lambda: ['/dev/serial/by-id/canable-one', '/dev/serial/by-id/canable-two'])
  assert protocol.configuration(path)['interface'].endswith('canable-one')
  assert protocol.configuration(path, preferred='/dev/serial/by-id/canable-two')['interface'].endswith('canable-two')
  assert protocol.configuration(path, exclude=['/dev/serial/by-id/canable-one'])['interface'].endswith('canable-two')


@pytest.mark.parametrize('transmit', [False, True])
def test_mode_verified_from_hardware_not_version_reply(monkeypatch, transmit):
  bus = slcan.Bus.__new__(slcan.Bus)
  bus.iface, bus.index, bus.transmit = '/dev/ttyACM0', ('same',), True
  monkeypatch.setattr(slcan, 'identity', lambda _: ('same',))
  sent = []
  def command(data, **kwargs):
    sent.append(data)
    bus.report = (b'L' if transmit else b'T', 0, 0)
  bus._command = command
  with pytest.raises(ValueError):
    bus.mode(transmit)
  assert not bus.transmit
  assert sent == [b'C\rS6\rA0\r' + (b'M0\r' if transmit else b'M1\r') + b'O\rQ\r']


def test_adapter_replacement_rejected_before_commands(monkeypatch):
  bus = slcan.Bus.__new__(slcan.Bus)
  bus.iface, bus.index = '/dev/ttyACM0', ('original',)
  monkeypatch.setattr(slcan, 'identity', lambda _: ('replacement',))
  bus._command = lambda *a, **k: pytest.fail('Must not write a replacement adapter')
  with pytest.raises(ValueError):
    bus.mode(True)


@pytest.mark.skipif(sys.platform != 'linux', reason='Linux serial watchdog')
@pytest.mark.parametrize('lost', ['eof', 'timeout'])
def test_serial_watchdog_closes_can_after_parent_disappears(lost):
  import pty
  import select
  import tty
  master, slave = pty.openpty()
  tty.setraw(slave)
  child = subprocess.Popen([sys.executable, '-c', slcan.WATCHDOG, str(slave)],
                           pass_fds=(slave,), stdin=subprocess.PIPE)
  try:
    child.stdin.write(b'1')
    child.stdin.flush()
    if lost == 'eof':
      child.stdin.close()
    assert select.select([master], [], [], 2)[0]
    assert os.read(master, 100) == b'C\r'
    assert child.wait(timeout=2) == 0
  finally:
    if child.poll() is None:
      child.kill()
      child.wait()
    if not child.stdin.closed:
      child.stdin.close()
    os.close(master)
    os.close(slave)
