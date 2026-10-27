"""Exact USB port ownership, quiet expiry and crash restoration contracts."""
import json
import os
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

import pytest

import host_lpm as p


@pytest.fixture
def bus(tmp_path):
  root = tmp_path / 'sys'
  device = root / 'devices/usb2/2-2'
  permit = root / 'devices/usb2/usb2-port2/usb3_lpm_permit'
  (device / 'power').mkdir(parents=True)
  permit.parent.mkdir(parents=True)
  permit.write_text('u1_u2')
  for name, value in {'idVendor':'1209', 'idProduct':'0001', 'busnum':'2', 'devnum':'7', 'speed':'5000'}.items():
    (device / name).write_text(value)
  for state in ('u1', 'u2'):
    (device / 'power' / ('usb3_hardware_lpm_' + state)).write_text('enabled')
  writes = []

  def write(path, value):
    assert path == permit
    writes.append(value)
    path.write_text(value)
    for state in ('u1', 'u2'):
      (device / 'power' / ('usb3_hardware_lpm_' + state)).write_text('disabled' if value == '0' else 'enabled')

  lease = p.PortLease(tmp_path / 'lease.json', 'boot', root=root, write=write)
  return SimpleNamespace(root=root, device=device, permit=permit, lease=lease, writes=writes, write=write)


@pytest.mark.parametrize('original', ['u1_u2', 'u1', 'u2'])
def test_exact_original_restored_and_repeated_activity_does_not_write(bus, original):
  bus.permit.write_text(original)
  for _ in range(100): bus.lease.activate(bus.device, bus.permit)
  assert bus.writes == ['0']
  assert json.loads(bus.lease.journal.read_text())['original'] == original
  bus.lease.restore(); bus.lease.restore()
  assert bus.writes == ['0', original]
  assert bus.permit.read_text() == original and not bus.lease.journal.exists()


def test_external_off_or_changed_policy_is_not_overwritten(bus):
  bus.permit.write_text('0')
  bus.lease.activate(bus.device, bus.permit); bus.lease.restore()
  assert bus.writes == [] and not bus.lease.journal.exists()
  bus.permit.write_text('u1_u2')
  bus.lease.activate(bus.device, bus.permit)
  bus.permit.write_text('u1')
  bus.lease.restore()
  assert bus.permit.read_text() == 'u1' and bus.writes == ['0']


def test_failed_disable_readback_rolls_back_before_leaving(bus):
  bus.lease.write = lambda path, value: path.write_text(value)
  with pytest.raises(RuntimeError, match='disable readback'):
    bus.lease.activate(bus.device, bus.permit)
  assert bus.permit.read_text() == 'u1_u2' and bus.lease.owned is None
  assert not bus.lease.journal.exists()


def test_failed_restore_retains_journal_for_next_helper(bus):
  bus.lease.activate(bus.device, bus.permit)
  bus.lease.write = lambda path, value: None
  with pytest.raises(RuntimeError, match='restore readback'): bus.lease.restore()
  assert bus.lease.journal.exists() and bus.lease.owned
  recovered = p.PortLease(bus.lease.journal, 'boot', root=bus.root, write=bus.write)
  recovered.recover()
  assert bus.permit.read_text() == 'u1_u2' and not recovered.journal.exists()


def test_crash_recovery_restores_port_even_after_device_departure(bus):
  bus.lease.activate(bus.device, bus.permit)
  # The journal owns the port, whose lifetime exceeds the attached USB device.
  (bus.device / 'devnum').unlink()
  recovered = p.PortLease(bus.lease.journal, 'boot', root=bus.root, write=bus.write)
  recovered.recover()
  assert bus.writes == ['0', 'u1_u2']


def test_old_boot_journal_does_not_touch_new_hardware(bus):
  bus.lease.activate(bus.device, bus.permit)
  recovered = p.PortLease(bus.lease.journal, 'different-boot', root=bus.root, write=bus.write)
  recovered.recover()
  assert bus.writes == ['0'] and not recovered.journal.exists()


def test_journal_cannot_target_unrelated_file(bus, tmp_path):
  target = tmp_path / 'usb3_lpm_permit'; target.write_text('0')
  bus.lease.journal.write_text(json.dumps({'boot':'boot', 'permit':str(target), 'original':'u1_u2'}))
  with pytest.raises(ValueError, match='port path'): bus.lease.recover()
  assert target.read_text() == '0' and bus.writes == []


def test_activity_idle_resume_eof_restore_in_order(bus):
  events = iter([(0., b'1'), (5., b'1'), (34., None), (35., None), (40., b'1'), (41., b'')])
  now, data = [0.], [None]
  def wait(*_):
    now[0], data[0] = next(events)
    return ([9] if data[0] is not None else [], [], [])
  p.serve(bus.lease, lambda: (bus.device, bus.permit), 9, clock=lambda:now[0], wait=wait, read=lambda *_:data[0])
  assert bus.writes == ['0', 'u1_u2', '0', 'u1_u2']


@pytest.mark.parametrize('failure', ['bad-token', 'disconnect', 'replaced'])
def test_faults_restore_owned_port(bus, failure):
  calls = [0]
  def locate():
    calls[0] += 1
    if calls[0] >= 3 and failure == 'disconnect': raise OSError('gone')
    if calls[0] >= 3 and failure == 'replaced': return bus.device.parent, bus.permit
    return bus.device, bus.permit
  tokens = iter([b'1', b'x' if failure == 'bad-token' else b'1'])
  with pytest.raises((ValueError, RuntimeError, OSError)):
    p.serve(bus.lease, locate, 9, wait=lambda *_:([9], [], []), read=lambda *_:next(tokens))
  assert bus.writes == ['0', 'u1_u2']


def test_find_requires_exact_claim_and_usb3(bus, monkeypatch):
  resolve, glob = Path.resolve, Path.glob
  monkeypatch.setattr(Path, 'glob', lambda path, pattern: iter([bus.device]) if path == bus.root/'bus/usb/devices' else glob(path, pattern))
  monkeypatch.setattr(Path, 'resolve', lambda path, *a, **kw: bus.permit.parent if path == bus.device/'port' else resolve(path, *a, **kw))
  assert p.find_device(2, 7, bus.root) == (bus.device, bus.permit)
  for bus_id, address in [(2, 8), (3, 7), (True, 7), (2, 128)]:
    with pytest.raises(ValueError): p.find_device(bus_id, address, bus.root)
  for filename, value in [('speed','480'), ('idProduct','0002')]:
    original = (bus.device/filename).read_text(); (bus.device/filename).write_text(value)
    with pytest.raises(ValueError): p.find_device(2, 7, bus.root)
    (bus.device/filename).write_text(original)


def test_explicit_off_does_not_start_privileged_child(monkeypatch):
  monkeypatch.setenv('CARROT_USB_LPM_OFF', '0')
  monkeypatch.setattr(p.subprocess, 'Popen', lambda *_a, **_k: pytest.fail('unexpected root helper'))
  lease = p.SessionLease(object()); lease.touch(); lease.close()
  assert lease.process is None


def test_frame_touch_never_waits_for_failed_helper(monkeypatch):
  closed = []
  child = SimpleNamespace(stdin=SimpleNamespace(fileno=lambda:9, close=lambda:closed.append(True)),
                          wait=lambda **_:pytest.fail('frame waited for helper'))
  lease = object.__new__(p.SessionLease)
  lease.process, lease.next_touch, lease.failed = child, 0., False
  def broken(*_): raise BrokenPipeError()
  monkeypatch.setattr(os, 'write', broken)
  lease.touch(); lease.touch()
  assert lease.failed and closed == [True]


@pytest.mark.skipif(sys.platform != 'linux', reason='Linux pipe/select lifetime')
def test_real_owner_pipe_eof_restores_port(bus):
  reader, writer = os.pipe()
  errors = []
  def worker():
    try: p.serve(bus.lease, lambda:(bus.device, bus.permit), reader)
    except BaseException as exc: errors.append(exc)
  thread = threading.Thread(target=worker)
  thread.start()
  try:
    os.write(writer, b'1')
    until = time.monotonic() + 2
    while time.monotonic() < until and bus.permit.read_text() != '0': time.sleep(.01)
    assert bus.permit.read_text() == '0'
  finally:
    os.close(writer)
    thread.join(3)
    os.close(reader)
  assert not thread.is_alive() and not errors
  assert bus.permit.read_text() == 'u1_u2'


def test_session_cleanup_failure_still_releases_power_owner():
  from server import CarrotSession
  session = object.__new__(CarrotSession)
  closed = []
  def fail(): raise RuntimeError('display close failed')
  session.hud_wifi = SimpleNamespace(close=fail)
  session.power_lease = SimpleNamespace(close=lambda:closed.append(True))
  with pytest.raises(RuntimeError, match='display close failed'): session.close()
  assert closed == [True]


def test_session_message_path_touches_lease_without_changing_inference():
  from server import CarrotSession
  from jetlink import protocol as P, request_chunks as C
  from jetlink.transport.base import Message
  session = object.__new__(CarrotSession)
  calls = []
  session.power_lease = SimpleNamespace(touch=lambda:calls.append('touch'))
  session.request_chunks = C.Receiver(); session.last_seq = 0
  session.t = SimpleNamespace(protocol_version=2); session.client = 'test'
  session.on_infer = lambda msg:calls.append(bytes(msg.payload))
  session.handle(Message(P.Msg.INFER_REQ, 7, 0, memoryview(b'unchanged model input')))
  assert calls == ['touch', b'unchanged model input'] and session.last_seq == 7


def test_session_leases_claimed_transport_before_read_ahead_wrapper(monkeypatch):
  import server
  raw = object()
  class Reader:
    def __init__(self, transport): self.transport = transport
  def initialize(session, transport):
    session.t, session.telemetry = transport, SimpleNamespace()
  seen = []
  monkeypatch.setattr(server.Session, '__init__', initialize)
  monkeypatch.setattr(server, 'DisplayTelemetry', lambda original:original)
  monkeypatch.setattr(server, 'HostForwarder', lambda:None)
  monkeypatch.setattr(server, 'ReadAheadTransport', Reader)
  monkeypatch.setattr(server.sys, 'platform', 'linux')
  monkeypatch.setattr(server, 'SessionLease', lambda transport:seen.append(transport))
  session = server.CarrotSession(raw)
  assert seen == [raw] and session.t.transport is raw


def test_default_jetson_claim_starts_owned_helper_and_throttles_heartbeats(monkeypatch):
  from jetlink.transport.usbbulk import UsbBulkTransport
  monkeypatch.setattr(p.sys,'platform','linux')
  monkeypatch.delenv('CARROT_USB_LPM_OFF',raising=False)
  original=Path.is_file
  monkeypatch.setattr(Path,'is_file',lambda path:True if path==Path('/etc/nv_tegra_release') else original(path))
  monkeypatch.setattr(p.os,'geteuid',lambda:1000,raising=False)
  monkeypatch.setattr(p.os,'set_blocking',lambda *_:None)
  raw=object.__new__(UsbBulkTransport)
  raw.handle=SimpleNamespace(getDevice=lambda:SimpleNamespace(getBusNumber=lambda:2,getDeviceAddress=lambda:7))
  calls=[]; closed=[]; sent=[]
  child=SimpleNamespace(stdin=SimpleNamespace(fileno=lambda:9,close=lambda:closed.append(True)),wait=lambda **_:0)
  def spawn(command,**kwargs):
    calls.append((command,kwargs));return child
  monkeypatch.setattr(p.subprocess,'Popen',spawn)
  monkeypatch.setattr(p.os,'write',lambda fd,data:sent.append((fd,data)))
  monkeypatch.setattr(p.time,'monotonic',lambda:10.)
  lease=p.SessionLease(raw);lease.touch();lease.touch();lease.close()
  assert len(calls)==1 and calls[0][0][:2]==['sudo','-n'] and calls[0][0][-2:]==['2','7']
  assert calls[0][1]['stdin']==p.subprocess.PIPE and sent==[(9,b'1')] and closed==[True]
