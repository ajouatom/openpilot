from types import SimpleNamespace
from pathlib import Path
import sys

import pytest

from openpilot.selfdrive.modeld.jetlink import daemon, link
from openpilot.selfdrive.modeld.jetlink.compat import ProtocolChoice
from jetlink.transport.base import LinkTimeout


def test_greet_only_changes_protocol_for_next_connection_after_failure():
  choice = ProtocolChoice('auto')
  transport = SimpleNamespace(protocol_version=choice.version)
  def hello():
    raise LinkTimeout('only 0 of 32 bytes arrived in time')
  with pytest.raises(LinkTimeout):
    daemon.greet(SimpleNamespace(hello=hello), choice)
  assert choice.version == 2
  assert transport.protocol_version == 3
  peer = {'protocol': 2}
  assert daemon.greet(SimpleNamespace(hello=lambda: peer), choice) == peer
  assert choice.version == 2


@pytest.mark.parametrize('mode', ['usb', 'ios', 'android'])
@pytest.mark.parametrize('speed', ['super-speed', 'super-speed-plus'])
def test_all_modes_accept_usb3(mode, speed):
  daemon.validate_speed(speed, mode)


def test_usb2_is_explicit_phone_only_without_deadline_change():
  for mode in ('ios', 'android'):
    daemon.validate_speed('high-speed', mode)
  with pytest.raises(RuntimeError, match='USB speed'):
    daemon.validate_speed('high-speed', 'usb')


@pytest.mark.parametrize('speed', ['full-speed', 'low-speed', 'UNKNOWN', ''])
@pytest.mark.parametrize('mode', ['usb', 'ios', 'android'])
def test_unusable_speed_is_rejected(mode, speed):
  with pytest.raises(RuntimeError, match='USB speed'):
    daemon.validate_speed(speed, mode)


@pytest.mark.parametrize('mode', ['ios', 'android'])
def test_phone_checks_data_role_not_power_role(tmp_path, monkeypatch, mode):
  role = tmp_path / 'current_dr'
  monkeypatch.setattr(daemon, 'DATA_ROLE', role)
  monkeypatch.setattr(daemon, 'TRANSPORT_MODE', mode)
  role.write_text('ufp\n')
  assert daemon.host_attached()
  role.write_text('dfp\n')
  assert not daemon.host_attached()
  role.unlink()
  assert not daemon.host_attached()


@pytest.mark.parametrize('mode,scenario', [
  ('usb', 'stop'), ('android', 'stop'), ('ios', 'stop'), ('auto', 'stop'),
  ('ios', 'retry'), ('auto', 'retry'), ('ios', 'detach'), ('auto', 'detach'),
  ('ios', 'unsafe'), ('auto', 'unsafe'),
  ('auto', 'constructor-unsafe'),
  ('auto', 'usb-success'), ('auto', 'usb-fail'), ('auto', 'ncm-unavailable'),
  ('auto', 'usb-v2'), ('auto', 'usb-exhausted'), ('auto', 'usb-pinned'),
  ('auto', 'usb-unsafe'), ('auto', 'brief-detach'),
  ('auto', 'usb-late'), ('auto', 'ncm-cleanup-unsafe'),
])
def test_daemon_wires_mode_protocol_and_separate_ios_owner(tmp_path, monkeypatch, mode, scenario):
  calls = []
  attached = [True]
  reports = []
  udcs = tmp_path / 'udc'
  (udcs / 'controller').mkdir(parents=True)
  (udcs / 'controller' / 'current_speed').write_text('super-speed\n')
  def path(value):
    if value == '/sys/class/udc':
      return udcs
    if value == daemon.SOCKET:
      return tmp_path / 'local.sock'
    return Path(value)
  monkeypatch.setattr(daemon, 'Path', path)
  monkeypatch.setattr(daemon, 'open', lambda *a: (tmp_path / 'owner.lock').open('w'), raising=False)
  monkeypatch.setattr(daemon.fcntl, 'flock', lambda *a: None)
  monkeypatch.setattr(daemon, 'transport_mode', lambda: mode)
  monkeypatch.setenv('JETLINK_PROTOCOL', '3' if scenario == 'usb-pinned' else 'auto')
  monkeypatch.setattr(daemon, 'TRANSPORT_MODE', 'usb')
  monkeypatch.setattr(daemon, 'update_affinity', lambda: None)
  monkeypatch.setattr(daemon, 'host_attached', lambda: attached[0])
  monkeypatch.setattr(daemon, 'publish', lambda state, **kw: reports.append((state, kw)))
  def sleep(delay):
    assert scenario != 'stop', 'must not rebind while waiting for the phone dial'
    if reports[-1][0] == 'blocked':
      raise KeyboardInterrupt
    if delay == 1:
      assert reports[-1][0] == 'waiting'
      attached[0] = True
    else:
      assert delay == 2
  monkeypatch.setattr(daemon.time, 'sleep', sleep)
  monkeypatch.setattr(daemon, 'gc', SimpleNamespace(disable=lambda: None, freeze=lambda: None, collect=lambda: None))
  monkeypatch.setattr(daemon.os, 'sched_setscheduler', lambda *a: None)
  monkeypatch.setattr(daemon.os, 'sched_setaffinity', lambda *a: None)
  listener = SimpleNamespace(bind=lambda *a: None, listen=lambda *a: None, settimeout=lambda *a: None,
                             close=lambda: calls.append('listener.close'))
  monkeypatch.setattr(daemon.socket, 'socket', lambda *a: listener)
  monkeypatch.setattr(daemon.os, 'chmod', lambda *a: None)
  monkeypatch.setattr(daemon, 'setup_gadget', lambda value: calls.append(('setup', value)))
  owner = SimpleNamespace(close=lambda: calls.append('owner.close'), bound_udc='controller', _configured=lambda: True)
  if scenario in ('usb-success', 'usb-fail', 'usb-v2', 'usb-exhausted', 'usb-pinned', 'usb-unsafe', 'usb-late'):
    monkeypatch.setattr(daemon.AutoTransport, 'GRACE', 0.)
  if scenario == 'usb-late':
    monkeypatch.setattr(daemon.AutoTransport, 'RETRY_GRACE', 0.)
  phone = SimpleNamespace()
  def make_owner(*a, **kw):
    if scenario == 'constructor-unsafe':
      raise daemon.UnsafeCleanupError('constructor could not release controller')
    return owner
  monkeypatch.setattr(daemon, 'CarrotTransport', make_owner)
  class Cable:
    def __init__(self, value):
      assert value is owner
      self.accepts = 0
      self.listener = object()
    def start(self):
      calls.append('cable.start')
      if scenario in ('ncm-unavailable', 'ncm-cleanup-unsafe'):
        self.listener = None
        raise daemon.MobileLinkError('NCM kernel support unavailable')
    def accept(self, timeout):
      assert timeout == 1.
      calls.append('cable.accept')
      self.accepts += 1
      if scenario == 'usb-late':
        raise LinkTimeout('USB server has not finished booting')
      if ((self.accepts == 1 and not (scenario == 'usb-fail' and calls.count('hello') == 1)) or
          (scenario in ('usb-v2', 'usb-exhausted') and calls.count('hello') < 2)):
        raise LinkTimeout('phone not dialed yet')
      return phone
    def close(self):
      calls.append('cable.close')
      if scenario == 'ncm-cleanup-unsafe':
        raise RuntimeError('owned network cleanup failed')
  monkeypatch.setattr(daemon, 'MobileCable', Cable)
  class Client:
    last_state = None
    spec = link.SPEC
    def __init__(self, transport, **kw):
      expected = owner if (scenario in ('usb-success', 'usb-late', 'ncm-unavailable') or
                          (scenario in ('usb-fail', 'usb-pinned', 'usb-unsafe') and calls.count('hello') == 0) or
                          (scenario in ('usb-v2', 'usb-exhausted') and calls.count('hello') < 2)) else (
        phone if mode in ('auto', 'ios') else owner)
      assert transport is expected
      assert transport.protocol_version == (2 if transport is owner and calls.count('hello') == 1 else 3)
      self.t = transport
    def hello(self, timeout=None):
      calls.append('hello')
      if self.t is owner and mode == 'auto':
        assert timeout == (None if scenario == 'ncm-unavailable' else 3.)
        if (scenario in ('usb-fail', 'usb-pinned', 'usb-exhausted', 'usb-unsafe') or
            (scenario == 'usb-v2' and calls.count('hello') == 1) or
            (scenario == 'usb-late' and calls.count('hello') <= 2)):
          raise LinkTimeout('bounded discovery write failed')
        if scenario == 'usb-v2':
          return {'protocol': 2, 'backend': 'trt', 'device': 'orin', 'carrot_host': 'jetson'}
        return {'protocol': 3, 'backend': 'ort', 'device': 'ane-Apple_M4', 'loaded': link.SPEC.sha256}
      return {'protocol': 3, 'backend': 'ort', 'device': 'ane-Apple_A17_Pro', 'loaded': link.SPEC.sha256} if mode == 'auto' else {'protocol': 3}
    def infer(self, images, packed, frame, reset):
      assert reset and packed.size == link.SPEC.packed_nelem
    def close(self):
      calls.append('client.close')
      if scenario in ('unsafe', 'usb-unsafe'):
        raise RuntimeError('session cleanup ownership unsafe')
      if self.t is owner:
        owner.close()
  monkeypatch.setattr(daemon, 'JetlinkClient', Client)
  monkeypatch.setitem(sys.modules, 'openpilot.common.params', SimpleNamespace(
    Params=lambda: SimpleNamespace(get_bool=lambda key: key == 'IsOffroad')))
  def prepare(client, peer, offroad, connected, progress, **kw):
    expected = 'usb' if scenario in ('usb-success', 'usb-v2', 'usb-late', 'ncm-unavailable') else ('ios' if mode == 'auto' else mode)
    assert kw['mode'] == expected and offroad() and connected()
    calls.append('prepare')
  monkeypatch.setattr(daemon, 'prepare', prepare)
  def serve(*a):
    calls.append('serve')
    if scenario in ('retry', 'detach', 'unsafe', 'brief-detach') and calls.count('serve') == 1:
      if scenario == 'detach':
        attached[0] = False
      if scenario == 'brief-detach':
        daemon.SESSION_DETACHED = True
      raise LinkTimeout('session ended')
    raise KeyboardInterrupt
  monkeypatch.setattr(daemon, 'serve_local', serve)
  with pytest.raises(KeyboardInterrupt):
    daemon.main()
  assert calls[0] == ('setup', mode)
  if scenario == 'constructor-unsafe':
    assert calls.count(('setup', mode)) == 1
    assert 'hello' not in calls and 'owner.close' not in calls
    assert reports[-2][0] == 'blocked'
    return
  if scenario == 'ncm-unavailable':
    assert calls.count(('setup', 'auto')) == calls.count(('setup', 'usb')) == 1
    assert calls.count('hello') == calls.count('serve') == 1
    assert calls.index('cable.close') < calls.index(('setup', 'usb'))
    assert any('retrying bulk USB' in report.get('error', '') for _, report in reports)
    assert not any(state == 'blocked' for state, _ in reports)
    return
  if scenario == 'ncm-cleanup-unsafe':
    assert calls.count(('setup', 'auto')) == 1 and ('setup', 'usb') not in calls
    assert reports[-2][0] == 'blocked' and 'hello' not in calls
    return
  if scenario == 'usb-late':
    assert calls.count('hello') == 3 and calls.count('serve') == 1
    assert calls.count(('setup', 'auto')) == 3
    assert not any(state == 'waiting' for state, _ in reports)  # no unplug
    return
  if scenario == 'usb-unsafe':
    assert calls.count(('setup', 'auto')) == 1
    assert calls.count('hello') == 1 and 'serve' not in calls
    assert reports[-2][0] == 'blocked'
    return
  if scenario in ('usb-success', 'usb-fail', 'usb-v2', 'usb-exhausted', 'usb-pinned'):
    hello, starts, accepts, closes = {
      'usb-success': (1, 1, 1, 2), 'usb-fail': (2, 2, 2, 3), 'usb-pinned': (2, 2, 3, 3),
      'usb-v2': (2, 2, 2, 4), 'usb-exhausted': (3, 3, 4, 5),
    }[scenario]
    assert calls.count('hello') == hello
    assert calls.count('cable.start') == starts
    assert calls.count('cable.accept') == accepts
    assert calls.count('owner.close') == closes
    assert calls.count(('setup', 'auto')) == starts
    assert 'teardown' not in calls
    return
  assert calls.count('owner.close') == (2 if scenario in ('detach', 'brief-detach') else 1)
  assert calls.index('hello') < calls.index('prepare')
  if mode in ('auto', 'ios'):
    assert calls.count('cable.accept') == {'stop': 2, 'retry': 3, 'detach': 4, 'brief-detach': 4, 'unsafe': 2}[scenario]
    assert calls.index('cable.start') < calls.index('cable.accept') < calls.index('hello')
    assert calls.index('client.close') < calls.index('cable.close') < calls.index('owner.close')
    assert 'teardown' not in calls
    assert calls.count('cable.start') == (2 if scenario in ('detach', 'brief-detach') else 1)
    assert calls.count('hello') == (2 if scenario in ('detach', 'brief-detach', 'retry') else 1)
    if scenario in ('detach', 'brief-detach'):
      waiting = [report for state, report in reports if state == 'waiting']
      assert len(waiting) == 1 and waiting[0]['peer'] is None
    if scenario == 'unsafe':
      blocked = [report for state, report in reports if state == 'blocked']
      assert len(blocked) == 1 and 'Unsafe Jetlink cleanup' in blocked[0]['error']
  else:
    assert 'cable.start' not in calls and 'teardown' not in calls
