import sys
from types import SimpleNamespace as NS

import pytest


@pytest.mark.parametrize('onroad,jetson,expected', [
  (True, True, {4}), (True, False, {0, 1, 2, 3}),
  (False, True, {0, 1, 2, 3}), (False, False, {0, 1, 2, 3}),
])
def test_jetson_placement_is_session_and_onroad_scoped(monkeypatch, onroad, jetson, expected):
  if sys.platform == 'win32':
    monkeypatch.setitem(sys.modules, 'fcntl', NS())
  from openpilot.selfdrive.modeld.jetlink import daemon
  monkeypatch.setitem(sys.modules, 'openpilot.common.params', NS(Params=lambda: NS(get_bool=lambda _: onroad)))
  monkeypatch.setattr(daemon, 'JETSON_SESSION', jetson)
  monkeypatch.setattr(daemon.Path, 'read_text', lambda _: '1')
  monkeypatch.setattr(daemon.Path, 'iterdir', lambda _: iter([NS(name='123'), NS(name='124')]))
  calls = []
  monkeypatch.setattr(daemon.os, 'SCHED_FIFO', 1, raising=False)
  monkeypatch.setattr(daemon.os, 'SCHED_OTHER', 0, raising=False)
  monkeypatch.setattr(daemon.os, 'sched_param', lambda v: v, raising=False)
  monkeypatch.setattr(daemon.os, 'sched_setscheduler', lambda *a: None, raising=False)
  monkeypatch.setattr(daemon.os, 'sched_setaffinity', lambda pid, cores: calls.append((pid, cores)), raising=False)
  daemon.update_affinity()
  assert calls == [(123, expected), (124, expected)]


def test_failed_jetson_session_does_not_leak_placement_to_next_peer(monkeypatch):
  if sys.platform == 'win32':
    monkeypatch.setitem(sys.modules, 'fcntl', NS())
  from openpilot.selfdrive.modeld.jetlink import daemon
  closed = []
  monkeypatch.setattr(daemon, 'PhasePublisher', lambda: NS(close=lambda: closed.append(True)))
  def fail(*args):
    assert daemon.JETSON_SESSION
    raise RuntimeError('test disconnect')
  monkeypatch.setattr(daemon, '_serve_local', fail)
  with pytest.raises(RuntimeError, match='test disconnect'):
    daemon.serve_local(None, None, {'carrot_host': 'jetson'})
  assert not daemon.JETSON_SESSION and closed == [True]
