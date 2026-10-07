from concurrent.futures import Future
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.modeld.jetlink import model, prepare
from test_hotplug_resources import joining_model


def test_constructor_never_prepares_gpu_or_connects(monkeypatch):
  monkeypatch.setitem(sys.modules, 'openpilot.selfdrive.modeld.parse_model_outputs', NS(Parser=lambda: None))
  monkeypatch.setitem(sys.modules, 'openpilot.system.hardware', NS(HARDWARE=NS(get_device_type=lambda: 'mici')))
  monkeypatch.setattr(model, 'Warp', lambda *a, **k: pytest.fail('eager GPU preparation'))
  monkeypatch.setattr(model, 'ClientConnection', lambda: pytest.fail('eager connection'))
  m = model.JoiningModel(NS(), 1344, 760)
  assert m.warp is None and m.preparation is None and m.small_runs == 0


@pytest.mark.parametrize('ready,allowed,outputs', [(False, True, 3), (True, False, 3), (True, True, 0), (True, True, 2)])
def test_preparation_waits_for_internal_outputs_and_ready_peer(monkeypatch, ready, allowed, outputs):
  m = joining_model(monkeypatch)
  m.warp = None
  m.ready, m.join_allowed, m.small_runs = ready, allowed, outputs
  monkeypatch.setattr(model, 'WarpPreparation', lambda *a: pytest.fail('premature preparation'))
  monkeypatch.setattr(model, 'ClientConnection', lambda: pytest.fail('premature connection'))
  assert m.run({}, {}, {}, False) == 'local'


def test_slow_preparation_keeps_internal_frames_and_gates_connection(monkeypatch):
  m = joining_model(monkeypatch)
  m.warp = None
  pending = NS(future=Future(), close=lambda: None)
  starts = []
  def start(*args):
    starts.append(args)
    return pending
  monkeypatch.setattr(model, 'WarpPreparation', start)
  monkeypatch.setattr(model, 'ClientConnection', lambda: pytest.fail('connection before warp ready'))
  for _ in range(100):
    assert m.run({}, {}, {}, False) == 'local'
  assert starts == [(1344, 760)] and m.small_runs == 103


def test_completed_preparation_obeys_current_join_permission(monkeypatch):
  m = joining_model(monkeypatch)
  m.warp = None
  future = Future()
  future.set_result((b'validated', 6.8))
  m.preparation = NS(future=future)
  m.join_allowed = False
  installed = []
  def install(*args, prepared):
    installed.append(prepared)
    return lambda *a: None
  monkeypatch.setattr(model, 'Warp', install)
  monkeypatch.setattr(model, 'ClientConnection', lambda: NS(future=Future()))
  assert m.run({}, {}, {}, False) == 'local'
  assert not installed and m.connection is None
  m.join_allowed = True
  assert m.run({}, {}, {}, False) == 'local'
  assert installed == [b'validated'] and m.preparation is None and m.connection is not None


def test_disconnect_cancels_preparation_and_keeps_local_model(monkeypatch):
  m = joining_model(monkeypatch)
  m.warp = None
  closed = []
  m.preparation = NS(future=Future(), close=lambda: closed.append(True))
  m.ready = False
  assert m.run({}, {}, {}, False) == 'local'
  assert closed == [True] and m.preparation is None and not m.active


@pytest.mark.parametrize('stage', ['worker', 'install'])
def test_failed_preparation_has_bounded_retry_and_never_enters_external_model(monkeypatch, stage):
  m = joining_model(monkeypatch)
  m.warp = None
  future = Future()
  if stage == 'worker':
    future.set_exception(ValueError('GPU unavailable'))
  else:
    future.set_result((b'bad', 1))
    def fail(*a, **k): raise ValueError('invalid executable')
    monkeypatch.setattr(model, 'Warp', fail)
  closed = []
  m.preparation = NS(future=future, close=lambda: closed.append(True))
  monkeypatch.setattr(model, 'WarpPreparation', lambda *a: pytest.fail('retry before deadline'))
  monkeypatch.setattr(model, 'ClientConnection', lambda: pytest.fail('failed preparation must keep local'))
  for _ in range(5):
    assert m.run({}, {}, {}, False) == 'local'
  assert len(closed) == 1 and m.next_join > 0 and m.warp is None and not m.active


@pytest.mark.parametrize('outcome', ['success', 'failure', 'cancel', 'timeout'])
def test_worker_is_bounded_reaped_and_cleans_private_artifacts(monkeypatch, tmp_path, outcome):
  entered, release = threading.Event(), threading.Event()
  calls = []
  class Process:
    returncode = None
    def __init__(self, args, **kwargs):
      self.target = Path(args[-1])
      calls.append(self.target.parent)
      self.target.write_bytes(b'executable')
      if outcome == 'failure':
        kwargs['stdout'].write(b'compiler failed')
      entered.set()
    def poll(self):
      if self.returncode is None and release.is_set():
        self.returncode = 1 if outcome == 'failure' else 0
      return self.returncode
    def kill(self):
      calls.append('killed')
      self.returncode = -9
    def wait(self):
      calls.append('reaped')
      return self.returncode
  monkeypatch.setattr(prepare.subprocess, 'Popen', Process)
  if hasattr(prepare.os, 'sched_setscheduler'):
    monkeypatch.setattr(prepare.os, 'sched_setscheduler', lambda *a: None)
    monkeypatch.setattr(prepare.os, 'sched_setaffinity', lambda *a: None)
  if outcome == 'timeout':
    monkeypatch.setattr(prepare, 'PREPARE_TIMEOUT', -.1)
  worker = prepare.WarpPreparation(1344, 760)
  assert entered.wait(2)
  try:
    if outcome == 'cancel':
      worker.close()
    elif outcome != 'timeout':
      release.set()
    worker.thread.join(2)
    assert not worker.thread.is_alive() and not calls[0].exists()
    if outcome == 'success':
      assert worker.future.result()[0] == b'executable'
    elif outcome == 'failure':
      with pytest.raises(RuntimeError, match='compiler failed'):
        worker.future.result()
    elif outcome == 'timeout':
      with pytest.raises(TimeoutError):
        worker.future.result()
    else:
      assert worker.future.cancelled()
    if outcome in ('cancel', 'timeout'):
      assert calls[-2:] == ['killed', 'reaped']
  finally:
    release.set()
    worker.close()
    worker.thread.join(2)
