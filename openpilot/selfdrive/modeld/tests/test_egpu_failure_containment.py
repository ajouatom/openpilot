"""Exercise deadline, scheduling, and disposal boundaries without a GPU."""
import ast
import io
from pathlib import Path
import subprocess
import threading
import time
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.modeld import precompiled_runner as runner


@pytest.mark.parametrize('recovery', [False, True])
def test_partial_response_cannot_escape_the_inference_deadline(monkeypatch, tmp_path, recovery):
  model = object.__new__(runner.PrecompiledModelState)
  model.process = SimpleNamespace(stdout=SimpleNamespace(fileno=lambda: 9), pid=987654321)
  model.file = SimpleNamespace(name=str(tmp_path / 'shared'))
  model.recovery_cancel = threading.Event() if recovery else None
  clock = [0.]
  class Selector:
    def __enter__(self): return self
    def __exit__(self, *args): pass
    def register(self, *args): pass
    def select(self, timeout):
      clock[0] += timeout
      return [True]
  monkeypatch.setattr(runner.selectors, 'DefaultSelector', Selector)
  monkeypatch.setattr(runner.os, 'read', lambda *args: b'partial')
  monkeypatch.setattr(runner.time, 'monotonic', lambda: clock[0])
  try:
    with pytest.raises(TimeoutError, match='worker timed out'):
      model._receive(.2)
    assert .2 <= clock[0] <= .25
  finally:
    model.process = None


def test_uninterruptible_exit_does_not_block_fallback_or_allow_another_owner(tmp_path):
  exited, reaped = threading.Event(), threading.Event()
  class Process:
    stdin, stdout = io.BytesIO(), io.BytesIO()
    def poll(self): return 0 if exited.is_set() else None
    def kill(self): self.killed = True
    def wait(self, timeout=None):
      if timeout is not None:
        raise subprocess.TimeoutExpired('worker', timeout)
      exited.wait(5)
      reaped.set()
  process = Process()
  model = object.__new__(runner.PrecompiledModelState)
  model.process, model.views, model.shared, model.output = process, {}, None, None
  model.failed = True
  model.file = (tmp_path / 'shared').open('wb')
  try:
    started = time.monotonic()
    model.close()
    assert time.monotonic() - started < .5
    assert process.killed and process in runner._retiring_workers
    with pytest.raises(RuntimeError, match='still exiting'):
      runner.PrecompiledModelState(1344, 760, tmp_path / 'model.pkl')
  finally:
    exited.set()
    assert reaped.wait(2)
    deadline = time.monotonic() + 2
    while process in runner._retiring_workers and time.monotonic() < deadline:
      time.sleep(.001)
  assert process not in runner._retiring_workers


def test_fallback_handler_discards_failed_camera_before_internal_inference():
  # Execute the real handler in a two-frame loop, without importing modeld's
  # native camera/GPU stack. A same-frame internal run would fail this test.
  source = Path(runner.__file__).with_name('modeld.py').read_text()
  tree = ast.parse(source)
  handler = next(node for node in ast.walk(tree) if isinstance(node, ast.ExceptHandler)
                 and any(isinstance(child, ast.Constant) and child.value == 'eGPU model failed, falling back to internal GPU'
                         for child in ast.walk(node)))
  body = ast.parse('for frame in (1, 2):\n if frame == 1:\n  pass\n published.append(frame)\n').body[0]
  body.body[0].body = handler.body
  module = ast.fix_missing_locations(ast.Module(body=[body], type_ignores=[]))
  reset = []
  internal = SimpleNamespace(reset=lambda: reset.append(True), run=lambda *a: pytest.fail('stale frame inference'))
  params = SimpleNamespace(get_bool=lambda _: True, put_bool=lambda *a: None)
  scope = {'params': params, 'small_model': internal, 'cloudlog': SimpleNamespace(exception=lambda *a: None),
           'queue_usbgpu_error_tmux': lambda *a: None, 'log_event': lambda *a, **kw: None, 'internal_needs_reset': True,
           'egpu_recovery': None, 'meta_main': SimpleNamespace(frame_id=41), 'published': []}
  exec(compile(module, '<modeld actual fallback>', 'exec'), scope)
  assert scope['published'] == [2] and scope['last_vipc_frame_id'] == 41
  assert scope['model'] is internal and reset == [True]


@pytest.mark.parametrize('runtime_sha', ['90970e01f2ba70d553294c0f9d5c6fe1678827f750705cab679a7b19b7bcb0b2',
                                         'c7c58622652231359ecea1a4253b63e157d002242bf596ede1b853f941f937ac'])
def test_usb_wait_only_adapts_verified_runtime_and_yields(monkeypatch, runtime_sha):
  import sys
  from openpilot.selfdrive.modeld import egpu_usb_wait as wait
  class AMD:
    sleep_timeout_ms = 200
    wait_timeout_ms = 30000
  class USB:
    def sleep(self, timeout): pass
  monkeypatch.setitem(sys.modules, 'tinygrad.runtime.ops_amd', SimpleNamespace(AMDDevice=AMD, USBIface=USB))
  old_sleep = USB.sleep
  assert not wait.configure_usb_wait({'runtime': {'sha256': 'unknown'}})
  assert USB.sleep is old_sleep and AMD.sleep_timeout_ms == 200
  sleeps = []
  monkeypatch.setattr(wait.time, 'sleep', sleeps.append)
  assert wait.configure_usb_wait({'runtime': {'sha256': runtime_sha}})
  USB().sleep(AMD.sleep_timeout_ms)
  assert sleeps == [.0002] and AMD.sleep_timeout_ms == 1
  assert AMD.wait_timeout_ms == 30000


def test_stall_diagnostic_is_bounded_and_excludes_idle(monkeypatch):
  from openpilot.selfdrive.modeld import precompiled_worker as worker
  progress = SimpleNamespace(current=('output_read', 1, 0.))
  calls = []
  # Four distinct stalled frames, with an idle phase between the first two.
  states = iter([('idle', 2, 0.), ('output_read', 2, 0.), ('output_read', 3, 0.), ('output_read', 4, 0.), None])
  parent = [9]
  def sleep(_):
    state = next(states)
    if state is None:
      parent[0] = 1
    else:
      progress.current = state
  monkeypatch.setattr(worker.os, 'getppid', lambda: parent[0])
  monkeypatch.setattr(worker.os, '_exit', lambda _: None)
  monkeypatch.setattr(worker.time, 'monotonic', lambda: 1.)
  monkeypatch.setattr(worker.time, 'sleep', sleep)
  monkeypatch.setattr(worker.faulthandler, 'dump_traceback', lambda **kw: calls.append(kw))
  worker.watch_parent(9, progress)
  assert len(calls) == 3
