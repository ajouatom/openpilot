import json
import threading
import time
from types import SimpleNamespace as NS

import pytest

from openpilot.common.egpu_recovery_status import read_status, recovery_badge, write_status
from openpilot.selfdrive.modeld import egpu_recovery as recovery


USB_ERROR = RuntimeError('bulk OUT 0x02 failed: Input/Output Error')


class Clock:
  now = 100.

  def __call__(self):
    return self.now


class Messages(dict):
  def __init__(self, now, moving=True, override=False):
    super().__init__(carState=NS(standstill=not moving, vEgo=10. if moving else 0., steeringPressed=override),
                     selfdriveState=NS(enabled=True), carControl=NS(latActive=True))
    self.valid = dict.fromkeys(self, True)
    self.alive = dict(self.valid)
    self.recv_time = dict.fromkeys(self, now)


class Task:
  def __init__(self, prepare, healthy):
    self.cancel, self.done = threading.Event(), threading.Event()
    self.error, self.ready_at = None, None
    self.model = NS()

  def take(self):
    return self.model


@pytest.fixture
def manager(tmp_path):
  (tmp_path / 'installed.json').write_text(json.dumps({'format': 'comma-generic-onnx'}))
  clock = Clock()
  value = recovery.EgpuRecovery(tmp_path / 'model.pkl', 1344, 760, clock=clock, task_factory=Task,
                                status_path=tmp_path / 'status.json', budget_path=tmp_path / 'budget.json')
  return value, clock


def advance(manager, seconds, *, present=True, work=.02, dropped=0):
  value, clock = manager
  for _ in range(round(seconds / .05)):
    clock.now += .05
    value.observe(True, work, dropped)
    value.tick(Messages(clock.now), present)


def test_recovery_waits_for_internal_health_and_backoff_while_moving(manager):
  value, _ = manager
  value.failed(USB_ERROR)
  advance(manager, 9.9)
  assert value.task is None
  advance(manager, .2)
  assert value.state == 'recovering' and value.attempts == 1
  assert not value.task.cancel.is_set()


@pytest.mark.parametrize('present,work,dropped', [(False, .02, 0), (True, .06, 0), (True, .02, 1)])
def test_absent_device_or_unhealthy_internal_model_never_starts(manager, present, work, dropped):
  value, _ = manager
  value.failed(USB_ERROR)
  advance(manager, 20, present=present, work=work, dropped=dropped)
  assert value.task is None and value.attempts == 0


@pytest.mark.parametrize('missing,work,dropped,gap', [(True, .02, 0, .05), (False, .051, 0, .05),
                                                  (False, .02, 1, .05), (False, .02, 0, .11)])
def test_internal_degradation_cancels_and_stops_retries(manager, missing, work, dropped, gap):
  value, clock = manager
  value.failed(USB_ERROR)
  advance(manager, 11)
  task = value.task
  clock.now += gap
  value.observe(not missing, work, dropped)
  assert task.cancel.is_set() and value.state == 'stopped'
  task.done.set()
  advance(manager, 200)
  assert value.attempts == 1 and value.task is None


@pytest.mark.parametrize('moving,override,expected', [(True, False, False), (True, True, True), (False, False, True)])
def test_ready_waits_for_the_same_jetlink_transition_gate(manager, moving, override, expected):
  value, clock = manager
  value.failed(USB_ERROR)
  advance(manager, 11)
  task = value.task
  task.ready_at = clock.now
  actual = value.tick(Messages(clock.now, moving, override), True)
  assert (actual is task.model) == expected
  assert value.state == ('idle' if expected else 'ready')


@pytest.mark.parametrize('bad', ['stale', 'invalid', 'dead', 'future'])
def test_ready_rejects_bad_vehicle_state_even_when_stopped(manager, bad):
  value, clock = manager
  value.failed(USB_ERROR)
  advance(manager, 11)
  value.task.ready_at = clock.now
  messages = Messages(clock.now, moving=False)
  if bad == 'stale':
    messages.recv_time['carState'] -= .3
  elif bad == 'future':
    messages.recv_time['carState'] += 1
  elif bad == 'invalid':
    messages.valid['carControl'] = False
  else:
    messages.alive['selfdriveState'] = False
  assert value.tick(messages, True) is None


def test_exactly_three_attempts_cleanup_serialized_and_budget_survives_modeld_restart(manager, capsys):
  value, clock = manager
  value.failed(USB_ERROR)
  for attempt in range(1, 4):
    advance(manager, recovery.DELAYS[attempt - 1] + 1)
    task = value.task
    assert task is not None and value.attempts == attempt
    task.error = USB_ERROR
    advance(manager, 3)
    assert value.task is task  # cleanup has not completed, so no overlapping owner
    task.done.set()
    value.tick(Messages(clock.now), True)
  assert value.state == 'stopped'
  events = [json.loads(line.removeprefix('[eGPU] ')) for line in capsys.readouterr().err.splitlines()]
  assert [e['attempt'] for e in events if e['event'] == 'retry_started'] == [1, 2, 3]
  assert [e['delay_s'] for e in events if e['event'] == 'retry_scheduled'] == list(recovery.DELAYS)
  assert sum(e['event'] == 'retry_failed' for e in events) == 3
  assert events[-1]['event'] == 'retry_stopped' and events[-1]['reason'] == 'attempts_exhausted'
  assert not any(e['event'] == 'retry_selected' for e in events)
  restarted = recovery.EgpuRecovery(value.path, 1344, 760, clock=clock,
                                    status_path=value.status_path, budget_path=value.budget_path)
  restarted.failed(USB_ERROR)
  assert restarted.exhausted and restarted.attempts == 3


def test_retry_logs_distinguish_probe_ready_handover_and_actual_live_output(manager, capsys):
  value, clock = manager
  value.failed(USB_ERROR)
  advance(manager, 11)
  value.task.ready_at = clock.now
  for _ in range(20):
    value.tick(Messages(clock.now), True)
  assert value.tick(Messages(clock.now, moving=False), True) is not None
  captured = capsys.readouterr()
  events = [json.loads(line.removeprefix('[eGPU] ')) for line in captured.err.splitlines()]
  assert not captured.out
  assert events[0]['error'] == str(USB_ERROR) and events[0]['active_model'] == 'internal'
  assert sum(e['event'] == 'retry_ready' for e in events) == 1
  assert events[-1]['event'] == 'retry_selected'
  assert not any(e['event'] == 'live_output_resumed' for e in events)


def test_usb_absence_explained_once_without_frame_rate_log_spam(manager, capsys):
  value, _ = manager
  value.failed(USB_ERROR)
  advance(manager, 60, present=False)
  events = [json.loads(line.removeprefix('[eGPU] ')) for line in capsys.readouterr().err.splitlines()]
  waits = [e for e in events if e['event'] == 'retry_waiting']
  assert len(waits) == 1 and waits[0]['reason'] == 'USB absent'
  assert not any(e['event'] == 'retry_started' for e in events)


@pytest.mark.parametrize('error', [ValueError('non-finite model output'), ValueError('precompiled PKL checksum mismatch'),
                                  RuntimeError('AMD failed with 1'), RuntimeError('precompiled checkpoint mismatch'),
                                  TimeoutError('some other operation'), RuntimeError('precompiled eGPU worker exited (-9)')])
def test_numerical_integrity_and_unknown_failures_are_not_retried(manager, error):
  value, _ = manager
  value.failed(error)
  advance(manager, 20)
  assert value.exhausted and value.attempts == 0


@pytest.mark.parametrize('code', [-1, 1])
def test_usb_runtime_identity_required_and_worker_timeout_is_supported(code):
  error = RuntimeError(f'AMD failed with {code}')
  assert not recovery.retryable(error, {})
  manifest = {'runtime': {'sha256': '90970e01f2ba70d553294c0f9d5c6fe1678827f750705cab679a7b19b7bcb0b2'}}
  assert recovery.retryable(error, manifest)
  assert recovery.retryable(TimeoutError('precompiled eGPU worker timed out'), {})


def test_rejected_artifact_is_never_unblocked(manager):
  value, _ = manager
  marker = value.path.parent / 'rejected'
  marker.write_text('retain')
  value.failed(USB_ERROR)
  advance(manager, 20)
  assert value.attempts == 0 and marker.read_text() == 'retain'


def test_hung_cleanup_has_deadline_and_cannot_launch_another_worker(manager):
  value, clock = manager
  value.failed(USB_ERROR)
  advance(manager, 11)
  task = value.task
  advance(manager, recovery.TASK_SECONDS + 1)
  assert value.state == 'stopped' and task.cancel.is_set()
  assert value.task is task and value.attempts == 1


@pytest.mark.parametrize('message', ['PCIe link not up (LTSSM=0x00)',
                                  'TimeoutError: BL not ready. Timed out after 10000 ms, condition not met: 0 != 2147483648'])
def test_temporary_initialization_failure_can_use_the_next_attempt(manager, message):
  value, clock = manager
  value.failed(USB_ERROR)
  advance(manager, 11)
  value.task.error = RuntimeError(message)
  value.task.done.set()
  value.tick(Messages(clock.now), True)
  assert value.state == 'waiting'
  advance(manager, 31)
  assert value.attempts == 2


def test_reset_internal_history_preserves_captured_buffers():
  import ast
  from pathlib import Path
  import numpy as np
  source = Path(__file__).parents[1] / 'modeld.py'
  tree = ast.parse(source.read_text(encoding='utf8'))
  cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'ModelState')
  method = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == 'reset')
  namespace = {}
  exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
  state = bytearray([1] * 16)
  tensor = NS(_buffer=lambda: NS(copyin=lambda value: state.__setitem__(slice(None), value)),
              numel=lambda: 4, dtype=NS(itemsize=4))
  model = NS(prev_desire=np.ones(8), npy={'prev_feat': np.ones(3)}, input_queues={'feat_q': tensor})
  namespace['reset'](model)
  assert not model.prev_desire.any() and not model.npy['prev_feat'].any() and not any(state)
  assert model.input_queues['feat_q'] is tensor


def test_status_ready_expiry_and_badge(tmp_path):
  path = tmp_path / 'status.json'
  write_status({'state': 'ready', 'updated': 100.}, path)
  assert recovery_badge(read_status(path, 101.)) == ('eGPU READY', 'ready')
  assert not read_status(path, 103.) and not read_status(path, 99.)
  path.write_text('invalid')
  assert not read_status(path, 101.)


def test_task_cancels_when_frame_thread_stalls_and_never_hands_over(monkeypatch):
  monkeypatch.setattr(recovery, 'background_scheduling', lambda: None)
  healthy = [True]
  entered = threading.Event()
  def prepare(cancel):
    entered.set()
    assert cancel.wait(2)
    raise InterruptedError('cancelled')
  task = recovery.RecoveryTask(prepare, lambda: healthy[0])
  assert entered.wait(2)
  healthy[0] = False
  assert task.done.wait(2)
  assert task.cancel.is_set() and task.take() is None


def test_task_transfers_only_prepared_worker_and_does_not_close_new_owner(monkeypatch):
  monkeypatch.setattr(recovery, 'background_scheduling', lambda: None)
  closed = []
  model = NS(process=NS(poll=lambda: None), close=lambda: closed.append(True), recovery_cancel=None)
  task = recovery.RecoveryTask(lambda cancel: model, lambda: True)
  deadline = time.monotonic() + 2
  while task.ready_at is None and time.monotonic() < deadline:
    time.sleep(.001)
  assert task.take() is model
  assert task.done.wait(2) and not closed


def test_ready_timeout_cleans_worker_without_waiting_on_frame_thread(monkeypatch):
  monkeypatch.setattr(recovery, 'background_scheduling', lambda: None)
  monkeypatch.setattr(recovery, 'READY_SECONDS', .02)
  entered, release = threading.Event(), threading.Event()
  def close():
    entered.set()
    assert release.wait(2)
  model = NS(process=NS(poll=lambda: None), close=close)
  task = recovery.RecoveryTask(lambda cancel: model, lambda: True)
  assert entered.wait(2)
  assert task.take() is None and not task.done.is_set()
  release.set()
  assert task.done.wait(2)
  assert isinstance(task.error, recovery.ReadyExpired)


@pytest.mark.parametrize('missing', [False, True])
def test_preparation_requires_five_outputs_and_resets_synthetic_history(tmp_path, monkeypatch, missing):
  from pathlib import Path
  from openpilot.selfdrive.modeld import precompiled_runner
  manifest = {'format': 'comma-generic-onnx'}
  path = tmp_path / 'model.pkl'
  path.write_bytes(b'test')
  (tmp_path / 'installed.json').write_text(json.dumps(manifest))
  read_text = Path.read_text
  monkeypatch.setattr(Path, 'read_text', lambda self, *a, **kw: 'MemAvailable: 8000000 kB' if str(self).replace('\\', '/') == '/proc/meminfo'
                      else read_text(self, *a, **kw))
  calls = []
  class Model:
    frame_size = 16
    def __init__(self, width, height, path, *, recovery_cancel):
      pass
    def run(self, *args):
      calls.append('run')
      return None if missing else {'plan': 0}
    def reset_after_probe(self):
      calls.append('reset')
    def close(self):
      calls.append('close')
  monkeypatch.setattr(precompiled_runner, 'PrecompiledModelState', Model)
  cancel = NS(is_set=lambda: False, wait=lambda _: False)
  if missing:
    with pytest.raises(RuntimeError, match='missing or late'):
      recovery.prepare_candidate(path, 10, 10, cancel, manifest)
    assert calls == ['run', 'close']
  else:
    assert isinstance(recovery.prepare_candidate(path, 10, 10, cancel, manifest), Model)
    assert calls == ['run'] * 5 + ['reset']
