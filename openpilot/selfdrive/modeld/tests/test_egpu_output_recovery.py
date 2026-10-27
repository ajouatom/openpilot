import json
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.modeld.egpu_output_recovery import OutputRecovery, RepeatedInvalidModelOutput
from openpilot.selfdrive.modeld.output_validation import NonFiniteModelOutput
from openpilot.selfdrive.modeld import precompiled_model as pm


class Runtime:
  def __init__(self, results):
    self.results = iter(results)
    self.state = 0
    self.saved = None
    self.inputs = []

  def init_recovery_state(self):
    pass

  def run(self):
    self.inputs.append(self.state)
    self.state += 1
    result = next(self.results)
    if isinstance(result, Exception):
      raise result
    if not np.isfinite(result).all():
      self.state = float('nan')
    return np.array(result)

  def save_recovery_state(self):
    self.saved = self.state

  def restore_recovery_state(self):
    self.state = self.saved


def setup(results):
  clock = SimpleNamespace(now=0.)
  runtime, events = Runtime(results), []
  recovery = OutputRecovery(runtime, 2, lambda *a, **kw: events.append((a, kw)), lambda: clock.now)
  return recovery, runtime, clock, events


def test_single_bad_frame_restores_history_and_next_good_frame_clears_streak():
  recovery, runtime, clock, events = setup([[1, 2], [np.nan, 2], [3, 4], [5, np.inf], [6, 7]])
  np.testing.assert_array_equal(recovery.run(), [1, 2])
  clock.now = .05
  assert recovery.run() is None
  assert runtime.state == 1 and recovery.last_good == 0
  clock.now = .1
  np.testing.assert_array_equal(recovery.run(), [3, 4])
  assert recovery.failures == 0
  clock.now = .15
  assert recovery.run() is None
  clock.now = .2
  np.testing.assert_array_equal(recovery.run(), [6, 7])
  assert runtime.inputs == [0, 1, 1, 2, 2]
  assert [a[0] for a, _ in events] == ['precompiledOutputHeld', 'precompiledOutputRecovered'] * 2


def test_two_consecutive_bad_frames_raise_without_repeated_hold():
  recovery, runtime, clock, events = setup([[1, 2], [np.nan, 2], [1, np.inf]])
  recovery.run()
  clock.now = .05
  assert recovery.run() is None
  clock.now = .1
  with pytest.raises(RepeatedInvalidModelOutput):
    recovery.run()
  assert len(events) == 1 and runtime.saved == 1


@pytest.mark.parametrize('age', [-.01, .101, 1.])
def test_stale_previous_result_cannot_be_held(age):
  recovery, _, clock, events = setup([[1, 2], [np.nan, 2]])
  recovery.run()
  clock.now = age
  with pytest.raises(NonFiniteModelOutput):
    recovery.run()
  assert not events


@pytest.mark.parametrize('result,error', [([np.nan, 2], NonFiniteModelOutput), ([1], ValueError),
                                        (TimeoutError('GPU stalled'), TimeoutError),
                                        (RuntimeError('USB disconnected'), RuntimeError)])
def test_startup_and_execution_contract_errors_never_hold(result, error):
  recovery, _, _, events = setup([result])
  with pytest.raises(error):
    recovery.run()
  assert not events


def test_rollback_failure_or_delay_cannot_acknowledge_hold():
  for broken in (False, True):
    recovery, runtime, clock, events = setup([[1, 2], [np.nan, 2]])
    recovery.run()
    def restore(broken=broken, clock=clock):
      if broken:
        raise ValueError('invalid eGPU recovery state')
      clock.now = .101
    runtime.restore_recovery_state = restore
    with pytest.raises(ValueError):
      recovery.run()
    assert not events


@pytest.mark.parametrize('change,allowed', [({}, True), ({'boot_id': 'now'}, False), ({'boot_id': ''}, False),
                                          ({'phase': 'load'}, False), ({'phase': 'boot_validation'}, False),
                                          ({'pickle_sha256': 'b'}, False), ({'rejected': False}, False)])
@pytest.mark.parametrize('error', ['ValueError: invalid precompiled model output',
                                 'openpilot.selfdrive.modeld.output_validation.NonFiniteModelOutput: invalid numbers',
                                 'openpilot.selfdrive.modeld.egpu_output_recovery.RepeatedInvalidModelOutput: two consecutive invalid eGPU outputs'])
def test_output_block_expires_only_with_matching_previous_boot_evidence(tmp_path, monkeypatch, change, allowed, error):
  from openpilot.selfdrive.modeld import egpu_worker_progress
  monkeypatch.setattr(egpu_worker_progress, 'boot_identity', lambda: 'now')
  failure = {'rejected': True, 'pickle_sha256': 'a', 'phase': 'inference', 'boot_id': 'old',
             'error': 'Traceback (most recent call last):\n' + error} | change
  (tmp_path / 'last_failure.json').write_text(json.dumps(failure))
  assert pm.retry_output_rejection(tmp_path, 'a') == allowed
  assert json.loads((tmp_path / 'last_failure.json').read_text()) == failure


@pytest.mark.parametrize('error', ['ValueError: invalid precompiled model output: count=1, expected=2',
                                 'ValueError: checkpoint mismatch', 'NonFiniteModelOutput: invalid\nValueError: checksum mismatch',
                                 'ValueError: quoted NonFiniteModelOutput: invalid'])
def test_contract_integrity_and_quoted_errors_remain_blocked(tmp_path, monkeypatch, error):
  from openpilot.selfdrive.modeld import egpu_worker_progress
  monkeypatch.setattr(egpu_worker_progress, 'boot_identity', lambda: 'now')
  (tmp_path / 'last_failure.json').write_text(json.dumps(
    {'rejected': True, 'pickle_sha256': 'a', 'phase': 'inference', 'boot_id': 'old', 'error': error}))
  assert not pm.retry_output_rejection(tmp_path, 'a')
