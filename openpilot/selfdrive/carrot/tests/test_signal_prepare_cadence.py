"""Camera capture, delayed publication and planner polling are distinct clocks."""
from copy import deepcopy

import pytest

from openpilot.selfdrive.carrot.signal_stop_prepare import SignalStopPrepare, PrepareAccel
from openpilot.selfdrive.carrot.tests.test_signal_stop_prepare import context, observation


def test_seven_hz_with_125ms_latency_confirms_without_stale_actuation():
  gate = SignalStopPrepare()
  cap = PrepareAccel()
  latest = None
  decisions = []
  for tick in range(41):
    now = tick*.05
    index = int((now-.125)/.15) if now >= .125 else -1
    if index < 0:
      continue
    latest = observation(index*.15)
    decision = gate.update(now, latest, **context())
    decisions.append(decision)
    target = cap.apply(1., 0., .05, decision.active)
    if now-latest['timestamp'] > .2:
      assert not decision.active and target == 1. and cap.cap is None
      assert decision.reason == 'awaiting_fresh'
    elif latest['timestamp'] >= .6:
      assert decision.active
  assert any(d.active for d in decisions)
  assert sum(d.entered for d in decisions) == 1


def primed():
  gate = SignalStopPrepare()
  for i in range(5):
    stamp = i*.15
    gate.update(stamp+.125, observation(stamp), **context())
    if i < 4:
      gate.update(stamp+.225, observation(stamp), **context())
  assert gate.active
  return gate


@pytest.mark.parametrize('change', ['unknown', 'green', 'missing', 'identity', 'session', 'capture_gap', 'late_new'])
def test_waiting_does_not_preserve_history_across_new_bad_evidence(change):
  gate = primed()
  assert gate.update(.825, observation(.6), **context()).reason == 'awaiting_fresh'
  stamp, now = .75, .875
  obs = observation(stamp)
  if change in ('unknown', 'green'):
    obs = observation(stamp, change)
  if change == 'missing':
    obs['tracks'] = []
  if change == 'identity':
    obs['tracks'][0]['id'] = 2
  if change == 'session':
    obs['session'] = 'restart'
  if change == 'capture_gap':
    stamp, now = .9, 1.025
    obs = observation(stamp)
  if change == 'late_new':
    now = .975
  assert not gate.update(now, obs, **context()).active
  stamp += .15
  assert not gate.update(now+.15, observation(stamp), **context()).active


@pytest.mark.parametrize('field', ['lead', 'gas', 'brake', 'turning', 'model_stopping'])
def test_context_cancellation_during_wait_clears_confirmation(field):
  gate = primed()
  ctx = context()
  ctx[field] = True
  assert not gate.update(.825, observation(.6), **ctx).active
  assert not gate.histories
  assert not gate.update(.875, observation(.75), **context()).active


def test_wait_is_bounded_and_duplicate_never_advances_capture_or_votes():
  gate = primed()
  before = deepcopy(gate.histories)
  for now in (.825, .9, 1., 1.049):
    assert not gate.update(now, observation(.6), **context()).active
    assert gate.histories == before and gate.last_capture == .6
  assert gate.update(1.051, observation(.6), **context()).reason == 'stale'
  assert not gate.histories


@pytest.mark.parametrize('kind', ['session', 'frame', 'stamp', 'version', 'stream', 'size'])
def test_old_but_unaccepted_or_malformed_sample_cannot_retain_history(kind):
  gate = primed()
  obs = observation(.6)
  if kind == 'session':
    obs['session'] = 'other'
  if kind == 'frame':
    obs['frame_id'] += 1
  if kind == 'stamp':
    obs['timestamp'] += .001
  if kind == 'version':
    obs['version'] = 2
  if kind == 'stream':
    obs['stream'] = 'wide'
  if kind == 'size':
    obs['size'] = [2688, 1520]
  assert gate.update(.825, obs, **context()).reason == 'stale'
  assert not gate.histories
