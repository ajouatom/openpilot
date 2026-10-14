"""Reviewed Santa Fe box geometry; these tests do not simulate image recognition."""
import pytest

from tools.signal_analysis.signal_upper_group_trial import UpperSignalGroupTrial as SignalStartGuard
from tools.signal_analysis.signal_upper_group_trial import compare_recorded_inputs
from openpilot.selfdrive.carrot.tests.test_signal_start_group import step


UPPER_RED = ([613, 131, 653, 151], [763, 120, 807, 142])
UPPER_GREEN = ([617, 128, 661, 150], [772, 119, 812, 139])
REAR_RED = [929, 350, 977, 374]


def acquire(guard, pair=UPPER_RED):
  for t in (0., .2, .4):
    step(guard, t, [(b, 'red') for b in (*pair, REAR_RED)])
  assert guard.hold


def test_two_upper_transitions_exclude_larger_but_far_lower_candidate():
  guard = SignalStartGuard()
  acquire(guard)
  for t in (.6, .8, 1., 1.2):
    assert step(guard, t, [(b, 'green') for b in UPPER_GREEN] + [(REAR_RED, 'red')]).hold
  decision = step(guard, 1.4, [(b, 'green') for b in UPPER_GREEN] + [(REAR_RED, 'red')])
  assert decision.released and decision.green_confirmed and decision.reason == 'green_group'
  # Unknown removes green authorization; loss of the pair reacquires lower red.
  for t in (1.6, 1.8, 2.):
    decision = step(guard, t, [(b, 'unknown') for b in UPPER_GREEN] + [(REAR_RED, 'red')])
    assert not decision.green_confirmed
  for t in (2.2, 2.4, 2.6):
    decision = step(guard, t, [(REAR_RED, 'red')])
  assert decision.hold and not decision.green_confirmed


@pytest.mark.parametrize('failure', ['one', 'new', 'unknown', 'duplicate', 'red', 'row', 'stale', 'restart'])
def test_upper_pair_does_not_turn_unqualified_green_into_release(failure):
  guard = SignalStartGuard()
  acquire(guard, () if failure == 'new' else UPPER_RED)
  for t in (.6, .8, 1., 1.2, 1.4, 1.6):
    pair = [(b, 'green') for b in UPPER_GREEN]
    if failure == 'one': pair = pair[:1]
    if failure == 'unknown': pair[1] = (UPPER_GREEN[1], 'unknown')
    if failure == 'duplicate': pair[1] = pair[0]
    if failure == 'red': pair[1] = (UPPER_RED[1], 'red')
    if failure == 'row': pair += [([690, 130, 730, 150], 'red')]
    if failure == 'restart': guard.session = 'restarted'
    if failure == 'stale':
      guard.update(t+1., dict(timestamp=t, frame_id=round(t*1000), session='camera',
                             tracks=[dict(box=b, raw=c) for b, c in pair]),
                   enabled=True, valid=True, drive=True, gas=False, speed=0., departure=True)
    else:
      result = step(guard, t, pair + [(REAR_RED, 'red')])
      assert result.hold and not result.green_confirmed
  assert guard.hold


@pytest.mark.parametrize('red', [
  [929, 200, 977, 224],  # Larger lower candidate, but insufficient separation.
  [929, 350, 1029, 374],  # Too wide to use this grouping hypothesis.
  [929, 350, 977, 398],  # Too tall; a nearby separate signal cannot be discarded.
  [620, 125, 660, 145],  # Same overhead row.
  [929, 90, 977, 114],  # Above the group.
])
def test_conflicting_red_geometry_remains_a_veto(red):
  guard = SignalStartGuard()
  acquire(guard)
  for t in (.6, .8, 1., 1.2, 1.4, 1.6):
    assert step(guard, t, [(b, 'green') for b in UPPER_GREEN] + [(red, 'red')]).hold


def test_recorded_input_comparison_reports_actual_change_and_reviewed_truth():
  rows = []
  for i in range(20):
    red = i < 5
    pair = UPPER_RED if red else UPPER_GREEN
    rows.append(dict(now=i*.1+.2, truth='red' if red else 'green',
                     observation=dict(timestamp=i*.1, frame_id=i, session='camera',
                       tracks=[dict(box=b, raw='red' if red else 'green') for b in pair] + [dict(box=REAR_RED, raw='red')]),
                     context=dict(enabled=True, valid=True, drive=True, gas=False, speed=0., departure=True)))
  result = compare_recorded_inputs(rows)
  assert result['baseline']['releases'] == 0
  assert result['trial']['releases'] == 1
  assert result['trial']['reviewed_red_releases'] == 0
  assert result['changes'] and result['releases'][0]['truth'] == 'green'
