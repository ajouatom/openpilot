"""Actual transition failures: resized housing and separate lower red signals."""
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.carrot.signal_start_guard import SignalStartGuard, observation_color
from openpilot.selfdrive.carrot.tests.test_signal_start_group import step
from openpilot.selfdrive.carrot.tests.test_signal_start_guard import held, observation
from tools.signal_analysis.replay_signal_assist import planner_class, REPO
from tools.signal_analysis.test_signal_assist_planner import inputs


@pytest.mark.parametrize('identity,history,box,release', [
  (317, True, [647, 286, 671, 298], True),
  (999, True, [647, 286, 671, 298], False),
  (317, False, [647, 286, 671, 298], False),
  (None, True, [647, 286, 671, 298], False),
  (317, True, [640, 286, 664, 298], False),
  (317, True, [647, 276, 671, 288], False),
])
def test_resize_needs_same_producer_identity_red_history_and_bounded_motion(identity, history, box, release):
  guard = SignalStartGuard()
  for i in range(20):
    t = i*.1
    red = i < 4
    track = dict(box=[647, 288, 683, 298] if red else box, raw='red' if red else 'green',
                 id=317 if red else identity, seen_red=history)
    d = guard.update(t, dict(timestamp=t, frame_id=i, session='camera', tracks=[track]),
                     enabled=True, valid=True, drive=True, gas=False, speed=0., departure=True)
  assert d.hold is not release and d.green_confirmed is release


LEFT, RIGHT = [572, 307, 600, 321], [652, 308, 680, 322]
LOW = [570, 363, 586, 371]


@pytest.mark.parametrize('conflict,release', [(LOW, True), ([570, 320, 586, 328], False),
  ([570, 363, 610, 383], False), ([660, 310, 674, 317], False), ([570, 280, 586, 288], False)])
def test_only_two_overhead_transitions_can_exclude_much_lower_smaller_red(conflict, release):
  guard = SignalStartGuard()
  released = []
  for i in range(30):
    d = step(guard, i*.1, [(LEFT, 'red' if i < 5 else 'green'),
                          (RIGHT, 'red' if i < 5 else 'green'), (conflict, 'red')])
    if d.released:
      released.append(i)
  assert bool(released) is release
  assert len(released) <= 1
  assert d.hold is not release
  assert d.green_confirmed is release
  if release:
    # Loss of the live pair removes the exception and reacquires the lower red.
    for i in range(30, 35):
      d = step(guard, i*.1, [(conflict, 'red')])
    assert d.hold and not d.green_confirmed


def test_single_green_cannot_exclude_even_lower_red():
  guard = SignalStartGuard()
  for i in range(25):
    d = step(guard, i*.1, [(RIGHT, 'red' if i < 5 else 'green'), (LOW, 'red')])
  assert d.hold and not d.green_confirmed


@pytest.mark.parametrize('failure', ['unknown', 'missing', 'invalid', 'restart', 'stale', 'red', 'moving'])
def test_green_input_expires_on_lost_evidence_context_or_completed_launch(failure):
  guard = held()
  kw = dict(enabled=True, valid=True, drive=True, gas=False, speed=0., departure=True)
  for t in (.7, 1.05, 1.4):
    d = guard.update(t+.35, observation(t, 'green'), **kw)
  assert d.green_confirmed
  obs = observation(1.8, 'green')
  if failure == 'unknown':obs['tracks'][0]['raw'] = 'unknown'
  if failure == 'missing':obs = None
  if failure == 'invalid':kw['valid'] = False
  if failure == 'restart':obs['session'] = 'new'
  if failure == 'stale':obs['timestamp'] = 1.
  if failure == 'red':obs['tracks'][0]['raw'] = 'red'
  if failure == 'moving':kw['speed'] = 5/3.6
  assert not guard.update(2., obs, **kw).green_confirmed
  if failure == 'invalid':
    # Repeating the previous green after an invalid context cannot restore it.
    kw['valid'] = True
    assert not guard.update(2.01, observation(1.4, 'green'), **kw).green_confirmed


@pytest.mark.parametrize('inhibit', ['brake', 'right_blinker', 'stay', 'user_stop', 'high', 'mode0'])
def test_transition_cannot_override_driver_stop_or_disabled_native_departure(inhibit):
  cls, ns = planner_class((REPO/'openpilot/selfdrive/carrot/carrot_functions.py').read_text(encoding='utf8'))
  p = cls(start_guard=SignalStartGuard())
  sm = inputs(distance=3.)
  sm['modelV2'].velocity.x = [0.]*33
  for i in range(70):
    if i >= 20:
      if inhibit == 'brake':sm['carState'].brakePressed = True
      if inhibit == 'right_blinker':sm['carState'].rightBlinker = True
      if inhibit == 'stay':
        ns['get_carrot_man'] = lambda sm: NS(trafficState=2, atcType='turn left', activeCarrot=1, xDistToTurn=0., desiredSpeed=80.)
      if inhibit == 'user_stop':p.user_stop_distance = 0.
      if inhibit == 'high':p.myDrivingMode = ns['DrivingMode'].High
      if inhibit == 'mode0':p.trafficLightDetectMode = 0
    t = i*.05
    p.update(sm, 80., 'acc', start_observation=observation(t, 'red' if i < 20 else 'green'),
             start_context=dict(now=t, enabled=True, valid=True, drive=True))
  assert not p.signal_green_active
  assert p.xState.name == 'e2eStopped'


def test_display_color_reports_current_conflict_and_never_latched_red():
  ctx = dict(enabled=True, valid=True, drive=True)
  obs = observation(1.)
  assert observation_color(obs, 1.2, **ctx) == 1
  obs['tracks'].append(dict(box=[740, 250, 780, 270], raw='green'))
  assert observation_color(obs, 1.2, **ctx) == 3
  assert observation_color(obs, 1.7, **ctx) == 0
  assert observation_color(obs, float('nan'), **ctx) == 0
  assert observation_color(None, 1.2, **ctx) == 0
  assert observation_color(obs, 1.2, **dict(ctx, enabled=False)) == 0


def test_fresh_conflicting_red_restarts_green_confirmation():
  guard = SignalStartGuard()
  for i in range(5):
    step(guard, i*.1, [(LEFT,'red'), (RIGHT,'red')])
  for i in range(5,10):
    step(guard, i*.1, [(LEFT,'green'), (RIGHT,'green')])
  conflict = [730,310,754,322]
  assert step(guard, 1., [(LEFT,'green'),(RIGHT,'green'),(conflict,'red')]).hold
  for i in range(11,18):
    assert step(guard, i*.1, [(LEFT,'green'),(RIGHT,'green')]).hold
  assert step(guard, 1.8, [(LEFT,'green'),(RIGHT,'green')]).released


def test_confirmed_transition_only_bridges_initial_low_speed_launch():
  guard = held()
  kw = dict(enabled=True,valid=True,drive=True,gas=False,speed=0.,departure=True)
  for t in (.7,1.05,1.4):
    guard.update(t+.35,observation(t,'green'),**kw)
  assert guard.update(2.,observation(1.8,'green'),**dict(kw,speed=.5)).green_confirmed
  assert not guard.update(2.2,observation(2.,'green'),**dict(kw,speed=2.)).green_confirmed
  assert not guard.update(2.4,observation(2.2,'green'),**kw).green_confirmed


@pytest.mark.parametrize('speed,standstill,immediate', [(0.,True,True),(.05,True,True),(.15,True,False),(0.,False,False)])
def test_confirmed_stopped_release_does_not_wait_for_a_second_timer(speed, standstill, immediate):
  cls, _ = planner_class((REPO/'openpilot/selfdrive/carrot/carrot_functions.py').read_text(encoding='utf8'))
  p = cls(start_guard=SignalStartGuard())
  sm = inputs(speed=speed,distance=3.)
  sm['carState'].standstill = standstill
  sm['modelV2'].velocity.x = [0.]*33
  released = False
  for i in range(40):
    t = i*.1
    p.update(sm,80.,'acc',start_observation=observation(t,'red' if i<5 else 'green'),
             start_context=dict(now=t,enabled=True,valid=True,drive=True))
    if p.start_decision.released:
      released = True
      assert (p.xState.name == 'e2eCruise') is immediate
      assert (p.stopping_count == 0) is immediate
      break
  assert released
