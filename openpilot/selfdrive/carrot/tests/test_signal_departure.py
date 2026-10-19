"""Stopped signal memory must not become a road-speed stop after lead departure."""
import ast
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.carrot.tests.test_signal_lead_handover import setup, update
from openpilot.selfdrive.carrot.tests.test_signal_start_guard import held, step
from tools.signal_analysis.replay_signal_assist import REPO


@pytest.mark.parametrize('guards', ['start', 'legacy', 'both'])
@pytest.mark.parametrize('loss', ['missing', 'outside'])
@pytest.mark.parametrize('speed', [.2, 1., 5/3.6, 5/3.6+.001, 21.25])
def test_lead_departure_then_loss(guards, loss, speed):
  p, sm = setup()
  if guards == 'start':
    p.signal_assist = p.signal_decision = None
  elif guards == 'legacy':
    p.start_guard = p.start_decision = None
  sm['radarState'].leadOne = NS(status=True, dRel=4., vLead=speed)
  update(p, sm, 1., 'unknown')
  sm['carState'].vEgo = sm['carState'].vEgoCluster = speed
  update(p, sm, 1.05, 'unknown')
  if loss == 'missing':
    sm['radarState'].leadOne.status = False
  else:
    sm['radarState'].leadOne.dRel = 1000.
  update(p, sm, 1.1, 'unknown')
  holds = [d.hold for d in (p.start_decision, p.signal_decision) if d is not None]
  if speed <= 5/3.6:
    assert all(holds) and p.xState.name == 'e2eStopped' and p.v_cruise == 0
  else:
    assert not any(holds)
    assert p.xState.name not in ('e2eStop', 'e2eStopped') and p.stop_dist == 1000.
    assert p.v_cruise > 0
    # Returning to a stop with no new red cannot revive an earlier intersection.
    sm['carState'].vEgo = sm['carState'].vEgoCluster = 0.
    update(p, sm, 1.15, 'unknown')
    assert not any(d.hold for d in (p.start_decision, p.signal_decision) if d)


@pytest.mark.parametrize('guards', ['start', 'legacy', 'both'])
@pytest.mark.parametrize('driver_hold', ['none', 'soft', 'user', 'stay'])
def test_already_applied_signal_stop_does_not_survive_departure(guards, driver_hold):
  p, sm = setup()
  if guards == 'start':
    p.signal_assist = p.signal_decision = None
  elif guards == 'legacy':
    p.start_guard = p.start_decision = None
  if driver_hold == 'soft':
    sm['carState'].softHoldActive = 1
  elif driver_hold == 'user':
    p.user_stop_distance = 0.
  elif driver_hold == 'stay':
    p.carrot_stay_stop = True
  sm['carState'].vEgo = sm['carState'].vEgoCluster = 21.25
  update(p, sm, 1., 'unknown')
  assert not any(d.hold for d in (p.start_decision, p.signal_decision) if d)
  assert (p.xState.name == 'e2eStopped') is (driver_hold != 'none')
  if driver_hold == 'none':
    assert p.stop_dist == 1000. and p.v_cruise > 0


@pytest.mark.parametrize('context', [{}, {'valid': False}, {'observation': None},
                                   {'observation': {'timestamp': float('nan')}}])
@pytest.mark.parametrize('speed', [21.25, -2.])
def test_held_start_guard_leaves_stopped_scope_even_with_bad_observation(context, speed):
  g = held()
  d = step(g, .7, speed=speed, **context)
  assert not d.hold and not g.hold and not d.green_confirmed and not d.released


def test_clock_rollback_cannot_reapply_stopped_hold_at_speed():
  g = held()
  assert not step(g, -1., speed=21.25).hold
  assert not step(g, 1., 'unknown', speed=0.).hold


@pytest.mark.parametrize('speed', [0., 1., 5/3.6, 21.25, -2.])
def test_final_output_rejects_stale_stopped_hold_at_speed(speed):
  path = REPO/'openpilot/selfdrive/controls/lib/longitudinal_planner.py'
  cls = next(n for n in ast.parse(path.read_text(encoding='utf8')).body
             if isinstance(n, ast.ClassDef) and n.name == 'LongitudinalPlanner')
  method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'apply_signal_start_guard')
  ns = {}
  exec(compile(ast.Module([method], type_ignores=[]), str(path), 'exec'), ns)
  output = NS(output_should_stop=False, output_v_target_now=20., output_a_target=-.7)
  ns['apply_signal_start_guard'](output, NS(start_decision=NS(hold=True)), speed)
  assert output.output_should_stop is (abs(speed) <= 5/3.6)
  assert output.output_v_target_now == (0. if abs(speed) <= 5/3.6 else 20.)
  assert output.output_a_target == -.7  # existing braking is never removed


@pytest.mark.parametrize('guards', ['start', 'legacy', 'both'])
def test_disable_cannot_leave_an_orphan_signal_stop_after_motion(guards):
  p, sm = setup()
  if guards == 'start':
    p.signal_assist = p.signal_decision = None
  elif guards == 'legacy':
    p.start_guard = p.start_decision = None
  for t, speed in [(1., 0.), (1.05, 21.25)]:
    sm['carState'].vEgo = sm['carState'].vEgoCluster = speed
    ctx = dict(now=t, enabled=False, valid=True, drive=True)
    p.update(sm, 80., 'acc', start_context=ctx, signal_context=ctx)
  assert p.xState.name not in ('e2eStop', 'e2eStopped')
  assert p.stop_dist == 1000. and p.v_cruise > 0


def test_fresh_moving_red_can_still_request_a_model_distance_stop():
  from tools.signal_analysis.test_signal_assist_planner import run
  p, sm = setup()
  sm['radarState'].leadOne = NS(status=True, dRel=4., vLead=10.)
  sm['carState'].vEgo = sm['carState'].vEgoCluster = 10.
  update(p, sm, 1., 'unknown')
  assert not p.start_guard.hold and not p.signal_assist.hold
  sm['radarState'].leadOne.status = False
  sm['modelV2'].position.x = [40.*i/32 for i in range(33)]
  run(p, sm, 'red', start=2., count=40)
  assert p.signal_assist.committed and not p.signal_assist.hold
  assert p.xState.name == 'e2eStop' and 0 < p.stop_dist < 40.


def test_new_stopped_red_needs_new_confirmation_after_departure():
  g = held()
  assert not step(g, .7, speed=21.25).hold
  assert not step(g, 1.05, 'unknown').hold
  assert not step(g, 1.4, 'red').hold
  assert step(g, 1.75, 'red').hold
