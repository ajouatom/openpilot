"""Exercise the production planner and its final veto, not a replacement policy."""
import ast
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.carrot.signal_assist import SignalAssist
from openpilot.selfdrive.carrot.signal_start_guard import SignalStartGuard
from openpilot.selfdrive.carrot.tests.test_signal_start_guard import observation
from tools.signal_analysis.replay_signal_assist import REPO, planner_class
from tools.signal_analysis.test_signal_assist_planner import inputs


def setup():
  cls, _ = planner_class((REPO/'openpilot/selfdrive/carrot/carrot_functions.py').read_text(encoding='utf8'))
  p = cls(start_guard=SignalStartGuard(), signal_assist=SignalAssist())
  sm = inputs(distance=320./31.)
  for i in range(15):
    update(p, sm, i*.05)
  assert p.start_decision.hold and p.signal_decision.hold
  return p, sm


def update(p, sm, t, color='red', mode='acc'):
  ctx = dict(now=t, enabled=True, valid=True, drive=True)
  old = dict(timestamp=t, frame_id=round(t*1000), session='camera', tracks=[dict(
    id=1, box=[640,250,680,270], state=color, age=0., observations=10, evidence={'raw':color})])
  p.update(sm, 80., mode, start_observation=observation(t, color), start_context=ctx,
           signal_observation=old, signal_context=ctx)


@pytest.mark.parametrize('distance,allowed', [(4., True), (11.99, True), (12., False),
  (12.01, False), (100., False), (0., False), (-1., False), (float('nan'), False), (float('inf'), False)])
@pytest.mark.parametrize('mode', ['acc', 'blended'])
def test_exact_existing_boundary_controls_both_auxiliary_vetoes(distance, allowed, mode):
  p, sm = setup()
  sm['radarState'].leadOne = NS(status=True, dRel=distance, vLead=1.)
  update(p, sm, 1., mode=mode)
  assert p.start_decision.hold is (not allowed)
  assert p.signal_decision.hold is (not allowed)
  assert p.xState.name == ('lead' if allowed else 'e2eStopped')
  assert not p.start_decision.green_confirmed and not p.start_decision.released
  if allowed:
    assert 'audioRefuse' not in p.events.events
    assert p.signal_observation_color == 1
    assert p.start_guard.hold and p.signal_assist.hold
    assert p.stop_dist == 1000.
    path = REPO/'openpilot/selfdrive/controls/lib/longitudinal_planner.py'
    cls = next(n for n in ast.parse(path.read_text(encoding='utf8')).body if isinstance(n, ast.ClassDef) and n.name=='LongitudinalPlanner')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name=='apply_signal_start_guard')
    ns = {}; exec(compile(ast.Module([method], type_ignores=[]), str(path), 'exec'), ns)
    output = NS(output_should_stop=False, output_v_target_now=1., output_a_target=.2)
    ns['apply_signal_start_guard'](output, p, sm['carState'].vEgo)
    assert not output.output_should_stop and output.output_v_target_now == 1.


@pytest.mark.parametrize('loss', ['missing', 'outside'])
def test_lead_exception_is_continuous_and_unknown_red_hold_returns_on_loss(loss):
  p, sm = setup()
  sm['radarState'].leadOne = NS(status=True, dRel=4., vLead=0.)
  for i in range(20, 40):
    update(p, sm, i*.05, 'unknown')
    assert p.xState.name == 'lead' and not p.start_decision.hold
  assert p.v_cruise > 0  # following target only; MPC still sees the stationary lead
  if loss == 'missing':
    sm['radarState'].leadOne.status = False
  else:
    sm['radarState'].leadOne.dRel = 12.
  update(p, sm, 2., 'unknown')
  assert p.start_decision.hold and p.xState.name == 'e2eStopped' and p.v_cruise == 0
  assert ('audioRefuse' in p.events.events) is (loss == 'missing')


@pytest.mark.parametrize('inhibit', ['soft_hold', 'user_stop'])
def test_driver_holds_survive_qualifying_lead(inhibit):
  p, sm = setup()
  sm['radarState'].leadOne = NS(status=True, dRel=4.)
  if inhibit == 'soft_hold':
    sm['carState'].softHoldActive = 1
  else:
    p.user_stop_distance = 0.
  update(p, sm, 1.)
  assert not p.start_decision.hold and p.xState.name == 'e2eStopped'
