"""Exercise the production planner output block without loading the native MPC."""
import ast
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib import longitudinal_preview as preview


@pytest.fixture
def planner_preview():
  lib = Path(__file__).resolve().parents[1] / 'lib'
  ns = {**vars(preview), 'np': np, 'DT_MDL': .05}
  helper = ast.parse((lib / 'drive_helpers.py').read_text(encoding='utf-8'))
  calc = next(n for n in helper.body if isinstance(n, ast.FunctionDef) and n.name == 'get_accel_from_plan')
  exec(compile(ast.Module(body=[calc], type_ignores=[]), str(lib / 'drive_helpers.py'), 'exec'), ns)
  tree = ast.parse((lib / 'longitudinal_planner.py').read_text(encoding='utf-8'))
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'LongitudinalPlanner')
  update = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'update')

  def assigns(node, name):
    return isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == name for t in node.targets)

  lo = next(i for i, n in enumerate(update.body) if assigns(n, 'lead_index'))
  hi = next(i for i, n in enumerate(update.body) if assigns(n, 'output_a_target_mpc'))
  code = compile(ast.Module(body=update.body[lo:hi+1], type_ignores=[]), str(lib / 'longitudinal_planner.py'), 'exec')
  times = 10 * (np.arange(17) / 32)**2
  state = SimpleNamespace(lead_preview=.87, mpc=SimpleNamespace(mode='acc', source='lead0'))

  def step(a_lead, *, a_ego=0., mode=preview.DRIVING_MODE_NORMAL, status=True, radar=True,
           track_id=52, gas=False, brake=False, reset=False, mpc_mode='acc', source='lead0', a_now=1.6, jerk=-.6):
    state.mpc.mode, state.mpc.source = mpc_mode, source
    state.v_desired_trajectory = 10 + a_now * times + .5 * jerk * times**2
    state.a_desired_trajectory = a_now + jerk * times
    lead = SimpleNamespace(status=status, radar=radar, radarTrackId=track_id, aLeadK=a_lead)
    absent = SimpleNamespace(status=False, radar=False, radarTrackId=-1, aLeadK=0.)
    rs = SimpleNamespace(leadOne=lead if source != 'lead1' else absent, leadTwo=lead if source == 'lead1' else absent)
    base = ns['get_accel_from_plan'](state.v_desired_trajectory, state.a_desired_trajectory, times, action_t=.35)[0]
    ns.update(self=state, sm={'radarState': rs, 'carState': SimpleNamespace(aEgo=a_ego, gasPressed=gas, brakePressed=brake)},
              carrot=SimpleNamespace(myDrivingMode=mode), reset_state=reset, action_t=.35,
              vEgoStopping=.05, CONTROL_N_T_IDX=times, output_a_target_base=base)
    exec(code, ns)
    return ns['output_a_target_mpc'], base, state.lead_preview

  return step


@pytest.mark.parametrize('mode', [1, 2, 3, 4])
@pytest.mark.parametrize('source', ['lead0', 'lead1', 'cruise'])
def test_zero_crossing_releases_preview_without_returning_abruptly_to_base(planner_preview, mode, source):
  before, base, offset = planner_preview(-.31, mode=mode, source=source)
  after, _, remaining = planner_preview(.50, mode=mode, source=source)
  assert remaining < offset
  assert before < after < base - .2
  assert after - before < .05
  for _ in range(60):
    target, base, remaining = planner_preview(.50, mode=mode, source=source)
  assert remaining == 0.
  assert target == pytest.approx(base)


def test_repeated_deadband_crossings_do_not_toggle_the_output(planner_preview):
  targets = [planner_preview(a)[0] for a in [-.31, .05, -.31, .05, -.31, .05]]
  assert max(abs(np.diff(targets))) < .05


@pytest.mark.parametrize('blocked', [{'gas': True}, {'brake': True}, {'reset': True}, {'mpc_mode': 'blended'}])
def test_driver_intervention_or_control_exit_discards_remaining_time(planner_preview, blocked):
  assert planner_preview(-1.)[2] > 0.
  kwargs = {'a_lead': .5}
  kwargs.update(blocked)
  target, base, remaining = planner_preview(**kwargs)
  assert remaining == 0.
  assert target == pytest.approx(base)
  # Re-engagement starts from zero; no old correction returns.
  assert planner_preview(.5)[2] == 0.


@pytest.mark.parametrize('mode', [1, 2, 3, 4])
@pytest.mark.parametrize('source', ['lead0', 'lead1', 'cruise'])
@pytest.mark.parametrize('missing', [{'status': False}, {'radar': False}, {'track_id': -1},
                                   {'a_lead': float('nan')}, {'a_ego': float('nan')}])
def test_lost_radar_support_releases_remaining_correction_on_current_plan(planner_preview, mode, source, missing):
  before, base, offset = planner_preview(-2., mode=mode, source=source)
  kwargs = {'a_lead': -2., 'mode': mode, 'source': source, **missing}
  target, _, remaining = planner_preview(**kwargs)
  assert remaining == pytest.approx(offset - .03)
  assert before < target < base - .2
  assert target - before < .05
  for _ in range(55):
    target, base, remaining = planner_preview(**kwargs)
  assert remaining == 0.
  assert target == pytest.approx(base)


def test_repeated_radar_vision_handoffs_do_not_toggle_to_base(planner_preview):
  targets = [planner_preview(-2., radar=radar)[0] for radar in [True, False]*30]
  assert max(abs(np.diff(targets))) < .06


@pytest.mark.parametrize('radar', [False, True])
@pytest.mark.parametrize('mode,limit', [(1, .09), (2, .10), (3, .08), (4, .08)])
def test_new_braking_plan_is_immediate_during_handoff(planner_preview, radar, mode, limit):
  planner_preview(-2., mode=mode)
  planner_preview(0., radar=False, mode=mode)
  target, base, _ = planner_preview(-4., radar=radar, track_id=63 if radar else -1,
                                  mode=mode, a_now=-3., jerk=-1.)
  assert target <= base
  assert target == pytest.approx(base - limit)
  assert target < -3.


@pytest.mark.parametrize('a_now,jerk', [(0., 0.), (.8, .3), (-.8, .3)])
def test_handoff_tail_does_not_brake_on_a_flat_or_rising_new_plan(planner_preview, a_now, jerk):
  planner_preview(-2.)
  target, base, remaining = planner_preview(0., radar=False, a_now=a_now, jerk=jerk)
  assert remaining > 0.
  assert target == pytest.approx(base)


@pytest.mark.parametrize('mode,limit', [(1, .09), (2, .10), (3, .08), (4, .08)])
def test_renewed_hard_braking_is_not_smoothed_by_a_release_filter(planner_preview, mode, limit):
  planner_preview(.5, mode=mode)
  target, base, offset = planner_preview(-4., mode=mode, a_now=-3., jerk=-1.)
  assert offset > .84
  assert target == pytest.approx(base - limit)
  assert target < -3.


@pytest.mark.parametrize('a_now,jerk', [(0., 0.), (.8, .3), (-.8, .3)])
def test_zero_or_rising_plan_does_not_inherit_spurious_braking(planner_preview, a_now, jerk):
  target, base, _ = planner_preview(.5, a_now=a_now, jerk=jerk)
  assert target == pytest.approx(base)
