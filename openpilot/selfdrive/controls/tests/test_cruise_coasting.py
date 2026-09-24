import ast
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.common.pid import PIDController
from openpilot.selfdrive.controls.lib.cruise_coasting import (
  BRAKE_RELEASE_JERK, CruiseCoastingPlan, coasting_percent, coasting_relief, no_coasting_lead,
)
# This existing fixture also supplies Windows-only Params/hardware import shims.
from openpilot.selfdrive.controls.tests.test_longcontrol_hyundai_tuning import DictParams, make_cp
import openpilot.selfdrive.controls.lib.longcontrol as lc


@pytest.mark.parametrize('value, expected', [(0, 0), (5, 5), (10, 10), (30, 10), (-2, 0),
                                            (None, 0), ('bad', 0), (float('nan'), 0), (float('inf'), 0)])
def test_setting_bounds(value, expected):
  assert coasting_percent(value) == expected


def test_margin_has_continuous_lower_and_upper_edges():
  target = 100 / 3.6
  for percent in (1, 5, 10):
    width = target * percent / 100
    assert coasting_relief(target, target, percent) == 0
    assert coasting_relief(target + .3 * width, target, percent) == pytest.approx(1)
    assert coasting_relief(target + .8 * width, target, percent) == pytest.approx(.5)
    assert coasting_relief(target + width, target, percent) == pytest.approx(0, abs=1e-12)
    assert coasting_relief(target + 2 * width, target, percent) == 0
    for edge in (0., .1, .6, 1.):
      below = coasting_relief(target + (edge - 1e-6) * width, target, percent)
      above = coasting_relief(target + (edge + 1e-6) * width, target, percent)
      assert abs(above - below) < 1e-7


def test_plan_requires_stable_target_and_cap_clearance():
  plan = CruiseCoastingPlan()
  args = {'enabled': True, 'percent': 5, 'set_speed': 100., 'target': 100 / 3.6, 'external_limit': 250 / 3.6, 'dt': .05}
  assert plan.update(**args) == 0
  for _ in range(19):
    assert plan.update(**args) == 0
  assert plan.update(**args) == args['target']
  # Reference stays fixed despite speed-calibration noise; it cannot ratchet.
  assert plan.update(**{**args, 'target': args['target'] + .01}) == args['target']
  assert plan.update(**{**args, 'set_speed': 99.}) == 0
  for _ in range(21):
    result = plan.update(**args)
  assert result > 0
  # A cap within the band vetoes relief before it lowers the ordinary target.
  assert plan.update(**{**args, 'external_limit': 103 / 3.6}) == 0
  assert plan.update(**args) == 0
  for value in (0., float('nan')):
    assert plan.update(**{**args, 'target': value}) == 0
  assert plan.update(**{**args, 'percent': 0}) == 0


def inputs():
  cs = SimpleNamespace(vEgo=102 / 3.6, aEgo=.1, vCruise=100., vCluRatio=1.,
                       softHoldActive=0, brakePressed=False, gasPressed=False, carrotCruise=0,
                       standstill=False, cruiseState=SimpleNamespace(standstill=False))
  plan = SimpleNamespace(aTarget=-.2, vTargetNow=100 / 3.6, jTargetNow=0., shouldStop=False,
                         longitudinalPlanSource='cruise', fcw=False, cruiseTarget=100.,
                         cruiseCoastingTarget=100 / 3.6, cruiseCoastingPercent=5)
  radar = SimpleNamespace(**{key: SimpleNamespace(status=False) for key in ('leadOne', 'leadTwo', 'leadCutInRisk')})
  return cs, plan, radar


def test_live_speed_ratio_does_not_restart_entry_or_move_reference():
  plan = CruiseCoastingPlan()
  set_speed = 90.
  reference = set_speed / 3.6 * .968
  # Cluster quantization and ego-speed noise change the ratio even at a fixed
  # set speed. These changes exceed the old 0.02 m/s restart threshold.
  for frame in range(201):
    ratio = .968 + .006 * np.sin(frame * .9)
    result = plan.update(enabled=True, percent=10, set_speed=set_speed,
                         target=set_speed / 3.6 * ratio, external_limit=200 / 3.6 * ratio, dt=.05)
    assert result == (0 if frame < 20 else reference)
    assert plan.target == reference
  # With the reference frozen, actual overspeed enters the relief band.
  assert coasting_relief(reference * 1.03, result, 10) == pytest.approx(1)


@pytest.mark.parametrize('ratio', [.94, 1.02])
def test_external_caps_protect_frozen_and_current_speed_bands(ratio):
  plan = CruiseCoastingPlan()
  args = {'enabled': True, 'percent': 5, 'set_speed': 100., 'target': 100 / 3.6,
          'external_limit': 250 / 3.6, 'dt': .05}
  for _ in range(21):
    reference = plan.update(**args)
  current = args['target'] * ratio
  # Place the cap between the two ceilings: checking just one is insufficient.
  cap = (current + reference) / 2 * 1.05
  assert plan.update(**{**args, 'target': current, 'external_limit': cap}) == 0
  assert plan.target == 0


@pytest.mark.parametrize('change', ['set_speed', 'percent', 'veto'])
def test_real_changes_restart_entry_and_capture_a_new_reference(change):
  plan = CruiseCoastingPlan()
  args = {'enabled': True, 'percent': 5, 'set_speed': 100., 'target': 100 / 3.6,
          'external_limit': 250 / 3.6, 'dt': .05}
  for _ in range(21):
    assert plan.update(**args) >= 0
  assert plan.stable_time == 1
  args['target'] = 98 / 3.6
  if change == 'veto':
    assert plan.update(**{**args, 'enabled': False}) == 0
  elif change == 'set_speed':
    args['set_speed'] = 99.
  else:
    args['percent'] = 6
  for frame in range(21):
    assert plan.update(**args) == (0 if frame < 20 else args['target'])


@pytest.fixture
def control(monkeypatch):
  monkeypatch.setattr(lc, 'Params', lambda: DictParams({'StoppingAccel': -50, 'LongTuningKpV': 100,
                                                     'LongTuningKiV': 200, 'LongTuningKf': 100}))
  def create(brand='hyundai'):
    cp = make_cp(brand)
    cp.longitudinalTuning = SimpleNamespace(kpBP=[0.], kpV=[1.], kiBP=[0.], kiV=[.2], kf=1.)
    c = lc.LongControl(cp)
    c.long_control_state = lc.LongCtrlState.pid
    return c
  return create


@pytest.mark.parametrize('brand', ['hyundai', 'gm', 'toyota'])
def test_zero_percent_matches_unmodified_pid_over_time(control, brand):
  c = control(brand)
  kp, ki = (1., 0.) if brand == 'hyundai' else (1., .2)
  legacy = PIDController(kp, ki, k_f=1., neg_limit=-3.5, pos_limit=2., rate=100)
  cs, plan, radar = inputs()
  plan.cruiseCoastingPercent = 0
  # Includes overspeed, acceleration, braking, integral history and refreshes.
  for frame in range(600):
    cs.vEgo = 27 + np.sin(frame / 30)
    cs.aEgo = np.cos(frame / 30) * .3
    plan.aTarget = np.sin(frame / 45) * .6
    error = plan.aTarget - cs.aEgo if brand == 'toyota' else plan.vTargetNow - cs.vEgo
    expected = legacy.update(error, speed=cs.vEgo, feedforward=plan.aTarget)
    actual, ff, jerk = c.update(True, cs, plan, (-3.5, 2.), .01, radar)
    assert actual == expected
    assert c.pid.i == legacy.i
    assert (ff, jerk) == (plan.aTarget, plan.jTargetNow)


def test_brake_release_is_gradual_and_never_positive(control):
  c = control()
  cs, plan, radar = inputs()
  baseline = plan.vTargetNow - cs.vEgo + plan.aTarget
  previous = baseline
  for _ in range(300):
    out, _, _ = c.update(True, cs, plan, (-3.5, 2.), .01, radar)
    assert 0 <= out - previous <= BRAKE_RELEASE_JERK * .01 + 1e-12
    assert baseline <= out <= 0
    previous = out
  assert out == 0
  # No delayed brake relief remains beyond the upper edge.
  cs.vEgo = 105 / 3.6
  out, _, _ = c.update(True, cs, plan, (-3.5, 2.), .01, radar)
  assert out == pytest.approx(plan.vTargetNow - cs.vEgo + plan.aTarget)


def test_no_integral_windup_and_no_extra_positive_acceleration(control):
  c = control('gm')
  cs, plan, radar = inputs()
  for _ in range(300):
    c.update(True, cs, plan, (-3.5, 2.), .01, radar)
  assert c.pid.i == 0
  # A positive command must retain the ordinary PID integration exactly.
  plan.aTarget = 1.5
  previous_i = c.pid.i
  out, _, _ = c.update(True, cs, plan, (-3.5, 2.), .01, radar)
  expected_i = previous_i + (plan.vTargetNow - cs.vEgo) * .2 * .01
  assert c.pid.i == pytest.approx(expected_i)
  assert out == pytest.approx(plan.vTargetNow - cs.vEgo + 1.5 + expected_i)
  assert c.coasting.correction == 0


@pytest.mark.parametrize('veto', ['leadOne', 'leadTwo', 'leadCutInRisk', 'source', 'fcw', 'gas', 'brake',
                                  'carrot', 'stale', 'target_change', 'disabled', 'negative_limit'])
def test_new_veto_restores_braking_without_coasting_delay(control, veto):
  c = control()
  cs, plan, radar = inputs()
  for _ in range(200):
    c.update(True, cs, plan, (-3.5, 2.), .01, radar)
  assert c.last_output_accel == 0
  age, limits = .01, (-3.5, 2.)
  if veto.startswith('lead'):
    getattr(radar, veto).status = True
  elif veto == 'source':
    plan.longitudinalPlanSource = 'lead0'
  elif veto == 'fcw':
    plan.fcw = True
  elif veto in ('gas', 'brake'):
    setattr(cs, veto + 'Pressed', True)
  elif veto == 'carrot':
    cs.carrotCruise = 1
  elif veto == 'stale':
    age = .201
  elif veto == 'target_change':
    cs.vCruise = 99
  elif veto == 'disabled':
    plan.cruiseCoastingPercent = 0
  elif veto == 'negative_limit':
    limits = (-3.5, -.1)
  plan.aTarget = -1.0
  out, _, _ = c.update(True, cs, plan, limits, age, radar)
  assert out == pytest.approx(plan.vTargetNow - cs.vEgo + plan.aTarget)
  assert c.coasting.correction == 0


def test_missing_optional_metadata_and_invalid_inputs_do_not_relax_braking(control):
  c = control()
  cs, plan, radar = inputs()
  del plan.cruiseCoastingTarget
  out, _, _ = c.update(True, cs, plan, (-3.5, 2.), .01, radar)
  assert out < 0
  for value in (float('nan'), float('inf'), -1.):
    assert coasting_relief(value, 100 / 3.6, 5) == 0
  assert not no_coasting_lead(SimpleNamespace())


@pytest.fixture
def planner_gate():
  # Execute the production integration method; only native MPC imports are skipped.
  path = Path(__file__).resolve().parents[1] / 'lib/longitudinal_planner.py'
  cls = next(n for n in ast.parse(path.read_text(encoding='utf-8')).body
             if isinstance(n, ast.ClassDef) and n.name == 'LongitudinalPlanner')
  method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'update_coasting')
  ns = {'CV': SimpleNamespace(KPH_TO_MS=1 / 3.6), 'get_carrot_man': lambda sm: sm.nav,
        'no_coasting_lead': no_coasting_lead}
  exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), ns)
  state = SimpleNamespace(coasting_percent=5, coasting=CruiseCoastingPlan(), dt=.05,
                          CP=SimpleNamespace(openpilotLongitudinalControl=True), reset_decel_timer=0,
                          mpc=SimpleNamespace(mode='acc', source='cruise', solution_status=0),
                          output_should_stop=False, fcw=False, v_cruise_kph=100.)
  cs, _, radar = inputs()
  class SM(dict):
    seen = {'carrotMan': True}
    nav = SimpleNamespace(desiredSpeed=250)
    valid = True
    def all_checks(self, service_list):
      return self.valid
  sm = SM(carState=cs, radarState=radar)
  carrot = SimpleNamespace(soft_hold_active=0, atc_active=False, lane_change_active=False,
                           xState=SimpleNamespace(name='e2eCruise'), eco_target_speed=0, v_cruise=100 / 3.6)
  args = {'v_cruise_kph': 100., 'v_cruise': 100 / 3.6, 'reset_state': False, 'force_slow_decel': False,
          'coasting_turn_blocked': False, 'cutin_predecel_limit': None}
  def step():
    ns['update_coasting'](state, sm, carrot, **args)
    return state.coasting_target
  return step, state, sm, carrot, args


@pytest.mark.parametrize('veto', ['lead', 'nav', 'stale_nav', 'invalid', 'turn', 'stop', 'source',
                                  'blended', 'solver', 'eco', 'atc', 'lane', 'cutin', 'force', 'reset'])
def test_planner_permission_vetoes_even_if_source_was_cruise(planner_gate, veto):
  step, state, sm, carrot, args = planner_gate
  for _ in range(21):
    value = step()
  assert value == pytest.approx(100 / 3.6)
  if veto == 'lead':
    sm['radarState'].leadTwo.status = True
  elif veto == 'nav':
    sm.nav.desiredSpeed = 103
  elif veto == 'stale_nav':
    sm.nav = None
  elif veto == 'invalid':
    sm.valid = False
  elif veto == 'turn':
    args['coasting_turn_blocked'] = True
  elif veto == 'stop':
    carrot.xState.name = 'e2eStop'
  elif veto == 'source':
    state.mpc.source = 'lead1'
  elif veto == 'blended':
    state.mpc.mode = 'blended'
  elif veto == 'solver':
    state.mpc.solution_status = 1
  elif veto == 'eco':
    carrot.eco_target_speed = 100
  elif veto == 'atc':
    carrot.atc_active = True
  elif veto == 'lane':
    carrot.lane_change_active = True
  elif veto == 'cutin':
    args['cutin_predecel_limit'] = -.5
  elif veto == 'force':
    args['force_slow_decel'] = True
  elif veto == 'reset':
    args['reset_state'] = True
  assert step() == 0


def test_planner_and_controller_coast_with_a_changing_live_ratio(planner_gate, control):
  step, state, sm, carrot, args = planner_gate
  c = control()
  cs, plan, radar = inputs()
  initial_ratio = .968
  reference = args['v_cruise_kph'] / 3.6 * initial_ratio
  for frame in range(100):
    ratio = initial_ratio + .006 * np.sin(frame * .9)
    sm['carState'].vCluRatio = ratio
    carrot.v_cruise = args['v_cruise'] = args['v_cruise_kph'] / 3.6 * ratio
    plan.cruiseCoastingTarget = step()
    assert plan.cruiseCoastingTarget == (0 if frame < 20 else reference)
    cs.vEgo = reference * 1.02
    plan.vTargetNow = reference
    for _ in range(5):
      c.update(True, cs, plan, (-3.5, 2.), .01, radar)
  assert c.last_output_accel == 0
  # A camera cap entering the frozen band cancels relief on the next plan.
  sm.nav.desiredSpeed = 103
  plan.cruiseCoastingTarget = step()
  assert plan.cruiseCoastingTarget == 0
  output, _, _ = c.update(True, cs, plan, (-3.5, 2.), .01, radar)
  assert output == pytest.approx(plan.vTargetNow - cs.vEgo + plan.aTarget)


@pytest.mark.parametrize('source', ['cam', 'bump', 'hda', 'hda_bump', 'section', 'hda_section', 'school', 'atc', 'vturn', 'route'])
@pytest.mark.parametrize('cap', [80., 100., 103., 105.])
def test_camera_bump_and_external_caps_restore_unmodified_braking(planner_gate, control, source, cap):
  step, state, sm, _, _ = planner_gate
  cs, plan, radar = inputs()
  c = control()
  for _ in range(21):
    plan.cruiseCoastingTarget = step()
  for _ in range(200):
    c.update(True, cs, plan, (-3.5, 2.), .01, radar)
  assert c.last_output_accel == 0
  # External caps are joined into cruise; the source label need not change.
  assert state.mpc.source == 'cruise'
  sm.nav = SimpleNamespace(desiredSpeed=cap, desiredSource=source)
  plan.cruiseCoastingTarget = step()
  assert plan.cruiseCoastingTarget == 0
  plan.aTarget = -1.2
  out, _, _ = c.update(True, cs, plan, (-3.5, 2.), .01, radar)
  assert out == pytest.approx(plan.vTargetNow - cs.vEgo + plan.aTarget)
  assert c.coasting.correction == 0


@pytest.mark.parametrize('stop_state', ['e2eStop', 'e2eStopped', 'e2ePrepare'])
def test_signal_stop_restores_braking_before_should_stop_flag(planner_gate, control, stop_state):
  step, state, _, carrot, _ = planner_gate
  cs, plan, radar = inputs()
  c = control()
  for _ in range(21):
    plan.cruiseCoastingTarget = step()
  for _ in range(200):
    c.update(True, cs, plan, (-3.5, 2.), .01, radar)
  assert c.last_output_accel == 0
  # Distant red-light deceleration can still have cruise source and shouldStop=False.
  carrot.xState.name = stop_state
  assert not state.output_should_stop and state.mpc.source == 'cruise'
  plan.cruiseCoastingTarget = step()
  assert plan.cruiseCoastingTarget == 0
  plan.aTarget = -1.0
  out, _, _ = c.update(True, cs, plan, (-3.5, 2.), .01, radar)
  assert out == pytest.approx(plan.vTargetNow - cs.vEgo + plan.aTarget)


def test_wire_metadata_defaults_and_roundtrip(control):
  from openpilot.cereal import log
  plan = log.LongitudinalPlan.new_message()
  assert plan.cruiseCoastingTarget == 0 and plan.cruiseCoastingPercent == 0
  plan.cruiseCoastingTarget = 100 / 3.6
  plan.cruiseCoastingPercent = 5
  with log.LongitudinalPlan.from_bytes(plan.to_bytes()) as decoded:
    assert decoded.cruiseCoastingTarget == pytest.approx(100 / 3.6)
    assert decoded.cruiseCoastingPercent == 5


def test_zero_setting_skips_planner_coasting_inputs(planner_gate):
  step, state, sm, _, _ = planner_gate
  state.coasting_percent = 0
  sm.clear()
  assert step() == 0


def test_catalog_and_persistent_default_agree():
  root = Path(__file__).resolve().parents[4]
  catalog = json.loads((root / 'openpilot/selfdrive/carrot_settings.json').read_text(encoding='utf-8'))
  def objects(value):
    if isinstance(value, dict):
      yield value
      for child in value.values():
        yield from objects(child)
    elif isinstance(value, list):
      for child in value:
        yield from objects(child)
  entries = list(objects(catalog))
  setting = next(x for x in entries if x.get('name') == 'CruiseCoastingPercent')
  assert (setting['default'], setting['min'], setting['max'], setting['unit'], setting['display_unit']) == (0, 0, 10, 1, 'percent')
  assert 'CruiseCoastingPercent' in next(x for x in entries if x.get('id') == 'CRUISE_CARROT')['params']
  assert '{"CruiseCoastingPercent", {PERSISTENT, INT, "0"}}' in (root / 'openpilot/common/params_keys.h').read_text(encoding='utf-8')
