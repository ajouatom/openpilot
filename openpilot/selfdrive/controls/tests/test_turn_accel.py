import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.turn_accel import get_future_curvature, limit_accel_in_turns
from openpilot.selfdrive.controls.lib.cutin_predecel import apply_cutin_predecel_accel_limit
from openpilot.selfdrive.modeld.constants import ModelConstants


TIMES = np.asarray(ModelConstants.T_IDXS)


def model(curvature=0.0, speed=10.0):
  curvature = np.broadcast_to(curvature, TIMES.shape).copy()
  return SimpleNamespace(
    position=SimpleNamespace(x=TIMES * speed, y=np.zeros_like(TIMES), z=np.zeros_like(TIMES)),
    velocity=SimpleNamespace(x=np.full_like(TIMES, speed)),
    orientationRate=SimpleNamespace(z=curvature * speed),
  )


def limit(m, speed=10.0, cruise=20.0, maximum=1.6, current=0.0):
  return limit_accel_in_turns(speed, get_future_curvature(m, current), [-2.0, maximum], 3.0,
                            model_msg=m, v_cruise=cruise, current_curvature=current)


def test_straight_road_keeps_normal_launch_acceleration():
  for speed in (0., .2, 3., 10., 30.):
    assert limit(model(speed=max(speed, 3.)), speed=speed) == [-2., 1.6]


@pytest.mark.parametrize('direction', [-1, 1])
def test_accelerating_toward_curve_limits_before_one_second_point(direction):
  m = model(np.where(TIMES >= 2., direction * .015, 0.))
  assert get_future_curvature(m, 0.) == 0.
  assert limit_accel_in_turns(10., 0., [-2., 1.6], 3.) == [-2., 1.6]
  ceiling = limit(m)[1]
  assert 0.2 < ceiling < .7
  # Independent physical check at the 30 m end of the lookahead.
  assert math.hypot(ceiling, (10.**2 + 2.*ceiling*30.) * .015) <= 2.1


def test_current_curve_does_not_release_when_future_path_straightens():
  assert limit(model(), speed=15., current=.012)[1] == 0.


def test_both_halves_of_s_curve_are_considered():
  m = model(np.where(TIMES < 1.5, -.012, .018))
  ceiling = limit(m)[1]
  assert math.hypot(ceiling, (100. + 2.*ceiling*30.) * .018) <= 2.1


def test_reduced_acceleration_can_still_leave_a_tight_launch():
  # Assuming the full 1.6 m/s2 first would predict an excessive turn speed.
  m = model(.15, speed=3.)
  assert .1 < limit(m, speed=.2)[1] < 1.0


def test_cruise_target_caps_predicted_speed_but_not_below_current_speed():
  m = model(.015)
  assert limit(m, cruise=10.)[1] > limit(m, cruise=20.)[1]
  assert limit(m, cruise=5.) == limit(m, cruise=10.)


def test_preview_stops_at_three_seconds():
  assert limit(model(np.where(TIMES > 4., .5, 0.))) == [-2., 1.6]


def test_slow_vehicle_does_not_react_to_unreachable_model_curve():
  m = model(np.where(TIMES >= 2., .2, 0.), speed=30.)
  assert limit(m, speed=3.) == [-2., 1.6]


def test_stationary_model_yaw_noise_does_not_block_acceleration():
  m = model(.8, speed=.01)
  m.orientationRate.z[:] = .01
  assert limit(m, speed=.2) == pytest.approx([-2., 1.6])


@pytest.mark.parametrize('field', ['position', 'velocity', 'orientationRate'])
def test_incomplete_prediction_preserves_current_curve_limit(field):
  m = model()
  setattr(getattr(m, field), 'z' if field == 'orientationRate' else 'x', [])
  assert limit(m, speed=15., current=.012)[1] == 0.


def test_nonfinite_preview_retains_current_curve_comfort_margin():
  m = model()
  m.position.x[5] = np.nan
  assert 0.0 < limit(m, current=.015)[1] < math.sqrt(2.1**2 - 1.5**2)


def test_invalid_far_tail_does_not_discard_near_curve():
  m = model(.015)
  before = limit(m)
  m.position.x[-1] = np.nan
  assert limit(m) == before


@pytest.mark.parametrize('maximum', [0., -.5, -1.5])
def test_braking_limits_are_preserved(maximum):
  assert limit(model(.1), maximum=maximum) == [-2., maximum]


def test_extra_acceleration_preference_cannot_exceed_combined_budget():
  m = model(np.where(TIMES >= 2., .015, 0.))
  # Once the curve is reachable, increasing the driver's acceleration preference
  # must not evade the same combined-acceleration constraint.
  for maximum in (1., 1.6, 2., 3.):
    ceiling = limit(m, maximum=maximum)[1]
    assert math.hypot(ceiling, (100. + 2.*ceiling*30.) * .015) <= 2.1


def test_moderate_curve_reserves_comfort_before_combined_limit_binds():
  m = model(.006)
  ceiling = limit(m)[1]
  assert .9 < ceiling < 1.4
  # The old combined envelope alone still permits the full normal acceleration.
  assert math.hypot(1.6, 100.*.006) < 2.1


def test_recorded_launch_curve_reduces_acceleration_before_and_during_turn():
  # Anonymized model geometry from an accelerating curve: no CAN, location,
  # device identifiers, or wall-clock timestamps are needed for this regression.
  path = Path(__file__).with_name('fixtures') / 'turn_accel_launch_curve.json'
  for row in json.loads(path.read_text(encoding='utf-8')):
    m = SimpleNamespace(**{key: SimpleNamespace(**row[key]) for key in ('position', 'velocity', 'orientationRate')})
    old = limit_accel_in_turns(row['speed'], get_future_curvature(m, 0.), [-2., 1.6], 3.)[1]
    new = limit(m, speed=row['speed'], cruise=row['cruise'], current=row['current_curvature'])[1]
    assert old >= row['minimum_old']
    assert 0. <= new <= row['maximum_new']


@pytest.mark.parametrize('previous_accel', [0., 1.5])
def test_planner_passes_preview_ceiling_to_mpc_with_existing_slew_limit(previous_accel):
  # Run the actual planner update through set_accel_limits. The native solver
  # is not needed to verify that the new ceiling reaches MPC and respects its
  # initial-state continuity constraint.
  path = Path(__file__).resolve().parents[1] / 'lib' / 'longitudinal_planner.py'
  tree = ast.parse(path.read_text(encoding='utf-8'))
  cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'LongitudinalPlanner')
  update = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == 'update')
  namespace = {'np': np, 'CV': SimpleNamespace(KPH_TO_MS=1/3.6), 'V_CRUISE_MAX': 145., 'V_CRUISE_UNSET': 255.,
               'LongCtrlState': SimpleNamespace(off='off'), 'ACCEL_MAX': 2., 'A_CRUISE_MIN': -2.,
               'get_future_curvature': get_future_curvature, 'limit_accel_in_turns': limit_accel_in_turns,
               'get_cutin_predecel_accel_limit': lambda _: None,
               'apply_cutin_predecel_accel_limit': apply_cutin_predecel_accel_limit}
  exec(compile(ast.Module(body=[update], type_ignores=[]), str(path), 'exec'), namespace)
  m = model(np.where(TIMES >= 2., .015, 0.))
  sm = {'modelV2': m, 'selfdriveState': SimpleNamespace(experimentalMode=False),
        'carState': SimpleNamespace(vEgo=10., vCruise=72., vCluRatio=1., standstill=False, gasPressed=False),
        'carControl': SimpleNamespace(orientationNED=[]), 'radarState': None,
        'controlsState': SimpleNamespace(longControlState='pid', forceDecel=False, desiredCurvature=0., curvature=0.)}
  carrot = SimpleNamespace(update=lambda *args: 72., mode='acc', soft_hold_active=False,
                           get_carrot_accel=lambda _: 1.6, leadAccelResponse=5, lane_change_active=False)
  captured = []

  class StopBeforeSolver(Exception):
    pass

  def stop(*args):
    raise StopBeforeSolver

  planner = SimpleNamespace(CP=SimpleNamespace(openpilotLongitudinalControl=True),
                            mpc=SimpleNamespace(set_accel_limits=lambda *args: captured.extend(args), set_cur_state=stop),
                            a_desired=previous_accel, reset_decel_timer=0, output_should_stop=False,
                            v_desired_filter=SimpleNamespace(update=lambda value: value),
                            parse_model=lambda _: (0., 0., 0., 0., 1.), update_lead_tracks=lambda _: (0, 0))
  with pytest.raises(StopBeforeSolver):
    namespace['update'](planner, sm, carrot)
  assert captured[0] == -2.
  assert captured[1] == pytest.approx(max(limit(m)[1], previous_accel - .05))
