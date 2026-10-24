from types import SimpleNamespace

import pytest

from openpilot.selfdrive.controls.lib.stop_go_assist import departure_boost, stop_guard

STOP = 4.3


def lead(dRel, vLead=0.0, aLeadK=0.0, status=True, radar=True):
  return SimpleNamespace(dRel=dRel, vLead=vLead, aLeadK=aLeadK, status=status, radar=radar)


def test_stop_guard_brakes_to_stop_distance():
  # 0.9 m/s with 0.58 m left needs -0.7; the MPC only asked for -0.3 (02_logs §23)
  assert stop_guard(-0.3, 0.9, lead(STOP + 0.58), STOP) == pytest.approx(-0.9 ** 2 / (2 * 0.58))


@pytest.mark.parametrize(('a_target', 'v_ego', 'lead_', 'expected'), (
  (-0.3, 0.9, lead(STOP - 1.0), -1.5),                 # already inside the stop distance: capped
  (-2.0, 0.9, lead(STOP + 0.58), -2.0),                # never weakens stronger braking
  (0.2, 0.9, lead(STOP + 0.58), 0.2),                  # does not create braking the planner did not ask for
  (-0.3, 3.5, lead(STOP + 0.58), -0.3),                # above the low-speed range
  (-0.3, 0.05, lead(STOP + 0.58), -0.3),               # standstill
  (-0.3, 0.9, lead(STOP + 0.58, vLead=0.5), -0.3),     # lead still moving
  (-0.3, 0.9, lead(STOP + 0.58, status=False), -0.3),  # no lead
))
def test_stop_guard_conditions(a_target, v_ego, lead_, expected):
  assert stop_guard(a_target, v_ego, lead_, STOP) == pytest.approx(expected)


def test_departure_boost_raises_toward_accelerating_lead():
  assert departure_boost(0.2, 0.0, lead(3.9, vLead=0.5, aLeadK=0.6), STOP, 0.5, 1.65, True) == 1.2
  assert departure_boost(0.2, 0.0, lead(3.9, vLead=0.5, aLeadK=0.6), STOP, 0.5, 0.8, True) == 0.8  # turn/accel limit


@pytest.mark.parametrize('kw', (
  dict(since_standstill=3.5),
  dict(v_ego=3.0),
  dict(a_target=0.0),                              # planner not asking to go
  dict(track_stable=False),
  dict(lead_=lead(3.9, vLead=0.2, aLeadK=0.6)),    # lead not faster enough
  dict(lead_=lead(3.9, vLead=0.5, aLeadK=0.3)),    # lead creeping, not accelerating
  dict(lead_=lead(3.4, vLead=0.5, aLeadK=0.6)),    # too close
  dict(lead_=lead(3.9, vLead=0.5, aLeadK=0.6, radar=False)),
))
def test_departure_boost_conditions(kw):
  args = dict(a_target=0.2, v_ego=0.0, lead_=lead(3.9, vLead=0.5, aLeadK=0.6), since_standstill=0.5, track_stable=True) | kw
  out = departure_boost(args['a_target'], args['v_ego'], args['lead_'], STOP, args['since_standstill'], 1.65, args['track_stable'])
  assert out == args['a_target']


def planner_hook():
  # LongitudinalPlanner.apply_stop_go_assist without importing the planner (msgq)
  import ast
  from pathlib import Path
  from openpilot.selfdrive.controls.lib import stop_go_assist
  path = Path(stop_go_assist.__file__).with_name('longitudinal_planner.py')
  cls = next(n for n in ast.parse(path.read_text(encoding='utf-8')).body if isinstance(n, ast.ClassDef) and n.name == 'LongitudinalPlanner')
  fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'apply_stop_go_assist')
  namespace = {'stop_guard': stop_guard, 'departure_boost': departure_boost, 'LEAD_ACCEL_MIN_TRACK_FRAMES': 3}
  exec(compile(ast.Module(body=[fn], type_ignores=[]), str(path), 'exec'), namespace)
  return namespace['apply_stop_go_assist']


def run_hook(mode, xstate='lead', traffic='off', should_stop=False, v=0.0, a=0.2, lead_=None, standstill=False):
  planner = SimpleNamespace(since_standstill=0.5, dt=0.05, mpc=SimpleNamespace(mode='acc'), output_a_target=a,
                            output_should_stop=should_stop, output_v_target_now=v, a_desired=a)
  cs = SimpleNamespace(standstill=standstill, gasPressed=False, brakePressed=False, vEgo=v)
  sm = {'carState': cs, 'radarState': SimpleNamespace(leadOne=lead_ or lead(3.9, vLead=0.5, aLeadK=0.6))}
  carrot = SimpleNamespace(stopGoAssist=mode, soft_hold_active=0, lane_change_active=False, stop_distance=STOP, v_cruise=10.0,
                           xState=SimpleNamespace(name=xstate), trafficState=SimpleNamespace(name=traffic))
  planner_hook()(planner, sm, carrot, False, 1.65, (10, 0))
  return planner


def test_hook_departure_boost_and_pid_target():
  p = run_hook(2)
  assert p.output_a_target == 1.2 and p.a_desired == 1.2 and p.output_v_target_now == pytest.approx(1.2 * 0.05)


@pytest.mark.parametrize('kw', (
  dict(mode=0), dict(mode=1), dict(mode=2, traffic='red'), dict(mode=2, xstate='e2eStopped'), dict(mode=2, xstate='e2eStop'),
  dict(mode=2, should_stop=True),
))
def test_hook_departure_gates(kw):
  assert run_hook(**kw).output_a_target == 0.2


def test_hook_stop_guard_lowers_pid_target():
  p = run_hook(1, v=0.9, a=-0.3, lead_=lead(STOP + 0.58))
  assert p.output_a_target == pytest.approx(-0.81 / 1.16)
  assert p.a_desired == p.output_a_target and p.output_v_target_now <= 0.9


def test_hook_tracks_time_since_standstill():
  p = run_hook(2, standstill=True)
  assert p.since_standstill == 0.0


def test_hook_boost_mode_also_guards():
  assert run_hook(2, v=0.9, a=-0.3, lead_=lead(STOP + 0.58)).output_a_target == pytest.approx(-0.81 / 1.16)
