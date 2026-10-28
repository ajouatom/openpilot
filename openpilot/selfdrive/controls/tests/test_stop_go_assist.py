from types import SimpleNamespace

import pytest

from openpilot.selfdrive.controls.lib import stop_go_assist as sga
from openpilot.selfdrive.controls.lib.stop_go_assist import DEPART_ACCEL, departure_go, soft_stop, stop_guard

STOP = 4.3
MIN = sga.SOFT_MIN_GAP


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


def test_soft_stop_eases_the_last_metre_when_there_is_room():
  # 10-28 521 s: -1.2 at StopReq -> jolt. With room, the decel eases to -0.5 at 0.5 m/s and -1.0 at 1.25 m/s.
  assert soft_stop(-1.2, 0.5, lead(STOP + 1.0)) == pytest.approx(-0.5)
  assert soft_stop(-1.4, 1.25, lead(STOP + 2.0)) == pytest.approx(-1.0)
  # 10-28 346 s at 0.74 m/s, 2.93 m from the stopped lead (inside the 4.0 m stop distance): -1.43 -> eased, stops ~2.5 m
  assert soft_stop(-1.43, 0.74, lead(2.93)) == pytest.approx(-0.5 - 0.24 / 1.5)


@pytest.mark.parametrize(('a_target', 'v_ego', 'lead_', 'expected'), (
  (-1.5, 1.0, lead(MIN + 0.1), -1.5),                  # no room left before the minimum gap: full braking
  (-1.5, 1.0, lead(MIN + 0.4), -1.0 / (2 * 0.4)),      # little room: eases only to what still stops at the minimum gap
  (-0.3, 1.0, lead(MIN + 0.4), -0.3),                  # never strengthens (the guard does)
  (-0.4, 0.5, lead(STOP + 1.0), -0.4),                 # already gentle
  (-1.2, 2.5, lead(STOP + 3.0), -1.2),                 # above the soft range: guard's job
  (-1.2, 0.5, lead(STOP + 1.0, vLead=0.5), -1.2),      # lead still moving
  (-1.2, 0.5, lead(STOP + 1.0, status=False), -1.2),   # no lead (red light): untouched
))
def test_soft_stop_conditions(a_target, v_ego, lead_, expected):
  assert soft_stop(a_target, v_ego, lead_) == pytest.approx(expected)


@pytest.mark.parametrize('planner', ('late', 'steady'))
@pytest.mark.parametrize('gap', (2.0, 3.0, 4.5, 6.0))
def test_soft_stop_closed_loop_keeps_the_minimum_gap(planner, gap):
  # Stopped lead from 3 m/s with guard + soft stop: a planner that brakes -1.5 only in the last 2 m, or -0.8 throughout.
  v, d, dt, at_stop_req = 3.0, STOP + gap, 0.01, None
  while v > 0.05:
    a_plan = (-1.5 if d - STOP < 2.0 else -0.5) if planner == 'late' else -0.8
    a = soft_stop(stop_guard(a_plan, v, lead(d), STOP), v, lead(d))
    if v < 0.7 and at_stop_req is None:
      at_stop_req = a  # decel when StopReq starts: the car carries it into the stop
    v, d = max(0.0, v + a * dt), d - v * dt
  assert d > MIN - 0.1
  assert at_stop_req >= -0.7


def test_departure_go_toward_a_lead_pulling_away():
  # 10-28 405 s: lead creeping away at aLeadK 0.2 with the gap 3.3 m under the 4.0 m stop distance
  assert departure_go(0.0, lead(3.3, vLead=0.4, aLeadK=0.2), 0.0, True)


@pytest.mark.parametrize('kw', (
  dict(since_standstill=3.5),
  dict(v_ego=3.0),
  dict(track_stable=False),
  dict(lead_=lead(3.3, vLead=0.2, aLeadK=0.6)),    # lead not faster enough
  dict(lead_=lead(3.3, vLead=0.5, aLeadK=0.05)),   # lead not accelerating
  dict(lead_=lead(2.4, vLead=0.5, aLeadK=0.6)),    # too close even for a lead pulling away
  dict(lead_=lead(3.3, vLead=0.5, aLeadK=0.6, radar=False)),
))
def test_departure_go_conditions(kw):
  args = dict(v_ego=0.0, lead_=lead(3.3, vLead=0.5, aLeadK=0.6), since_standstill=0.5, track_stable=True) | kw
  assert not departure_go(args['v_ego'], args['lead_'], args['since_standstill'], args['track_stable'])


def planner_hook():
  # LongitudinalPlanner.apply_stop_go_assist without importing the planner (msgq)
  import ast
  from pathlib import Path
  path = Path(sga.__file__).with_name('longitudinal_planner.py')
  cls = next(n for n in ast.parse(path.read_text(encoding='utf-8')).body if isinstance(n, ast.ClassDef) and n.name == 'LongitudinalPlanner')
  fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'apply_stop_go_assist')
  namespace = {'stop_guard': stop_guard, 'soft_stop': soft_stop, 'departure_go': departure_go, 'DEPART_ACCEL': DEPART_ACCEL,
               'LEAD_ACCEL_MIN_TRACK_FRAMES': 3}
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


def test_hook_departure_go_and_pid_target():
  p = run_hook(2)
  assert p.output_a_target == 1.2 and p.a_desired == 1.2 and p.output_v_target_now == pytest.approx(1.2 * 0.05)


def test_hook_departure_overrides_the_mpc_stop_inside_the_stop_distance():
  p = run_hook(2, should_stop=True, a=0.0, lead_=lead(3.3, vLead=0.4, aLeadK=0.2))
  assert not p.output_should_stop and p.output_a_target == 1.2


@pytest.mark.parametrize('kw', (
  dict(mode=0), dict(mode=1), dict(mode=2, traffic='red'), dict(mode=2, xstate='e2eStopped'), dict(mode=2, xstate='e2eStop'),
))
def test_hook_departure_gates(kw):
  p = run_hook(should_stop=True, **kw)
  assert p.output_a_target == 0.2 and p.output_should_stop


def test_hook_stop_guard_lowers_pid_target():
  p = run_hook(1, v=0.9, a=-0.3, lead_=lead(STOP + 0.58))
  assert p.output_a_target == pytest.approx(-0.81 / 1.16)
  assert p.a_desired == p.output_a_target and p.output_v_target_now <= 0.9


def test_hook_soft_stop_raises_pid_target():
  p = run_hook(1, v=0.5, a=-1.2, lead_=lead(STOP + 1.0))
  assert p.output_a_target == pytest.approx(-0.5) and p.a_desired == pytest.approx(-0.5)


def test_hook_tracks_time_since_standstill():
  p = run_hook(2, standstill=True)
  assert p.since_standstill == 0.0


def test_hook_boost_mode_also_guards():
  assert run_hook(2, v=0.9, a=-0.3, lead_=lead(STOP + 0.58)).output_a_target == pytest.approx(-0.81 / 1.16)
