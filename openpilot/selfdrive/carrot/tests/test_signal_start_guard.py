import ast
import json
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.carrot.signal_start_guard import SignalStartGuard, publish_start_observation, SignalStartRuntime
from tools.signal_analysis.test_signal_assist_planner import inputs
from tools.signal_analysis.replay_signal_assist import planner_class, REPO


def observation(t, color='red', box=None, session='camera'):
  return dict(timestamp=t, frame_id=round(t*1000), session=session,
              tracks=[dict(box=box or [640, 250, 680, 270], raw=color)])


def step(g, t, color='red', **kwargs):
  obs = kwargs.pop('observation', observation(t, color))
  context = dict(enabled=True, valid=True, drive=True, gas=False, speed=0., departure=True)
  context.update(kwargs)
  return g.update(t+.35, obs, **context)


def held():
  g = SignalStartGuard()
  assert not step(g, 0).hold
  assert step(g, .35).hold
  return g


def test_red_blocks_once_and_never_starts_moving_braking():
  g = SignalStartGuard()
  for i in range(30):
    assert not step(g, i*.35, speed=5).hold
  g = SignalStartGuard()
  decisions = [step(g, i*.35) for i in range(20)]
  assert all(d.hold for d in decisions[1:])
  assert sum(d.warn for d in decisions) == 1


@pytest.mark.parametrize('kind', ['missing', 'unknown', 'stale', 'invalid', 'restart_green', 'other_green', 'mixed'])
def test_uncertainty_does_not_release_red(kind):
  g = held()
  for i in range(3, 12):
    t = i*.35
    obs = observation(t, 'green')
    valid = True
    if kind == 'missing': obs = None
    if kind == 'unknown': obs = observation(t, 'unknown')
    if kind == 'stale': obs['timestamp'] = t-1
    if kind == 'invalid': valid = False
    if kind == 'restart_green': obs['session'] = 'restarted'
    if kind == 'other_green': obs['tracks'][0]['box'] = [850, 250, 890, 270]
    if kind == 'mixed': obs['tracks'].append(dict(box=[740, 250, 780, 270], raw='red'))
    d = step(g, t, observation=obs, valid=valid)
    assert d.hold and not d.released and not d.warn


def test_same_housing_fresh_green_removes_veto_only_after_confirmation():
  g = held()
  assert step(g, .7, 'green').hold
  assert step(g, 1.05, 'green').hold
  d = step(g, 1.4, 'green')
  assert d.released and not d.hold


def test_repeats_future_gaps_and_malformed_do_not_confirm():
  for obs in (observation(0), observation(100), {'timestamp': float('nan')},
              observation(0, box=[0, 0, 0, 0])):
    g = SignalStartGuard()
    for i in range(5):
      assert not step(g, i*.05, observation=obs).hold
  g = SignalStartGuard()
  for i in range(5):
    assert not step(g, i).hold


def test_invalid_between_green_samples_restarts_confirmation():
  g = held()
  step(g, .7, 'green')
  step(g, 1.05, 'green', valid=False)
  assert step(g, 1.4, 'green').hold


def test_gas_override_cooldown_and_disable_park():
  g = held()
  assert not step(g, .7, gas=True).hold
  for i in range(3, 25):
    assert not step(g, i*.35).hold
  for k in ('enabled', 'drive'):
    g = held()
    assert not step(g, .7, **{k: False}).hold
  g = held()
  assert not step(g, -1., gas=True).hold


def test_warning_waits_for_departure_request():
  g = SignalStartGuard()
  step(g, 0, departure=False)
  d = step(g, .35, departure=False)
  assert d.hold and not d.warn
  assert step(g, .7).warn
  assert not step(g, 1.05).warn


def test_red_reacquires_after_settling_pitch_but_green_cannot_walk_anchor():
  g = SignalStartGuard()
  step(g, 0)
  box = [640, 260, 680, 280]
  assert not step(g, .35, observation=observation(.35, box=box)).hold
  assert step(g, .7, observation=observation(.7, box=box)).hold
  for i in range(3, 10):
    other = [640, 260+i*10, 680, 280+i*10]
    assert step(g, i*.35, observation=observation(i*.35, 'green', other)).hold


def test_publisher_opt_in_raw_only_and_neural_confidence(tmp_path):
  path = tmp_path/'obs.json'
  rec = dict(neural=True, session='a', frame_id=1, timestamp_eof=10**9,
             tracks=[dict(box=[640, 250, 680, 270], label='red', score=.9, neural_age=0., age=0)])
  publish_start_observation(rec, tmp_path, path)
  assert not path.exists()
  (tmp_path/'start_guard_enabled').write_text('1')
  for label, score, age, expected in [('red', .9, 0, 'red'), ('green', .9, 0, 'green'),
                                       ('green,left', .9, 0, 'unknown'), ('red', .7, 0, 'unknown'),
                                       ('red', float('nan'), 0, 'unknown'), ('green', float('inf'), 0, 'unknown'),
                                       ('red', .9, .1, 'unknown'), ('red,left', .9, 0, 'red')]:
    rec['tracks'][0].update(label=label, score=score, age=age)
    publish_start_observation(rec, tmp_path, path)
    assert json.loads(path.read_text())['tracks'][0]['raw'] == expected


def test_runtime_separate_endpoint_and_fresh_vehicle_context(tmp_path):
  enable, obs = tmp_path/'enabled', tmp_path/'obs'
  enable.write_text('1')
  obs.write_text(json.dumps(dict(version=1, stream='road', size=[1344, 760], **observation(1))))
  runtime = SignalStartRuntime(enable, obs)
  services = ('carState', 'modelV2', 'radarState', 'selfdriveState', 'carControl')
  class SM(dict): pass
  sm = SM(carState=NS(canValid=True, gearShifter='drive'), selfdriveState=NS(enabled=True), carControl=NS(longActive=True))
  sm.seen = sm.valid = sm.alive = dict.fromkeys(services, True)
  sm.recv_time = dict.fromkeys(services, 1.)
  record, context = runtime.read(sm, 1.1)
  assert record['frame_id'] == 1000 and context['valid'] and context['enabled'] and context['drive']
  assert not runtime.read(sm, 2)[1]['valid']


@pytest.mark.parametrize('lead', [False, True])
@pytest.mark.parametrize('mode', ['acc', 'blended'])
def test_actual_planner_red_wins_model_go_outside_lead_range_and_user_stop(lead, mode):
  cls, _ = planner_class((REPO/'openpilot/selfdrive/carrot/carrot_functions.py').read_text(encoding='utf-8'))
  p = cls(start_guard=SignalStartGuard())
  sm = inputs(lead=lead)
  sm['radarState'].leadOne.dRel = 102.
  warnings = []
  for i in range(30):
    t = i*.05
    p.user_stop_distance = 50.
    p.update(sm, 80., mode, start_observation=observation(t),
             start_context=dict(now=t, enabled=True, valid=True, drive=True))
    warnings.extend(p.events.events)
    if i >= 3:
      assert p.start_decision.hold and p.v_cruise == 0 and p.stop_dist == 0 and p.xState.name == 'e2eStopped'
      assert p.mode == 'acc'
  assert warnings.count('audioRefuse') == (0 if lead else 1)
  sm['carState'].gasPressed = True
  p.update(sm, 80., 'acc', start_observation=observation(2),
           start_context=dict(now=2, enabled=True, valid=True, drive=True))
  assert not p.start_decision.hold and p.xState.name == 'e2eCruise'


def test_actual_planner_green_does_not_override_original_stop():
  cls, _ = planner_class((REPO/'openpilot/selfdrive/carrot/carrot_functions.py').read_text(encoding='utf-8'))
  p = cls(start_guard=SignalStartGuard())
  sm = inputs(distance=3.)
  sm['modelV2'].velocity.x = [0.]*33
  for i in range(12):
    t = i*.35
    p.update(sm, 80., 'acc', start_observation=observation(t, 'red' if i < 4 else 'green'),
             start_context=dict(now=t+.35, enabled=True, valid=True, drive=True))
  assert not p.start_decision.hold and p.v_cruise == 0 and p.xState.name == 'e2eStopped'


@pytest.mark.parametrize('mode', ['acc', 'blended'])
@pytest.mark.parametrize('model_go', [False, True])
@pytest.mark.parametrize('inhibit', ['none', 'left_blinker', 'soft_hold', 'detect_only'])
def test_confirmed_transition_supplies_normal_departure_with_driver_gates(mode, model_go, inhibit):
  from openpilot.selfdrive.carrot.tests.test_signal_start_group import SMALL, RIGHT

  cls, _ = planner_class((REPO/'openpilot/selfdrive/carrot/carrot_functions.py').read_text(encoding='utf-8'))
  p = cls(start_guard=SignalStartGuard())
  sm = inputs(distance=3.)
  sm['modelV2'].velocity.x = [0.]*33
  releases = []
  for i in range(100):
    t = i*.05
    if i == 20:
      if model_go:
        sm['modelV2'].position.x = [100.*j/32 for j in range(33)]
        sm['modelV2'].velocity.x = [0.]+[15.]*32
      sm['carState'].leftBlinker = inhibit == 'left_blinker'
      sm['carState'].softHoldActive = int(inhibit == 'soft_hold')
      if inhibit == 'detect_only':
        p.trafficLightDetectMode = 1
    tracks = [(SMALL, 'red'), (RIGHT, 'red')] if i < 20 else [(RIGHT, 'green')]
    obs = dict(timestamp=t, frame_id=i, session='camera', tracks=[dict(box=b, raw=c) for b, c in tracks])
    p.update(sm, 80., mode, start_observation=obs,
             start_context=dict(now=t+.2, enabled=True, valid=True, drive=True))
    if p.start_decision.released:
      releases.append(p.start_decision.reason)
    if 4 <= i < 20:
      assert p.start_decision.hold and p.xState.name == 'e2eStopped' and p.v_cruise == 0
  assert releases == ['green_transition'] and not p.start_decision.hold
  if inhibit == 'none':
    assert p.xState.name == 'e2eCruise' and p.v_cruise > 0
  elif inhibit == 'soft_hold':
    # Soft hold is carried separately from the cruise speed setting.
    assert p.xState.name == 'e2eStopped' and p.soft_hold_active > 0
  else:
    assert p.xState.name == 'e2eStopped' and p.v_cruise == 0


@pytest.mark.parametrize('hold,accel', [(True, 1.2), (True, -1.5), (False, 1.2)])
def test_production_output_veto_after_mpc_and_coasting(hold, accel):
  path = REPO/'openpilot/selfdrive/controls/lib/longitudinal_planner.py'
  cls = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef) and n.name == 'LongitudinalPlanner')
  method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'apply_signal_start_guard')
  update = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'update')
  ns = {}
  exec(compile(ast.Module([method], type_ignores=[]), str(path), 'exec'), ns)
  p = NS(output_should_stop=False, output_v_target_now=2., output_a_target=accel)
  p.apply_signal_start_guard = lambda carrot: ns['apply_signal_start_guard'](p, carrot)
  # Execute the real last output operations, including the preceding coast call.
  p.update_coasting = lambda *a: setattr(p, 'output_a_target', accel)
  p.apply_signal_stop_prepare = lambda *a: None
  ns.update(self=p, carrot=NS(start_decision=NS(hold=hold)), sm={}, v_cruise_kph=80.,
            v_cruise=20., reset_state=False, force_slow_decel=False, coasting_turn_blocked=False, cutin_predecel_limit=None)
  exec(compile(ast.Module(update.body[-3:], type_ignores=[]), str(path), 'exec'), ns)
  assert p.output_should_stop == hold
  assert p.output_v_target_now == (0. if hold else 2.)
  assert p.output_a_target == (min(accel, 0.) if hold else accel)
