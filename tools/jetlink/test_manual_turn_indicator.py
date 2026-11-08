import math
from types import SimpleNamespace as N

import pytest

from blink_observation import allowed
from test_indicator_reconstruction import scene
from openpilot.selfdrive.controls.lib.desire_lib.boundary_indicator import BoundaryIndicator
from openpilot.selfdrive.controls.lib.desire_lib.indicator_observer import ModelIndicator, following_road_curve
from openpilot.selfdrive.controls.lib.desire_lib.manual_turn_indicator import ManualTurnMovement


def drive(direction=1, fault=None):
  cs, md = scene()
  detector, isolation = ModelIndicator(), BoundaryIndicator()
  sign = 1 if direction == 1 else -1
  history = []
  for i in range(65):
    now = 10 + i * .05
    md.frameId = i
    cs.vEgo = 5.
    cs.steeringPressed = i >= 25
    cs.steeringTorque = 400 * sign if cs.steeringPressed else 0.
    cs.steeringAngleDeg = min(150, max(0, i - 25) * 8) * sign
    yaw = -math.radians(12) * sign if i >= 29 else 0.
    md.laneLineProbs = [0.] * 4
    if fault == 'no_driver':
      cs.steeringPressed = False
    elif fault == 'brief_torque':
      cs.steeringPressed = i == 25
    elif fault == 'wrong_yaw':
      yaw *= -1
    elif fault == 'no_rotation':
      yaw = 0.
    elif fault == 'small_angle':
      cs.steeringAngleDeg = min(40, abs(cs.steeringAngleDeg)) * sign
    elif fault == 'moderate_turn':
      cs.steeringAngleDeg = (20 + min(22.5, abs(cs.steeringAngleDeg))) * sign
    elif fault == 'held_angle':
      cs.steeringAngleDeg = 100 * sign
    elif fault == 'park':
      cs.gearShifter = 'park'
    elif fault == 'reverse':
      cs.gearShifter = 'reverse'
    elif fault == 'stale_pose':
      yaw = None
    elif fault == 'nan':
      yaw = math.nan
    elif fault == 'stationary':
      cs.vEgo = 0.
    elif fault == 'fast':
      cs.vEgo = 61 / 3.6
    elif fault == 'stalk':
      cs.blinkerLever = 2
    elif fault == 'lamps':
      cs.rightBlinker = True
    elif fault == 'duplicate_frame':
      md.frameId = 0
    elif fault in ('slow_relabel', 'fast_relabel'):
      cs.vEgo = (8 if fault == 'slow_relabel' else 35) / 3.6
      isolation.relabel_until = now + 2.
    isolation.update(now, cs, md, enabled=False, valid=True, busy=False)
    detector.update(now, cs, md, valid=fault != 'invalid', lateral_active=False, desire_enabled=False,
                    path_enabled=False, busy=fault == 'busy', isolation=isolation,
                    manual_turn_enabled=fault != 'disabled', yaw_rate=yaw)
    if detector.active:
      history.append((now, detector.active.direction))
  return detector, isolation, cs, md, history


@pytest.mark.parametrize('direction', [1, 2])
def test_manual_turn_starts_without_atc_lanes_or_lateral_activation(direction):
  detector, isolation, cs, md, history = drive(direction)
  assert history and detector.active.source == 'manual_turn'
  assert {value[1] for value in history} == {direction}
  assert isolation.filter_lamp(direction) == 0
  assert md.meta.desireState == [1., 0., 0., 0., 0., 0., 0., 0.]
  md.meta.observedIndicator = 'left' if direction == 1 else 'right'
  md.meta.laneChangeState = 'off'
  assert allowed(cs, N(latActive=False), md, 'manual_turn', history[-1][0])


@pytest.mark.parametrize('fault', ['no_driver', 'brief_torque', 'wrong_yaw', 'no_rotation', 'small_angle',
  'held_angle', 'park', 'reverse', 'stale_pose', 'nan', 'stationary', 'fast', 'stalk', 'lamps',
  'duplicate_frame', 'invalid', 'busy', 'disabled'])
def test_counterexamples_never_request(fault):
  assert not drive(fault=fault)[4]


@pytest.mark.parametrize('direction', [1, 2])
def test_moderate_driver_turn_requests_indicator(direction):
  assert drive(direction, 'moderate_turn')[4]


def test_low_speed_lane_relabels_do_not_hide_an_independent_manual_turn():
  assert drive(fault='slow_relabel')[4]
  assert not drive(fault='fast_relabel')[4]


@pytest.mark.parametrize('last_lamp', [0., 1.75])
def test_second_turn_requires_fresh_steering_after_both_rearm_intervals(last_lamp):
  cs, md = scene()
  detector, isolation = ModelIndicator(), BoundaryIndicator()
  isolation.own(10., 1)
  cs.vEgo = 5.
  md.laneLineProbs = [0.] * 4
  ready = max(2., last_lamp + 1.)
  for i in range(90):
    t = i * .05
    md.frameId = i
    cs.leftBlinker = t < last_lamp
    cs.steeringPressed = .25 <= t <= .75 or t >= ready + .1
    cs.steeringTorque = -400. if cs.steeringPressed else 0.
    cs.steeringAngleDeg = -55. if .25 <= t <= .75 else -min(48., max(0., t-ready-.1) * 180)
    isolation.update(10.+t, cs, md, enabled=False, valid=True, busy=False)
    detector.update(10.+t, cs, md, valid=True, lateral_active=False, desire_enabled=False, path_enabled=False,
                    busy=False, isolation=isolation, manual_turn_enabled=True, yaw_rate=.22 if cs.steeringPressed else 0.)
    if detector.active:
      assert ready+.1 < t < ready+1 and detector.active.direction == 2
      break
  else:
    pytest.fail('fresh second turn did not activate')


@pytest.mark.parametrize('direction', [1, 2])
def test_normal_return_and_opposite_driver_cancellation(direction):
  sign = 1 if direction == 1 else -1
  cs, _ = scene()
  cs.steeringAngleDeg, cs.steeringTorque, cs.steeringPressed = 80 * sign, 0., False
  movement = ManualTurnMovement(10., direction)
  for i in range(1, 20):
    assert movement.update(10 + i * .05, cs, -math.radians(10) * sign)
  cs.steeringAngleDeg = 14 * sign
  for i in range(20, 25):
    assert movement.update(10 + i * .05, cs, 0.)
  assert not movement.update(11.31, cs, 0.)
  assert movement.reason == 'completed'
  movement = ManualTurnMovement(20., direction)
  cs.steeringTorque, cs.steeringPressed, cs.steeringAngleDeg = -400 * sign, True, 20 * sign
  assert movement.update(20.05, cs, -math.radians(5) * sign)
  cs.steeringAngleDeg = -1 * sign
  assert not movement.update(20.10, cs, 0.)
  assert movement.reason == 'driver_opposite'


def test_stop_keeps_hold_and_invalid_pose_releases_it():
  cs, _ = scene()
  cs.vEgo, cs.steeringAngleDeg = 0., 0.
  movement = ManualTurnMovement(10., 1)
  for i in range(1, 101):
    assert movement.update(10 + i * .05, cs, 0.)
  assert not movement.update(15.05, cs, None)
  assert movement.reason == 'inputs'


def test_recognized_road_bend_is_excluded():
  cs, md = scene()
  md.laneLineProbs = [.9] * 4
  curvature = .01
  for line in md.laneLines:
    line.y = [y + .5 * curvature * x * x for x, y in zip(line.x, line.y, strict=True)]
  assert following_road_curve(md, 5., .05)
  from openpilot.selfdrive.controls.lib.desire_lib.manual_turn_indicator import ManualTurnIndicator
  detector = ManualTurnIndicator()
  for i in range(60):
    cs.vEgo, cs.steeringPressed, cs.steeringTorque = 5., True, -400.
    cs.steeringAngleDeg = -min(150, i * 8)
    assert detector.update(10 + i * .05, cs, i, .05, allowed=True,
                           following_curve=following_road_curve(md, 5., .05)) is None
  assert detector.reason == 'road_curve'


@pytest.mark.parametrize('reason', ['disabled', 'invalid', 'opposite_lamp', 'stalk', 'pose'])
def test_active_turn_revokes_on_new_input(reason):
  detector, isolation, cs, md, history = drive()
  now = history[-1][0] + .05
  if reason == 'opposite_lamp':
    cs.rightBlinker = True
  if reason == 'stalk':
    cs.blinkerLever = 2
  detector.update(now, cs, md, valid=reason != 'invalid', lateral_active=False, desire_enabled=False,
                  path_enabled=False, busy=False, isolation=isolation,
                  manual_turn_enabled=reason != 'disabled', yaw_rate=None if reason == 'pose' else -.2)
  assert detector.active is None


@pytest.mark.parametrize('enabled', [False, True])
@pytest.mark.parametrize('atc', [0, 1, 2, 3])
def test_source_loop_uses_independent_option_with_lateral_control_off(monkeypatch, enabled, atc):
  import json
  import sys
  from contextlib import nullcontext
  import blink_source as source
  from test_blinker import Vehicle
  from openpilot.selfdrive.carrot.bluetooth import model

  clock, sent, received = [10.], [], set()
  sm = Vehicle(speed=5., engaged=False)
  sm['selfdriveState'].atcTurnStartMonoTime = 0
  sm['selfdriveState'].atcTurnDirection = 'none'
  sm['carState'].steeringTorque, sm['carState'].steeringPressed = 400., True
  sm['carState'].blinkerLever = 0
  sm['carState'].trailerConnected = False
  sm['modelV2'] = N(meta=N(desire='none', autoTurnSignal='none', autoTurnSignalLaneChange=False,
    observedIndicator='none', observedIndicatorMonoTime=0, observedIndicatorSource='manual_turn',
    indicatorIsolationMask=1, laneChangeState='off'))
  sm['carrotMan'] = N(atcType='none', xDistToTurn=0)
  sm.services = list(sm)
  sm.valid, sm.alive = dict.fromkeys(sm, True), dict.fromkeys(sm, True)

  def tick(_):
    i = len(sent)
    clock[0] = 10 + i * .05
    sm.recv_time = dict.fromkeys(sm, clock[0])
    sm['modelV2'].meta.observedIndicator = 'left' if 2 <= i < 8 else 'none'
    sm['modelV2'].meta.observedIndicatorMonoTime = int(10.1e9) if i >= 2 else 0

  def receive(_):
    i = len(sent)
    received.add(i)
    return b'' if i == 10 else json.dumps({'stamp': 10 + i * .05, 'state': 'ready'}).encode()

  sm.update = tick
  monkeypatch.setitem(sys.modules, 'openpilot.common.params', N(Params=lambda: N(
    get=lambda _: b'cp', get_int=lambda key: 1 if key == 'AutoBlinkerControl' else atc if key == 'AutoTurnControl' else 0,
    get_bool=lambda key: enabled and key == 'ManualTurnIndicator')))
  monkeypatch.setitem(sys.modules, 'openpilot.cereal', N(messaging=N(SubMaster=lambda _: sm),
    car=N(CarParams=N(from_bytes=lambda _: nullcontext(N(carFingerprint='HYUNDAI_IONIQ_5_PE'))))))
  sock = N(setblocking=lambda _: None, recv=receive, send=lambda raw: sent.append(json.loads(raw)))
  monkeypatch.setattr(source.socket, 'socket', lambda **kw: sock)
  monkeypatch.setattr(source.select, 'select', lambda *args: ([sock] if len(sent) not in received else [], [], []))
  monkeypatch.setattr(source.time, 'monotonic', lambda: clock[0])
  monkeypatch.setattr(source.time, 'sleep', lambda _: None)
  monkeypatch.setattr(model, 'read_json', lambda *args: {})
  monkeypatch.setattr(model, 'config', dict)
  monkeypatch.setattr(model, 'atomic_json', lambda *args: None)
  monkeypatch.setattr(source.Publisher, 'update', lambda *args, **kwargs: None)
  source.main(0)
  holds = [value['hold'] for value in sent if value['hold']]
  assert bool(holds) is enabled
  if enabled:
    assert {h['direction'] for h in holds} == {'left'}
    assert len({h['id'] for h in holds}) == 1
    assert all(value['hold'] is None for value in sent[8:])
