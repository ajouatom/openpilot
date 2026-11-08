import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from openpilot.cereal import car, log
from openpilot.selfdrive.controls.lib.desire_lib.tesla_turn import TeslaTurnJourney, turn_signal_context


N = 10_000_000_000


def messages():
  pose = log.LivePose.new_message(inputsOK=True, sensorsOK=True, posenetOK=True)
  pose.orientationNED.valid = True
  pose.orientationNED.z = .5
  class Messages(dict):
    pass
  sm = Messages(livePose=pose, selfdriveState=log.SelfdriveState.new_message(active=True),
                carControl=car.CarControl.new_message())
  sm.seen, sm.valid, sm.alive = [dict.fromkeys(sm, True) for _ in range(3)]
  sm.recv_time = dict.fromkeys(sm, 10.)
  sm.logMonoTime = dict.fromkeys(sm, N)
  return sm


@pytest.mark.parametrize("invalid", [
  "", "unseen", "stale_arrival", "stale_source", "future_source", "invalid_message", "dead",
  "inputs", "sensors", "posenet", "orientation", "nan", "disabled", "fault", "inactive", "stale_permission",
])
def test_native_pose_and_permission_require_original_fresh_valid_samples(invalid):
  sm = messages()
  state = car.CarState.new_message(latEnabled=True)
  if invalid == "unseen":
    sm.seen['livePose'] = False
  elif invalid == "stale_arrival":
    sm.recv_time['livePose'] -= .4
  elif invalid == "stale_source":
    sm.logMonoTime['livePose'] -= 400_000_000
  elif invalid == "future_source":
    sm.logMonoTime['livePose'] += 1
  elif invalid == "invalid_message":
    sm.valid['livePose'] = False
  elif invalid == "dead":
    sm.alive['livePose'] = False
  elif invalid in ("inputs", "sensors", "posenet"):
    setattr(sm['livePose'], invalid + "OK", False)
  elif invalid == "orientation":
    sm['livePose'].orientationNED.valid = False
  elif invalid == "nan":
    sm['livePose'].orientationNED.z = float("nan")
  elif invalid == "disabled":
    state.latEnabled = False
  elif invalid == "fault":
    state.steerFaultTemporary = True
  elif invalid == "inactive":
    sm['selfdriveState'].active = False
  elif invalid == "stale_permission":
    sm.recv_time['selfdriveState'] -= .4
  allowed, pose = turn_signal_context(sm, state, 10., N, False)
  assert allowed == (not invalid)
  if invalid in ("", "disabled", "fault", "inactive", "stale_permission"):
    assert pose == (N, .5)
  else:
    assert pose is None


def test_always_lateral_is_permission_not_a_fault_or_freshness_override():
  sm = messages()
  sm['selfdriveState'].active = False
  state = car.CarState.new_message(latEnabled=True)
  assert turn_signal_context(sm, state, 10., N, True)[0]
  state.latEnabled = False
  assert not turn_signal_context(sm, state, 10., N, True)[0]
  state.latEnabled = True
  sm.valid['carControl'] = False
  assert not turn_signal_context(sm, state, 10., N, True)[0]


def advance(journey, tick, yaw=0., angle=90., speed=5., nav="turn left", distance=20, desire=1, **overrides):
  state = NS(vEgo=speed, steeringAngleDeg=angle, steeringPressed=False, steeringTorque=0.)
  state.__dict__.update(overrides)
  return journey.update(10. + tick * .05, state, (N + tick * 50_000_000, yaw), nav, distance, desire, True, True)


@pytest.mark.parametrize("fault", ["gap", "backwards", "jump", "nan_angle", "nan_speed"])
def test_pose_discontinuity_never_finishes_a_previously_entered_turn(fault):
  journey = TeslaTurnJourney()
  for i in range(12):
    assert advance(journey, i, yaw=-i * .05) == 1
  assert journey.entered
  tick, yaw, angle, speed = 12, -.55, 10., 5.
  if fault == "gap":
    tick = 30
  elif fault == "backwards":
    tick = 3
  elif fault == "jump":
    yaw = 2.
  elif fault == "nan_angle":
    angle = float("nan")
  else:
    speed = float("nan")
  assert advance(journey, tick, yaw=yaw, angle=angle, speed=speed) == 1
  assert not journey.entered


def test_heading_wrap_and_stopped_drift_do_not_fabricate_turn_rotation():
  journey = TeslaTurnJourney()
  advance(journey, 0, yaw=-3.1)
  advance(journey, 1, yaw=3.1)
  assert journey.rotation == pytest.approx(2 * math.pi - 6.2)
  for i in range(2, 15):
    advance(journey, i, yaw=3.1 - (i - 1) * .05, speed=0., angle=0.)
  assert not journey.entered
  assert journey.rotation < math.radians(20)


@pytest.mark.parametrize("driver_abort", [False, True])
def test_opposite_torque_needs_wheel_past_zero_and_manual_abort_never_replays(driver_abort):
  journey = TeslaTurnJourney()
  for i in range(5):
    advance(journey, i, yaw=-i * .05)
  assert advance(journey, 5, yaw=-.2, angle=20., steeringPressed=True, steeringTorque=-1.) == 1
  result = advance(journey, 6, yaw=-.2, angle=-.1 if driver_abort else 0., steeringPressed=True, steeringTorque=-1.)
  assert result == (0 if driver_abort else 1)
  if driver_abort:
    assert advance(journey, 7, yaw=-.2) == 0


@pytest.mark.parametrize("same_side", [False, True])
@pytest.mark.parametrize("delay", [1, 80])
def test_following_turn_is_bounded_and_only_handed_over_after_completion(same_side, delay):
  journey = TeslaTurnJourney()
  for i in range(12):
    advance(journey, i, yaw=-i * .05)
  direction = 1 if same_side else 2
  nav = "turn left" if same_side else "turn right"
  assert advance(journey, 12, yaw=-.55, nav=nav, distance=100, desire=direction) == 1
  for tick in range(13, 13 + delay):
    assert advance(journey, tick, yaw=-.55, nav=nav, distance=100, desire=direction) == 1
  assert advance(journey, 13 + delay, yaw=-.55, angle=0., nav=nav, distance=100, desire=direction) == (direction if delay == 1 else 0)
  assert not journey.entered
  assert advance(journey, 14 + delay, yaw=-.55, angle=0., nav=nav, distance=100, desire=direction) == (direction if delay == 1 else 0)


def test_journey_timeout_does_not_rearm_same_turn():
  journey = TeslaTurnJourney()
  assert advance(journey, 0) == 1
  assert advance(journey, 1200) == 0
  assert advance(journey, 1201, distance=100) == 0


def test_modeld_passes_validated_pose_and_hold_permission_to_actual_desire_update():
  path = Path(__file__).parents[2] / "modeld/modeld.py"
  tree = ast.parse(path.read_text(encoding='utf8'))
  init = next(node for node in ast.walk(tree) if isinstance(node, ast.Assign) and
              ast.unparse(node.targets[0]) == "(turn_hold_allowed, turn_pose)" and isinstance(node.value, ast.Tuple))
  call = next(node for node in ast.walk(tree) if isinstance(node, ast.Expr) and
              isinstance(node.value, ast.Call) and ast.unparse(node.value.func) == "DH.update")
  sm = messages()
  sm['carState'] = car.CarState.new_message(latEnabled=True)
  sm['livePose'].angularVelocityDevice.valid = True
  sm['livePose'].angularVelocityDevice.z = -.2
  sm['carrotMan'], sm['radarState'] = None, None
  sm.valid['radarState'] = sm.alive['radarState'] = True
  sm.recv_time['radarState'] = 10.
  received = []
  namespace = {"DH": NS(tesla_auto_signals=True, update=lambda *args, **kwargs: received.append(kwargs)), "sm": sm,
               "indicator_now": 10., "indicator_valid": True, "turn_signal_context": turn_signal_context,
               "params": NS(get_bool=lambda name: False), "time": NS(clock_gettime_ns=lambda clock: N, CLOCK_MONOTONIC=1),
               "modelv2_send": NS(modelV2=log.ModelDataV2.new_message()), "lane_change_prob": .5}
  block = next(node.body for node in ast.walk(tree) if isinstance(getattr(node, 'body', None), list) and init in node.body)
  code = compile(ast.Module(body=block[block.index(init):block.index(call) + 1], type_ignores=[]), str(path), "exec")
  exec(code, namespace)
  assert received[-1]["turn_hold_allowed"]
  assert received[-1]["turn_pose"] == (N, .5)
  assert received[-1]["assistant_valid"]
  assert received[-1]['indicator_yaw_rate'] == pytest.approx(-.2)
  sm.recv_time['radarState'] -= .4
  exec(code, namespace)
  assert not received[-1]["assistant_valid"]
  assert received[-1]["turn_hold_allowed"] and received[-1]["turn_pose"] == (N, .5)
  sm.logMonoTime['livePose'] -= 400_000_000
  exec(code, namespace)
  assert not received[-1]["turn_hold_allowed"] and received[-1]["turn_pose"] is None
  assert received[-1]['indicator_yaw_rate'] is None
  sm.logMonoTime['livePose'] = N
  sm.recv_time['livePose'] -= .3
  exec(code, namespace)
  assert received[-1]['indicator_yaw_rate'] is None
