import pytest

from openpilot.cereal import car, log
from openpilot.selfdrive.car.tesla_lane_signals import TeslaLaneSignals
from openpilot.selfdrive.car.tesla_turn_signals import TeslaTurnSignals
from openpilot.selfdrive.car.tests.test_tesla_turn_signals import N, echo, feedback, intent
from openpilot.selfdrive.carrot.xiaoge.xiaoge_vision import XiaogeVisionResult
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper


def vision(stamp, left=0, right=0, valid=True):
  return XiaogeVisionResult(left, right, valid, stamp, False, False, False, 0)


def context(direction=1, distance=1.8, frame=1, torque=True):
  cs = car.CarState.new_message(canValid=True, gearShifter="drive", turnSignalDriverValid=True,
                               vEgo=15., steeringPressed=torque, steeringTorque=(1. if direction == 1 else -1.) if torque else 0.)
  cc = car.CarControl.new_message()
  md = log.ModelDataV2.new_message(frameId=frame)
  left, right = (-distance, 3.6-distance) if direction == 1 else (distance-3.6, distance)
  md.laneLineProbs = [.9] * 4
  md.laneLines = [{"x": list(range(33)), "y": [y] * 33} for y in (left-3.6, left, right, right+3.6)]
  md.roadEdges = [{"y": [left-4.1] * 33}, {"y": [right+4.1] * 33}]
  md.roadEdgeStds = [.1, .1]
  md.meta.desireState = [1., 0., 0., 0., 0.]
  return cs, cc, md


def drive(observer, direction=1, *, mutate=None, until=48):
  result = None
  for i in range(until):
    cs, cc, md = context(direction, max(.6, 1.8-max(0, i-22)*.055), i+1)
    now = N + i * 50_000_000
    v = vision(N + i//8 * 400_000_000)
    if mutate:
      v = mutate(i, cs, cc, md, v, now)
    result = observer.update(cs, cc, md, v, now, allowed=True)
    if result:
      return result, cs, cc, md, v, now
  return None, cs, cc, md, v, now


@pytest.mark.parametrize("direction", [1, 2])
def test_manual_dashed_400ms_cadence_drives_existing_executor_without_steering(direction):
  observer = TeslaLaneSignals(manual=True)
  request, cs, cc, md, v, now = drive(observer, direction)
  assert request and request[0] == direction and request[1] > 1 << 31
  assert not cc.latActive and not cc.enabled and str(md.meta.desire) == "none"
  signals = TeslaTurnSignals(True)
  feedback(signals, now)
  sends = signals.update(cs, cc, md.meta, True, now, lamp_intent=request)
  assert len(sends) == 1 and sends[0].dat[1] & 3 == direction
  assert signals.session.lamp_only and not signals.session.turn
  echo(signals, sends[0], now+1)
  feedback(signals, now+2, ui=(2, 0) if direction == 1 else (0, 2),
           front=(1, 0) if direction == 1 else (0, 1))
  cs.leftBlinker, cs.rightBlinker = direction == 1, direction == 2
  # Genuine-stalk filtering is the production steering isolation, not a fake desire.
  helper = DesireHelper(tesla_auto_signals=True)
  assert helper._update_driver_blinker(cs)[0] == 0
  assert helper._update_driver_blinker(cs)[0] == 0
  assert observer.update(cs, cc, md, v, now+10_000_000, allowed=True) == request
  # Even a repeated model wakeup must revoke stale evidence immediately.
  assert observer.update(cs, cc, md, vision(now-900_000_000), now+20_000_000, allowed=True) is None
  feedback(signals, now+21_000_000, ui=(2, 0) if direction == 1 else (0, 2),
           front=(1, 0) if direction == 1 else (0, 1))
  cancelled = signals.update(cs, cc, md.meta, True, now+22_000_000)
  assert signals.session.cancelling and cancelled[0].dat[1] & 3 == 3


@pytest.mark.parametrize("direction", [1, 2])
@pytest.mark.parametrize("gate", ["solid", "unknown", "invalid", "stale", "future", "repeat", "lane", "road",
                                "confidence", "blindspot", "curve", "gentle", "driver", "trailer", "fault", "nan"])
def test_manual_admission_rejects_unsupported_evidence(direction, gate):
  def mutate(i, cs, cc, md, v, now):
    if gate in ("solid", "unknown"):
      v = vision(v.lane_received_nanos, **{("left" if direction == 1 else "right"): 1 if gate == "solid" else -1})
    elif gate == "invalid":
      v = vision(v.lane_received_nanos, valid=False)
    elif gate in ("stale", "future", "repeat"):
      v = vision(now-900_000_000 if gate == "stale" else now+1 if gate == "future" else N)
    elif gate == "lane":
      md.laneLines[0 if direction == 1 else 3].y = [0.] * 33
    elif gate == "road":
      md.roadEdges[direction-1].y = [0.] * 33
    elif gate == "confidence":
      md.laneLineProbs = [.1] * 4
    elif gate == "blindspot":
      cs.leftBlindspot = cs.rightBlindspot = True
    elif gate == "curve":
      cs.steeringPressed = False
    elif gate == "gentle":
      md.laneLines[1].y = [-1.8] * 33
      md.laneLines[2].y = [1.8] * 33
    elif gate == "driver":
      cs.turnSignalDriverDirection = direction
    elif gate == "trailer":
      cs.trailerConnected = True
    elif gate == "fault":
      cs.steerFaultTemporary = True
    elif gate == "nan":
      cs.steeringTorque = float("nan")
    return v
  assert drive(TeslaLaneSignals(manual=True), direction, mutate=mutate)[0] is None


def test_disabled_and_stalkless_do_not_gain_support():
  assert drive(TeslaLaneSignals())[0] is None
  assert drive(TeslaLaneSignals(manual=True), mutate=lambda i, cs, cc, md, v, n:
               (setattr(cs, "turnSignalDriverValid", False) or v))[0] is None


@pytest.mark.parametrize("failure", ["driver_counter", "hazards", "context", "model_reset", "gap", "return", "timeout"])
def test_active_lifecycle_revokes_without_relabelling_or_replay(failure):
  observer = TeslaLaneSignals(manual=True)
  request, cs, cc, md, v, now = drive(observer)
  assert request
  md.frameId += 1
  later = now + 50_000_000
  if failure == "driver_counter":
    cs.leftBlinkerStalkCount += 1
  elif failure == "hazards":
    cs.leftBlinker = cs.rightBlinker = True
  elif failure == "model_reset":
    md.frameId = 0
  elif failure == "gap":
    later += 200_000_000
  elif failure == "return":
    for line in md.laneLines:
      line.y = [y-.3 for y in line.y]
  elif failure == "timeout":
    observer.active.started = later/1e9 - 12.
  assert observer.update(cs, cc, md, vision(later), later, allowed=failure != "context") is None
  assert observer.active is None
  assert observer.request_id == request[1]


@pytest.mark.parametrize("source", ["desire", "path"])
@pytest.mark.parametrize("active", [False, True])
def test_optional_predictions_need_unique_frames_and_lateral_control(source, active):
  observer = TeslaLaneSignals(**{source: True})
  result = None
  for i in range(24):
    cs, cc, md = context(frame=i+1, torque=False)
    cc.latActive = active
    if source == "desire":
      md.meta.desireState = [.1, .0, .0, .85, .05]
    else:
      cc.actuators.curvature = -.002
      md.position.t = [0., 1., 1.5, 2., 2.5, 3.]
      md.position.x = [0., 5., 10., 15., 20., 25.]
      md.position.y = [0., -.2, -.8, -1.5, -2., -2.2]
    now = N + i*50_000_000
    v = vision(N + i//8*400_000_000)
    result = observer.update(cs, cc, md, v, now, allowed=True)
    if result:
      break
  assert bool(result) == active
  if active:
    assert observer.active.source == "model_"+source
    assert str(md.meta.desire) == "none"


def test_repeated_timestamps_do_not_confirm_prediction_or_lane():
  observer = TeslaLaneSignals(desire=True)
  cs, cc, md = context(torque=False)
  cc.latActive = True
  md.meta.desireState = [.1, 0., 0., .9, 0.]
  for i in range(20):
    assert observer.update(cs, cc, md, vision(N), N+i*10_000_000, allowed=True) is None
  assert observer.active is None


def test_model_turn_probability_does_not_restore_navigation_preannounce():
  observer = TeslaLaneSignals(desire=True, path=True)
  def mutate(i, cs, cc, md, v, now):
    cc.latActive = True
    cs.steeringPressed = False
    md.meta.desireState = [.01, .98, 0., .01, 0.]
    return v
  assert drive(observer, mutate=mutate)[0] is None


def test_existing_atc_intent_cannot_be_relabelled_as_lamp_only():
  signals = TeslaTurnSignals(True)
  cs, cc, md = context()
  feedback(signals, N)
  assert not signals.update(cs, cc, intent(1, 1), True, N, lamp_intent=(2, (1 << 31)+1))
  assert signals.session is None and signals.status == "blocked:lateral_inactive"


@pytest.mark.parametrize("field,value", [("vEgo", 0.), ("vEgo", float("nan")), ("trailerConnected", True),
                                       ("steerFaultTemporary", True), ("steerFaultPermanent", True)])
def test_executor_rechecks_manual_permission(field, value):
  signals = TeslaTurnSignals(True)
  cs, cc, md = context()
  setattr(cs, field, value)
  feedback(signals, N)
  assert not signals.update(cs, cc, md.meta, True, N, lamp_intent=(1, (1 << 31)+1))
  assert signals.session is None


def test_prediction_loses_lateral_permission_even_on_a_repeated_model_frame():
  observer = TeslaLaneSignals(desire=True)
  def mutate(i, cs, cc, md, v, now):
    cc.latActive = True
    cs.steeringPressed = False
    md.meta.desireState = [.1, 0., 0., .85, .05]
    return v
  request, cs, cc, md, v, now = drive(observer, mutate=mutate)
  assert request
  cc.latActive = False
  assert observer.update(cs, cc, md, v, now+10_000_000, allowed=True) is None
  assert observer.active is None


def test_consumed_prediction_cannot_replay_until_neutral_and_dark_rearm():
  observer = TeslaLaneSignals(desire=True)
  def mutate(i, cs, cc, md, v, now):
    cc.latActive = True
    cs.steeringPressed = False
    md.meta.desireState = [.1, 0., 0., .85, .05]
    return v
  request, cs, cc, md, v, now = drive(observer, mutate=mutate)
  assert request
  observer.active.started = now/1e9 - 12.
  for i in range(1, 100):
    stamp = now+i*50_000_000
    md.frameId += 1
    assert observer.update(cs, cc, md, vision(stamp), stamp, allowed=True) is None
  assert observer.prediction_consumed and observer.request_id == request[1]
  md.meta.desireState = [1., 0., 0., 0., 0.]
  md.frameId += 1
  stamp += 50_000_000
  assert observer.update(cs, cc, md, vision(stamp), stamp, allowed=True) is None
  assert not observer.prediction_consumed
  md.meta.desireState = [.1, 0., 0., .85, .05]
  for i in range(1, 7):
    md.frameId += 1
    later = stamp+i*50_000_000
    second = observer.update(cs, cc, md, vision(later), later, allowed=True)
    if second:
      break
  assert second == (request[0], request[1]+1)


@pytest.mark.parametrize("direction", [1, 2])
def test_manual_crossing_completes_after_relabel_and_settle(direction):
  observer = TeslaLaneSignals(manual=True)
  request, _, cc, md, _, now = drive(observer, direction)
  assert request
  movement = observer.active
  distances = [.9, .7, .5, .3, .1, -.1, 3.4, 3.2, 3., 2.8, 2.6, 2.4, 2.2, 2., 1.8] + [1.8]*12
  for i, distance in enumerate(distances, 1):
    cs, _, next_model = context(direction, distance, md.frameId+i)
    stamp = now+i*50_000_000
    result = observer.update(cs, cc, next_model, vision(stamp), stamp, allowed=True)
    if result is None:
      break
    assert result == request
  assert observer.active is None
  assert movement.reason == "completed"
  assert observer.reason == "completed_or_cancelled"
  assert observer.request_id == request[1]


def test_atc_priority_cancels_owned_observation_without_relabelling_the_session():
  observer = TeslaLaneSignals(manual=True)
  request, cs, cc, md, v, now = drive(observer)
  signals = TeslaTurnSignals(True)
  feedback(signals, now)
  frame = signals.update(cs, cc, md.meta, True, now, lamp_intent=request)[0]
  echo(signals, frame, now+1)
  feedback(signals, now+2, ui=(2, 0), front=(1, 0))
  assert observer.update(cs, cc, md, v, now+10_000_000, allowed=True, busy=True) is None
  cc.latActive = True
  packets = signals.update(cs, cc, intent(2, 7), True, now+10_000_000)
  assert signals.session.cancelling and signals.session.request_id == request[1] and signals.session.lamp_only
  assert [p.dat[1] & 3 for p in packets] == [3]
  assert observer.active is None
