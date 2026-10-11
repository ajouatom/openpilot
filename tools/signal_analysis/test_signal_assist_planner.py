from pathlib import Path
from types import SimpleNamespace as NS

from tools.signal_analysis.replay_signal_assist import planner_class, REPO
from openpilot.selfdrive.carrot.signal_assist import SignalAssist


def inputs(speed=0., distance=100., lead=False):
  return dict(carState=NS(vCluRatio=1., softHoldActive=0, vEgo=speed, aEgo=0., vEgoCluster=speed,
                         gasPressed=False, brakePressed=False, steeringAngleDeg=0., leftBlinker=False, rightBlinker=False),
              selfdriveState=NS(personality=1), radarState=NS(leadOne=NS(status=lead, dRel=10.)),
              modelV2=NS(position=NS(x=[distance*i/32 for i in range(33)], y=[0.]*33),
                         velocity=NS(x=[speed]+[15.]*32), leadsV3=[]))


def run(p, sm, state, start=0., count=20):
  for i in range(count):
    t = start + i * .05
    obs = dict(timestamp=t, frame_id=round(t*20), session='same', tracks=[
      dict(id=1, box=[640, 250, 680, 270], state=state, age=0., observations=10, evidence={'raw': state})])
    p.update(sm, 80., 'acc', signal_observation=obs, signal_context=dict(now=t, enabled=True, valid=True, drive=True))


def planner():
  cls, _ = planner_class((REPO/'openpilot/selfdrive/carrot/carrot_functions.py').read_text(encoding='utf-8'))
  return cls(SignalAssist())


def test_red_hold_wins_over_model_go_and_lead_handover():
  p = planner(); sm = inputs(lead=True)
  run(p, sm, 'red')
  assert p.xState.name == 'e2eStopped' and p.stop_dist == 0 and p.v_cruise == 0
  run(p, sm, 'unknown', 1.)
  assert p.xState.name == 'e2eStopped'


def test_moving_red_enters_existing_stop_and_speed_cap():
  p = planner(); sm = inputs(speed=10., distance=40.)
  run(p, sm, 'red')
  assert p.xState.name == 'e2eStop'
  assert 0 < p.stop_dist < 40. and p.v_cruise < 80/3.6
  assert 0 < p.comfort_brake <= 2.4


def test_green_does_not_force_departure_against_original_model():
  p = planner(); sm = inputs(distance=3.)
  sm['modelV2'].velocity.x = [0.] * 33
  run(p, sm, 'red')
  run(p, sm, 'green', 1.)
  assert not p.signal_decision.hold
  assert p.xState.name == 'e2eStopped' and p.v_cruise == 0


def test_driver_gas_releases_hold_through_original_override():
  p = planner(); sm = inputs()
  run(p, sm, 'red')
  sm['carState'].gasPressed = True
  run(p, sm, 'red', 1., 1)
  assert not p.signal_decision.hold and p.xState.name == 'e2eCruise'
