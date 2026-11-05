"""Paddle stop (card publishes vCruise 0) hands off to the light even behind a moving lead, and keeps stopping."""
from types import SimpleNamespace as NS

from tools.signal_analysis.replay_signal_assist import REPO, planner_class
from tools.signal_analysis.test_signal_assist_planner import inputs


def planner():
  cls, _ = planner_class((REPO/'openpilot/selfdrive/carrot/carrot_functions.py').read_text(encoding='utf-8'))
  return cls()


def red_scene(speed=10., lead_v=8.):
  sm = inputs(speed=speed, distance=40.)
  sm['modelV2'].velocity.x = [speed] + [0.] * 32  # model plans a stop at 40 m
  sm['radarState'].leadOne = NS(status=True, dRel=30., vLead=lead_v)
  return sm


def run(p, sm, v_cruise_kph, n=20):
  for _ in range(n):
    p.update(sm, v_cruise_kph, 'acc')


def test_paddle_takes_the_light_behind_a_moving_lead():
  p, sm = planner(), red_scene()
  run(p, sm, 0.)
  assert p.trafficState.name == 'red' and p.xState.name == 'e2eStop'


def test_without_paddle_a_moving_lead_still_wins():
  p, sm = planner(), red_scene()
  run(p, sm, 80.)
  assert p.xState.name == 'lead'


def test_stopped_lead_is_still_followed_during_paddle():
  p, sm = planner(), red_scene(lead_v=0.)
  run(p, sm, 0.)
  assert p.xState.name == 'lead'


def test_paddle_stop_survives_green_flicker_after_hand_off():
  p, sm = planner(), red_scene()
  run(p, sm, 0.)
  sm['modelV2'].velocity.x = [10.] + [15.] * 32  # model now says go: green
  run(p, sm, 80.)  # cruise.py restored the set speed after the hand-off
  assert p.trafficState.name == 'green'
  assert p.xState.name == 'e2eStop' and p.stop_dist < 300.


def test_gas_releases_the_paddle_stop():
  p, sm = planner(), red_scene()
  run(p, sm, 0.)
  sm['modelV2'].velocity.x = [10.] + [15.] * 32
  sm['carState'].gasPressed = True
  run(p, sm, 80., 1)
  sm['carState'].gasPressed = False
  run(p, sm, 80.)
  assert p.xState.name != 'e2eStop'


def test_normal_red_stop_still_exits_on_green():
  sm = red_scene()
  sm['radarState'].leadOne = NS(status=False, dRel=0., vLead=0.)
  p = planner()
  run(p, sm, 80.)
  assert p.xState.name == 'e2eStop'
  sm['modelV2'].velocity.x = [10.] + [15.] * 32
  run(p, sm, 80.)
  assert p.xState.name == 'e2eCruise'


def test_paddle_stop_target_ignores_the_go_plan_of_a_flicker():
  p, sm = planner(), red_scene()
  run(p, sm, 0.)
  before = p.stop_dist
  sm['modelV2'].position.x = [140. * i / 32 for i in range(33)]  # green frames plan to go ~140 m
  sm['modelV2'].velocity.x = [10.] + [15.] * 32
  run(p, sm, 80., 5)
  assert p.xState.name == 'e2eStop' and p.stop_dist <= before
