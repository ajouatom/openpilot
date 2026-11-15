"""Red-light slow-roll hold in CarrotPlanner: below 3 m/s a red stop (e2eStop) keeps its stop target through green/off
frames, so a "go" plan seen just before the stop line does not accelerate the car; once stopped a green starts as before.
"""
from types import SimpleNamespace as NS

from tools.signal_analysis.replay_signal_assist import REPO, planner_class
from tools.signal_analysis.test_signal_assist_planner import inputs


def planner():
  cls, _ = planner_class((REPO/'openpilot/selfdrive/carrot/carrot_functions.py').read_text(encoding='utf-8'))
  return cls()


def no_lead_red(speed=10.):
  sm = inputs(speed=speed, distance=40.)
  sm['modelV2'].velocity.x = [speed] + [0.] * 32  # model plans a stop at 40 m
  sm['radarState'].leadOne = NS(status=False, dRel=0., vLead=0.)
  return sm


def run(p, sm, v_cruise_kph, n=20):
  for _ in range(n):
    p.update(sm, v_cruise_kph, 'acc')


def test_normal_red_stop_still_exits_on_green():
  p, sm = planner(), no_lead_red()
  run(p, sm, 80.)
  assert p.xState.name == 'e2eStop'
  sm['modelV2'].velocity.x = [10.] + [15.] * 32
  run(p, sm, 80.)
  assert p.xState.name == 'e2eCruise'


def test_red_stop_keeps_its_target_through_a_go_plan_while_creeping_to_the_line():
  # night drive log: at 2 km/h, 2 m before a red line, the model planned go and the car rolled 1.4 -> 10 km/h
  p, sm = planner(), no_lead_red()
  run(p, sm, 80.)
  assert p.xState.name == 'e2eStop'
  sm['carState'].vEgo = sm['carState'].vEgoCluster = 2.
  sm['modelV2'].velocity.x = [2.] + [15.] * 32
  run(p, sm, 80.)
  assert p.xState.name == 'e2eStop' and p.stop_dist < 300.
  sm['carState'].vEgo = sm['carState'].vEgoCluster = 0.
  run(p, sm, 80.)  # stopped at the line: the green now starts as before
  assert p.xState.name == 'e2eCruise'
