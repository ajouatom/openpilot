"""Moving preparation ends near a lead/at standstill; stopped mode 1 == mode 2."""
import ast
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.carrot.signal_stop_prepare import SignalStopPrepare, PrepareAccel, prepare_lead_blocked
from openpilot.selfdrive.carrot.signal_start_guard import SignalStartGuard
from openpilot.selfdrive.carrot.tests.test_signal_stop_prepare import observation
from tools.signal_analysis.replay_signal_assist import planner_class, REPO
from tools.signal_analysis.test_signal_assist_planner import inputs


@pytest.mark.parametrize('index', [0, 1])
@pytest.mark.parametrize('distance,blocked', [(4., True), (50., True), (50.01, False), (80., False),
                                           (-1., True), (float('nan'), True), (float('inf'), True)])
def test_both_leads_use_fixed_boundary(index, distance, blocked):
  leads = [NS(status=False), NS(status=False)]
  leads[index] = NS(status=True, dRel=distance)
  assert prepare_lead_blocked(NS(leadOne=leads[0], leadTwo=leads[1])) is blocked


@pytest.mark.parametrize('distance,active', [(10., False), (50., False), (50.1, True), (80., True)])
def test_far_lead_remains_in_mpc_and_close_lead_cancels_preparation(distance, active):
  cls, _ = planner_class((REPO/'openpilot/selfdrive/carrot/carrot_functions.py').read_text(encoding='utf8'))
  p = cls(stop_prepare=SignalStopPrepare()); sm = inputs(speed=15., distance=160., lead=True)
  sm['radarState'].leadOne.dRel = distance
  for i in range(25):
    t = i*.05
    p.update(sm, 80., 'acc', prepare_observation=observation(t), prepare_context=dict(
      now=t, enabled=True, valid=True, drive=True, turning=False, geometry=lambda b:(30.,70.), actuate=True))
  assert p.prepare_control_active is active
  assert sm['radarState'].leadOne.status and sm['radarState'].leadOne.dRel == distance
  assert p.stop_dist == 1000. and p.xState.name == ('e2eStopPrepare' if active else 'lead')
  sm['radarState'].leadOne.dRel = 20.
  p.update(sm, 80., 'acc', prepare_observation=observation(1.3), prepare_context=dict(
    now=1.3, enabled=True, valid=True, drive=True, turning=False, geometry=lambda b:(30.,70.), actuate=True))
  assert not p.prepare_control_active and p.xState.name == 'lead'


@pytest.mark.parametrize('speed,soft,dist,active', [(15.,0,80.,True), (15.,0,50.,False),
  (0.,0,80.,False), (1.,0,80.,False), (15.,1,80.,False), (82/3.6,0,80.,False)])
def test_final_cap_rechecks_current_lead_speed_and_hold(speed, soft, dist, active):
  path = REPO/'openpilot/selfdrive/controls/lib/longitudinal_planner.py'
  tree = ast.parse(path.read_text(encoding='utf8'))
  cls = next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='LongitudinalPlanner')
  method = next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='apply_signal_stop_prepare')
  ns = {}; exec(compile(ast.Module([method],type_ignores=[]),str(path),'exec'),ns)
  p = NS(output_a_target=1., output_v_target_now=20., output_j_target_now=1., output_should_stop=False,
         mpc=NS(mode='acc'), dt=.05, a_desired=1., coasting_target=70.,
         _signal_prepare_accel=PrepareAccel(), _signal_prepare_time=None, _signal_prepare_previous=None)
  sm = inputs(speed=speed,lead=True); sm['carState'].softHoldActive=soft; sm['radarState'].leadOne.dRel=dist
  ns['apply_signal_stop_prepare'](p,sm,NS(prepare_decision=NS(active=True),prepare_control_active=True),False,now=10.)
  assert (p.output_a_target < 0) is active
  assert not p.output_should_stop


@pytest.mark.parametrize('lead', [False, True])
def test_stopped_modes_have_identical_red_unknown_green_and_departure(lead):
  cls, _ = planner_class((REPO/'openpilot/selfdrive/carrot/carrot_functions.py').read_text(encoding='utf8'))
  traces=[]
  for mode in (1,2):
    p=cls(start_guard=SignalStartGuard(),stop_prepare=SignalStopPrepare()); sm=inputs(speed=0.,lead=lead)
    trace=[]
    for i in range(70):
      t=i*.05; color='red' if i<15 else 'unknown' if i<30 else 'green'
      start=dict(timestamp=t,frame_id=i,session='same',tracks=[dict(id=1,box=[640,250,680,270],raw=color,seen_red=True)])
      p.update(sm,80.,'acc',start_observation=start,start_context=dict(now=t,enabled=True,valid=True,drive=True),
        prepare_observation=observation(t,color),prepare_context=dict(now=t,enabled=True,valid=True,drive=True,
          turning=False,geometry=lambda b:(30.,70.),actuate=mode==2))
      assert not p.prepare_control_active
      trace.append((p.xState,p.start_decision,p.stop_dist,p.v_cruise))
    traces.append(trace)
  assert traces[0] == traces[1]
  assert any(row[1].hold for row in traces[0]) and any(row[1].released for row in traces[0])
