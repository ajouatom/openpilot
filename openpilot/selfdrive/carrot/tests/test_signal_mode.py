import ast
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.carrot import signal_mode as policy
from openpilot.selfdrive.carrot.signal_start_guard import SignalStartRuntime
from openpilot.selfdrive.carrot.signal_stop_prepare import SignalStopPrepare
from openpilot.selfdrive.carrot.tests.test_signal_assist_runtime import SM
from openpilot.selfdrive.carrot.tests.test_signal_stop_prepare import observation
from tools.signal_analysis.replay_signal_assist import planner_class, REPO
from tools.signal_analysis.test_signal_assist_planner import inputs


@pytest.mark.parametrize('value,expected', [(0,0),(1,1),(2,2),(-1,0),(3,0),(None,0),('bad',0)])
def test_mode_values_fail_closed(value,expected):
  assert policy.read_mode(NS(get_int=lambda k:value))==expected


def test_missing_native_key_is_off_until_rebuild():
  class OldParams:
    def get_int(self,key):raise RuntimeError('unknown native key')
  assert policy.read_mode(OldParams())==0


@pytest.mark.parametrize('mode', [0,1,2])
def test_mode_owns_all_production_flags(monkeypatch,mode):
  from openpilot.selfdrive.modeld.signal_color_shadow import requested as observer
  from openpilot.selfdrive.modeld.signal_tracking_shadow import tracking_requested
  from openpilot.selfdrive.carrot.signal_display import display_requested
  from openpilot.selfdrive.carrot.signal_assist_runtime import requested
  monkeypatch.setattr(policy,'signal_mode',lambda:mode)
  assert observer()==tracking_requested()==display_requested()==(mode>0)
  for name in ('start_guard_enabled','stop_prepare_enabled'):
    assert requested(policy.DIRECTORY/name)==(mode>0)
  assert not requested(policy.DIRECTORY/'assist_enabled')


def test_off_on_ignores_pre_enable_observation_and_clears_red(monkeypatch,tmp_path):
  state=[0];monkeypatch.setattr(policy,'signal_mode',lambda:state[0])
  path=tmp_path/'observation'
  runtime=SignalStartRuntime(observation_path=path)
  sm=SM();obs,ctx=runtime.read(sm,10.)
  assert not ctx['enabled'] and runtime.guard is not None
  state[0]=1
  def packet(t,fid):
    return dict(version=1,stream='road',size=[1344,760],session='mode',timestamp=t,frame_id=fid,
                tracks=[dict(box=[640,250,680,270],raw='red')])
  path.write_text(json.dumps(packet(10.1,1)))
  assert runtime.read(sm,10.6)[0] is None
  for t,fid in [(10.7,2),(10.9,3)]:
    sm.recv_time=dict.fromkeys(sm.recv_time,t);path.write_text(json.dumps(packet(t,fid)))
    obs,ctx=runtime.read(sm,t);ctx.pop('now')
    dec=runtime.guard.update(t,obs,**ctx,gas=False,speed=0.,departure=True)
  assert dec.hold and dec.warn
  state[0]=0
  obs,ctx=runtime.read(sm,11.7);ctx.pop('now')
  dec=runtime.guard.update(11.7,obs,**ctx,gas=False,speed=0.,departure=True)
  assert not dec.hold and not dec.warn


@pytest.mark.parametrize('mode', [0,1,2])
def test_mode_one_notifies_without_moving_deceleration(monkeypatch,mode):
  monkeypatch.setattr(policy,'signal_mode',lambda:0)  # unrelated real transports OFF
  cls,_=planner_class((REPO/'openpilot/selfdrive/carrot/carrot_functions.py').read_text(encoding='utf-8'))
  p=cls(stop_prepare=SignalStopPrepare());sm=inputs(speed=15.,distance=160.);sounds=[]
  for i in range(25):
    t=i*.05
    p.update(sm,80.,'acc',prepare_observation=observation(t),prepare_context=dict(now=t,enabled=mode>0,
      valid=True,drive=True,turning=False,geometry=lambda b:(30.,70.),actuate=mode==2))
    sounds+=p.events.events
  assert p.prepare_decision.active==(mode>0)
  assert sounds.count('trafficSignChanged')==int(mode>0)
  assert p.prepare_control_active==(mode==2)
  assert (p.xState.name=='e2eStopPrepare')==(mode==2)
  source=REPO/'openpilot/selfdrive/controls/lib/longitudinal_planner.py'
  tree=ast.parse(source.read_text(encoding='utf-8'));klass=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='LongitudinalPlanner')
  method=next(n for n in klass.body if isinstance(n,ast.FunctionDef) and n.name=='apply_signal_stop_prepare')
  ns={};exec(compile(ast.Module([method],type_ignores=[]),str(source),'exec'),ns)
  target=NS(output_a_target=1.,output_v_target_now=20.,output_j_target_now=1.,output_should_stop=False,
    mpc=NS(mode='acc'),dt=.05,a_desired=1.,coasting_target=70.)
  sm['carState'].aEgo=0.
  sm['radarState'].leadTwo=NS(status=False)
  ns['apply_signal_stop_prepare'](target,sm,p,False,now=1.3)
  assert (target.output_a_target<0)==(mode==2)
  if mode!=2:assert target.output_a_target==1. and target.coasting_target==70.
  else:
    for next_mode in (1,0):
      t=1.4+(1-next_mode)*.1
      p.update(sm,80.,'acc',prepare_observation=observation(t),prepare_context=dict(now=t,enabled=next_mode>0,
        valid=True,drive=True,turning=False,geometry=lambda b:(30.,70.),actuate=False))
      target.output_a_target=1.
      ns['apply_signal_stop_prepare'](target,sm,p,False,now=t)
      assert target.output_a_target==1. and p.xState.name!='e2eStopPrepare'


def test_prepare_mode_change_requires_new_observation(monkeypatch):
  from openpilot.selfdrive.carrot.signal_assist_runtime import SignalAssistRuntime
  from openpilot.selfdrive.carrot.signal_stop_prepare_runtime import SignalPrepareRuntime
  state=[1];monkeypatch.setattr(policy,'signal_mode',lambda:state[0])
  runtime=SignalPrepareRuntime();runtime.enabled=False
  # Isolate transport/mode boundary from camera validity, which has real-message tests.
  monkeypatch.setattr(SignalAssistRuntime,'read',lambda *a:(observation(10.),dict(now=10.,enabled=True,valid=True,drive=True)))
  obs,ctx=runtime.read({});assert obs is not None and not ctx['actuate']
  runtime.prepare.active=True;state[0]=2
  monkeypatch.setattr(SignalAssistRuntime,'read',lambda *a:(observation(10.),dict(now=10.1,enabled=True,valid=True,drive=True)))
  obs,ctx=runtime.read({})
  assert obs is None and ctx['actuate'] and not runtime.prepare.active


def test_mode_two_uses_measured_housing_backend_with_egpu():
  from openpilot.selfdrive.modeld.signal_neural_shadow import automatic_backend
  params=NS(get_int=lambda k:2,get_bool=lambda k:k=='UsbGpuActive')
  assert automatic_backend(params)=='tracking_cpu'


def test_catalog_menu_default_and_registration():
  catalog=json.loads((REPO/'openpilot/selfdrive/carrot_settings.json').read_text(encoding='utf-8'))
  def find(obj):
    if isinstance(obj,dict):
      if obj.get('name')==policy.PARAM:return obj
      for v in obj.values():
        result=find(v)
        if result:return result
    if isinstance(obj,list):
      for v in obj:
        result=find(v)
        if result:return result
  item=find(catalog)
  assert (item['min'],item['max'],item['default'])==(0,2,0)
  assert (REPO/'openpilot/selfdrive/carrot_settings.json').read_text(encoding='utf-8').count('"SignalAssistMode"')==2
  assert '{"SignalAssistMode", {PERSISTENT, INT, "0"}}' in (REPO/'openpilot/common/params_keys.h').read_text()
