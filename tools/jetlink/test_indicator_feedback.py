from types import SimpleNamespace as N
import pytest
from blink_feedback import ObservationFeedback
from blink_observation import allowed
from openpilot.selfdrive.controls.lib.desire_lib.indicator_feedback import TapEcho
from openpilot.selfdrive.controls.lib.desire_lib.indicator_observer import ModelIndicator, Movement
from openpilot.selfdrive.controls.lib.desire_lib.boundary_indicator import BoundaryIndicator
from test_indicator_reconstruction import scene
from test_blinker_boundary import scene as boundary_scene, update


def context():
  cs, md = scene()
  cs.leftBlinkerStalkCount, cs.rightBlinkerStalkCount = 9, 255
  md.meta.observedIndicatorMonoTime = int(10e9)
  hold = dict(id='session:1', source='observation', direction='right', time=10.)
  f = ObservationFeedback(); f.prepare(10., hold, md.meta, cs)
  f.sent(10., {'stamp':10., 'state':'ready'}, True)
  return cs, md, hold, f


def test_sent_tap_wraparound_is_not_driver_cancel_but_later_tap_is():
  cs, md, hold, f = context(); echo = TapEcho()
  cs.blinkerLever=1; cs.rightBlinkerStalkCount=0; cs.rightBlinker=True
  assert echo.matches(10.1,cs,2,10.,f.value)
  cs.blinkerLever=0
  f.sent(10.2, {'stamp':10.2,'state':'sending'}, True)
  assert not echo.matches(10.2,cs,2,10.,f.value)
  cs.blinkerLever=1; cs.rightBlinkerStalkCount=1
  assert echo.matches(10.21,cs,2,10.,f.value)  # Stock neutral interleaved our pulse train.
  f.sent(10.4, {'stamp':10.4,'state':'waiting'}, True)
  assert not echo.matches(10.4,cs,2,10.,f.value)


@pytest.mark.parametrize('fault', ['unsubmitted','stale','wrong_request','wrong_direction','latched','opposite_count','second_count','opposite_lamp','expired','missing_counts'])
def test_echo_never_masks_unmatched_driver_input(fault):
  cs, md, hold, f = context(); cs.blinkerLever=1; cs.rightBlinkerStalkCount=0
  now=10.1
  if fault=='unsubmitted': f.sent(10., {'stamp':10.,'state':'ready'}, False)
  if fault=='stale': now=10.2
  if fault=='wrong_request': f.value['tap']['request']=9.
  if fault=='wrong_direction': f.value['tap']['direction']=1
  if fault=='latched': cs.blinkerLever=2
  if fault=='opposite_count': cs.leftBlinkerStalkCount=10
  if fault=='second_count': cs.rightBlinkerStalkCount=4
  if fault=='opposite_lamp': cs.leftBlinker=True
  if fault=='expired': now=10.7; f.value['stamp']=now
  if fault=='missing_counts': del cs.rightBlinkerStalkCount
  assert not TapEcho().matches(now,cs,2,10.,f.value)


def test_continuous_tap_cannot_be_masked_by_refreshing_lease():
  cs,md,hold,f=context();cs.blinkerLever=1;cs.rightBlinkerStalkCount=0; echo=TapEcho()
  assert echo.matches(10.05,cs,2,10.,f.value)
  f.sent(10.35,{'stamp':10.35,'state':'sending'},True)
  assert not echo.matches(10.35,cs,2,10.,f.value)


def test_real_observer_and_source_keep_owned_echo_then_cancel_latched_driver():
  cs,md,hold,f=context();d=ModelIndicator();iso=BoundaryIndicator()
  d.active=Movement(10.,2,'manual',cs,md);d.request_time=10.
  cs.blinkerLever=1;cs.rightBlinkerStalkCount=0;cs.rightBlinker=True
  args=dict(valid=True,lateral_active=False,desire_enabled=False,path_enabled=False,busy=False,isolation=iso,feedback=f.value)
  d.update(10.1,cs,md,**args)
  assert d.active is not None
  echo=TapEcho().matches(10.1,cs,2,10.,f.value)
  assert allowed(cs,N(latActive=False),md,'manual',10.1,holding=True,tap_echo=echo)
  assert not allowed(cs,N(latActive=False),md,'manual',10.1,holding=False,tap_echo=echo)
  cs.blinkerLever=2
  d.update(10.12,cs,md,**args)
  assert d.active is None and d.reason=='cancelled'


def test_early_lamp_requires_exact_ack_and_expires_or_clears_on_override():
  cs,md,h,f=context();cs.rightBlinker=True
  assert f.activation(10.1,h,cs,{'stamp':10.1,'state':'sending'},False,None,True) is None
  cs.rightBlinker=False
  assert f.activation(10.3,h,cs,{'stamp':10.3,'state':'waiting'},False,None,True) is None
  assert f.activation(10.4,h,cs,{'stamp':10.4,'state':'waiting'},True,None,True)==h['id']
  assert f.activation(10.9,h,cs,{'stamp':10.9,'state':'waiting'},True,None,True) is None
  cs.rightBlinker=True
  f.activation(11.,h,cs,{'stamp':11.,'state':'sending'},False,None,True)
  assert f.activation(11.1,h,cs,{'stamp':11.1,'state':'driver_override'},True,h['id'],True) is None
  cs.rightBlinker=False
  assert f.activation(11.2,h,cs,{'stamp':11.2,'state':'waiting'},True,None,True) is None


@pytest.mark.parametrize('direction',[1,2])
def test_prediction_earlier_by_roughly_point_three_seconds_with_real_motion(direction):
  cs,md=boundary_scene(direction); d=BoundaryIndicator(); first=None
  for i in range(70):
    now=10+i*.05; y=1.8-max(0,i-30)*.04
    md.frameId=i
    md.laneLines[1].y=[-y if direction==1 else -(3.6-y)]
    md.laneLines[2].y=[3.6-y if direction==1 else y]
    update(d,now,cs,md,enabled=True,valid=True,busy=False)
    if d.request_direction and first is None: first=(i,y)
  assert first and 1.08 < first[1] < 1.2
  assert 53-first[0] in (5,6,7)  # Old 0.9 m threshold reached at frame 53.


def test_early_prediction_does_not_trigger_far_from_boundary_even_at_high_rate():
  cs,md=boundary_scene(2); d=BoundaryIndicator()
  for i in range(36):
    now=10+i*.05; y=2.2-max(0,i-30)*.15
    md.frameId=i;md.laneLines[1].y=[y-3.6];md.laneLines[2].y=[y]
    update(d,now,cs,md,enabled=True,valid=True,busy=False)
    assert not d.request_direction

@pytest.mark.parametrize('driver_override',[False,True])
@pytest.mark.parametrize('complete_ack',[False,True])
def test_actual_source_model_roundtrip_survives_echo_and_waits_for_ack(monkeypatch,driver_override,complete_ack):
  import sys,json,types
  from contextlib import nullcontext
  import blink_source as source
  from test_blinker import Vehicle
  from openpilot.selfdrive.carrot.bluetooth import model
  from openpilot.selfdrive.carrot.bluetooth.physical import HOLD_PATH
  clock,sent,saved=[10.],[],{}
  sm=Vehicle();cs,md=scene();cs.leftBlinkerStalkCount=0;cs.rightBlinkerStalkCount=0
  sm['carState']=cs;sm['modelV2']=md
  md.meta=N(desire='none',autoTurnSignal='none',autoTurnSignalLaneChange=False,laneChangeState='off',
    observedIndicator='none',observedIndicatorMonoTime=0,observedIndicatorSource='manual',indicatorIsolationMask=0)
  sm['carrotMan']=N(atcType='none',xDistToTurn=0)
  sm['livePose']=N(inputsOK=True,sensorsOK=True,posenetOK=True,orientationNED=N(valid=True,z=0.))
  sm.services=list(sm);sm.valid=dict.fromkeys(sm,True);sm.alive=dict.fromkeys(sm,True)
  detector=ModelIndicator();isolation=BoundaryIndicator();held=[]
  def tick(_):
    i=len(sent);clock[0]=10+i*.05;sm.recv_time=dict.fromkeys(sm,clock[0])
    cs.blinkerLever=1 if i in (6,8) else 0
    cs.rightBlinkerStalkCount=0 if i<6 else 1 if i<8 else 2
    cs.rightBlinker=6<=i<=8
    if driver_override and i>=18:cs.blinkerLever=2;cs.leftBlinkerStalkCount=1
    sm['selfdriveState'].atcTurnStartMonoTime=0;sm['selfdriveState'].atcTurnDirection='none'
    isolation.request_time=10.1
    if i==2:isolation.own(clock[0],2)
    detector.update(clock[0],cs,md,valid=True,lateral_active=False,desire_enabled=False,path_enabled=False,busy=False,
      isolation=isolation,manual_edge=2 if i==2 else 0,feedback=saved.get(HOLD_PATH,{}))
    md.meta.observedIndicator='right' if detector.active else 'none'
    md.meta.observedIndicatorMonoTime=int(detector.request_time*1e9)
    md.meta.indicatorIsolationMask=isolation.blocked
  sm.update=tick
  fake_params=types.ModuleType('openpilot.common.params')
  fake_params.Params=lambda:N(get=lambda _:b'cp',get_int=lambda k:1 if k=='AutoBlinkerControl' else 0,
    get_bool=lambda k:k=='ManualLaneChangeIndicator')
  fake_cereal=types.ModuleType('openpilot.cereal');fake_cereal.messaging=N(SubMaster=lambda _:sm)
  fake_cereal.car=N(CarParams=N(from_bytes=lambda _:nullcontext(N(carFingerprint='HYUNDAI_IONIQ_5_PE'))))
  monkeypatch.setitem(sys.modules,'openpilot.common.params',fake_params);monkeypatch.setitem(sys.modules,'openpilot.cereal',fake_cereal)
  fed=[-1]
  def recv(_):
    i=len(sent)
    if i>=22:return b''
    fed[0]=i
    h=next((x['hold'] for x in sent if x['hold']),None)
    state='driver_override' if driver_override and i>=18 else 'waiting' if i>=12 else 'sending' if i>=3 else 'ready'
    return json.dumps(dict(stamp=10+i*.05,state=state,accepted_id=h['id'] if h and complete_ack and i>=12 else None)).encode()
  sock=N(setblocking=lambda _:None,recv=recv,send=lambda raw:sent.append(json.loads(raw)))
  monkeypatch.setattr(source.socket,'socket',lambda **kw:sock)
  monkeypatch.setattr(source.select,'select',lambda *a:([sock] if len(sent)>=22 or fed[0]!=len(sent) else [],[],[]))
  monkeypatch.setattr(source.time,'monotonic',lambda:clock[0]);monkeypatch.setattr(source.time,'sleep',lambda _:None)
  monkeypatch.setattr(model,'read_json',lambda path,*a:saved.get(path,{}));monkeypatch.setattr(model,'config',dict)
  def write(path,value):
    saved[path]=value
    if path==HOLD_PATH:held.append((len(sent)-1,value))
  monkeypatch.setattr(model,'atomic_json',write)
  source.main(0)
  assert all(sent[i]['hold'] for i in range(2,18))
  assert len({sent[i]['hold']['id'] for i in range(2,18)})==1
  assert not any(v['direction'] for i,v in held if i<12)
  assert any(v['direction']=='right' for i,v in held if 12<=i<18)==complete_ack
  if driver_override:
    assert sent[-1]['hold'] is None
    assert all(v['direction'] is None for i,v in held if i>=18)
