import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from blink_policy import Policy
from blink_can import frames, inactive, crc
from blink_protocol import wire_record
from test_blinker import setup, packet


def update(p, t=10., mode=1, commands=(), **kw):
  values = dict(now=t, mode=mode, ok=True, commands=commands, speed=0., angle=0., gear='drive', remote=True)
  values.update(kw)
  return p.update(**values)


def event(t=10., direction='left', ident='a'):
  return dict(id=ident, time=t, direction=direction)


@pytest.mark.parametrize('mode,speed,held', [(0,0,False),(1,0,True),(1,5/3.6,True),(2,0,True),(1,6/3.6,False)])
def test_manual_threshold_and_mode(mode,speed,held):
  taps, hold = update(Policy(), mode=mode, speed=speed, commands=[event()])
  assert bool(hold) == held
  assert bool(taps) != held


def test_manual_hold_survives_departure_then_completes_turn():
  p = Policy()
  update(p, commands=[event()])
  assert update(p,t=11,speed=3)[1]
  assert update(p,t=12,speed=3,angle=40)[1]
  assert update(p,t=13,speed=3,angle=5)[1]
  assert update(p,t=13.41,speed=3,angle=5)[1] is None


def test_wrong_direction_steering_does_not_complete_selected_turn():
  p = Policy()
  update(p,commands=[event()])
  update(p,t=11,speed=3,angle=-60)
  update(p,t=12,speed=3,angle=0)
  assert update(p,t=13,speed=3)[1]


def test_same_button_cancels_opposite_switches_and_duplicates_do_nothing():
  p = Policy()
  _, first = update(p,commands=[event()])
  assert update(p,t=10.1,commands=[event()])[1]['id'] == first['id']
  assert update(p,t=10.2,commands=[event(10.2,'right','b')])[1]['direction'] == 'right'
  assert update(p,t=10.3,commands=[event(10.3,'right','c')])[1] is None


@pytest.mark.parametrize('kw', [dict(ok=False),dict(mode=0),dict(remote=False),dict(gear='reverse'),dict(angle=float('nan')),dict(t=311)])
def test_manual_hold_cancel_paths(kw):
  p=Policy()
  update(p,commands=[event()])
  assert update(p,**kw)[1] is None


@pytest.mark.parametrize('kind,mode,allowed',[('turn',1,True),('turn',2,True),('lane',1,False),('lane',2,True)])
def test_automatic_modes_and_end(kind,mode,allowed):
  p=Policy()
  update(p,mode=mode)
  assert bool(update(p,t=11,mode=mode,automatic=(kind,'left'))[1]) == allowed
  if allowed:
    assert update(p,t=12,mode=mode,automatic=(kind,'left'))[1]
  assert update(p,t=13,mode=mode)[1] is None


def test_no_join_at_startup_setting_change_or_after_stale_model():
  p=Policy()
  assert update(p,automatic=('turn','left'))[1] is None
  assert update(p,t=11,mode=2,automatic=('turn','left'))[1] is None
  update(p,t=12,mode=2)
  assert update(p,t=13,mode=2,automatic=('turn','left'))[1]
  assert update(p,t=14,mode=2,auto_valid=False)[1] is None
  assert update(p,t=15,mode=2,automatic=('turn','left'))[1] is None
  update(p,t=16,mode=2)
  assert update(p,t=17,mode=2,automatic=('turn','left'))[1]


def test_manual_cancel_suppresses_current_automatic_episode():
  p=Policy()
  update(p)
  update(p,t=11,automatic=('turn','left'))
  assert update(p,t=12,commands=[event(12)],automatic=('turn','left'))[1] is None
  assert update(p,t=13,automatic=('turn','left'))[1] is None
  update(p,t=14)
  assert update(p,t=15,automatic=('turn','left'))[1]



def test_reverse_waits_for_actual_idle_and_keeps_only_latest(tmp_path):
  clock,bus,c=setup(tmp_path)
  c.accept(packet())
  for offset,_ in c.pending['schedule'][:]:
    clock[0]=11+offset;c.tick()
  bus.idle=False
  clock[0]=11.8;bus.latest[0x3c1]=(clock[0],bus.original)
  c.accept(packet('right','second',clock[0]));c.tick()
  assert c.pending is None and c.queued['id']=='second'
  clock[0]=12.;bus.latest[0x3c1]=(clock[0],bus.original)
  c.accept(packet('left','third',clock[0]));c.tick()
  assert c.queued['id']=='third'
  clock[0]=13.;bus.latest[0x3c1]=(clock[0],bus.original)
  bus.idle=True
  c.accept(dict(token='session',ok=True,guard_deadline=13.3,events=[]));c.tick()
  assert c.pending['id']=='third' and c.queued is None


@pytest.mark.parametrize('fault',['driver','guard','disconnect','timeout'])
def test_waiting_intention_revoked_without_later_replay(tmp_path,fault):
  clock,bus,c=setup(tmp_path)
  c.owned=dict(original=bus.original,until=20)
  bus.idle=False
  p=packet('right');p['events'][0]['device']='remote';p['devices']=['remote']
  c.accept(p)
  assert c.queued
  clock[0]=11.1
  bus.latest[0x3c1]=(clock[0],bytes(8) if fault=='driver' else bus.original)
  if fault=='guard':c.guard_deadline=0
  elif fault=='disconnect':c.accept(dict(token='session',ok=True,guard_deadline=11.4,events=[],devices=[]))
  elif fault=='timeout':clock[0]=14.3;c.guard_deadline=14.6;bus.latest[0x3c1]=(clock[0],bus.original)
  c.tick()
  assert c.queued is None and not bus.sent


def test_only_one_touch_can_is_sent_for_a_continuing_hold(tmp_path):
  clock,bus,c=setup(tmp_path)
  h=dict(id='hold',direction='left',source='turn',deadline=11.39)
  for dt in (0.,.037063,.086550,.217974,.257611,.297840,.6,1.,2.,3.,5.):
    clock[0]=11+dt;bus.latest[0x3c1]=(clock[0],bus.original)
    c.accept(dict(token='session',ok=True,guard_deadline=clock[0]+.3,events=[],hold=h));c.tick()
  assert len(bus.sent)==6 and c.status['accepted_id']=='hold'
  assert bus.modes==[True,False]


def test_native_hold_reader_clears_stale_and_preserves_unrelated_vehicles(tmp_path):
  import json
  from types import SimpleNamespace
  from openpilot.selfdrive.carrot.bluetooth.physical import HoldReader,FINGERPRINT
  path=tmp_path/'lease.json'
  params=SimpleNamespace(get_int=lambda _:1)
  reader=HoldReader(FINGERPRINT,params,path)
  path.write_text(json.dumps(dict(stamp=10.,mode=1,fingerprint=FINGERPRINT,direction='left',override=True)))
  assert reader.read(True,10.1)==2
  assert reader.read(True,10.36)==1
  assert reader.read(False,10.2)==1
  assert HoldReader('other',params,path).read(True,10.1)==0
  params.get_int=lambda _:0
  assert reader.read(True,10.7)==0


@pytest.mark.parametrize('value',[{}, {'stamp':float('nan')}, {'stamp':12}, {'stamp':10,'fingerprint':'other','mode':1,'direction':'left'}])
def test_invalid_native_hold_lease_never_sets_hold(value):
  from openpilot.selfdrive.carrot.bluetooth.physical import lease_value
  assert lease_value(value,10.1)==0


def test_unaccepted_start_expires_without_becoming_invisible_toggle():
  p=Policy()
  update(p,commands=[event()])
  assert p.acknowledged(13.9,None)
  assert p.acknowledged(14.01,None) is None
  assert update(p,t=15,commands=[event(15,'left','next')])[1]
  held=p.hold
  assert p.acknowledged(20,held['id'])
  assert p.acknowledged(20,held['id'],driver_override=True) is None


@pytest.mark.parametrize('direction,desire', [('left','turnLeft'),('right','turnRight')])
def test_audio_cue_starts_before_desire_and_holds_until_desire_ends(direction,desire):
  from blink_policy import AutomaticSignals
  a,p=AutomaticSignals(),Policy()
  def step(t,cue=0.,atc_type='none',turn='none'):
    auto=a.update(t,1,True,cue,direction,atc_type,turn)
    return update(p,t=t,automatic=auto)[1]
  assert step(10) is None
  first=step(11,11,'turn '+direction)
  assert first and first['direction']==direction
  assert step(12,11,'turn '+direction)['id']==first['id']
  assert step(13,11,'turn '+direction,desire)['id']==first['id']
  assert step(14,11,'none',desire)['id']==first['id']
  assert step(15,11) is None
  assert step(15.1,11,'turn '+direction) is None


@pytest.mark.parametrize('fault', ['startup','stale','future','invalid','disabled','setting','navigation','opposite'])
def test_audio_cue_never_replays_or_starts_wrong_turn(fault):
  from blink_policy import AutomaticSignals
  a=AutomaticSignals()
  if fault!='startup':a.update(10,1,True,0.,'none','none','none')
  mode=0 if fault=='disabled' else 2 if fault=='setting' else 1
  cue=9. if fault=='stale' else 12. if fault=='future' else 11.
  atc_type='none' if fault=='navigation' else 'turn left'
  desire='turnRight' if fault=='opposite' else 'none'
  assert a.update(11,mode,fault!='invalid',cue,'left',atc_type,desire) is None
  assert a.update(11.1,1,True,cue,'left','turn left','none') is None


def test_audio_cue_cancel_before_entry_and_stale_recovery():
  from blink_policy import AutomaticSignals
  a=AutomaticSignals()
  a.update(10,1,True,0.,'none','none','none')
  assert a.update(11,1,True,11.,'left','turn left','none')==('turn','left')
  assert a.update(11.1,1,True,11.,'left','none','none') is None
  assert a.update(11.2,1,True,11.,'left','turn left','none') is None
  assert a.update(12,1,True,12.,'left','turn left','none')
  assert a.update(12.1,1,False,0.,'none','','none') is None
  assert a.update(12.2,1,True,12.,'left','turn left','turnLeft') is None


def test_audio_cue_same_button_cancel_does_not_restart_on_turn_entry():
  from blink_policy import AutomaticSignals
  a,p=AutomaticSignals(),Policy()
  a.update(10,1,True,0.,'none','none','none');update(p)
  auto=a.update(11,1,True,11.,'left','turn left','none')
  assert update(p,t=11,automatic=auto)[1]
  assert update(p,t=11.1,automatic=auto,commands=[event(11.1)])[1] is None
  auto=a.update(12,1,True,11.,'left','turn left','turnLeft')
  assert update(p,t=12,automatic=auto)[1] is None


def test_mode_two_lane_start_still_waits_and_does_not_join_on_setting_change():
  from blink_policy import AutomaticSignals
  a,p=AutomaticSignals(),Policy()
  def step(t,mode,lane=None):
    auto=a.update(t,mode,True,0.,'none','fork left','none',lane)
    return update(p,t=t,mode=mode,automatic=auto)[1]
  assert step(10,1) is None
  assert step(11,1,'left') is None
  assert step(12,2,'left') is None
  assert step(13,2,'left') is None
  assert step(14,2) is None
  assert step(15,2,'left')['source']=='lane'
  assert step(16,2) is None


def test_actual_audio_event_produces_timestamp_and_direction_only_on_cue():
  import ast
  from types import SimpleNamespace as NS
  path=Path(__file__).resolve().parents[2]/'openpilot/selfdrive/selfdrived/selfdrived.py'
  tree=ast.parse(path.read_text(encoding='utf8'))
  cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='SelfdriveD')
  fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='update_atc_events')
  ns={'time':NS(monotonic_ns=lambda:11000000000),
      'LaneChangeDirection':NS(left='left',right='right'),
      'EventName':NS(audioTurn='turn',audioLaneChange='lane')}
  exec(compile(ast.Module(body=[fn],type_ignores=[]),str(path),'exec'),ns)
  class SM(dict):
    alive={'carrotMan':True}
  sm=SM(carrotMan=NS(atcType='none'));events=[]
  obj=NS(sm=sm,events=NS(add=events.append),atc_type_last='',atc_turn_start=0,atc_turn_direction='none')
  run=ns['update_atc_events']
  for value in ('fork left prepare','fork left'):
    sm['carrotMan'].atcType=value;run(obj)
  assert events==['lane'] and obj.atc_turn_start==0
  sm['carrotMan'].atcType='turn left';run(obj)
  assert events==['lane','turn'] and obj.atc_turn_start==11000000000 and obj.atc_turn_direction=='left'
  run(obj)
  assert events==['lane','turn']
  sm['carrotMan'].atcType='none';run(obj)
  sm['carrotMan'].atcType='turn right';run(obj)
  assert events==['lane','turn','turn'] and obj.atc_turn_direction=='right'


def test_audio_cue_schema_roundtrip():
  pytest.importorskip('capnp')
  from openpilot.cereal import log
  msg=log.SelfdriveState.new_message(atcTurnStartMonoTime=11000000000,atcTurnDirection='left')
  with log.SelfdriveState.from_bytes(msg.to_bytes()) as read:
    assert read.atcTurnStartMonoTime==11000000000 and str(read.atcTurnDirection)=='left'


def test_actual_source_loop_starts_from_audio_cue_before_model_turn(monkeypatch):
  import blink_source as source
  import json
  import types
  from contextlib import nullcontext
  from types import SimpleNamespace as NS
  from test_blinker import Vehicle
  from openpilot.selfdrive.carrot.bluetooth import model
  clock,sent=[10.],[]
  sm=Vehicle()
  sm['modelV2']=NS(meta=NS(desire='none',autoTurnSignal='none',autoTurnSignalLaneChange=False))
  sm['carrotMan']=NS(atcType='none')
  sm.services=list(sm);sm.valid=dict.fromkeys(sm,True);sm.alive=dict.fromkeys(sm,True)
  def tick(_):
    i=len(sent);clock[0]=10.+i*.1
    sm.recv_time=dict.fromkeys(sm,clock[0])
    sm['selfdriveState'].atcTurnStartMonoTime=10100000000 if i else 0
    sm['selfdriveState'].atcTurnDirection='left' if i else 'none'
    sm['carrotMan'].atcType='turn left' if i else 'none'
    sm['modelV2'].meta.desire='turnLeft' if i==2 else 'none'
  sm.update=tick
  fake_params=types.ModuleType('openpilot.common.params')
  fake_params.Params=lambda:NS(get=lambda _:b'cp',get_int=lambda _:1)
  fake_cereal=types.ModuleType('openpilot.cereal')
  fake_cereal.messaging=NS(SubMaster=lambda _:sm)
  fake_cereal.car=NS(CarParams=NS(from_bytes=lambda _:nullcontext(NS(carFingerprint='HYUNDAI_IONIQ_5_PE'))))
  monkeypatch.setitem(sys.modules,'openpilot.common.params',fake_params)
  monkeypatch.setitem(sys.modules,'openpilot.cereal',fake_cereal)
  sock=NS(setblocking=lambda _:None,recv=lambda _:b'',send=lambda raw:sent.append(json.loads(raw)))
  monkeypatch.setattr(source.socket,'socket',lambda **kw:sock)
  monkeypatch.setattr(source.select,'select',lambda *a:([sock] if len(sent)==4 else [],[],[]))
  monkeypatch.setattr(source.time,'monotonic',lambda:clock[0])
  monkeypatch.setattr(source.time,'sleep',lambda _:None)
  monkeypatch.setattr(model,'read_json',lambda *a:{})
  monkeypatch.setattr(model,'config',dict)
  monkeypatch.setattr(model,'atomic_json',lambda *a:None)
  source.main(0)
  assert sent[0]['hold'] is None
  assert sent[1]['hold']['source']=='turn' and sent[1]['hold']['direction']=='left'
  assert sent[2]['hold']['id']==sent[1]['hold']['id']
  assert sent[3]['hold'] is None
