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
