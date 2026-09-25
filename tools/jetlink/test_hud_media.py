import sys
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).parent))
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'openpilot/selfdrive/carrot/cluster'))
from hud_navi import Assembler, fragments, CHUNK, HEADER, MAX_EVENT
from hud_stats import VehicleSystemStats, VehicleCpuOverlay


def test_large_keyframe_reassembles_without_truncation():
  raw=bytes(range(256))*977
  a=Assembler(); parts=list(fragments(raw,123,4))
  assert len(parts)>1 and max(map(len,parts))<=CHUNK+HEADER.size
  for part in parts[:-1]:assert a.feed(part,1.) is None
  assert a.feed(parts[-1],1.1)==(123,4,raw)


def test_missing_reordered_stale_and_oversized_fragments_are_discarded():
  parts=list(fragments(b'x'*(CHUNK*3),123,1)); a=Assembler()
  assert a.feed(parts[0],0) is None
  assert a.feed(parts[2],.1) is None
  assert a.feed(parts[1],.2) is None
  assert a.feed(parts[0],1) is None
  assert a.feed(parts[1],3.1) is None
  assert a.feed(HEADER.pack(1,1,0,MAX_EVENT+1)+b'x',4) is None
  assert a.feed(next(fragments(b'fresh',123,2)),4)==(123,2,b'fresh')


class FakeSM:
  def __init__(self):
    self.valid={'deviceState':True}; self.alive=dict(self.valid)
    self.device=SimpleNamespace(cpuUsagePercent=[0,20,40,60,80,100,10,30],memoryUsagePercent=42,freeSpacePercent=73)
  def update(self,timeout):pass
  def __getitem__(self,key):return self.device


def test_stats_use_all_vehicle_cores_including_idle_and_never_host_fallback():
  sm=FakeSM(); adapter=VehicleSystemStats(sm=sm)
  stats=adapter.sample()
  assert stats.cpu_core_percents==(0,20,40,60,80,100,10,30)
  assert stats.cpu_used_percent==42.5
  assert stats.memory_used_percent==42 and stats.disk_used_percent==27
  overlay=VehicleCpuOverlay(sm=sm)
  assert overlay.sample_text()=='CPU 42%'
  sm.alive['deviceState']=False
  assert adapter.sample().cpu_core_percents==()
  assert overlay.sample_text()=='CPU --'


def test_invalid_percentages_do_not_become_plausible_vehicle_values():
  sm=FakeSM();sm.device.cpuUsagePercent=[float('nan'),-1,101,0]
  sm.device.freeSpacePercent=float('inf')
  stats=VehicleSystemStats(sm=sm).sample()
  assert stats.cpu_core_percents==(None,None,None,0)
  assert stats.cpu_used_percent==0 and stats.disk_used_percent is None

def test_host_read_ahead_owns_payload_and_stays_bounded():
  import queue, time
  sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'third_party/jetlink'))
  from host_reader import ReadAheadTransport
  from jetlink.transport.base import Message, LinkTimeout
  class Transport:
    def __init__(self):self.inputs=queue.Queue();self.buf=bytearray(4)
    def recv(self,timeout):
      try:seq=self.inputs.get(timeout=timeout)
      except queue.Empty:raise LinkTimeout('idle')
      self.buf[:]=bytes([seq])*4
      return Message(1,seq,0,memoryview(self.buf))
  transport=Transport();reader=ReadAheadTransport(transport)
  try:
    for i in (1,2,3,4,5):transport.inputs.put(i)
    first=reader.recv(1)
    time.sleep(.05)
    assert bytes(first.payload)==b'\x01'*4
    assert reader.queue.qsize()<=2
    assert [reader.recv(1).seq for _ in range(4)]==[2,3,4,5]
  finally:reader.close()
  assert not reader.thread.is_alive()

def test_video_gap_waits_for_keyframe_instead_of_decoding_dependent_frames():
  from openpilot.cereal import log
  from hud_navi import RemoteMediaSocket
  packets=[]
  for sequence,message_type,flags in [(1,2,1),(2,3,1),(3,3,0),(5,3,0),(6,3,1)]:
    event=log.Event.new_message(carrotNaviMedia={'schemaVersion':1,'sessionId':'trial','kind':'render','name':'map_main',
      'sequence':sequence,'messageType':message_type,'flags':flags,'present':True,'payload':b'video'})
    packets.extend(fragments(event.to_bytes(),10,sequence))
  class FakeSocket:
    def recv(self,size):
      if packets:return packets.pop(0)
      raise BlockingIOError
  reader=object.__new__(RemoteMediaSocket)
  reader.sock=FakeSocket();reader.assembler=Assembler();reader.previous=None;reader.waiting=set();reader.sequences={}
  assert [e.carrotNaviMedia.sequence for e in reader.drain()]==[1,2,3,6]


def test_snapshot_loss_is_not_an_ignition_off_command(monkeypatch):
  import base64
  import hud
  current=[(10.,{'params':{'IsOnroad':base64.b64encode(b'1').decode()}})]
  monkeypatch.setattr(hud,'read_snapshot',lambda:current[0])
  params=hud.DisplayParams()
  assert params.get_bool('IsOnroad')
  current[0]=None;params.next_read=0
  assert params.get_bool('IsOnroad')
  current[0]=(11.,{'params':{'IsOnroad':base64.b64encode(b'0').decode()}});params.next_read=0
  assert not params.get_bool('IsOnroad')
