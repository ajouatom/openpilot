"""Wire format and transport gates for the BLE output of the existing blinker policy."""
import hashlib
import hmac
import struct

from openpilot.selfdrive.carrot.bluetooth.b1_monitor import Intent, command, compatible, SERVICE


def test_command_wire_vector_fits_default_att_payload():
  key=bytes(range(32))
  packet=command(key, 0x12345678, 1800, 14, 2, .35)
  assert len(packet)==20
  assert struct.unpack('<IIHBB', packet[:12])==(0x12345678,1800,14,2,70)
  assert packet[12:]==hmac.new(key,packet[:12],hashlib.sha256).digest()[:8]
  assert compatible({'name':'Carrot-B1','uuids':[SERVICE]})
  assert not compatible({'name':'Carrot-B1','uuids':[]})


def source(now, ident='a', direction='left', **kwargs):
  event={'id':ident, 'time':now, 'direction':direction, 'device':'remote'}
  return {'stamp':now, 'events':[event], 'devices':['remote'], 'ok':True, **kwargs}


def test_one_request_once_and_exact_completion():
  intent=Intent()
  now=intent.started+.1
  data=source(now)
  assert intent.choose(data,'ready',0,now)[:2]==(1,'a')
  intent.inflight=(7,'a',now)
  assert intent.choose(data,'sending',0,now+.02)[0]==0
  hold={**data['events'][0], 'source':'turn'}
  data.update(events=[],hold=hold,b1_hold=True)
  assert intent.choose(data,'ready',6,now+.03)[0]==0
  assert intent.choose(data,'ready',7,now+.03)[0]==4
  data['hold']=None
  assert intent.choose(data,'ready',7,now+.04)[0]==0


def test_rapid_opposite_request_and_loss_of_remote():
  for connected in (False,True):
    intent=Intent()
    now=intent.started+.1
    intent.inflight=(4,'a',now)
    data=source(now,'b','right')
    assert intent.choose(data,'sending',0,now)[0]==0
    data.update(events=[],devices=['remote'] if connected else [])
    op,ident,_=intent.choose(data,'ready',4,now+.05)
    assert (op,ident)==((2,'b') if connected else (0,None))


def test_stale_missing_fault_and_override_do_not_actuate():
  for state,age,ok in [('ready',.21,True),('ready',0,False),('missing',0,True),('error',0,True),('driver_override',0,True)]:
    intent=Intent()
    now=intent.started+.1
    assert intent.choose(source(now,ok=ok),state,0,now+age)[0]==3


def test_monitor_joins_source_only_after_board_authentication(monkeypatch):
  import asyncio
  import sys
  import time
  from types import SimpleNamespace

  from tools.jetlink import blink_protocol  # noqa: TID251 -- shared subprocess protocol
  from openpilot.selfdrive.carrot.bluetooth.b1_monitor import Monitor, META, CONTROL

  async def run():
    monitor = Monitor()
    monitor.attrs = {META: 'meta', CONTROL: 'command'}
    monitor.params = SimpleNamespace(get_bool=lambda name: name == 'IsOnroad')
    monkeypatch.setitem(sys.modules, 'openpilot.common.params', SimpleNamespace(Params=lambda: monitor.params))
    started, feedback = [], []
    worker = SimpleNamespace(process=SimpleNamespace(poll=lambda: None), send=feedback.append)
    def factory(*args):
      started.append(args)
      return worker
    monkeypatch.setattr(blink_protocol, 'Process', factory)
    key = bytes(range(32))
    seq, ack, writes = 0, 0, []

    async def board(path, interface, method, sig, body, reply_timeout):
      nonlocal seq, ack
      if path == 'meta':
        return (struct.pack('<IIHBBBH', 71, 1000, ack, 1, 0, 2, seq),)
      payload = body[0]
      assert payload[12:] == hmac.digest(key, payload[:12], 'sha256')[:8]
      _, _, seq, op, _ = struct.unpack('<IIHBB', payload[:12])
      writes.append(op)
      if op in (1, 2):
        ack = seq

    monitor.client.call = board
    await monitor.exchange(key)
    assert writes == [3] and not started
    monitor.next_start = 0
    data = source(time.monotonic())
    def latest():
      data['stamp'] = time.monotonic()
      return data
    worker.latest = latest
    await monitor.exchange(key)
    assert started == [('blink_source.py','ble')]
    assert writes[-1] == 1
    await monitor.exchange(key)
    assert feedback[-1]['accepted_id'] == 'a'
    assert writes[-1] == 0
    data['ok'] = False
    await monitor.exchange(key)
    assert writes[-1] == 3

  asyncio.run(run())


def test_enrollment_requires_one_named_service_and_proof(monkeypatch, tmp_path):
  import asyncio
  from types import SimpleNamespace
  from openpilot.selfdrive.carrot.bluetooth import b1_monitor as b1

  devices = [{'name':'Carrot-B1','address':'00:00:00:00:00:01','uuids':[SERVICE],'paired':True,'rssi':-42}]
  for name in ('ENROLLMENT','SETUP','STAGED','BADGE'):
    monkeypatch.setattr(b1, name, tmp_path / name)
  monkeypatch.setattr(b1, 'parked', lambda: True)
  monkeypatch.setattr(b1, 'radio_present', lambda: True)
  async def enabled():
    return True
  monkeypatch.setattr(b1,'radio_enabled',enabled)
  async def noop(*args):
    pass
  monkeypatch.setattr(b1.asyncio,'sleep',noop)
  async def run():
    monitor=b1.Monitor()
    seq, key = 0, b''
    async def call(path, interface, method, sig, body):
      nonlocal seq, key
      if method=='ReadValue':
        return (struct.pack('<IIHBBBH', 42, 1000, 0, 0, 0, 2, seq),)
      if path=='claim':
        key=hashlib.sha256(body[0][4:]).digest()
      else:
        packet=body[0]
        assert packet[10]==3 and hmac.digest(key,packet[:12],'sha256')[:8]==packet[12:]
        seq=struct.unpack_from('<H',packet,8)[0]
    async def snapshot():
      return {'devices':devices}
    async def discover(_):
      monitor.attrs={b1.META:'meta',b1.CONTROL:'control',b1.CLAIM:'claim'}
    monitor.client=SimpleNamespace(scan=noop,snapshot=snapshot,call=call)
    monitor.discover=discover
    devices.append({**devices[0],'address':'00:00:00:00:00:02'})
    await monitor.enroll()
    assert not b1.ENROLLMENT.exists()
    devices.pop()
    await monitor.enroll()
    assert b1.read_json(b1.ENROLLMENT)['key']==key.hex()
  asyncio.run(run())


def test_badge_expires_and_hides_if_unregistered(tmp_path):
  from openpilot.selfdrive.ui.b1_badge import status
  import json
  reg, data = tmp_path/'enrolled', tmp_path/'status'
  assert status(10,reg,data) is None
  reg.touch()
  assert status(10,reg,data)=='offline'
  for state in ('auth','missing','ready','error'):
    data.write_text(json.dumps({'time':10,'state':state}))
    assert status(11,reg,data)==state
    assert status(12,reg,data)=='offline'
