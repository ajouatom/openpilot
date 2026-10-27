import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import blink_protocol as protocol
from blink_can import Controller, frames, crc
import blink_can
from blink_source import Events, vehicle_ready


class Bus:
  def __init__(self, clock):
    self.clock = clock
    self.original = bytes.fromhex('8f30300000000000')
    self.latest = {0x3C1: (clock[0], self.original)}
    self.sent, self.modes = [], []
    self.idle = True

  def neutral(self, now):
    return self.original if self.idle else None

  def mode(self, value):
    self.modes.append(value)

  def send(self, data):
    self.sent.append(data)


def setup(tmp_path):
  clock = [10.0]
  bus = Bus(clock)
  c = Controller(bus, 'session', tmp_path / 'counter.json', lambda: clock[0])
  clock[0] = 11.0
  bus.latest[0x3C1] = (11.0, bus.original)
  return clock, bus, c


def packet(direction='left', ident='one', now=11.0):
  return {'token': 'session', 'ok': True, 'guard_deadline': now + 0.34, 'events': [{'id': ident, 'direction': direction, 'deadline': now + 0.39}]}


@pytest.mark.parametrize('direction,count', [('left', 6), ('right', 7)])
def test_bounded_transaction_no_repeat_and_counter_persistence(tmp_path, direction, count):
  clock, bus, c = setup(tmp_path)
  command = packet(direction)
  c.accept(command)
  schedule = c.pending['schedule']
  for offset, _ in schedule:
    clock[0] = 11.0 + offset
    c.tick()
    c.accept(command)
  assert len(bus.sent) == count and bus.modes == [True, False]
  assert all(crc(b[1:]) == b[0] for b in bus.sent)
  assert json.loads(c.state_path.read_text())['counter'] == 5
  c.accept(command)
  assert c.pending is None
  c.accept(packet('right', ident='next', now=clock[0]))
  c.tick()
  assert c.pending is not None  # No five-second cooldown after idle feedback.


def test_recorded_vectors_and_counter_wrap_preserve_unrelated_bytes():
  original = bytes.fromhex('8f30300000000000')
  assert frames(original, 'left', 11)[0][1].hex() == '9cc0304010000000'
  right = frames(original, 'right', 13)
  assert right[0][1].hex() == '40e0300041000000'
  assert right[-1][1].hex() == '5900300000000000'
  assert right[0][1][2:3] + right[0][1][5:] == original[2:3] + original[5:]
  with pytest.raises(ValueError):
    frames(original, 'both', 3)
  with pytest.raises(ValueError):
    frames(original, 'left', 15)
  with pytest.raises(ValueError):
    frames(bytes(8), 'left', 3)


@pytest.mark.parametrize('change', ['token', 'guard', 'expiry', 'future', 'direction', 'inactive'])
def test_disallowed_commands_never_open_tx(tmp_path, change):
  clock, bus, c = setup(tmp_path)
  p = packet()
  if change == 'token':
    p['token'] = 'old-session'
  if change == 'guard':
    p['ok'] = False
  if change == 'expiry':
    p['events'][0]['deadline'] = 10.0
  if change == 'future':
    p['events'][0]['deadline'] = 30.0
  if change == 'direction':
    p['events'][0]['direction'] = 'both'
  if change == 'inactive':
    bus.idle = False
  c.accept(p)
  c.tick()
  assert not bus.sent and not bus.modes


@pytest.mark.parametrize('failure', ['guard', 'driver', 'late'])
def test_mid_transaction_failure_stops_without_catching_up(tmp_path, failure):
  clock, bus, c = setup(tmp_path)
  c.accept(packet())
  c.tick()
  assert len(bus.sent) == 1
  if failure == 'guard':
    c.guard_deadline = 0
  if failure == 'driver':
    bus.latest[0x3C1] = (11.0, bytes(8))
  if failure == 'late':
    clock[0] += 0.12
  c.tick()
  assert c.pending is None and c.status['accepted_id'] is None
  assert len(bus.sent) == 1


def test_no_worker_or_usb_traffic_without_explicit_peer_capability():
  client = SimpleNamespace()
  sender = protocol.Sender(client, {'carrot_host': 'jetson'})
  sender.send(client)
  sender.close()
  assert sender.worker is None


def test_source_and_wire_drop_old_future_disconnected_and_cancelled_input():
  source = Events(10)
  event = {'id': 'one', 'address': 'A', 'time': 11.0, 'action': 'blinkerLeft'}
  config = {'devices': {'A': {'enabled': True, 'mapping': {'left': 'blinkerLeft'}}}}
  status = {'time': 11.0, 'grabbed': ['A']}
  def read(**kw):
    return source.read({'events': [dict(event, **kw)]}, config, status, {}, {}, 11.1)
  assert read()[0]['direction'] == 'left'
  for changes in ({'time': 9.9}, {'time': 12}, {'time': float('nan')}, {'action': 'laneLeft'}, {'repeat': True}):
    assert read(**changes) == []
  assert source.read({'events': [event]}, config, status, {}, {'A': 11.0}, 11.1) == []
  status['grabbed'] = []
  assert read() == []
  value = {'stamp': 11.0, 'ok': True, 'events': [{'id': 'one', 'time': 10.9, 'direction': 'left'}]}
  wire = protocol.wire_record(value, 'session', 100.0, 11.1)
  assert wire['guard_deadline'] == pytest.approx(111.35) and wire['events'][0]['deadline'] == pytest.approx(111.3)
  assert protocol.wire_record(value, 'session', 100.0, 11.3) is None


def test_host_ignores_old_sequence_and_does_not_start_without_negotiation():
  import server
  session = object.__new__(server.CarrotSession)
  session.last_seq = 4
  sent = []
  session.blinker = SimpleNamespace(send=sent.append)
  session.handle(SimpleNamespace(msg_type=protocol.DATA, seq=4, payload=b'{}'))
  assert not sent
  host = protocol.Host()
  host.send(b'{}')
  assert host.worker is None


def test_sender_uses_separate_bounded_message_and_expires_clock_anchor(monkeypatch):
  clock, sent = [1.], []
  monkeypatch.setattr(protocol.time, 'monotonic', lambda: clock[0])
  value = {'stamp':1., 'ok':True, 'events':[{'id':'one','time':.95,'direction':'left'}]}
  monkeypatch.setattr(protocol, 'Process', lambda *a: SimpleNamespace(latest=lambda:value, close=lambda:None, send=lambda _:True))
  client = SimpleNamespace(last_state={}, _next_seq=lambda:1,
                           _expect=lambda *a:SimpleNamespace(payload=protocol.encode({'ready':True,'token':'t','clock':100.})),
                           t=SimpleNamespace(send_json=lambda *a:None, send=lambda *a:sent.append(a)))
  sender = protocol.Sender(client, {'carrot_host':'jetson',protocol.CAPABILITY:True})
  sender.send(client)
  kind, _, parts = sent[0]
  assert kind == protocol.DATA and len(parts) == 1 and isinstance(parts[0],bytes)
  assert json.loads(parts[0])['events'][0]['deadline'] == pytest.approx(100.35)
  clock[0] = 4.1
  value['stamp'] = clock[0]
  sender.send(client)
  assert len(sent) == 1
  sender.close()


@pytest.mark.parametrize('valid', [False, True])
def test_real_bus_setup_requires_one_shot_readback_before_opening_socket(monkeypatch, valid):
  bus = object.__new__(blink_can.Bus)
  bus.iface, bus.index, bus.sock = 'can1', '12', None
  calls, opened = [], []
  monkeypatch.setattr(blink_can, 'identity', lambda _: '12')
  def run(*args):
    calls.append(args)
    if args[0] == 'ip':
      return json.dumps([{'linkinfo': {'info_data': {'bittiming': {'bitrate': 500000},
                         'ctrlmode': ['ONE-SHOT'] if valid else ['LISTEN-ONLY']}}}])
    return ''
  bus.run = run
  def create(*args):
    opened.append(args)
    return SimpleNamespace(setsockopt=lambda *a: None, bind=lambda *a: None, setblocking=lambda *a: None)
  monkeypatch.setattr(blink_can.socket, 'socket', create)
  # Linux CAN constants are a device API; Windows does not expose them.
  for name, value in {'PF_CAN':29,'SOCK_RAW':3,'CAN_RAW':1,'SOL_CAN_RAW':101,'CAN_RAW_LOOPBACK':3,
                      'CAN_RAW_FILTER':1,'CAN_RAW_ERR_FILTER':2}.items():
    monkeypatch.setattr(blink_can.socket, name, value, raising=False)
  if valid:
    bus.mode(True)
    assert len(opened) == 1
  else:
    with pytest.raises(ValueError, match='CAN mode'):
      bus.mode(True)
    assert not opened
    assert not any(args[-1] == 'up' for args in calls)


class Vehicle(dict):
  def __init__(self, gear='drive', speed=25., engaged=True):
    super().__init__(carState=SimpleNamespace(gearShifter=gear, vEgo=speed, canValid=True, steeringAngleDeg=0., leftBlinker=False, rightBlinker=False,
                                            latEnabled=True, steerFaultTemporary=False, steerFaultPermanent=False),
                     carControl=SimpleNamespace(enabled=engaged, latActive=engaged, longActive=engaged),
                     selfdriveState=SimpleNamespace(enabled=engaged, active=engaged),
                     pandaStates=[SimpleNamespace(controlsAllowed=engaged)])
    self.services = list(self)
    self.valid = dict.fromkeys(self, True)
    self.alive = dict.fromkeys(self, True)
    self.recv_time = dict.fromkeys(self, 10.)


@pytest.mark.parametrize('gear,speed,engaged', [('park',0.,False),('drive',25.,False),('drive',25.,True),('reverse',-2.,False)])
def test_manual_indicator_allowed_while_driving_or_engaged(gear, speed, engaged):
  assert vehicle_ready(Vehicle(gear, speed, engaged), 10.1, 'HYUNDAI_IONIQ_5_PE')


@pytest.mark.parametrize('fault', ['stale','future','invalid','dead','can','speed','gear','panda','profile'])
def test_manual_indicator_still_rejects_unknown_vehicle_or_stale_state(fault):
  sm = Vehicle()
  fingerprint = 'HYUNDAI_IONIQ_5_PE'
  if fault == 'stale':
    sm.recv_time['carState'] = 9.
  if fault == 'future':
    sm.recv_time['carState'] = 11.
  if fault == 'invalid':
    sm.valid['carState'] = False
  if fault == 'dead':
    sm.alive['carState'] = False
  if fault == 'can':
    sm['carState'].canValid = False
  if fault == 'speed':
    sm['carState'].vEgo = float('nan')
  if fault == 'gear':
    sm['carState'].gearShifter = 'unknown'
  if fault == 'panda':
    sm['pandaStates'] = []
  if fault == 'profile':
    fingerprint = 'OTHER_VEHICLE'
  assert not vehicle_ready(sm, 10.1, fingerprint)


def test_disabled_local_profile_overrides_packaged_trial(tmp_path):
  path = tmp_path/'blinker.json'
  path.write_text(json.dumps({'enabled':False,'profile':protocol.PROFILE,'interface':'can1'}))
  assert protocol.configuration(path) is None
  path.write_text(json.dumps({'enabled':True,'profile':protocol.PROFILE,'interface':'can1'}))
  assert protocol.configuration(path)['interface'] == 'can1'


def test_optional_driver_never_loads_on_other_kernel(monkeypatch):
  import blink_driver
  monkeypatch.setattr(blink_driver.platform, 'release', lambda:'other-kernel')
  monkeypatch.setattr(blink_driver.subprocess, 'run', lambda *a,**kw:pytest.fail('unexpected driver operation'))
  assert blink_driver.prepare() is False


def test_bundled_driver_sources_match_pinned_manifest():
  import hashlib
  source = Path(__file__).with_name('peak_usb')
  manifest = json.loads((source/'sources.json').read_text())
  for name, entry in manifest.items():
    assert hashlib.sha256((source/name).read_bytes()).hexdigest() == entry['sha256']
    assert entry['url'].startswith('https://raw.githubusercontent.com/gregkh/linux/v5.15.148/')


def test_source_uses_time_after_receiving_vehicle_messages(monkeypatch):
  import blink_source as source
  import types
  from contextlib import nullcontext
  from openpilot.selfdrive.carrot.bluetooth import model
  clock, sent = [10.], []
  sm = Vehicle()
  def update(_):
    clock[0] += .001
    sm.recv_time = dict.fromkeys(sm.services, clock[0])
  sm.update = update
  fake_params = types.ModuleType('openpilot.common.params')
  fake_params.Params = lambda: SimpleNamespace(get=lambda _:b'car-params', get_int=lambda _:0)
  fake_cereal = types.ModuleType('openpilot.cereal')
  fake_cereal.messaging = SimpleNamespace(SubMaster=lambda _:sm)
  fake_cereal.car = SimpleNamespace(CarParams=SimpleNamespace(from_bytes=lambda _:nullcontext(SimpleNamespace(carFingerprint='HYUNDAI_IONIQ_5_PE'))))
  monkeypatch.setitem(sys.modules, 'openpilot.common.params', fake_params)
  monkeypatch.setitem(sys.modules, 'openpilot.cereal', fake_cereal)
  sock = SimpleNamespace(setblocking=lambda _:None, recv=lambda _:b'', send=lambda raw:sent.append(json.loads(raw)))
  monkeypatch.setattr(source.socket, 'socket', lambda **kw:sock)
  monkeypatch.setattr(source.select, 'select', lambda *a:([sock] if sent else [],[],[]))
  monkeypatch.setattr(source.time, 'monotonic', lambda:clock[0])
  monkeypatch.setattr(source.time, 'sleep', lambda _:None)
  monkeypatch.setattr(model, 'read_json', lambda *a:{})
  monkeypatch.setattr(model, 'config', dict)
  source.main(0)
  assert len(sent) == 1 and sent[0]['ok'] is True
  assert sent[0]['stamp'] == clock[0]
