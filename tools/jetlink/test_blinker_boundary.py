import math
from types import SimpleNamespace as N

import pytest

from openpilot.selfdrive.controls.lib.desire_lib.boundary_indicator import BoundaryIndicator
from blink_boundary import BoundaryEvents, vehicle_allowed
from blink_protocol import wire_record


def scene(direction=1):
  cs = N(
    leftBlinker=False,
    rightBlinker=False,
    blinkerLever=0,
    canValid=True,
    gearShifter='drive',
    trailerConnected=False,
    steerFaultTemporary=False,
    steerFaultPermanent=False,
    vEgo=15.0,
    steeringPressed=True,
    steeringTorque=300.0 if direction == 1 else -300.0,
  )
  md = N(frameId=0, laneLines=[N(y=[-5.4]), N(y=[-1.8]), N(y=[1.8]), N(y=[5.4])], laneLineProbs=[0.9] * 4)
  return cs, md


def drive(b, cs, md, direction=1, *, enabled=True, valid=True, busy=False):
  seen = []
  # Wait through startup's all-dark guard, then approach the selected boundary.
  for i in range(65):
    t = 10 + i * 0.05
    y = 1.8 - max(0, i - 30) * 0.04
    md.frameId = i
    md.laneLines[1].y = [-y if direction == 1 else -(3.6 - y)]
    md.laneLines[2].y = [3.6 - y if direction == 1 else y]
    b.update(t, cs, md, enabled=enabled, valid=valid, busy=busy)
    if b.request_direction:
      seen.append((t, b.request_direction, b.request_time))
  return seen


@pytest.mark.parametrize('direction', [1, 2])
def test_both_directions_need_real_driver_motion_and_emit_one_edge(direction):
  b = BoundaryIndicator()
  cs, md = scene(direction)
  seen = drive(b, cs, md, direction)
  assert seen and {v[1] for v in seen} == {direction}
  assert len({v[2] for v in seen}) == 1
  assert b.filter_lamp(direction) == 0


@pytest.mark.parametrize(
  'field,value',
  [
    ('steeringPressed', False),
    ('steeringTorque', -300.0),
    ('steeringTorque', math.nan),
    ('vEgo', 8.0),
    ('vEgo', math.nan),
    ('gearShifter', 'park'),
    ('canValid', False),
    ('trailerConnected', True),
    ('steerFaultTemporary', True),
    ('leftBlinker', True),
    ('rightBlinker', True),
    ('blinkerLever', 1),
  ],
)
def test_driver_vehicle_and_existing_signal_gates(field, value):
  b = BoundaryIndicator()
  cs, md = scene()
  setattr(cs, field, value)
  assert not drive(b, cs, md)
  assert not vehicle_allowed(cs, 1)


@pytest.mark.parametrize('key', ['enabled', 'valid', 'busy'])
def test_disabled_invalid_busy_do_not_request(key):
  cs, md = scene()
  assert not drive(BoundaryIndicator(), cs, md, **{key: key == 'busy'})


@pytest.mark.parametrize('probs', [[0.9, 0.01, 0.9, 0.9], [0.9, 0.9, 0.01, 0.9], [0.9, math.nan, 0.9, 0.9]])
def test_old_visible_line_is_not_enough(probs):
  cs, md = scene()
  md.laneLineProbs = probs
  assert not drive(BoundaryIndicator(), cs, md)


def test_inhibit_survives_disable_dark_flash_and_restart_journal():
  b = BoundaryIndicator()
  cs, md = scene()
  seen = drive(b, cs, md)
  stamp = seen[0][2]
  journal = {'time': stamp, 'direction': 1}
  instances = (b, BoundaryIndicator(journal))
  for b in instances:
    for i in range(300):
      now = stamp + 0.2 + i * 0.05
      cs.leftBlinker = (i % 16) < 8
      b.update(now, cs, md, enabled=False, valid=True, busy=False)
      assert b.filter_lamp(1) == 0
      assert b.request_direction == 0
    cs.leftBlinker = False
    for i in range(23):
      b.update(stamp + 16 + i * 0.05, cs, md, enabled=False, valid=True, busy=False)
    assert b.filter_lamp(1) == 1


def test_stale_future_or_malformed_journal_cannot_create_request():
  cs, md = scene()
  for journal in ({'time': -99.0, 'direction': 1}, {'time': 100.0, 'direction': 1}, [], {'time': math.nan, 'direction': 1}):
    b = BoundaryIndicator(journal)
    b.update(10.0, cs, md, enabled=False, valid=True, busy=False)
    assert b.blocked == 0 and b.request_direction == 0


def test_jumps_gaps_and_duplicate_frames_never_trigger():
  for mode in ('jump', 'gap', 'duplicate'):
    b = BoundaryIndicator()
    cs, md = scene()
    for i in range(100):
      md.frameId = i if mode != 'duplicate' else 1
      y = 1.8 if i % 2 else 0.6
      md.laneLines[1].y = [-y]
      md.laneLines[2].y = [3.6 - y]
      b.update(10 + i * (0.5 if mode == 'gap' else 0.05), cs, md, enabled=True, valid=True, busy=False)
      assert b.request_direction == 0


def test_emission_requires_origin_record_and_consumes_rejected_edges():
  b = BoundaryEvents()
  saved = []
  assert not b.read(10.0, 0, 0.0, allowed=True, record=saved.append)
  events = b.read(11.0, 1, 10.99, allowed=True, record=saved.append)
  assert saved == [{'time': 10.99, 'direction': 1}] and len(events) == 1
  assert not b.read(11.05, 1, 10.99, allowed=True, record=saved.append)
  assert not b.read(12.0, 2, 11.99, allowed=False, record=saved.append)
  assert not b.read(12.05, 2, 11.99, allowed=True, record=saved.append)

  def fail(_):
    raise OSError('read-only')

  assert not b.read(13.0, 1, 12.99, allowed=True, record=fail)
  wire = wire_record({'stamp': 11.0, 'ok': True, 'events': events, 'hold': None}, 'test', 0.0, 11.0)
  assert len(wire['events']) == 1 and wire['hold'] is None


def test_lamp_and_synthesized_stalk_feedback_cannot_request_steering(monkeypatch):
  import sys

  monkeypatch.setitem(sys.modules, 'openpilot.common.params', N(Params=lambda: N()))
  monkeypatch.setitem(sys.modules, 'openpilot.system.hardware', N(PC=True, TICI=False))
  from openpilot.selfdrive.controls.lib import desire_helper as dh
  from openpilot.selfdrive.controls.tests.test_desire_helper import TestDesireHelperDriverIntent
  from openpilot.cereal import log

  monkeypatch.setattr(dh, 'Params', lambda: N())
  t = TestDesireHelperDriverIntent()
  t.setup_method()
  t.helper.bluetooth_commands = N(read=lambda **kw: None)
  t.carrot_man.atcType = 'none'
  t.helper.boundary_indicator.blocked = 1
  from openpilot.selfdrive.controls.tests.test_desire_helper import make_car_state

  cs = make_car_state(True)
  cs.leftBlinkerStalkCount = 0
  cs.rightBlinkerStalkCount = 0
  for i in range(30):
    cs.leftBlinkerStalkCount = i
    t.helper.update(cs, N(), True, 0.1, t.carrot_man, N())
    assert t.helper.lane_change_state == log.LaneChangeState.off
    assert t.helper.desire == log.Desire.none
  t.helper.boundary_indicator.blocked = 0
  t.update(False)
  t.update(True)
  t.update(True)
  assert t.helper.lane_change_state == log.LaneChangeState.laneChangeStarting


def test_actual_source_loop_emits_once_after_journal_and_never_holds(monkeypatch):
  import sys
  import json
  import types
  from contextlib import nullcontext
  import blink_source as source
  from test_blinker import Vehicle
  from openpilot.selfdrive.carrot.bluetooth import model

  clock, sent, saved = [10.0], [], []
  sm = Vehicle()
  cs = sm['carState']
  base, _ = scene()
  for k, v in vars(base).items():
    setattr(cs, k, v)
  sm['modelV2'] = N(
    meta=N(
      desire='none', autoTurnSignal='none', autoTurnSignalLaneChange=False, laneChangeState='off', manualLaneIndicator='none', manualLaneIndicatorMonoTime=0
    )
  )
  sm['carrotMan'] = N(atcType='none', xDistToTurn=0)
  sm['livePose'] = N(inputsOK=True, sensorsOK=True, posenetOK=True, orientationNED=N(valid=True, z=0.0))
  sm.services = list(sm)
  sm.valid = dict.fromkeys(sm, True)
  sm.alive = dict.fromkeys(sm, True)

  def tick(_):
    i = len(sent)
    clock[0] = 10.0 + i * 0.05
    sm.recv_time = dict.fromkeys(sm, clock[0])
    sm['selfdriveState'].atcTurnStartMonoTime = 0
    sm['selfdriveState'].atcTurnDirection = 'none'
    sm['modelV2'].meta.manualLaneIndicator = 'left' if 2 <= i < 5 else 'none'
    sm['modelV2'].meta.manualLaneIndicatorMonoTime = 10100000000 if i >= 2 else 0

  sm.update = tick
  fake_params = types.ModuleType('openpilot.common.params')
  fake_params.Params = lambda: N(
    get=lambda _: b'cp', get_int=lambda k: 1 if k == 'AutoBlinkerControl' else 0, get_bool=lambda k: k == 'ManualLaneChangeIndicator'
  )
  fake_cereal = types.ModuleType('openpilot.cereal')
  fake_cereal.messaging = N(SubMaster=lambda _: sm)
  fake_cereal.car = N(CarParams=N(from_bytes=lambda _: nullcontext(N(carFingerprint='HYUNDAI_IONIQ_5_PE'))))
  monkeypatch.setitem(sys.modules, 'openpilot.common.params', fake_params)
  monkeypatch.setitem(sys.modules, 'openpilot.cereal', fake_cereal)
  fed = [-1]

  def recv(_):
    if len(sent) >= 8:
      return b''
    fed[0] = len(sent)
    return json.dumps({'stamp': 10.0 + len(sent) * 0.05, 'state': 'ready'}).encode()

  def send(raw):
    value = json.loads(raw)
    if value['events']:
      assert any(path.name == 'boundary-indicator.json' for path, _ in saved)
    sent.append(value)

  sock = N(setblocking=lambda _: None, recv=recv, send=send)
  monkeypatch.setattr(source.socket, 'socket', lambda **kw: sock)
  monkeypatch.setattr(source.select, 'select', lambda *a: ([sock] if len(sent) >= 8 or fed[0] != len(sent) else [], [], []))
  monkeypatch.setattr(source.time, 'monotonic', lambda: clock[0])
  monkeypatch.setattr(source.time, 'sleep', lambda _: None)
  monkeypatch.setattr(model, 'read_json', lambda *a: {})
  monkeypatch.setattr(model, 'config', dict)
  monkeypatch.setattr(model, 'atomic_json', lambda path, value: saved.append((path, value)))
  source.main(0)
  events = [e for v in sent for e in v['events']]
  assert len(events) == 1 and events[0]['direction'] == 'left'
  assert all(v['hold'] is None for v in sent)


@pytest.mark.parametrize("direction", [1, 2])
def test_real_capnp_boundary_request_roundtrip(direction):
  from openpilot.cereal import log

  event = log.Event.new_message()
  model = event.init("modelV2")
  model.meta.manualLaneIndicator = direction
  model.meta.manualLaneIndicatorMonoTime = 12345678900
  with log.Event.from_bytes(event.to_bytes()) as decoded:
    assert str(decoded.modelV2.meta.manualLaneIndicator) == ("left" if direction == 1 else "right")
    assert decoded.modelV2.meta.manualLaneIndicatorMonoTime == 12345678900
