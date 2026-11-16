import sys
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.car import hyundai_kick as kick
from tools.jetlink.blink_owner import Owner, GRACE, external_output_allowed  # noqa: TID251

# joongyu01/carrot-wip-dev: carrot-wip's KickOutput lease is inert here (HyundaiSafetyFlags.CANFD_BLINKER_KICK = 0, see
# values.py); our Kick runs on every CAN-FD / CGW1 car without it, tested in test_aprk_blinkers.py.
pytestmark = pytest.mark.skip(reason='KickOutput is inert on joongyu01/carrot-wip-dev (CANFD_BLINKER_KICK = 0)')


@pytest.fixture
def output(monkeypatch):
  values = {'CanfdBlinkerKick': True, 'IsOnroad': True}
  params = NS(get_bool=lambda key: values.get(key, False))
  cp = NS(brand='hyundai', flags=0, passive=False, safetyConfigs=[NS(safetyParam=32768)])
  owner = NS(held=False, renewed=10., acquire=lambda ready: setattr(owner, 'held', ready))
  monkeypatch.setattr(kick, 'Owner', lambda name: owner)
  return kick.KickOutput(cp, params), values


@pytest.mark.parametrize('onroad', [False, True])
@pytest.mark.parametrize('spas', [False, True])
@pytest.mark.parametrize('enabled', [False, True])
def test_external_gate(onroad, spas, enabled):
  values = {'IsOnroad':onroad, 'CanfdBlinkerControl':spas, 'CanfdBlinkerKick':enabled}
  assert external_output_allowed(NS(get_bool=values.get)) == (onroad and not spas and not enabled)


@pytest.fixture
def source_output(output, monkeypatch):
  gate, values = output
  incoming, feedback = {}, []
  worker = NS(process=NS(poll=lambda: None), latest=lambda: incoming,
              send=lambda value: feedback.append(value), close=lambda **kw: None)
  monkeypatch.setattr(kick, 'Process', lambda *args: worker)
  cs = NS(canValid=True, gearShifter='drive', blinkerLever=False, leftBlinker=False, rightBlinker=False)
  return gate, values, incoming, feedback, cs


def hold_record(record, now, ident='hold-1', direction='left', started=None):
  record.update(stamp=now, ok=True, events=[], hold={
    'id': ident, 'direction': direction, 'time': now if started is None else started, 'source': 'lane'})


def submitted(side):
  data = bytearray(32)
  data[16] = side << 5
  return [(0x16A, bytes(data), 0)]


def test_common_hold_acknowledgement_and_stop(source_output):
  gate, _, record, feedback, cs = source_output
  hold_record(record, 10.)
  assert gate.request(cs, 10., True, True) == 3
  gate.feedback(cs, 10., [])
  assert feedback[-1]['accepted_id'] is None
  gate.feedback(cs, 10.1, submitted(3))
  assert feedback[-1]['accepted_id'] == 'hold-1'
  for now in (11., 25., 60.):
    hold_record(record, now, started=10.)
    assert gate.request(cs, now, True, True) == 3
  record.update(stamp=60.1, hold=None)
  assert gate.request(cs, 60.1, True, True) == 0
  gate.feedback(cs, 60.1, [])
  assert feedback[-1]['accepted_id'] is None


@pytest.mark.parametrize('fault', ['old_record', 'future_record', 'invalid', 'owner', 'lever', 'hazard', 'gear', 'opposite', 'cancel'])
def test_interrupted_hold_cannot_resume_same_id(source_output, fault):
  gate, _, record, _, cs = source_output
  hold_record(record, 10.)
  assert gate.request(cs, 10., True, True) == 3
  hold_record(record, 10.1, started=10.)
  record['stamp'] = 9. if fault == 'old_record' else 11. if fault == 'future_record' else 10.1
  record['ok'] = fault != 'invalid'
  cs.blinkerLever = fault == 'lever'
  cs.leftBlinker = fault == 'hazard'
  cs.rightBlinker = fault in ('hazard', 'opposite')
  cs.gearShifter = 'park' if fault == 'gear' else 'drive'
  if fault == 'cancel':
    record['hold'] = None
  assert gate.request(cs, 10.1, True, fault != 'owner') == 0
  cs.blinkerLever = cs.leftBlinker = cs.rightBlinker = False
  cs.gearShifter = 'drive'
  hold_record(record, 10.2, started=10.)
  assert gate.request(cs, 10.2, True, True) == 0
  hold_record(record, 10.3, ident='hold-2')
  assert gate.request(cs, 10.3, True, True) == 3


def test_explicit_tap_is_once_and_bounded_after_handshake(source_output):
  gate, _, record, _, cs = source_output
  record.update(stamp=10., ok=True, hold=None, events=[{'id': 'tap', 'time':10., 'direction':'right'}])
  assert gate.request(cs, 10., True, True) == 4
  record.update(stamp=10.5, events=[])
  assert gate.request(cs, 10.5, True, True) == 4
  gate.feedback(cs, 10.5, submitted(4))
  record['stamp'] = 11.49
  assert gate.request(cs, 11.49, True, True) == 4
  record['stamp'] = 11.51
  assert gate.request(cs, 11.51, True, True) == 0


def test_output_toggle_closes_worker_without_replaying_old_request(source_output):
  gate, values, record, _, cs = source_output
  hold_record(record, 10.)
  assert gate.request(cs, 10., True, True) == 3
  values['CanfdBlinkerKick'] = False
  assert gate.request(cs, 10.1, True, False) == 0
  assert gate.worker is None
  values['CanfdBlinkerKick'] = True
  hold_record(record, 11.2, started=10.)
  assert gate.request(cs, 11.2, True, True) == 0


@pytest.mark.parametrize('mode', [0, 1, 2])
@pytest.mark.parametrize('selected', [False, True])
@pytest.mark.parametrize('legacy_mode', [0, 1, 2, 3, 4])
def test_common_detector_settings_match_external_output(mode, selected, legacy_mode):
  import ast
  from pathlib import Path
  path = Path(__file__).parents[2] / 'controls/lib/desire_helper.py'
  method = next(node for node in ast.walk(ast.parse(path.read_text(encoding='utf-8')))
                if isinstance(node, ast.FunctionDef) and node.name == '_update_params_periodic')
  scope = {'RUNTIME':Path('.'), 'read_json':lambda *args:{}, 'time':NS(monotonic=lambda:10.), 'CV':NS(KPH_TO_MS=1/3.6)}
  exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), scope)
  for native in (False, True):
    bools = dict.fromkeys(('ManualLaneChangeIndicator', 'ManualTurnIndicator', 'ModelDesireIndicator', 'ModelPathIndicator'), selected)
    bools['CanfdBlinkerKick'] = native
    ints = {'AutoBlinkerControl':mode, 'CanfdBlinkerKickModelMode':legacy_mode}
    reads = []
    def get_int(key):
      reads.append(key)
      return ints.get(key, 0)
    helper = NS(frame=100, kick_supported=native, external_indicator_supported=not native,
                boundary_indicator_supported=True, indicator_journal_time=0.,
                params=NS(get_bool=lambda key:bools.get(key, False), get_int=get_int, get_float=lambda key:0.))
    scope['_update_params_periodic'](helper)
    for name in ('boundary_indicator_enabled', 'manual_turn_indicator_enabled', 'model_desire_indicator_enabled', 'model_path_indicator_enabled'):
      assert getattr(helper, name) == (selected and mode in (1, 2))
    assert 'CanfdBlinkerKickModelMode' not in reads


def test_kick_uses_advertised_common_voice_route():
  from openpilot.selfdrive.car.voice_indicator import native_indicator
  from opendbc.car.hyundai.values import HyundaiFlags
  flags = HyundaiFlags.CANFD | HyundaiFlags.CANFD_HDA2 | HyundaiFlags.ENABLE_BLINKERS
  cp = NS(brand='hyundai', passive=False, flags=flags, safetyConfigs=[NS(safetyParam=32768)])
  assert native_indicator(cp) is None
  cp.safetyConfigs[0].safetyParam = 0
  assert native_indicator(cp) == 'SPAS2'


def test_toggle_and_stale_input_release_ownership(output):
  gate, values = output
  assert gate.update(True, 10.)
  values['CanfdBlinkerKick'] = False
  assert not gate.update(True, 10.01)
  assert not gate.owner.held
  values['CanfdBlinkerKick'] = True
  assert gate.update(True, 10.1)
  assert not gate.update(False, 10.11)
  assert not gate.owner.held
  values['IsOnroad'] = False
  assert not gate.update(True, 10.2)


@pytest.mark.parametrize('direction', [1, 2])
def test_waits_for_model_lamp_isolation(output, monkeypatch, tmp_path, direction):
  gate, _ = output
  monkeypatch.setattr(kick, 'RUNTIME', tmp_path)
  assert gate.update(True, 10.)
  data = bytearray(32)
  data[16] = (direction + 2) << 5
  frames = [(0x16A, bytes(data), 0), (0x165, bytes(24), 0), (0x1A0, bytes(32), 0)]
  model = NS(meta=NS(indicatorIsolationMask=0))
  assert gate.isolate(frames, model, 10.) == frames[2:]
  journal = kick.read_json(tmp_path/'boundary-indicator.json', {})
  assert journal['mask'] == direction
  model.meta.indicatorIsolationMask = direction
  assert gate.isolate(frames, model, 10.05) == frames
  gate.owner.held = False
  assert gate.isolate(frames, model, 10.06) == frames[2:]


def test_journal_failure_blocks_lamp(output, monkeypatch):
  gate, _ = output
  gate.update(True, 10.)
  monkeypatch.setattr(kick, 'read_json', lambda *args: {})
  def fail(*args):
    raise OSError('unavailable')
  monkeypatch.setattr(kick, 'atomic_json', fail)
  data = bytearray(32)
  data[16] = 3 << 5
  assert gate.isolate([(0x16A, data, 0)], NS(meta=NS(indicatorIsolationMask=1)), 10.) == []


def lever_frame(direction, stamp=10., bus=0):
  data = bytearray.fromhex('8f30300000000000')
  data[3] = 0x40 if direction & 1 else 0
  data[4] = 1 if direction & 2 else 0
  data[0] = kick.crc(data[1:])
  return (int(stamp * 1e9), [(0x3C1, bytes(data), bus)])


@pytest.mark.parametrize('direction', [1, 2])
def test_verified_driver_press_is_published_and_stops_native_output(output, direction):
  gate, _ = output
  gate.update(True, 10.)
  cs = NS(canValid=True)
  gate.observe_driver([lever_frame(0)], cs, 10.)
  assert cs.turnSignalDriverValid and cs.turnSignalDriverDirection == 0
  gate.observe_driver([lever_frame(direction, 10.01)], cs, 10.01)
  assert cs.turnSignalDriverValid and cs.turnSignalDriverDirection == direction
  assert (cs.leftBlinkerStalkCount, cs.rightBlinkerStalkCount) == (int(direction == 1), int(direction == 2))
  assert not gate.update(True, 10.01)
  assert gate.owner.held  # External adapters stay excluded while the driver takes over.


def test_short_driver_press_survives_release_in_same_can_batch(output):
  gate, _ = output
  gate.update(True, 10.)
  cs = NS(canValid=True)
  gate.observe_driver([lever_frame(0)], cs, 10.)
  gate.observe_driver([lever_frame(1, 10.01), lever_frame(0, 10.02)], cs, 10.02)
  assert cs.turnSignalDriverValid and cs.turnSignalDriverDirection == 0 and cs.leftBlinkerStalkCount == 1
  assert not gate.update(True, 10.02)


@pytest.mark.parametrize('enabled', [False, True])
def test_ordinary_stalk_counters_work_without_kick_ownership(output, enabled):
  gate, values = output
  values['CanfdBlinkerKick'] = enabled
  cs = NS(canValid=True, leftBlinkerStalkCount=254, rightBlinkerStalkCount=7)
  gate.observe_driver([], cs, 10.)
  assert not cs.turnSignalDriverValid
  assert (cs.leftBlinkerStalkCount, cs.rightBlinkerStalkCount) == (254, 7)
  cs = NS(canValid=True, leftBlinkerStalkCount=1, rightBlinkerStalkCount=8)
  gate.observe_driver([], cs, 10.01)
  assert (cs.leftBlinkerStalkCount, cs.rightBlinkerStalkCount) == (1, 8)


def test_counter_source_handoff_does_not_create_a_press(output):
  gate, values = output
  gate.update(True, 10.)
  cs = NS(canValid=True, leftBlinkerStalkCount=7, rightBlinkerStalkCount=2)
  gate.observe_driver([lever_frame(0)], cs, 10.)
  # An unverified legacy count must not enter the trusted counter stream.
  cs = NS(canValid=True, leftBlinkerStalkCount=8, rightBlinkerStalkCount=2)
  gate.observe_driver([lever_frame(0, 10.01)], cs, 10.01)
  assert cs.leftBlinkerStalkCount == 7
  values['CanfdBlinkerKick'] = False
  gate.update(True, 10.02)
  cs = NS(canValid=True, leftBlinkerStalkCount=8, rightBlinkerStalkCount=2)
  gate.observe_driver([], cs, 10.02)
  assert cs.leftBlinkerStalkCount == 7
  cs = NS(canValid=True, leftBlinkerStalkCount=9, rightBlinkerStalkCount=3)
  gate.observe_driver([], cs, 10.03)
  assert (cs.leftBlinkerStalkCount, cs.rightBlinkerStalkCount) == (8, 3)
  values['CanfdBlinkerKick'] = True
  gate.update(True, 10.04)
  cs = NS(canValid=True, leftBlinkerStalkCount=9, rightBlinkerStalkCount=3)
  gate.observe_driver([lever_frame(0, 10.04)], cs, 10.04)
  assert (cs.leftBlinkerStalkCount, cs.rightBlinkerStalkCount) == (8, 3)
  cs = NS(canValid=True, leftBlinkerStalkCount=10, rightBlinkerStalkCount=3)
  gate.observe_driver([lever_frame(1, 10.05)], cs, 10.05)
  assert (cs.leftBlinkerStalkCount, cs.rightBlinkerStalkCount) == (9, 3)


@pytest.mark.parametrize('blocked', ['owner', 'old_owner', 'off', 'offroad', 'invalid_can', 'bad_crc',
                                   'wrong_bus', 'tx_echo', 'old_frame', 'future_frame', 'short_frame'])
def test_unverified_lever_cannot_bypass_lamp_isolation(output, blocked):
  gate, values = output
  gate.update(True, 10.)
  cs = NS(canValid=blocked != 'invalid_can')
  if blocked == 'owner':
    gate.owner.held = False
  if blocked == 'old_owner':
    gate.owner.renewed = 9.
  values['CanfdBlinkerKick'] = blocked != 'off'
  values['IsOnroad'] = blocked != 'offroad'
  gate.observe_driver([lever_frame(0)], cs, 10.)
  frame = lever_frame(1, 9. if blocked == 'old_frame' else 11. if blocked == 'future_frame' else 10.01,
                      1 if blocked == 'wrong_bus' else 0x80 if blocked == 'tx_echo' else 0)
  if blocked in ('bad_crc', 'short_frame'):
    data = frame[1][0][1]
    frame[1][0] = (0x3C1, bytes([data[0] ^ 1]) + data[1:] if blocked == 'bad_crc' else data[:7], 0)
  gate.observe_driver([frame], cs, 10.01)
  assert cs.turnSignalDriverDirection == 0 and cs.leftBlinkerStalkCount == 0


def test_lease_handoff_and_stale_recovery_require_neutral(output):
  gate, _ = output
  gate.update(True, 10.)
  cs = NS(canValid=True)
  gate.observe_driver([lever_frame(1)], cs, 10.)
  assert not cs.turnSignalDriverValid
  gate.observe_driver([lever_frame(0, 10.01), lever_frame(1, 10.02)], cs, 10.02)
  assert cs.turnSignalDriverValid and cs.leftBlinkerStalkCount == 1
  gate.owner.renewed = 10.5
  gate.observe_driver([], cs, 10.5)
  assert not cs.turnSignalDriverValid
  gate.observe_driver([lever_frame(1, 10.51)], cs, 10.51)
  assert not cs.turnSignalDriverValid and cs.leftBlinkerStalkCount == 1
  gate.observe_driver([lever_frame(0, 10.52), lever_frame(1, 10.53)], cs, 10.53)
  assert cs.turnSignalDriverValid and cs.leftBlinkerStalkCount == 2


def test_spas_lamp_frames_never_create_a_driver_press(output):
  gate, _ = output
  gate.update(True, 10.)
  cs = NS(canValid=True)
  gate.observe_driver([lever_frame(0)], cs, 10.)
  data = bytearray(32)
  data[16] = 3 << 5
  gate.observe_driver([(10_010_000_000, [(0x16A, bytes(data), 0), (0x413, bytes(8), 0)])], cs, 10.01)
  assert cs.turnSignalDriverValid and cs.turnSignalDriverDirection == 0 and cs.leftBlinkerStalkCount == 0


@pytest.mark.parametrize('direction', [1, 2])
@pytest.mark.parametrize('quick_release', [False, True])
def test_verified_driver_is_recognized_while_native_lamp_mask_is_active(output, direction, quick_release):
  import ast
  from pathlib import Path
  from openpilot.selfdrive.controls.lib.desire_lib.boundary_indicator import BoundaryIndicator

  # Execute the production decoding methods without platform-only modeld imports.
  path = Path(__file__).parents[2] / 'controls/lib/desire_helper.py'
  methods = [n for n in ast.walk(ast.parse(path.read_text(encoding='utf-8')))
             if isinstance(n, ast.FunctionDef) and n.name in ('_update_driver_blinker', '_update_stalk')]
  ns = dict(BLINKER_NONE=0, BLINKER_LEFT=1, BLINKER_RIGHT=2, DT_MDL=.05)
  exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), 'exec'), ns)
  observer = BoundaryIndicator()
  observer.own(10., direction)
  helper = NS(boundary_indicator=observer, kick_supported=True, tesla_auto_signals=False,
              _driver_signal_counts=None, _manual_signal_direction=0, _manual_signal_grace=0.,
              driver_blinker_state=0, laneChangeNeedTorque=0, stalk_counts=None)
  gate, _ = output
  gate.update(True, 10.)
  cs = NS(canValid=True, leftBlinker=direction == 1, rightBlinker=direction == 2)
  gate.observe_driver([lever_frame(0)], cs, 10.)
  assert ns['_update_driver_blinker'](helper, cs)[0] == 0
  assert ns['_update_stalk'](helper, cs) == 0
  frames = [lever_frame(direction, 10.01)]
  if quick_release:
    frames.append(lever_frame(0, 10.02))
  gate.observe_driver(frames, cs, 10.02)
  assert ns['_update_driver_blinker'](helper, cs) == (direction, True, True)
  assert ns['_update_stalk'](helper, cs) == direction
  assert observer.blocked == direction and observer.isolate_until == 22.
  gate.observe_driver([lever_frame(0, 10.03)], cs, 10.03)
  assert ns['_update_driver_blinker'](helper, cs)[0] == direction
  assert ns['_update_stalk'](helper, cs) == 0
  gate.owner.held = False
  gate.observe_driver([], cs, 10.04)
  assert ns['_update_driver_blinker'](helper, cs)[0] == 0


@pytest.mark.skipif(sys.platform != 'linux', reason='real fcntl ownership')
@pytest.mark.parametrize('external_name', ['USB2CAN', 'BT2CAN'])
def test_native_waits_for_external_release_and_receiver_expiry(output, tmp_path, external_name):
  gate, values = output
  clock = [10.]
  external = Owner(external_name, tmp_path, lambda: clock[0])
  gate.owner = Owner('CANFD Kick', tmp_path, lambda: clock[0])
  assert external.acquire(True)
  assert not gate.update(True, clock[0])
  external.release()
  clock[0] += .1
  assert not gate.update(True, clock[0])
  clock[0] += GRACE
  assert gate.update(True, clock[0])
  assert not external.acquire(True)
  values['CanfdBlinkerKick'] = False
  assert not gate.update(True, clock[0])
  assert not external.acquire(True)
  clock[0] += GRACE + .01
  assert external.acquire(True)


@pytest.mark.parametrize('blocked', [None, 'stale_model', 'stale_control', 'invalid_can', 'dashcam', 'off', 'no_isolation', 'driver'])
def test_production_card_send_boundary(output, monkeypatch, tmp_path, blocked):
  import ast
  from pathlib import Path
  from openpilot.cereal import car, log
  from openpilot.selfdrive.pandad import can_list_to_can_capnp

  gate, values = output
  record = {}
  hold_record(record, 10.)
  worker = NS(process=NS(poll=lambda:None), latest=lambda:record, send=lambda value:True, close=lambda **kw:None)
  monkeypatch.setattr(kick, 'Process', lambda *args:worker)
  values['ImpactDashcamReboot'] = blocked == 'dashcam'
  values['CanfdBlinkerKick'] = blocked != 'off'
  monkeypatch.setattr(kick, 'RUNTIME', tmp_path)
  path = Path(__file__).parents[1] / 'card.py'
  method = next(n for n in ast.walk(ast.parse(path.read_text(encoding='utf-8')))
                if isinstance(n, ast.FunctionDef) and n.name == 'controls_update')
  ns = {'car':car, 'REPLAY':False, 'can_list_to_can_capnp':can_list_to_can_capnp,
        'time':NS(monotonic=lambda:10., monotonic_ns=lambda:10_000_000_000)}
  exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), ns)
  model = log.ModelDataV2.new_message()
  model.meta.indicatorIsolationMask = 0 if blocked == 'no_isolation' else 1
  class Messages(dict):
    def all_alive(self, services):
      return True
  sm = Messages(modelV2=model.as_reader(), radarState=None)
  sm.valid = sm.alive = dict.fromkeys(('carControl', 'modelV2', 'radarState'), True)
  sm.recv_time = dict.fromkeys(sm.valid, 10.)
  if blocked in ('stale_model', 'stale_control'):
    sm.recv_time['modelV2' if blocked == 'stale_model' else 'carControl'] = 9.
  controller = NS(kick_output_ready=False)
  payload = bytearray(32)
  payload[16] = 3 << 5
  def apply(*args):
    frames = [(0x16A, bytes(payload), 0)] if controller.kick_output_ready and controller.kick_direction else []
    return car.CarControl.Actuators.new_message().as_reader(), frames
  stages = ('apply', 'sendcan', 'total')
  sent = []
  card = NS(kick_output=gate, spas_signals=NS(enabled=False), native_voice_indicator=None, initialized_prev=True,
            params=gate.params, sm=sm, CI=NS(CC=controller, apply=apply), tesla_signals=NS(configured=False),
            lateral_startup=NS(update=lambda *args:True), pm=NS(send=lambda *args:sent.append(args)),
            card_diag_recv_ns=10_000_000_000, card_diag_stage_names=stages,
            card_diag_stage_current=dict.fromkeys(stages, 0), card_diag_stage_sum_us=dict.fromkeys(stages, 0),
            card_diag_stage_max_us=dict.fromkeys(stages, 0), card_diag_process_max_us=0,
            card_diag_slow_process=0, card_diag_frames=0)
  cs = car.CarState.new_message(canValid=blocked != 'invalid_can', gearShifter='drive')
  if blocked == 'driver':
    gate.update(True, 10.)
    gate.observe_driver([lever_frame(0, 9.99), lever_frame(1, 10.)], cs, 10.)
  ns['controls_update'](card, cs.as_reader(), car.CarControl.new_message().as_reader())
  with log.Event.from_bytes(sent[-1][1]) as event:
    assert len(event.sendcan) == int(blocked is None)
