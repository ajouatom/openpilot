import sys
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.car import hyundai_kick as kick
from tools.jetlink.blink_owner import Owner, GRACE, external_output_allowed  # noqa: TID251


@pytest.fixture
def output(monkeypatch):
  values = {'CanfdBlinkerKick': True, 'IsOnroad': True}
  params = NS(get_bool=lambda key: values.get(key, False))
  cp = NS(brand='hyundai', passive=False, safetyConfigs=[NS(safetyParam=32768)])
  owner = NS(held=False, acquire=lambda ready: setattr(owner, 'held', ready))
  monkeypatch.setattr(kick, 'Owner', lambda name: owner)
  return kick.KickOutput(cp, params), values


@pytest.mark.parametrize('onroad', [False, True])
@pytest.mark.parametrize('spas', [False, True])
@pytest.mark.parametrize('enabled', [False, True])
def test_external_gate(onroad, spas, enabled):
  values = {'IsOnroad':onroad, 'CanfdBlinkerControl':spas, 'CanfdBlinkerKick':enabled}
  assert external_output_allowed(NS(get_bool=values.get)) == (onroad and not spas and not enabled)


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


@pytest.mark.parametrize('blocked', [None, 'stale_model', 'stale_control', 'invalid_can', 'dashcam', 'off', 'no_isolation'])
def test_production_card_send_boundary(output, monkeypatch, tmp_path, blocked):
  import ast
  from pathlib import Path
  from openpilot.cereal import car, log
  from openpilot.selfdrive.pandad import can_list_to_can_capnp

  gate, values = output
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
    frames = [(0x16A, bytes(payload), 0)] if controller.kick_output_ready else []
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
  ns['controls_update'](card, cs.as_reader(), car.CarControl.new_message().as_reader())
  with log.Event.from_bytes(sent[-1][1]) as event:
    assert len(event.sendcan) == int(blocked is None)
