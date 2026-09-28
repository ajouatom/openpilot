import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.cereal import car, log
from openpilot.selfdrive.monitoring import dm2d, policy
from openpilot.selfdrive.monitoring.dm2 import DriverMonitoring2


class Messages(dict):
  healthy = True

  def __init__(self, **messages):
    super().__init__(messages)
    self.logMonoTime = dict.fromkeys(messages, int(100e9))

  def all_checks(self, _):
    return self.healthy


def parked_messages():
  return Messages(carState=car.CarState.new_message(canValid=True, gearShifter='park', standstill=True),
                  selfdriveState=log.SelfdriveState.new_message(enabled=False, active=False))


@pytest.mark.parametrize('fault', ['drive', 'neutral', 'unknown', 'reverse', 'raw_speed', 'filtered_speed',
                                 'nan_speed', 'nan_raw', 'not_stopped', 'enabled', 'active', 'can_invalid',
                                 'unhealthy', 'stale_car', 'stale_control', 'future', 'demo'])
def test_invalid_or_nonparked_state_never_unlocks(fault):
  sm = parked_messages()
  cs, state = sm['carState'], sm['selfdriveState']
  if fault in ('drive', 'neutral', 'unknown', 'reverse'):
    cs.gearShifter = fault
  elif fault in ('raw_speed', 'nan_raw'):
    cs.vEgoRaw = .001 if fault == 'raw_speed' else float('nan')
  elif fault in ('filtered_speed', 'nan_speed'):
    cs.vEgo = .02 if fault == 'filtered_speed' else float('nan')
  elif fault == 'not_stopped':
    cs.standstill = False
  elif fault in ('enabled', 'active'):
    setattr(state, fault, True)
  elif fault == 'can_invalid':
    cs.canValid = False
  elif fault == 'unhealthy':
    sm.healthy = False
  elif fault in ('stale_car', 'stale_control'):
    sm.logMonoTime['carState' if fault == 'stale_car' else 'selfdriveState'] = int(99e9)
  elif fault == 'future':
    sm.logMonoTime['carState'] = int(101e9)
  assert not dm2d.parked_reset_eligible(sm, 100., demo=fault == 'demo')


@pytest.mark.parametrize('experimental', [False, True])
@pytest.mark.parametrize('camera', [False, True])
@pytest.mark.parametrize('always_on', [False, True])
def test_confirmed_parking_clears_warning_counts_and_both_lockouts(experimental, camera, always_on):
  dm = DriverMonitoring2(experimental=experimental, always_on=always_on)
  dm.configure_context(100., camera)
  dm.too_distracted = True
  dm.alert_3_cnt, dm.no_response_cnt, dm.cnt_since_alert_3 = 2, 1, 120
  dm.lockout_time = 100
  dm.awareness = -.1
  dm.alert_level = policy.AlertLevel.three
  dm.timing_crossed_terminal = True
  sm = parked_messages()
  assert dm2d.parked_reset_eligible(sm, 100.)
  for frame in range(20):
    assert not dm.update_parked_reset(100. + frame * .05, True)
    assert dm.too_distracted
  assert dm.update_parked_reset(101., True)
  packet = dm.get_state_packet().driverMonitoringState
  assert not packet.lockout and not packet.alwaysOnLockout and not packet.noResponseForceDecel
  assert dm.awareness == dm.last_vision_awareness == dm.last_wheeltouch_awareness == 1.
  assert dm.alert_level == policy.AlertLevel.none
  assert (dm.alert_3_cnt, dm.no_response_cnt, dm.cnt_since_alert_3, dm.lockout_time) == (0, 0, 0, 0)
  assert not dm.timing_crossed_terminal and dm.interaction_grace_remaining == 0
  assert not dm.update_parked_reset(101.05, True)  # one reset per parking stop
  # Normal monitoring resumes afterwards; the reset never disables DM.
  dm.update_parked_reset(101.1, False)
  dm.configure_context(101.1, False)
  for _ in range(301):
    dm.run_without_camera(False, True, False, False)
  assert dm.alert_level == policy.AlertLevel.one


@pytest.mark.parametrize('interruption', ['invalid', 'gap', 'backwards'])
def test_partial_park_confirmation_cannot_survive_a_gap(interruption):
  dm = DriverMonitoring2()
  dm.too_distracted = True
  for frame in range(19):
    assert not dm.update_parked_reset(100. + frame * .05, True)
  now = {'invalid': 100.95, 'gap': 102., 'backwards': 99.}[interruption]
  assert not dm.update_parked_reset(now, interruption != 'invalid')
  for frame in range(1, 20):
    assert not dm.update_parked_reset(now + frame * .05, True)
  assert dm.too_distracted


def persistence_method():
  path = Path(__file__).resolve().parents[1] / 'selfdrived/selfdrived.py'
  cls = next(n for n in ast.parse(path.read_text(encoding='utf8')).body if isinstance(n, ast.ClassDef) and n.name == 'SelfdriveD')
  method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'update_dm_lockout')
  namespace = {'time': SimpleNamespace(monotonic=lambda: 100.)}
  exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
  return namespace['update_dm_lockout']


def test_saved_lockout_clears_on_valid_release_and_can_lock_again(monkeypatch):
  class Params:
    locked = True
    writes = []

    def get_bool(self, key):
      assert key == 'DriverTooDistracted'
      return self.locked

    def put_bool(self, key, value):
      assert key == 'DriverTooDistracted' and isinstance(value, bool)
      self.locked = value
      self.writes.append(value)

  params = Params()
  sm = Messages(driverMonitoringState=log.DriverMonitoringState.new_message(lockout=False))
  state = SimpleNamespace(params=params, sm=sm, dm_lockout_set=True)
  update = persistence_method()
  sm.healthy = False
  update(state)
  sm.healthy = True
  sm.logMonoTime['driverMonitoringState'] = int(99e9)
  update(state)
  assert params.locked and params.writes == []
  sm.logMonoTime['driverMonitoringState'] = int(100e9)
  update(state)
  update(state)
  assert not params.locked and params.writes == [False]
  monkeypatch.setattr(policy, 'Params', lambda: params)
  assert not DriverMonitoring2().too_distracted  # DM restart cannot restore old lock
  sm['driverMonitoringState'].lockout = True
  update(state)
  assert params.locked and params.writes == [False, True]


@pytest.mark.parametrize('experimental', [False, True])
@pytest.mark.parametrize('camera', [False, True])
def test_dispatcher_publishes_parked_release(monkeypatch, experimental, camera):
  sm = parked_messages()
  sm.update(modelV2=log.ModelDataV2.new_message(), radarState=log.RadarState.new_message())
  sm.logMonoTime['driverStateV2'] = 0
  sm.updated = {'carParams': False, 'radarState': True, 'driverStateV2': False}
  clock, packets = [100.], []
  dm = DriverMonitoring2(experimental=experimental)
  dm.too_distracted = True
  dm.alert_3_cnt, dm.no_response_cnt = 2, 1
  dm.awareness = -.1

  def update(_):
    clock[0] += .05
    for service in ('carState', 'selfdriveState', 'driverStateV2'):
      sm.logMonoTime[service] = int(clock[0] * 1e9)
  sm.update = update
  monkeypatch.setattr(dm2d.messaging, 'SubMaster', lambda *a, **k: sm)
  monkeypatch.setattr(dm2d.messaging, 'PubMaster', lambda *a: SimpleNamespace(send=lambda _, p: packets.append(p.to_dict())), raising=False)
  monkeypatch.setattr(dm2d.messaging, 'sub_sock', lambda *a, **k: None, raising=False)
  monkeypatch.setattr(dm2d.messaging, 'drain_sock', lambda *a, **k: [], raising=False)
  monkeypatch.setattr(dm2d, 'DriverMonitoring2', lambda **k: dm)
  monkeypatch.setattr(dm2d, 'CommandReader', lambda *a: SimpleNamespace(read=lambda **k: None))
  monkeypatch.setattr(dm2d, 'CameraAvailability', lambda: SimpleNamespace(update=lambda *a: camera))
  monkeypatch.setattr(dm2d, 'camera_sample_usable', lambda *a: camera)
  monkeypatch.setattr(dm2d.time, 'monotonic', lambda: clock[0])

  class Done(Exception):
    pass

  class Rate:
    frame = 0

    def keep_time(self):
      self.frame += 1
      if self.frame == 25:
        raise Done
  monkeypatch.setattr(dm2d, 'Ratekeeper', lambda *a, **k: Rate())
  with pytest.raises(Done):
    dm2d.run_dm2(SimpleNamespace(get_bool=lambda _: False), experimental)
  assert packets[0]['driverMonitoringState']['lockout']
  assert packets[-1]['valid'] and not packets[-1]['driverMonitoringState']['lockout']
